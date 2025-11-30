#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>

using boost::asio::serial_port_base;
namespace asio = boost::asio;

using quat_t = std::array<double,4>; // x, y, z, w

// --------- Inline quaternion / euler helpers ----------
static inline quat_t quat_normalize(const quat_t &q) {
	double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	if (n < 1e-12) return {0.0, 0.0, 0.0, 1.0};
	return {q[0]/n, q[1]/n, q[2]/n, q[3]/n};
}

static inline quat_t quat_multiply(const quat_t &a, const quat_t &b) {
	// Hamilton product: q = a * b
	quat_t r;
	r[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]; // w
	r[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1]; // x
	r[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0]; // y
	r[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3]; // z
	return r;
}

static inline double quat_dot(const quat_t &a, const quat_t &b) {
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

static inline quat_t quat_slerp(const quat_t &q1_in, const quat_t &q2_in, double t) {
	// Slerp with fallback to lerp for small angles
	quat_t q1 = q1_in;
	quat_t q2 = q2_in;
	double cosom = quat_dot(q1, q2);

	// if opposite, flip one to take shorter path
	if (cosom < 0.0) {
		cosom = -cosom;
		q2 = {-q2[0], -q2[1], -q2[2], -q2[3]};
	}

	const double EPS = 1e-6;
	if ((1.0 - cosom) > EPS) {
		double omega = std::acos(cosom);
		double sinom = std::sin(omega);
		double s1 = std::sin((1.0 - t) * omega) / sinom;
		double s2 = std::sin(t * omega) / sinom;
		quat_t r = {
			q1[0]*s1 + q2[0]*s2,
			q1[1]*s1 + q2[1]*s2,
			q1[2]*s1 + q2[2]*s2,
			q1[3]*s1 + q2[3]*s2
		};
		return quat_normalize(r);
	} else {
		// very close - use linear interpolation
		quat_t r = {
			q1[0] * (1.0 - t) + q2[0] * t,
			q1[1] * (1.0 - t) + q2[1] * t,
			q1[2] * (1.0 - t) + q2[2] * t,
			q1[3] * (1.0 - t) + q2[3] * t 
		};
		return quat_normalize(r);
	}
}

// Convert roll, pitch, yaw (rad) -> quaternion (x,y,z,w)
static inline quat_t euler_to_quat(double roll, double pitch, double yaw) {
	double cy = std::cos(yaw * 0.5);
	double sy = std::sin(yaw * 0.5);
	double cp = std::cos(pitch * 0.5);
	double sp = std::sin(pitch * 0.5);
	double cr = std::cos(roll * 0.5);
	double sr = std::sin(roll * 0.5);

	quat_t q;
	q[3] = cr * cp * cy + sr * sp * sy; // w
	q[0] = sr * cp * cy - cr * sp * sy; // x
	q[1] = cr * sp * cy + sr * cp * sy; // y
	q[2] = cr * cp * sy - sr * sp * cy; // z
	return q;
}

// Extract roll, pitch, yaw from quaternion
static inline std::array<double,3> quat_to_euler(const quat_t &q) {
	// q: x,y,z,w
	double x = q[0], y = q[1], z = q[2], w = q[3];

	// roll (x-axis rotation)
	double sinr_cosp = 2.0 * (w * x + y * z);
	double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
	double roll = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis)
	double sinp = 2.0 * (w * y - z * x);
	double pitch;
	if (std::fabs(sinp) >= 1.0)
		pitch = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
	else
		pitch = std::asin(sinp);

	// yaw (z-axis)
	double siny_cosp = 2.0 * (w * z + x * y);
	double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
	double yaw = std::atan2(siny_cosp, cosy_cosp);

	return {roll, pitch, yaw};
}

// Build small rotation quaternion from angular velocity vector (rad/s) and dt
static inline quat_t delta_quat_from_omega(double wx, double wy, double wz, double dt) {
	double omega_norm = std::sqrt(wx*wx + wy*wy + wz*wz);

	if (omega_norm < 1e-12) {
		return {0.0, 0.0, 0.0, 1.0};
	}

	double theta_over_two = 0.5 * omega_norm * dt;
	double s = std::sin(theta_over_two);
	double c = std::cos(theta_over_two);
	double ux = wx / omega_norm;
	double uy = wy / omega_norm;
	double uz = wz / omega_norm;

	return {
		ux * s,
		uy * s,
		uz * s,
		c
	};
}

class RP2040IMUNode {
public:
	RP2040IMUNode(ros::NodeHandle& nh): io(), serial(io), nh_(nh){
		nh_.param<std::string>("port", param_port, "/dev/ttyIMU");
		nh_.param<int>("baud_rate", param_baud_rate, 115200);
		nh_.param<double>("accel_correction_gain", accel_gain, 0.03);
		nh_.param<double>("gyro_scale", gyro_scale, 1.15);

		imu_raw_pub = nh_.advertise<sensor_msgs::Imu>("/rp2040_imu/data_raw", 20);
		imu_data_pub = nh_.advertise<sensor_msgs::Imu>("/rp2040_imu/data", 20);
		temp_pub = nh_.advertise<sensor_msgs::Temperature>("/rp2040_imu/temperature", 1, true);

		orientation = {0.0, 0.0, 0.0, 1.0};
		last_time = ros::Time::now();
		fused_prescaler = 0;

		openSerial();
		ROS_INFO("rp2040_imu_node started. port: %s, baud: %d, accel_gain: %f", param_port.c_str(), param_baud_rate, accel_gain);
	}

	//cleanup
	~RP2040IMUNode() {
		try {
			if (serial.is_open()) serial.close();
		} catch(...) {}
	}

	void spin() {
		asio::streambuf buf;
		std::istream is(&buf);

		while (ros::ok()) {
			try {
				// read until newline (blocks, but ok here)
				std::size_t n = asio::read_until(serial, buf, '\n');
				std::string line;
				std::getline(is, line);
				boost::algorithm::trim(line);

				if (line.size() >= 2 && line.front() == '#' && line.back() == '#') {
					std::string payload = line.substr(1, line.size()-2);
					std::vector<std::string> toks;
					boost::split(toks, payload, boost::is_any_of(","), boost::token_compress_on);
					if (toks.size() == 6) {
						double ax = std::stod(toks[0]);
						double ay = std::stod(toks[1]);
						double az = std::stod(toks[2]);
						double gx = std::stod(toks[3]);
						double gy = std::stod(toks[4]);
						double gz = std::stod(toks[5]);
						publishImu(ax, ay, az, gx, gy, gz);
					}
					else if (toks.size() == 1) {
						double temp = std::stod(toks[0]);
						publishTemperature(temp);
					}
				}

				if (buf.size() > 4096){
					buf.consume(buf.size());
				}
			} catch (std::exception &e) {
				ROS_WARN("IMU disconnected, trying to reconnect...");
				try {
					serial.close();
				} catch(...) {}

				ros::Duration(2.0).sleep();
				openSerial();
			}
			ros::spinOnce();
		}
	}

private:

	ros::NodeHandle nh_;

	ros::Publisher temp_pub;
	ros::Publisher imu_raw_pub;
	ros::Publisher imu_data_pub;

	std::string param_port;
	int param_baud_rate;
	double accel_gain;
	double gyro_scale;

	asio::io_context io;
	asio::serial_port serial;

	quat_t orientation;
	ros::Time last_time;
	int fused_prescaler;

	void openSerial() {
		try {
			if (serial.is_open())
				serial.close();

			serial.open(param_port);
			serial.set_option(serial_port_base::baud_rate(param_baud_rate));
			serial.set_option(serial_port_base::character_size(8));
			serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
			serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
			serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
			ROS_INFO("Opened serial port %s @ %d", param_port.c_str(), param_baud_rate);
		} catch (std::exception &e) {
			ROS_ERROR("Failed to open serial port %s: %s", param_port.c_str(), e.what());
		}
	}

	void publishTemperature(double value) {
		sensor_msgs::Temperature msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "rp2040_imu_link";
		msg.temperature = value;
		msg.variance = 0.0;
		temp_pub.publish(msg);
	}

	void publishImu(double ax_g, double ay_g, double az_g, double gx_deg, double gy_deg, double gz_deg) {
		// Time delta
		ros::Time now = ros::Time::now();
		double dt = (now - last_time).toSec();
		last_time = now;
		if (dt <= 0.0 || dt > 0.05) {
			// if abnormal dt, assume standard 0.01 (or simply skip gyro integration)
			dt = 0.01;
		}

		// --- Gyro integration
		double convert = gyro_scale * M_PI / 180.0;
		double wx = gx_deg * convert;
		double wy = gy_deg * convert;
		double wz = gz_deg * convert;

		quat_t dq = delta_quat_from_omega(wx, wy, wz, dt);
		orientation = quat_multiply(orientation, dq);
		orientation = quat_normalize(orientation);

		// --- Accel correction (acc in G's)
		double ax = ax_g, ay = ay_g, az = az_g;
		double acc_norm = std::sqrt(ax*ax + ay*ay + az*az);
		if (acc_norm > 1e-3) {
			// Normalize accel vector
			ax /= acc_norm; ay /= acc_norm; az /= acc_norm;

			// derive pitch & roll from accel (gravity vector)
			// pitch = atan2(-ax, sqrt(ay^2 + az^2))
			double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
			double roll  = std::atan2(ay, az);

			// keep yaw from current global quaternion
			auto e = quat_to_euler(orientation);
			double yaw = e[2];

			quat_t q_acc = euler_to_quat(roll, pitch, yaw);

			// slerp toward accel-derived quaternion by accel_gain
			orientation = quat_slerp(orientation, q_acc, accel_gain);
			orientation = quat_normalize(orientation);
		}

		// Build and publish IMU message
		sensor_msgs::Imu imu_msg;
		imu_msg.header.stamp = now;
		imu_msg.header.frame_id = "rp2040_imu_link";

		imu_msg.orientation.x = 0;
		imu_msg.orientation.y = 0;
		imu_msg.orientation.z = 0;
		imu_msg.orientation.w = 0;
		imu_msg.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

		imu_msg.angular_velocity.x = wx;
		imu_msg.angular_velocity.y = wy;
		imu_msg.angular_velocity.z = wz;
		imu_msg.angular_velocity_covariance = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};

		// linear acceleration: convert G -> m/s^2
		const double G = 9.80665;
		imu_msg.linear_acceleration.x = ax_g * G;
		imu_msg.linear_acceleration.y = ay_g * G;
		imu_msg.linear_acceleration.z = az_g * G;
		imu_msg.linear_acceleration_covariance = {0.04,0,0,0,0.04,0,0,0,0.04};

		imu_raw_pub.publish(imu_msg);

		fused_prescaler++;

		//publish every third message, 100hz raw, 33hz fused
		if(fused_prescaler % 4 == 3){
			fused_prescaler = 0;

			imu_msg.orientation.x = orientation[0];
			imu_msg.orientation.y = orientation[1];
			imu_msg.orientation.z = orientation[2];
			imu_msg.orientation.w = orientation[3];
			imu_msg.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
			imu_data_pub.publish(imu_msg);
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "rp2040_imu_node");
	ros::NodeHandle nh("~");

	try {
		RP2040IMUNode node(nh);
		node.spin();
	} catch (std::exception &e) {
		ROS_FATAL("Fatal exception in rp2040_imu_node: %s", e.what());
		return 1;
	}

	return 0;
}
