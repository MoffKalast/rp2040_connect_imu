// rp2040_imu_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>
#include <iostream>

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

class RP2040IMUNode : public rclcpp::Node {
public:
	RP2040IMUNode()
	: Node("rp2040_imu_node"),
	  io_(), serial_(io_), fused_prescaler(0)
	{
		// Declare parameters with defaults (fixed at startup)
		this->declare_parameter<std::string>("port", "/dev/ttyIMU");
		this->declare_parameter<int>("baud_rate", 115200);
		this->declare_parameter<double>("accel_correction_gain", 0.03);
		this->declare_parameter<double>("gyro_scale", 1.15);

		this->get_parameter("port", param_port);
		this->get_parameter("baud_rate", param_baud_rate);
		this->get_parameter("accel_correction_gain", accel_gain);
		this->get_parameter("gyro_scale", gyro_scale);

		// Publishers
		imu_raw_pub = this->create_publisher<sensor_msgs::msg::Imu>("/rp2040_imu/data_raw", rclcpp::QoS(20));
		imu_data_pub = this->create_publisher<sensor_msgs::msg::Imu>("/rp2040_imu/data", rclcpp::QoS(20));
		// emulate latched behavior with transient_local QoS
		auto temp_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
		temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>("/rp2040_imu/temperature", temp_qos);

		orientation = {0.0, 0.0, 0.0, 1.0};
		last_time = this->now();

		openSerial();

		RCLCPP_INFO(this->get_logger(), "rp2040_imu_node started. port: %s, baud: %d, accel_gain: %f",
		            param_port.c_str(), param_baud_rate, accel_gain);
	}

	~RP2040IMUNode() {
		try {
			if (serial_.is_open()) serial_.close();
		} catch(...) {}
	}

	// Blocking spin-like loop, keeps behavior close to original node
	void spin() {
		asio::streambuf buf;
		std::istream is(&buf);

		while (rclcpp::ok()) {
			try {
				// read until newline (blocks)
				std::size_t n = asio::read_until(serial_, buf, '\n');
				(void)n;
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
				RCLCPP_WARN(this->get_logger(), "IMU disconnected or read error (%s), trying to reconnect...", e.what());
				try {
					serial_.close();
				} catch(...) {}

				// sleep for 2 seconds before retry
				rclcpp::sleep_for(std::chrono::milliseconds(2000));
				openSerial();
			}

			// allow rclcpp to process callbacks if any (none by default here)
			rclcpp::spin_some(this->get_node_base_interface());
		}
	}

private:
	// node members
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub;

	std::string param_port;
	int param_baud_rate;
	double accel_gain;
	double gyro_scale;

	asio::io_context io_;
	asio::serial_port serial_;

	quat_t orientation;
	rclcpp::Time last_time;
	int fused_prescaler;

	void openSerial() {
		try {
			if (serial_.is_open())
				serial_.close();

			serial_.open(param_port);
			serial_.set_option(serial_port_base::baud_rate(param_baud_rate));
			serial_.set_option(serial_port_base::character_size(8));
			serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
			serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
			serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
			RCLCPP_INFO(this->get_logger(), "Opened serial port %s @ %d", param_port.c_str(), param_baud_rate);
		} catch (std::exception &e) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", param_port.c_str(), e.what());
		}
	}

	void publishTemperature(double value) {
		sensor_msgs::msg::Temperature msg;
		msg.header.stamp = this->now();
		msg.header.frame_id = "rp2040_imu_link";
		msg.temperature = value;
		msg.variance = 0.0;
		temp_pub->publish(msg);
	}

	void publishImu(double ax_g, double ay_g, double az_g, double gx_deg, double gy_deg, double gz_deg) {
		// Time delta
		rclcpp::Time now = this->now();
		double dt = (now - last_time).seconds();
		last_time = now;
		if (dt <= 0.0 || dt > 0.05) {
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
		sensor_msgs::msg::Imu imu_msg;
		imu_msg.header.stamp = now;
		imu_msg.header.frame_id = "rp2040_imu_link";

		// orientation initially zero for raw message
		imu_msg.orientation.x = 0.0;
		imu_msg.orientation.y = 0.0;
		imu_msg.orientation.z = 0.0;
		imu_msg.orientation.w = 0.0;
		imu_msg.orientation_covariance = std::array<double,9>{0, 0, 0, 0, 0, 0, 0, 0, 0};

		imu_msg.angular_velocity.x = wx;
		imu_msg.angular_velocity.y = wy;
		imu_msg.angular_velocity.z = wz;
		imu_msg.angular_velocity_covariance = std::array<double,9>{0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};

		// linear acceleration: convert G -> m/s^2
		const double G = 9.80665;
		imu_msg.linear_acceleration.x = ax_g * G;
		imu_msg.linear_acceleration.y = ay_g * G;
		imu_msg.linear_acceleration.z = az_g * G;
		imu_msg.linear_acceleration_covariance = std::array<double,9>{0.04,0,0,0,0.04,0,0,0,0.04};

		imu_raw_pub->publish(imu_msg);

		fused_prescaler++;

		// publish every third message (approx same behavior as original)
		if(fused_prescaler == 3){
			fused_prescaler = 0;

			imu_msg.orientation.x = orientation[0];
			imu_msg.orientation.y = orientation[1];
			imu_msg.orientation.z = orientation[2];
			imu_msg.orientation.w = orientation[3];
			imu_msg.orientation_covariance = std::array<double,9>{0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
			imu_data_pub->publish(imu_msg);
		}
	}
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	try {
		auto node = std::make_shared<RP2040IMUNode>();
		node->spin();
	} catch (std::exception &e) {
		RCLCPP_FATAL(rclcpp::get_logger("rp2040_imu_node"), "Fatal exception in rp2040_imu_node: %s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}