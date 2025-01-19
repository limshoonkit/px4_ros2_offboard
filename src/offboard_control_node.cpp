#include <px4_msgs/msg/px4io_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/parameter.hpp>

#include <bits/stdc++.h>
#include <chrono>
#include <string>

#include "flight_patterns.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace uosm
{
namespace px4
{

constexpr float PUBLISH_RATE(20.0);                 // pose publishing rate
constexpr float THRESHOLD_ORIGIN(0.1);              // threshold for origin

// Math utility
template <typename T>
inline static T calculate_vector_norm(const px4_msgs::msg::VehicleOdometry& odom)
{
	return static_cast<T>(std::sqrt(
		std::pow(odom.position[0], 2) +
		std::pow(odom.position[1], 2)));
}

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string px4_namespace) : Node("offboard_control_node")
	{
		// Params setup
		this->declare_parameter("flight_pattern", rclcpp::ParameterValue(0)); // Default to CIRCULAR
    	rclcpp::Parameter flight_pattern_ = this->get_parameter("flight_pattern");
		const auto flight_pattern_int_ = flight_pattern_.as_int();
		if (flight_pattern_int_ < 0 || flight_pattern_int_ > 4)
		{
			RCLCPP_ERROR(this->get_logger(), "Invalid flight pattern %ld!", flight_pattern_int_);
			return;
		}
	
		this->declare_parameter("max_iter", rclcpp::ParameterValue(2));
		this->declare_parameter("dt", rclcpp::ParameterValue(0.05f));
		this->declare_parameter("radius", rclcpp::ParameterValue(0.80f));
		this->declare_parameter("height", rclcpp::ParameterValue(1.00f));
		this->declare_parameter("speed", rclcpp::ParameterValue(0.30f));
		this->declare_parameter("min_speed", rclcpp::ParameterValue(0.05f));
		this->declare_parameter("offset_x", rclcpp::ParameterValue(0.00f));
		this->declare_parameter("offset_y", rclcpp::ParameterValue(0.00f));
		this->declare_parameter("offset_z", rclcpp::ParameterValue(0.00f));
		this->declare_parameter("frequency", rclcpp::ParameterValue(0.00f));
		this->declare_parameter("ngram_vertices", rclcpp::ParameterValue(7));
		this->declare_parameter("ngram_step", rclcpp::ParameterValue(2));

		flight_params_ = {
			this->get_parameter("dt").as_double(),
			this->get_parameter("radius").as_double(),
			this->get_parameter("height").as_double(),
			this->get_parameter("speed").as_double(),
			this->get_parameter("min_speed").as_double(),
			this->get_parameter("offset_x").as_double(),
			this->get_parameter("offset_y").as_double(),
			this->get_parameter("offset_z").as_double(),
			this->get_parameter("frequency").as_double(),
			static_cast<int>(this->get_parameter("ngram_vertices").as_int()),
			static_cast<int>(this->get_parameter("ngram_step").as_int()),
			static_cast<int>(this->get_parameter("max_iter").as_int())};

		if (std::__gcd(flight_params_.ngram_vertices, flight_params_.ngram_step) != 1)
		{
			RCLCPP_ERROR(this->get_logger(), "NGram_vertices and NGram_step must be co-prime!");
			return;
		}

		pattern_ = static_cast<flight_pattern::PatternFactory::PatternType>(flight_pattern_int_);
		RCLCPP_INFO(this->get_logger(), "Flight Pattern: %d, Flight Height: %.2f", pattern_, flight_params_.height);

		// Publishers & Subscribers setup
		rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
    	const auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", qos);
		vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command");

		px4io_status_sub_ = this->create_subscription<px4_msgs::msg::Px4ioStatus>(px4_namespace + "in/px4io_status", qos,
																				  [this](const px4_msgs::msg::Px4ioStatus::SharedPtr msg)
																				  {
																					  px4io_status_ = *msg;
																				  });

		vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(px4_namespace + "in/vehicle_status", qos,
																					  [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
																					  {
																						  vehicle_status_ = *msg;
																					  });

		vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(px4_namespace + "in/vehicle_odometry", qos,
																						  [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
																						  {
																						      vehicle_odometry_ = *msg;
																						  });

		traj.position = {static_cast<float>(flight_params_.offset_x), static_cast<float>(flight_params_.offset_y), static_cast<float>(flight_params_.height)};
		traj.yaw = 0.0f;

		is_init = true;
	}

	void arm();
	void disarm();
	void switch_to_auto_land_mode();
	void switch_to_offboard_mode();
	void publish_offboard_control_mode();

	void publish_trajectory_setpoint();
	void request_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	
	bool is_init = false;
	flight_pattern::PatternParameters flight_params_;
	flight_pattern::PatternFactory::PatternType pattern_;
	px4_msgs::msg::Px4ioStatus px4io_status_;
	px4_msgs::msg::VehicleStatus vehicle_status_;
	px4_msgs::msg::VehicleOdometry vehicle_odometry_;

	px4_msgs::msg::TrajectorySetpoint traj;

private:

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	rclcpp::Subscription<Px4ioStatus>::SharedPtr px4io_status_sub_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);

	uint8_t service_result_;
	bool service_done_;
};

/**
 *  Assume following Flight Modes are set https://docs.px4.io/main/en/flight_modes_mc/
 *  1: Manual
 *  2: Position
 *  3: Offboard
 *  4: Auto Land
 *  5: Altitude
 *  6: Stabilized
 */
/**
 * @brief Send a command to switch to offboard mode
 */
void OffboardControl::switch_to_auto_land_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to Auto Land mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4);
}

/**
 * @brief Send a command to switch to offboard mode
 */
void OffboardControl::switch_to_offboard_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	RCLCPP_INFO(this->get_logger(), "requesting disarm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

/**
 * @brief Publish the offboard control mode.
 *        Only position and attitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 */
void OffboardControl::publish_trajectory_setpoint()
{
	traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(traj);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                           std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void OffboardControl::response_callback(
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
{
	auto status = future.wait_for(1s);
	if (status == std::future_status::ready)
	{
		auto reply = future.get()->reply;
		service_result_ = reply.result;
		switch (service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "command reply unknown");
			break;
		}
		service_done_ = true;
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
	}
}

} // namespace uosm
} // namespace px4

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	int fcu_timeout_count = 0;
	bool isCompleted = false;
    bool isReady = false;
    int wait_period_sec = 2;

	auto node = std::make_shared<uosm::px4::OffboardControl>("/fmu/");
	if (node->is_init)
	{
		rclcpp::Rate rate(uosm::px4::PUBLISH_RATE);
		RCLCPP_INFO(node->get_logger(), "Waiting FCU connection");
		// Wait for FCU connection
		while (rclcpp::ok() && !node->px4io_status_.status_fmu_ok)
		{
			rclcpp::spin_some(node);
			rate.sleep();
			fcu_timeout_count++;
			if (fcu_timeout_count > 100)
			{
				RCLCPP_ERROR(node->get_logger(), "No FCU connection!");
				return 1;
			}
		}

		// Send a few setpoints before starting
		for (int i = 100; rclcpp::ok() && i > 0; --i)
		{
			node->publish_trajectory_setpoint();
			rclcpp::spin_some(node);
			rate.sleep();
		}
		auto pattern = uosm::px4::flight_pattern::PatternFactory::createPattern(node->pattern_, node->flight_params_);
		auto last_request = node->now();

		while (rclcpp::ok())
		{
			auto nav_state = node->vehicle_status_.nav_state;
			auto arming_state = node->vehicle_status_.arming_state;
			if (arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED && isCompleted)
			{
				RCLCPP_INFO(node->get_logger(), "Flight mission completed, Exiting!");
				rclcpp::shutdown();
				return 0;
			}
			node->publish_trajectory_setpoint();
			rclcpp::spin_some(node);
			rate.sleep();

			// Switch to offboard mode
			if (nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
				nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND &&
				(node->now() - last_request > rclcpp::Duration::from_seconds(wait_period_sec)))
			{
				node->switch_to_offboard_mode();
				last_request = node->now();
			}
			else
			{
				// Arm the vehicle
				if (arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
					(node->now() - last_request > rclcpp::Duration::from_seconds(wait_period_sec)) &&
					!isCompleted)
				{
					node->arm();
					last_request = node->now();
				}

				// Start the flight pattern
				if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
					arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
					!isCompleted)
				{
					if (!isReady)
					{
						pattern->hover(node->traj);
						last_request = node->now();
						isReady = true;
					}

					// HOVER for wait_period_sec, then run the flight pattern
					if (isReady && (node->now() - last_request) > rclcpp::Duration::from_seconds(wait_period_sec))
					{
						pattern->run(node->traj);
						RCLCPP_INFO(node->get_logger(), "Setting Trajectory (x = %.2f m, y = %.2f m, z = %.2f m, yaw = %.2f rad)", node->traj.position[0], node->traj.position[1], node->traj.position[2], node->traj.yaw);
						isCompleted = pattern->is_done();
					}
				}

				// Return to original position, then land
				if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
					arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
					isCompleted)
				{
					auto pos_z = node->vehicle_odometry_.position[2];
					auto norm_xy = uosm::px4::calculate_vector_norm<float>(node->vehicle_odometry_);
					if (pos_z >= uosm::px4::THRESHOLD_ORIGIN && norm_xy >= uosm::px4::THRESHOLD_ORIGIN)
					{
						pattern->hover(node->traj);
						RCLCPP_WARN(node->get_logger(), "Returning to hover at origin! x = %.2f m, y = %.2f m, z = %.2f m", node->vehicle_odometry_.position[0], node->vehicle_odometry_.position[1], pos_z);
					}

					if(nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
					{
						node->switch_to_auto_land_mode();
					}
				}
			}
		}
	}

	rclcpp::shutdown();
	return 0;
}