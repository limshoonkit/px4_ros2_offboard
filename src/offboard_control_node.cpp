#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

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

constexpr float PUBLISH_RATE(20.0f);	   // publishing rate

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
		const auto qos_profile = rclcpp::QoS(10)
									 .reliability(rclcpp::ReliabilityPolicy::BestEffort)
									 .durability(rclcpp::DurabilityPolicy::TransientLocal)
									 .history(rclcpp::HistoryPolicy::KeepLast);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", qos_profile);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", qos_profile);
		vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command");

		vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(px4_namespace + "out/vehicle_status", qos_profile,
																					  [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
																					  {
																						  vehicle_status_ = *msg;
																					  });

		vehicle_gp_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(px4_namespace + "out/vehicle_global_position", qos_profile,
																						  [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
																						  {
																							  vehicle_gp_ = *msg;
																						  });

		traj.position = {static_cast<float>(flight_params_.offset_x), static_cast<float>(flight_params_.offset_y), static_cast<float>(flight_params_.height)};
		traj.yaw = 0.0f;

		is_init_ = true;
	}

	void arm();
	void disarm();

	void switch_to_offboard_mode();
	void publish_offboard_control_mode();

	void publish_trajectory_setpoint();
	void request_landing(float lat = 0.0f, float lon = 0.0f, float alt = 0.0f);
	void request_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f);

	bool is_init_ = false;
	flight_pattern::PatternParameters flight_params_;
	flight_pattern::PatternFactory::PatternType pattern_;

	px4_msgs::msg::VehicleStatus vehicle_status_;
	px4_msgs::msg::VehicleGlobalPosition vehicle_gp_;

	px4_msgs::msg::TrajectorySetpoint traj;

private:
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gp_sub_;

	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);

	uint8_t service_result_;
};

/**
 * @brief Send a command to switch to offboard mode
 * https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/Commander.cpp#L367
 * PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
 */
void OffboardControl::switch_to_offboard_mode()
{
	RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	
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
	msg.attitude = true;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 */
void OffboardControl::publish_trajectory_setpoint()
{
	traj.position[2] *= -1; // invert (NED)
	traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(traj);
	// RCLCPP_INFO(this->get_logger(), "Setting Trajectory (x = %.2f m, y = %.2f m, z = %.2f m, yaw = %.2f rad)", traj.position[0], traj.position[1], traj.position[2], traj.yaw);
}

void OffboardControl::request_landing(float lat, float lon, float alt)
{
	// https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
	msg.param1 = 0.0f; // Abort Alt
	msg.param2 = 0.0f; // Land Mode
	// https://docs.px4.io/main/en/advanced_config/parameter_reference.html#MPC_LAND_SPEED
	msg.param3 = NAN;	   // empty
	msg.param4 = traj.yaw; // Yaw (rad)
	msg.param5 = lat;	   // lat
	msg.param6 = lon;	   // lon
	msg.param7 = alt;	   // alt (m)
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
																				 std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Land command send");
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

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
	constexpr int wait_period_sec_ = 2;
	constexpr int hover_period_sec_ = 20;
	int preflight_check_timeout_count_ = 0;

	enum STATE
	{
		DISARMED = 0,
		OFFBOARD_ARMED,
		HOVERING,
		FLYING,
		LANDING
	} state_;

	state_ = STATE::DISARMED;
	bool is_done_ = false;

	auto node = std::make_shared<uosm::px4::OffboardControl>("/fmu/");
	if (node->is_init_)
	{
		rclcpp::Rate rate(uosm::px4::PUBLISH_RATE);
		RCLCPP_INFO(node->get_logger(), "Waiting Pre-flight Check");

		// Check vehicle pre-flight status
		while (rclcpp::ok() && !node->vehicle_status_.pre_flight_checks_pass)
		{
			rclcpp::spin_some(node);
			rate.sleep();
			preflight_check_timeout_count_++;
			if (preflight_check_timeout_count_ > 100)
			{
				RCLCPP_ERROR(node->get_logger(), "Pre-flight Check Failed!");
				return 1;
			}
		}

		const float home_lat = node->vehicle_gp_.lat;
		const float home_lon = node->vehicle_gp_.lon;
		const float home_alt = node->vehicle_gp_.alt;

		RCLCPP_ERROR(node->get_logger(), "Pre-flight Check OK, home position: (lat = %.6f, lon = %.6f, alt = %.2f m)", home_lat, home_lon, home_alt);
		auto pattern = uosm::px4::flight_pattern::PatternFactory::createPattern(node->pattern_, node->flight_params_);
		auto last_request = node->now();

		while (rclcpp::ok())
		{
			auto nav_state = node->vehicle_status_.nav_state;
			auto arming_state = node->vehicle_status_.arming_state;
			node->publish_offboard_control_mode();
			node->publish_trajectory_setpoint();
			rclcpp::spin_some(node);
			rate.sleep();

			switch (state_)
			{
			case STATE::DISARMED:
				// RCLCPP_INFO(node->get_logger(), "STATE::DISARMED");
				if (is_done_)
				{
					RCLCPP_INFO(node->get_logger(), "Flight mission completed, Exiting!");
					rclcpp::shutdown();
					return 0;
				}
				state_ = STATE::OFFBOARD_ARMED;
				break;
			case STATE::OFFBOARD_ARMED:
				// RCLCPP_INFO(node->get_logger(), "STATE::OFFBOARD_ARMED");
				if (is_done_)
				{
					state_ = STATE::DISARMED;
				}
				if ((node->now() - last_request).seconds() > wait_period_sec_)
				{
					node->switch_to_offboard_mode();
					node->arm();
					last_request = node->now();
				}

				if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
					arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
				{
					state_ = STATE::HOVERING;
				}
				break;
			case STATE::HOVERING:
				// RCLCPP_INFO(node->get_logger(), "STATE::HOVERING");
				node->traj.yaw = M_PI_2; // NED convertion, Adjust with π/2 radians, Assuming vehicle starting at 0 radians in ENU frame
				pattern->hover(node->traj);
				if ((node->now() - last_request).seconds() > hover_period_sec_)
				{
					if (is_done_)
					{
						state_ = STATE::LANDING;
					}
					else
					{
						state_ = STATE::FLYING;
					}
				}
				break;
			case STATE::FLYING:
				// RCLCPP_INFO(node->get_logger(), "STATE::FLYING");
				pattern->run(node->traj);
				if (pattern->is_done())
				{
					is_done_ = true;
					last_request = node->now();
					state_ = STATE::HOVERING;
				}
				break;
			case STATE::LANDING:
				// RCLCPP_INFO(node->get_logger(), "STATE::LANDING");
				if (nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
				{
					node->request_landing(home_lat, home_lon, home_alt);
					state_ = STATE::DISARMED;
				}
				break;
			default:
				RCLCPP_INFO(node->get_logger(), "STATE::UNKNOWN");
				break;
			}
		}
	}

	rclcpp::shutdown();
	return 0;
}