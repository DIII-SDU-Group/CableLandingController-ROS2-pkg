#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <stdint.h>
#include <functional>
#include <memory>
#include <thread>
#include <climits>
#include <math.h>
#include <chrono>
#include <iostream>
#include <queue>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "iii_interfaces/msg/control_state.hpp"
#include "iii_interfaces/action/takeoff.hpp"
#include "iii_interfaces/action/landing.hpp"
#include "iii_interfaces/action/fly_to_position.hpp"

#include "geometry.h"
#include "blocking_queue.h"

#include "PositionMPCStepFunction.h"
#include "rt_nonfinite.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define ROS_DEFAULT_API

using namespace std::chrono_literals;

#define LOG_INFO(str) RCLCPP_INFO(this->get_logger(),str)

typedef Eigen::Matrix<float, 3, 1> pos3_t;
typedef Eigen::Matrix<float, 4, 1> pos4_t;
typedef Eigen::Matrix<float, 6, 1> state3_t;
typedef Eigen::Matrix<float, 8, 1> state4_t;

enum state_t {
	init = 0,
	on_ground_non_offboard,
	in_flight_non_offboard,
	arming,
	setting_offboard,
	taking_off,
	hovering,
	landing,
	in_positional_flight
};

enum request_type_t {
	cancel_request,
	takeoff_request,
	landing_request,
	fly_to_position_request
};

struct takeoff_request_params_t {
	float takeoff_altitude;
};

struct fly_to_position_request_params_t {
	pos4_t target_position;
};

struct request_t {
	rclcpp_action::GoalUUID action_id;
	request_type_t request_type;
	void *request_params;


	bool operator==(const request_t & rhs) const {

		return action_id == rhs.action_id && request_type == rhs.request_type && 
			request_params == rhs.request_params;

	}

	void operator=(const request_t & rhs) {

		action_id = rhs.action_id;
		request_type = rhs.request_type;
		request_params = rhs.request_params;

	}
};

enum request_reply_type_t {
	accept,
	reject,
	success,
	fail,
	cancel
};

struct request_reply_t {
	rclcpp_action::GoalUUID action_id;
	request_reply_type_t reply_type;

	bool operator==(const request_reply_t & rhs) const {

		return action_id == rhs.action_id && reply_type == rhs.reply_type;

	}

	void operator=(const request_reply_t & rhs) {

		action_id = rhs.action_id;
		reply_type = rhs.reply_type;

	}
};

enum request_queue_action_t {
	yes,
	no,
	if_match
};

/*****************************************************************************/
// Class
/*****************************************************************************/

class CableLandingController : public rclcpp::Node {
public:
	using Takeoff = iii_interfaces::action::Takeoff;
	using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

	using Landing = iii_interfaces::action::Landing;
	using GoalHandleLanding = rclcpp_action::ServerGoalHandle<Landing>;

	using FlyToPosition = iii_interfaces::action::FlyToPosition;
	using GoalHandleFlyToPosition = rclcpp_action::ServerGoalHandle<FlyToPosition>;

	CableLandingController(const std::string & node_name="cable_landing_controller", 
			const std::string & node_namespace="/cable_landing_controller", 
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~CableLandingController();

private:
	// Takeoff action:
	rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;

	rclcpp_action::GoalResponse handleGoalTakeoff(
		const rclcpp_action::GoalUUID & uuid, 
		std::shared_ptr<const Takeoff::Goal> goal
	);
	rclcpp_action::CancelResponse handleCancelTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
	void handleAcceptedTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
	void followTakeoffCompletion(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

	// Landing action:
	rclcpp_action::Server<Landing>::SharedPtr landing_server_;

	rclcpp_action::GoalResponse handleGoalLanding(
		const rclcpp_action::GoalUUID & uuid, 
		std::shared_ptr<const Landing::Goal> goal
	);
	rclcpp_action::CancelResponse handleCancelLanding(const std::shared_ptr<GoalHandleLanding> goal_handle);
	void handleAcceptedLanding(const std::shared_ptr<GoalHandleLanding> goal_handle);
	void followLandingCompletion(const std::shared_ptr<GoalHandleLanding> goal_handle);

	// Fly to position action:
	rclcpp_action::Server<FlyToPosition>::SharedPtr fly_to_position_server_;

	rclcpp_action::GoalResponse handleGoalFlyToPosition(
		const rclcpp_action::GoalUUID & uuid, 
		std::shared_ptr<const FlyToPosition::Goal> goal
	);
	rclcpp_action::CancelResponse handleCancelFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);
	void handleAcceptedFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);
	void followFlyToPositionCompletion(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);

	// General member variables:
	state_t state_ = init;

	uint8_t arming_state_; // armed = 4
	uint8_t nav_state_; // offboard = 14
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	quat_t odom_q_;
	vector_t odom_ang_vel_;
	vector_t odom_pos_;
	vector_t odom_vel_;

	std::vector<state4_t> planned_trajectory_;
	state4_t trajectory_target_;

	std::mutex odometry_mutex_;
	std::mutex planned_trajectory_mutex_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	BlockingQueue<request_t> request_queue_;
	BlockingQueue<request_reply_t> request_reply_queue_;

	rclcpp::Rate request_reply_poll_rate_;
	rclcpp::Rate request_completion_poll_rate_;

	rclcpp::TimerBase::SharedPtr main_state_machine_timer_;

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

	rclcpp::Publisher<iii_interfaces::msg::ControlState>::SharedPtr control_state_pub_;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_traj_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planned_target_pub_;

	// General member methods:
	void stateMachineCallback();
	void odometryCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg);

	bool isOffboard();
	bool isArmed();

	void setModeOffboard();

	void land();

	void arm();
	void disarm();

	void publishVehicleCommand(uint16_t command, float param1 = 0.0,
					 float param2 = 0.0,
					 float param3 = 0.0,
					 float param4 = 0.0,
					 float param5 = 0.0,
					 float param6 = 0.0,
					 float param7 = 0.0) const;
	void publishOffboardControlMode() const;
	void publishControlState();
	void publishTrajectorySetpoint(state4_t set_point) const;

	void publishPlannedTrajectory();

	state4_t loadVehicleState();
	geometry_msgs::msg::PoseStamped loadVehiclePose();
	nav_msgs::msg::Path loadPlannedPath();
	geometry_msgs::msg::PoseStamped loadPlannedTarget();

	state4_t stepPositionMPC(state4_t vehicle_state, state4_t target, bool reset);

	void clearPlannedTrajectory();
	void setTrajectoryTarget(state4_t target);

};



/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);