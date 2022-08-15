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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "iii_interfaces/msg/control_state.hpp"
#include "iii_interfaces/action/takeoff.hpp"

#include "geometry.h"
#include "blocking_queue.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define ROS_DEFAULT_API

using namespace std::chrono_literals;

typedef Eigen::Matrix<float, 3, 1> pos3_t;
typedef Eigen::Matrix<float, 4, 1> pos4_t;
typedef Eigen::Matrix<float, 6, 1> state3_t;
typedef Eigen::Matrix<float, 8, 1> state4_t;

typedef enum {
	init = 0,
	on_ground_non_offboard,
	in_flight_non_offboard,
	on_ground,
	taking_off,
	hovering,
	in_positional_flight
} state_t;

typedef enum {
	cancel_request,
	takeoff_request,
	fly_to_position_request
} request_type_t;

typedef struct {
	float takeoff_altitude;
} *takeoff_request_params_t;

typedef struct {
	pos4_t target_position;
} *fly_to_position_request_params_t;

struct request_struct {
	rclcpp_action::GoalUUID & action_id;
	request_type_t request_type;
	void *request_params;


	bool operator==(const struct request_struct & rhs) const {

		return action_id == rhs.action_id && request_type == rhs.request_type && 
			request_params == rhs.request_params;

	}

	struct request_struct & operator=(const struct request_struct & rhs) const {

		struct request_struct ret_val = {
			.action_id = rhs.action_id,
			.request_type = rhs.request_type,
			.request_params = rhs.request_params
		};

		return ret_val;

	}
};

typedef struct request_struct request_t;

typedef enum {
	accept,
	reject,
	success,
	fail
} request_reply_type_t;

struct request_reply_struct {
	rclcpp_action::GoalUUID & action_id;
	request_reply_type_t reply_type;

	bool operator==(const struct request_reply_struct & rhs) const {

		return action_id == rhs.action_id && reply_type == rhs.reply_type;

	}

	struct request_reply_struct & operator=(const struct request_reply_struct & rhs) const {

		struct request_reply_struct ret_val = {
			.action_id = rhs.action_id,
			.reply_type = rhs.reply_type
		};

		return ret_val;

	}
};

typedef struct request_reply_struct request_reply_t;

/*****************************************************************************/
// Class
/*****************************************************************************/

class CableLandingController : public rclcpp::Node {
public:
	using Takeoff = iii_interfaces::action::Takeoff;
	using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

	CableLandingController(const std::string & node_name="cable_landing_controller", 
			const std::string & node_namespace="/cable_landing_controller", 
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~CableLandingController();

private:
	// Action stuff:
	rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;

	rclcpp_action::GoalResponse handleGoalTakeoff(
		const rclcpp_action::GoalUUID & uuid, 
		std::shared_ptr<const Takeoff::Goal> goal
	);
	rclcpp_action::CancelResponse handleCancelTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
	void handleAcceptedTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
	void followTakeoffCompletion(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

	// General member variables:
	state_t state_ = init;

	uint8_t arming_state_; // armed = 4
	uint8_t nav_state_; // offboard = 14
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	quat_t odom_q_;
	vector_t odom_ang_vel_;
	vector_t odom_pos_;
	vector_t odom_vel_;

	std::mutex odometry_mutex_;

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

	//rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

	// General member methods:
	void stateMachineCallback();
	void odometryCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg);

	bool isOffboard();
	bool isArmed();

	void arm();
	void disarm();

	void publishVehicleCommand(uint16_t command, float param1 = 0.0,
					 float param2 = 0.0,
					 float param3 = 0.0,
					 float param4 = 0.0,
					 float param5 = 0.0,
					 float param6 = 0.0,
					 float param7 = 0.0) const;
	void publishControlState();
	void publishTrajectorySetpoint(state4_t set_point) const;

	state4_t loadVehicleState();

	void resetPositionMPC();
	state4_t stepPositionMPC(state4_t vehicle_state, state4_t target);

	//void publish_offboard_control_mode() const;

};



/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);