/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "cable_landing_controller_sw.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CableLandingController::CableLandingController(const std::string & node_name, 
						const std::string & node_namespace, const rclcpp::NodeOptions & options) : 
	Node(node_name, node_namespace, options),
	request_queue_(1), 
	request_reply_queue_(2),
	request_reply_poll_rate_(50ms),
	request_completion_poll_rate_(100ms) {

	this->declare_parameter<int>("controller_period_ms", 100);
	this->declare_parameter<float>("landed_altitude_threshold", 0.5);
	this->declare_parameter<float>("reached_position_euclidean_distance_threshold", 0.1);

	quat_t temp_q(1,0,0,0);
	vector_t temp_vec(0,0,0);

	odom_q_ = temp_q;
	odom_pos_ = temp_vec;
	odom_vel_ = temp_vec;
	odom_ang_vel_ = temp_vec;

	// Action stuff
	this->takeoff_server_ = rclcpp_action::create_server<Takeoff>(
		this,
		"takeoff",
		std::bind(&CableLandingController::handleGoalTakeoff, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&CableLandingController::handleCancelTakeoff, this, std::placeholders::_1),
		std::bind(&CableLandingController::handleAcceptedTakeoff, this, std::placeholders::_1)
	);

	//offboard_control_mode_publisher_ =
	//	this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
	trajectory_setpoint_pub_ =
		this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
	vehicle_command_pub_ =
		this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in", 10);

	control_state_pub_ = 
		this->create_publisher<iii_interfaces::msg::ControlState>("control_state", 10);

	// check nav_state if in offboard (14)
	// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
	vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/vehicle_status/out",
		10,
		[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
			arming_state_ = msg->arming_state;
			nav_state_ = msg->nav_state;
		}
	);

	// get common timestamp
	timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
		"/fmu/timesync/out",
		10,
		[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
			timestamp_.store(msg->timestamp);
		}
	);

	odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(  ////
		"/fmu/vehicle_odometry/out", 10,
		std::bind(&CableLandingController::odometryCallback, this, std::placeholders::_1));

	main_state_machine_timer_ = this->create_wall_timer(
		100ms, std::bind(&CableLandingController::stateMachineCallback, this));

}

CableLandingController::~CableLandingController() {

	RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control..");
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); 
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	
}

rclcpp_action::GoalResponse CableLandingController::handleGoalTakeoff(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const Takeoff::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received takeoff goal request with target altitude %d", goal->target_altitude);
	
	(void)uuid;

	float min_target_altitude;
	this->get_parameter("min_target_altitude", min_target_altitude);

	if (goal->target_altitude < min_target_altitude)
		return rclcpp_action::GoalResponse::REJECT;

	takeoff_request_params_t params;
	params->takeoff_altitude = goal->target_altitude;

	request_t request = {
		.action_id = (rclcpp_action::GoalUUID &)uuid,
		.request_type = takeoff_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false)) 
		return rclcpp_action::GoalResponse::REJECT;

	rclcpp_action::GoalUUID &action_id = (rclcpp_action::GoalUUID &)uuid;
 
	request_reply_t reply = {
		.action_id = action_id
	};
	request_reply_queue_.Peak(reply, true);

	while (reply.action_id != action_id) {

		request_reply_poll_rate_.sleep();

		request_reply_queue_.Peak(reply, true);

	}

	rclcpp_action::GoalResponse return_response;

	switch (reply.reply_type) {

	default:
	case reject:
	case fail:

		if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue

		return_response = rclcpp_action::GoalResponse::REJECT;

		break;
	
	case accept:

		if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue

	case success:

		return_response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		
		break;

	}

	return return_response;

}

rclcpp_action::CancelResponse CableLandingController::handleCancelTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	return rclcpp_action::CancelResponse::REJECT;

}

void CableLandingController::handleAcceptedTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	using namespace std::placeholders;

	std::thread{ std::bind(&CableLandingController::followTakeoffCompletion, this, _1), goal_handle}.detach();

}

void CableLandingController::followTakeoffCompletion(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	rclcpp_action::GoalUUID & action_id = (rclcpp_action::GoalUUID &)goal_handle->get_goal_id();

	auto feedback = std::make_shared<Takeoff::Feedback>();
	auto & altitude = feedback->altitude;

	auto result = std::make_shared<Takeoff::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			state4_t vechicle_state = loadVehicleState();
			altitude = vechicle_state(2);

			goal_handle->publish_feedback(feedback);

			request_completion_poll_rate_.sleep();

		}

		if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
			

		switch (reply.reply_type) {

		default:
		case reject:
		case fail:

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
		
		case accept:

			break;

		case success:

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

void CableLandingController::stateMachineCallback() {

	float landed_altitude_threshold;
	this->get_parameter("landed_altitude_threshold", landed_altitude_threshold);

	float reached_pos_euc_dist_thresh;
	this->get_parameter("reached_position_euclidean_distance_threshold", reached_pos_euc_dist_thresh);

	state4_t veh_state = loadVehicleState();

	state4_t set_point = veh_state;

	static state4_t fixed_reference = set_point;

	rclcpp_action::GoalUUID temp_uuid;
	
	static request_t request = {
		.action_id = (rclcpp_action::GoalUUID &)temp_uuid
	};

	bool offboard = isOffboard();
	bool armed = isArmed();

	auto notifyCurrentRequest = [&](request_reply_type_t reply_type) -> bool {

		request_reply_t reply = {
			.action_id = request.action_id,
			.reply_type = reply_type
		};

		request_reply_queue_.Push(reply, true);

	};

	auto rejectPendingRequest = [&]() -> bool {

		if (request_queue_.Pop(request, false)) {

			notifyCurrentRequest(reject);

			return true;

		} else {

			return false;

		}
	};

	auto tryPendingRequest = [&](request_type_t request_type, bool do_notify, bool do_pop) -> bool {

		bool result;

		if (do_pop)
			result = request_queue_.Pop(request, false);
		else
			result = request_queue_.Peak(request, false);

		if (result) {

			if (request.request_type == request_type) {

				if (do_notify) notifyCurrentRequest(accept);
				return true;

			} else {
				
				if (do_notify) notifyCurrentRequest(reject);
				return false;

			}

		} else return false;

	};

	auto setZeroVelocity = [](state4_t state) -> state4_t {

		for (int i = 4; i < 8; i++) state(i) = 0;

		return state;

	};

	auto appendZeroVelocity = [](pos4_t pos) -> state4_t {

		state4_t state(
			pos(0),
			pos(1),
			pos(2),
			pos(3),
			0,
			0,
			0,
			0
		);

		return state;

	};

	auto reachedPosition = [this, reached_pos_euc_dist_thresh](state4_t state, state4_t target) -> bool {

		return (state-target).norm() <= reached_pos_euc_dist_thresh;

	};

	switch (state_) {

	case init:
	default:

		if (!offboard && veh_state(2) < landed_altitude_threshold) {

			state_ = on_ground_non_offboard;

		} else if(!offboard && veh_state(2) >= landed_altitude_threshold) {

			state_ = in_flight_non_offboard;

		} else if (offboard && veh_state(2) < landed_altitude_threshold) {

			set_point = setZeroVelocity(set_point);

			state_ = on_ground;

		} else if(offboard && veh_state(3) >= landed_altitude_threshold) {

			fixed_reference = setZeroVelocity(set_point);

			set_point = fixed_reference;

			state_ = hovering;

		}

		break;

	case on_ground_non_offboard:
		
		if (!offboard and veh_state(2) >= landed_altitude_threshold) {

			state_ = in_flight_non_offboard;

		} else if(offboard && veh_state(2) < landed_altitude_threshold) {

			set_point = setZeroVelocity(set_point);

			state_ = on_ground;

		} else {

			rejectPendingRequest();

		} 

		break;

	case in_flight_non_offboard:
		
		if (!offboard && veh_state(2) < landed_altitude_threshold) {

			state_ = on_ground_non_offboard;

		} else if (offboard) {

			fixed_reference = setZeroVelocity(veh_state);

			set_point = fixed_reference;

			state_ = hovering;

		} else {

			rejectPendingRequest();

		}

		break;
	
	case on_ground:
		
		if (!offboard) {
			
			state_ = on_ground_non_offboard;

		} else if(tryPendingRequest(takeoff_request, true, true)) {

			arm();

			takeoff_request_params_t takeoff_params = (takeoff_request_params_t)request.request_params;
			float takeoff_altitude = takeoff_params->takeoff_altitude;

			fixed_reference = setZeroVelocity(veh_state);
			fixed_reference(2) = takeoff_altitude;

			resetPositionMPC();
			set_point = stepPositionMPC(veh_state, fixed_reference);

			state_ = taking_off;
			
		} else {

			set_point = setZeroVelocity(veh_state);

		}

		break;

	case taking_off:
		
		if (!offboard || !armed) {

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			state_ = init;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			notifyCurrentRequest(success);

			fixed_reference = fixed_reference; // For explicability

			set_point = fixed_reference;

			state_ = hovering;

		} else {

			rejectPendingRequest();

			set_point = stepPositionMPC(veh_state, fixed_reference);

		}

		break;

	case hovering:
		
		if (!offboard || !armed) {

			rejectPendingRequest();

			state_ = init;

		} else if (tryPendingRequest(fly_to_position_request, true, true)) {

			fly_to_position_request_params_t request_params = (fly_to_position_request_params_t)request.request_params;
			pos4_t target_position = request_params->target_position;

			fixed_reference = appendZeroVelocity(target_position);

			resetPositionMPC();
			set_point = stepPositionMPC(veh_state, fixed_reference);

			state_ = in_positional_flight;

		} else {

			set_point = fixed_reference;

		}

		break;

	case in_positional_flight:

		if (!offboard || !armed) {

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			state_ = init;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			notifyCurrentRequest(success);

			fixed_reference = fixed_reference; // For explicability

			set_point = fixed_reference;

			state_ = hovering;

		} else {

			rejectPendingRequest();

			set_point = stepPositionMPC(veh_state, fixed_reference);

		}

		break;
	
	}

	publishControlState();
	publishTrajectorySetpoint(set_point);

}

void CableLandingController::odometryCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

	quat_t q(
		msg->q[0],
		msg->q[1],
		msg->q[2],
		msg->q[3]
	);

	vector_t ang_vel(
		msg->rollspeed,
		msg->pitchspeed,
		msg->yawspeed
	);

	vector_t pos(
		msg->x,
		msg->y,
		msg->z
	);

	vector_t vel(
		msg->vx,
		msg->vy,
		msg->vz
	);

	odometry_mutex_.lock(); {

		odom_q_ = q;
		odom_ang_vel_ = ang_vel;
		odom_pos_ = pos;
		odom_vel_ = vel;

	} odometry_mutex_.unlock();

}

bool CableLandingController::isOffboard() {

	return nav_state_ == 14;

}

bool CableLandingController::isArmed() {

	return arming_state_ == 4;

}

/**
 * @brief Send a command to Arm the vehicle
 */
void CableLandingController::arm() {

	//armed = true;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");

}


/**
 * @brief Send a command to Disarm the vehicle
 */
void CableLandingController::disarm() {

	//armed = false;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void CableLandingController::publishVehicleCommand(uint16_t command, float param1,
					      float param2, float param3, float param4,
					      float param5, float param6,
					      float param7) const {

	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_pub_->publish(msg);

}

void CableLandingController::publishControlState() {

	iii_interfaces::msg::ControlState msg;

	switch (state_) {

	default:
	case init:
		
		msg.state = msg.CONTROL_STATE_INIT;
		break;

	case on_ground_non_offboard:

		msg.state = msg.CONTROL_STATE_ON_GROUND_NON_OFFBOARD;
		break;

	case in_flight_non_offboard:

		msg.state = msg.CONTROL_STATE_IN_FLIGHT_NON_OFFBOARD;
		break;

	case on_ground:

		msg.state = msg.CONTROL_STATE_ON_GROUND;
		break;

	case taking_off:

		msg.state = msg.CONTROL_STATE_TAKING_OFF;
		break;

	case hovering:

		msg.state = msg.CONTROL_STATE_HOVERING;
		break;

	case in_positional_flight:

		msg.state = msg.CONTROL_STATE_IN_POSITIONAL_FLIGHT;
		break;
	
	}

	control_state_pub_->publish(msg);

}

/**
 * @brief Publish a trajectory setpoint
 */
void CableLandingController::publishTrajectorySetpoint(state4_t set_point) const {

	static rotation_matrix_t R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));

	vector_t pos(
		set_point(0),
		set_point(1),
		set_point(2)
	);

	pos = R_NED_to_body_frame.transpose() * pos;

	vector_t vel(
		set_point(4),
		set_point(5),
		set_point(6)
	);

	vel = R_NED_to_body_frame.transpose() * vel;

	float yaw = -set_point(3);

	float yaw_rate = -set_point(7);

	px4_msgs::msg::TrajectorySetpoint msg{};

	msg.timestamp = timestamp_.load();
	msg.x = pos(0);
	msg.y = pos(1);
	msg.z = pos(2);
	msg.yaw = yaw;
	msg.vx = vel(0);
	msg.vy = vel(1);
	msg.vz = vel(2);
	msg.yawspeed = yaw_rate;

	RCLCPP_INFO(this->get_logger(),  "\n Publishing trajectory setpoint: \n x: %f, y: %f, z: %f, yaw: %f, vx: %f, vy: %f, vz: %f, yawspeed: %f",
		pos(0), pos(1), pos(2), yaw, vel(0), vel(1), vel(2), yaw_rate);
	//msg.acceleration	// in meters/sec^2
	//msg.jerk			// in meters/sec^3
	//msg.thrust		// normalized thrust vector in NED

	trajectory_setpoint_pub_->publish(msg);

}


state4_t CableLandingController::loadVehicleState() {

	static rotation_matrix_t R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));

	quat_t q;
	vector_t ang_vel;
	vector_t pos;
	vector_t vel;

	odometry_mutex_.lock(); {

		q = odom_q_;
		ang_vel = odom_ang_vel_;
		pos = odom_pos_;
		vel = odom_vel_;

	} odometry_mutex_.unlock();

	pos = R_NED_to_body_frame * pos;
	vel = R_NED_to_body_frame * vel;

	orientation_t eul = quatToEul(q);

	state4_t vehicle_state(
		pos(0),
		pos(1),
		pos(2),
		-eul(2),
		vel(0),
		vel(1),
		vel(2),
		-ang_vel(2)
	);

	return vehicle_state;

}

void CableLandingController::resetPositionMPC() {



}

state4_t CableLandingController::stepPositionMPC(state4_t vehicle_state, state4_t target) {

	return target;

}

///**
// * @brief Publish the offboard control mode.
// *        For this example, only position and altitude controls are active.
// */
//void CableLandingController::publish_offboard_control_mode() const {
//
//	OffboardControlMode msg{};
//	msg.timestamp = timestamp_.load();
//	msg.position = true;
//	msg.velocity = true;
//	msg.acceleration = false;
//	msg.attitude = false;
//	msg.body_rate = false;
//
//	offboard_control_mode_publisher_->publish(msg);
//
//}
//
//
//
//

int main(int argc, char* argv[]) {
	std::cout << "Starting cable landing controller node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CableLandingController>());

	rclcpp::shutdown();
	return 0;
}