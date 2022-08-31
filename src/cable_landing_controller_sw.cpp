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
	this->declare_parameter<float>("landed_altitude_threshold", 0.15);
	this->declare_parameter<float>("reached_position_euclidean_distance_threshold", 0.1);
	this->declare_parameter<float>("minimum_target_altitude", 1);

	this->declare_parameter<double>("position_MPC_dt", 0.1);

	this->declare_parameter<double>("position_MPC_vx_max", 10.);
	this->declare_parameter<double>("position_MPC_vy_max", 10.);
	this->declare_parameter<double>("position_MPC_vz_max", 10.);
	this->declare_parameter<double>("position_MPC_vpsi_max", M_PI_2);

	this->declare_parameter<double>("position_MPC_ax_max", 10.);
	this->declare_parameter<double>("position_MPC_ay_max", 10.);
	this->declare_parameter<double>("position_MPC_az_max", 10.);
	this->declare_parameter<double>("position_MPC_apsi_max", M_PI_4);

	this->declare_parameter<double>("position_MPC_wx", 0.);
	this->declare_parameter<double>("position_MPC_wy", 0.);
	this->declare_parameter<double>("position_MPC_wz", 0.);
	this->declare_parameter<double>("position_MPC_wpsi", 10.);

	this->declare_parameter<double>("position_MPC_wvx", 10.00);
	this->declare_parameter<double>("position_MPC_wvy", 10.00);
	this->declare_parameter<double>("position_MPC_wvz", 10.00);
	this->declare_parameter<double>("position_MPC_wvpsi", 1.);

	this->declare_parameter<double>("position_MPC_wax", 10.00);
	this->declare_parameter<double>("position_MPC_way", 10.00);
	this->declare_parameter<double>("position_MPC_waz", 10.00);
	this->declare_parameter<double>("position_MPC_wapsi", 1.);

	this->declare_parameter<double>("position_MPC_wx_last", 100.);
	this->declare_parameter<double>("position_MPC_wy_last", 100.);
	this->declare_parameter<double>("position_MPC_wz_last", 100.);
	this->declare_parameter<double>("position_MPC_wpsi_last", 10.);

	this->declare_parameter<double>("position_MPC_wvx_last", 10);
	this->declare_parameter<double>("position_MPC_wvy_last", 10);
	this->declare_parameter<double>("position_MPC_wvz_last", 10);
	this->declare_parameter<double>("position_MPC_wvpsi_last", 1.);

	this->declare_parameter<double>("position_MPC_wax_last", 10);
	this->declare_parameter<double>("position_MPC_way_last", 10);
	this->declare_parameter<double>("position_MPC_waz_last", 10);
	this->declare_parameter<double>("position_MPC_wapsi_last", 1.);

	this->declare_parameter<double>("cable_landing_MPC_dt", 0.1);

	this->declare_parameter<double>("cable_landing_MPC_vx_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_vy_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_vz_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_vpsi_max", M_PI_2);

	this->declare_parameter<double>("cable_landing_MPC_ax_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_ay_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_az_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_apsi_max", M_PI_4);

	this->declare_parameter<double>("cable_landing_MPC_wx", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wy", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wz", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wpsi", 0.);

	this->declare_parameter<double>("cable_landing_MPC_wvx", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wvy", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wvz", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wvpsi", 0.);

	this->declare_parameter<double>("cable_landing_MPC_wx_last", 10.);
	this->declare_parameter<double>("cable_landing_MPC_wy_last", 10.);
	this->declare_parameter<double>("cable_landing_MPC_wz_last", 1.);
	this->declare_parameter<double>("cable_landing_MPC_wpsi_last", 10.);

	this->declare_parameter<double>("cable_landing_MPC_wvx_last", 5.);
	this->declare_parameter<double>("cable_landing_MPC_wvy_last", 5.);
	this->declare_parameter<double>("cable_landing_MPC_wvz_last", 5.);
	this->declare_parameter<double>("cable_landing_MPC_wvpsi_last", 5.);

	quat_t temp_q(1,0,0,0);
	vector_t temp_vec(0,0,0);

	odom_q_ = temp_q;
	odom_pos_ = temp_vec;
	odom_vel_ = temp_vec;
	odom_ang_vel_ = temp_vec;

	// tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Takeoff action:
	this->takeoff_server_ = rclcpp_action::create_server<Takeoff>(
		this,
		"takeoff",
		std::bind(&CableLandingController::handleGoalTakeoff, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&CableLandingController::handleCancelTakeoff, this, std::placeholders::_1),
		std::bind(&CableLandingController::handleAcceptedTakeoff, this, std::placeholders::_1)
	);

	// Landing action:
	this->landing_server_ = rclcpp_action::create_server<Landing>(
		this,
		"landing",
		std::bind(&CableLandingController::handleGoalLanding, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&CableLandingController::handleCancelLanding, this, std::placeholders::_1),
		std::bind(&CableLandingController::handleAcceptedLanding, this, std::placeholders::_1)
	);

	// Fly to position action:
	this->fly_to_position_server_ = rclcpp_action::create_server<FlyToPosition>(
		this,
		"fly_to_position",
		std::bind(&CableLandingController::handleGoalFlyToPosition, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&CableLandingController::handleCancelFlyToPosition, this, std::placeholders::_1),
		std::bind(&CableLandingController::handleAcceptedFlyToPosition, this, std::placeholders::_1)
	);

	// Cable landing action:
	this->cable_landing_server_ = rclcpp_action::create_server<CableLanding>(
		this,
		"cable_landing",
		std::bind(&CableLandingController::handleGoalCableLanding, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&CableLandingController::handleCancelCableLanding, this, std::placeholders::_1),
		std::bind(&CableLandingController::handleAcceptedCableLanding, this, std::placeholders::_1)
	);

	// Publishers and subscriptions:
	planned_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_trajectory", 10);
	planned_macro_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_macro_trajectory", 10);
	planned_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("planned_target", 10);

	offboard_control_mode_pub_ =
		this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/offboard_control_mode/in", 10);
	trajectory_setpoint_pub_ =
		this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/trajectory_setpoint/in", 10);
	vehicle_command_pub_ =
		this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/vehicle_command/in", 10);

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

	powerline_sub_ = this->create_subscription<iii_interfaces::msg::Powerline>(
		"/pl_mapper/powerline", 10,
		std::bind(&CableLandingController::powerlineCallback, this, std::placeholders::_1));

	main_state_machine_timer_ = this->create_wall_timer(
		100ms, std::bind(&CableLandingController::stateMachineCallback, this));

}

CableLandingController::~CableLandingController() {

	RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control..");
	land();
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


	if (state_ != on_ground_non_offboard)
		return rclcpp_action::GoalResponse::REJECT;

	takeoff_request_params_t *params = new takeoff_request_params_t;
	params->takeoff_altitude = goal->target_altitude;

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	request_t request = {
		.action_id = action_id,
		.request_type = takeoff_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false)) 
		return rclcpp_action::GoalResponse::REJECT;

	rclcpp_action::GoalResponse return_response;

	return_response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

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

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<Takeoff::Feedback>();

	auto result = std::make_shared<Takeoff::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			state4_t vechicle_state = loadVehicleState();
			feedback->altitude = vechicle_state(2);

			goal_handle->publish_feedback(feedback);

			request_completion_poll_rate_.sleep();

		}

		if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
			

		switch (reply.reply_type) {

		default:
		case cancel:
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

rclcpp_action::GoalResponse CableLandingController::handleGoalLanding(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const Landing::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received landing goal request");
	
	(void)uuid;

	if (state_ != hovering)
		return rclcpp_action::GoalResponse::REJECT;

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	request_t request = {
		.action_id = action_id,
		.request_type = landing_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false)) 
		return rclcpp_action::GoalResponse::REJECT;

	rclcpp_action::GoalResponse return_response;

	return_response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

	return return_response;

}

rclcpp_action::CancelResponse CableLandingController::handleCancelLanding(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	return rclcpp_action::CancelResponse::REJECT;

}

void CableLandingController::handleAcceptedLanding(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	using namespace std::placeholders;

	std::thread{ std::bind(&CableLandingController::followLandingCompletion, this, _1), goal_handle}.detach();

}

void CableLandingController::followLandingCompletion(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<Landing::Feedback>();

	auto result = std::make_shared<Landing::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			state4_t vechicle_state = loadVehicleState();
			feedback->altitude = vechicle_state(2);

			goal_handle->publish_feedback(feedback);

			request_completion_poll_rate_.sleep();

		}

		if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
			

		switch (reply.reply_type) {

		default:
		case cancel:
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

rclcpp_action::GoalResponse CableLandingController::handleGoalFlyToPosition(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const FlyToPosition::Goal> goal
) {

	// //LOG_INFO("a");

	RCLCPP_DEBUG(this->get_logger(), "Received fly to position goal request");
	
	(void)uuid;

	if (state_ != hovering)
		return rclcpp_action::GoalResponse::REJECT;

	// //LOG_INFO("b");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	fly_to_position_request_params_t *params = new fly_to_position_request_params_t;

	geometry_msgs::msg::PoseStamped target_pose = tf_buffer_->transform(goal->target_pose, "world");

	// //LOG_INFO("c");

	quat_t quat(
		target_pose.pose.orientation.w,
		target_pose.pose.orientation.x,
		target_pose.pose.orientation.y,
		target_pose.pose.orientation.z
	);

	orientation_t eul = quatToEul(quat);

	pos4_t target_position;
	target_position(0) = target_pose.pose.position.x;
	target_position(1) = target_pose.pose.position.y;
	target_position(2) = target_pose.pose.position.z;
	target_position(3) = eul(2);

	params->target_position = target_position;

	// //LOG_INFO("d");

	request_t request = {
		.action_id = action_id,
		.request_type = fly_to_position_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false)) 
		return rclcpp_action::GoalResponse::REJECT;

	// //LOG_INFO("e");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse CableLandingController::handleCancelFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received fly to position cancel request");

	// //LOG_INFO("f");
	
	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	request_t request = {
		.action_id = action_id,
		.request_type = cancel_request,
		.request_params = NULL
	};

	// //LOG_INFO("g");

	if (!request_queue_.Push(request, false))
		return rclcpp_action::CancelResponse::REJECT;

	// //LOG_INFO("h");

	return rclcpp_action::CancelResponse::ACCEPT;

}

void CableLandingController::handleAcceptedFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

	// //LOG_INFO("i");

	using namespace std::placeholders;

	std::thread{ std::bind(&CableLandingController::followFlyToPositionCompletion, this, _1), goal_handle}.detach();

}

void CableLandingController::followFlyToPositionCompletion(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

	// //LOG_INFO("j");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<FlyToPosition::Feedback>();

	auto result = std::make_shared<FlyToPosition::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};

	// //LOG_INFO("k");
	
	while(true) {

		// //LOG_INFO("l");

		while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			// //LOG_INFO("m");

			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();
			nav_msgs::msg::Path planned_path = loadPlannedPath();

			feedback->vehicle_pose = vehicle_pose;
			feedback->planned_path = planned_path;

			goal_handle->publish_feedback(feedback);

			// //LOG_INFO("n");

			request_completion_poll_rate_.sleep();

		}

		// //LOG_INFO("o");

		if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue

		// //LOG_INFO("p");
			
		switch (reply.reply_type) {

		default:
		case cancel:

		// //LOG_INFO("q");

			if (goal_handle->is_canceling()) {

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
		// //LOG_INFO("r");
		case fail:

		// //LOG_INFO("s");
			result->success = false;
			goal_handle->abort(result);

			return;
			break;
		
		case accept:
		// //LOG_INFO("t");

			break;

		case success:

		// //LOG_INFO("u");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

rclcpp_action::GoalResponse CableLandingController::handleGoalCableLanding(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const CableLanding::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received cable landing goal request");
	
	(void)uuid;

	if (state_ != hovering)
		return rclcpp_action::GoalResponse::REJECT;

	int cable_id = goal->target_cable_id;

	if (!updateTargetCablePose(cable_id)) {

		clearTargetCable();

		return rclcpp_action::GoalResponse::REJECT;

	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	cable_landing_request_params_t *params = new cable_landing_request_params_t;

	params->cable_id = cable_id;

	request_t request = {
		.action_id = action_id,
		.request_type = cable_landing_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false))  {

		clearTargetCable();

		return rclcpp_action::GoalResponse::REJECT;

	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse CableLandingController::handleCancelCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received cable landing cancel request");
	
	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	request_t request = {
		.action_id = action_id,
		.request_type = cancel_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false))
		return rclcpp_action::CancelResponse::REJECT;

	return rclcpp_action::CancelResponse::ACCEPT;

}

void CableLandingController::handleAcceptedCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

	using namespace std::placeholders;

	std::thread{ std::bind(&CableLandingController::followCableLandingCompletion, this, _1), goal_handle}.detach();

}

void CableLandingController::followCableLandingCompletion(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<CableLanding::Feedback>();

	auto result = std::make_shared<CableLanding::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();
			nav_msgs::msg::Path planned_path = loadPlannedPath();
			nav_msgs::msg::Path planned_macro_path = loadPlannedMacroPath();
			geometry_msgs::msg::PoseStamped planned_target = loadPlannedTarget();

			float distance = 0;
			distance += pow(vehicle_pose.pose.position.x - planned_target.pose.position.x, 2);
			distance += pow(vehicle_pose.pose.position.y - planned_target.pose.position.y, 2);
			distance += pow(vehicle_pose.pose.position.z - planned_target.pose.position.z, 2);
			distance = sqrt(distance);


			feedback->vehicle_pose = vehicle_pose;
			feedback->planned_path = planned_path;
			feedback->planned_macro_path = planned_macro_path;
			feedback->distance_vehicle_to_cable = distance;

			goal_handle->publish_feedback(feedback);

			request_completion_poll_rate_.sleep();

		}

		if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
	
		switch (reply.reply_type) {

		default:
		case cancel:

			if (goal_handle->is_canceling()) {

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

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

	static state4_t prev_veh_state;
	static state4_t veh_state;
	prev_veh_state = veh_state;
	veh_state = loadVehicleState();

	static state4_t set_point;
	static state4_t prev_set_point;
	prev_set_point = set_point;
	set_point = veh_state;

	static state4_t fixed_reference = veh_state;

	rclcpp_action::GoalUUID temp_uuid;
	
	static request_t request = {
		.action_id = (rclcpp_action::GoalUUID &)temp_uuid
	};

	static int arm_cnt = 0;
	static int offboard_cnt = 0;
	static int land_cnt = 0;
	static int target_cable_cnt = 0;

	static int cable_id = -1;

	bool offboard = isOffboard();
	bool armed = isArmed();

	// if (offboard)
	// 	LOG_INFO("offboard");
	// else
	// 	LOG_INFO("not offboard");

	//if (armed)
	//	//LOG_INFO("armed");
	//else
	//	//LOG_INFO("not armed");

	auto notifyCurrentRequest = [&](request_reply_type_t reply_type) -> bool {

		request_reply_t reply = {
			.action_id = request.action_id,
			.reply_type = reply_type
		};

		request_reply_queue_.Push(reply, true);

		return true;

	};

	auto rejectPendingRequest = [&]() -> bool {

		if (request_queue_.Pop(request, false)) {

			notifyCurrentRequest(reject);

			return true;

		} else {

			return false;

		}
	};

	auto tryPendingRequest = [&](request_type_t request_type, 
			request_queue_action_t do_notify, request_queue_action_t do_pop) -> bool {

		bool result;

		if (do_pop == yes)
			result = request_queue_.Pop(request, false);
		else
			result = request_queue_.Peak(request, false);

		if (result) {

			if (request.request_type == request_type) {

				if (do_pop == if_match) {

					result = request_queue_.Pop(request, false);

					if (do_notify == yes || do_notify == if_match) notifyCurrentRequest(accept);

				}

				return true;

			} else {
				
				if (do_notify == yes && do_pop == yes) notifyCurrentRequest(reject);
				return false;

			}

		} else return false;

	};

	auto currentRequestIsCancelled = [&](request_queue_action_t do_notify, request_queue_action_t do_pop) -> bool {

		// //LOG_INFO("b1");

		request_t tmp_request;

		bool result;

		RCLCPP_DEBUG(this->get_logger(), "Dummy write");

		if (do_pop == yes)
			result = request_queue_.Pop(tmp_request, false);
		else
			result = request_queue_.Peak(tmp_request, false);

		// //LOG_INFO("b3");

		if (result && tmp_request.action_id == request.action_id) {

		// //LOG_INFO("b4");

			if (do_pop == if_match) {

		// //LOG_INFO("b5");

				result = request_queue_.Pop(request, false);

		// //LOG_INFO("b6");

				if (do_notify == yes || do_notify == if_match) notifyCurrentRequest(cancel);

		// //LOG_INFO("b7");

			}

		// //LOG_INFO("b8");

			return true;

		} else if (result) {

		// //LOG_INFO("b9");

			if (do_notify == yes && do_pop == yes) notifyCurrentRequest(reject);

		////LOG_INFO("b10");
			return false;

		}

	};

	auto setZeroVelocity = [](state4_t state) -> state4_t {

		for (int i = 4; i < 12; i++) state(i) = 0;

		return state;

	};

	auto setNanVelocity = [](state4_t state) -> state4_t {

		for (int i = 4; i < 12; i++) state(i) = NAN;

		return state;

	};

	auto appendZeroVelocity = [](pos4_t pos) -> state4_t {

		state4_t state;

		for (int i = 0; i < 4; i++) {

			state(i) = pos(i);
			state(i+4) = 0;
			state(i+8) = 0;

		}

		return state;

	};

	auto reachedPosition = [this, reached_pos_euc_dist_thresh](state4_t state, state4_t target) -> bool {

		for(int i=4;i<12;i++) target(i) = 0;
		for(int i=8;i<12;i++) state(i) = 0;

		float norm = (state-target).norm();

		//LOG_INFO(std::to_string(norm));

		return norm <= reached_pos_euc_dist_thresh;

	};

	switch (state_) {

	case init:
	default:

		if (!offboard && veh_state(2) < landed_altitude_threshold) {

			state_ = on_ground_non_offboard;

		} else if(!offboard && veh_state(2) >= landed_altitude_threshold) {

			state_ = in_flight_non_offboard;

		} else if (offboard && veh_state(2) < landed_altitude_threshold) {

			set_point = setNanVelocity(veh_state);

			land();

			state_ = init;

		} else if(offboard && veh_state(3) >= landed_altitude_threshold) {

			fixed_reference = setZeroVelocity(veh_state);

			set_point = fixed_reference;

			state_ = hovering;

		}

		break;

	case on_ground_non_offboard:

		//RCLCPP_INFO(this->get_logger(), "State: %d", state_);
		//RCLCPP_INFO(this->get_logger(), "z: %f", veh_state(2));
		
		if (!offboard && veh_state(2) >= landed_altitude_threshold && armed) {

			state_ = in_flight_non_offboard;

		} else if (offboard) {

			state_ = init;

		} else if(tryPendingRequest(takeoff_request, yes, yes)) {

			arm();

			takeoff_request_params_t *takeoff_params = (takeoff_request_params_t *)request.request_params;
			float takeoff_altitude = takeoff_params->takeoff_altitude;

			delete request.request_params;

			fixed_reference = setZeroVelocity(veh_state);
			fixed_reference(2) = takeoff_altitude;

			setTrajectoryTarget(fixed_reference);

			set_point = setNanVelocity(veh_state);

			arm_cnt = 10;

			state_ = arming;

		} else {

			set_point = setNanVelocity(veh_state);

			rejectPendingRequest();

		} 

		break;

	case in_flight_non_offboard:

		//RCLCPP_INFO(this->get_logger(), "State: %d", state_);
		//RCLCPP_INFO(this->get_logger(), "z: %f", veh_state(2));
		
		if (!offboard && veh_state(2) < landed_altitude_threshold || !armed) {

			state_ = on_ground_non_offboard;

		} else if (offboard) {

			fixed_reference = setZeroVelocity(veh_state);

			set_point = fixed_reference;

			state_ = hovering;

		} else {

			set_point = setNanVelocity(veh_state);

			rejectPendingRequest();

		}

		break;
	
	case arming:
		
		if (arm_cnt == 0 && !armed) {

			set_point = setNanVelocity(veh_state);

			clearPlannedTrajectory();

			disarm();

			notifyCurrentRequest(fail);
			
			state_ = on_ground_non_offboard;

		} else if(armed) {

			set_point = setNanVelocity(veh_state);
			
			setModeOffboard();

			offboard_cnt = 10;

			state_ = setting_offboard;

		} else {

			rejectPendingRequest();

			set_point = setNanVelocity(veh_state);

			arm_cnt--;

		}

		break;

	case setting_offboard:

		if (offboard_cnt == 0 && !offboard) {

			set_point = setNanVelocity(veh_state);

			clearPlannedTrajectory();

			disarm();

			notifyCurrentRequest(fail);
			
			state_ = on_ground_non_offboard;

		} else if (offboard) {

			fixed_reference = fixed_reference;

			set_point = fixed_reference;

			state_ = taking_off;

		} else {

			rejectPendingRequest();

			set_point = setNanVelocity(veh_state);

			offboard_cnt--;
			
		}

		break;

	case taking_off:
		
		if (!offboard || !armed) {

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			clearPlannedTrajectory();

			state_ = init;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			RCLCPP_INFO(this->get_logger(), "Takeoff successful");

			notifyCurrentRequest(success);

			fixed_reference = fixed_reference; // For explicability

			set_point = fixed_reference;

			clearPlannedTrajectory();

			state_ = hovering;

		} else {

			rejectPendingRequest();

			set_point = fixed_reference;

		}

		break;

	case hovering:

		if (!offboard || !armed) {

			rejectPendingRequest();

			state_ = init;

		} else if (tryPendingRequest(landing_request, if_match, if_match)) {

			land();

			set_point = setNanVelocity(veh_state);

			land_cnt = 100;

			state_ = landing;

		} else if (tryPendingRequest(fly_to_position_request, if_match, if_match)) {

			fly_to_position_request_params_t *request_params = (fly_to_position_request_params_t *)request.request_params;
			pos4_t target_position = request_params->target_position;

			delete request.request_params;

			fixed_reference = appendZeroVelocity(target_position);

			setTrajectoryTarget(fixed_reference);

			set_point = stepPositionMPC(prev_veh_state, fixed_reference, true);

			state_ = in_positional_flight;

		} else if (tryPendingRequest(cable_landing_request, if_match, if_match)) {

			cable_landing_request_params_t *request_params = (cable_landing_request_params_t *)request.request_params;
			cable_id = request_params->cable_id;
			
			delete request.request_params;

			target_cable_cnt = 10;

			target_cable_cnt = updateTargetCablePose(cable_id) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = loadTargetCableState();

			setTrajectoryTarget(fixed_reference);

			set_point = stepCableLandingMPC(veh_state, fixed_reference, true);

			state_ = during_cable_landing;


		} else {

			rejectPendingRequest();

			set_point = fixed_reference;

		}

		break;

	case landing:

		if (!offboard || !armed) {

			notifyCurrentRequest(success);

			set_point = setNanVelocity(veh_state);

			state_ = init;

		} else if (land_cnt == 0) {

			notifyCurrentRequest(fail);

			set_point = setNanVelocity(veh_state);

			state_ = init;

		} else {

			rejectPendingRequest();

			set_point = setNanVelocity(veh_state);

			land_cnt--;

		}

		break;

	case in_positional_flight:

		//LOG_INFO("1");

		if (!offboard || !armed) {

			//LOG_INFO("a1");

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			state_ = init;

		 } else if (currentRequestIsCancelled(if_match, if_match)) {

		 	//LOG_INFO("a2");

		 	fixed_reference = setZeroVelocity(veh_state); // For explicability

		 	clearPlannedTrajectory();

		 	set_point = fixed_reference;

		 	state_ = hovering;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			//LOG_INFO("a3");

			notifyCurrentRequest(success);

			clearPlannedTrajectory();

			fixed_reference = setZeroVelocity(fixed_reference); // For explicability

			set_point = fixed_reference;

			state_ = hovering;

		} else {

			//LOG_INFO("2");

			rejectPendingRequest();

			set_point = stepPositionMPC(veh_state, fixed_reference, false);
			// set_point = fixed_reference;

		}

		break;

	case during_cable_landing:

		if (!offboard || !armed) {

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			state_ = init;

		} else if (currentRequestIsCancelled(if_match, if_match)) {

			fixed_reference = setZeroVelocity(veh_state); // For explicability

			clearPlannedTrajectory(); // Also clear macro trajectory
			clearTargetCable();

			fixed_reference = setZeroVelocity(veh_state);

			set_point = fixed_reference;

			state_ = hovering;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			notifyCurrentRequest(success);

			clearPlannedTrajectory();
			clearTargetCable();

			fixed_reference = setZeroVelocity(fixed_reference); // For explicability

			set_point = fixed_reference;

			state_ = on_cable_armed;

		} else if (target_cable_cnt == 0) {

			notifyCurrentRequest(fail);

			fixed_reference = setZeroVelocity(veh_state); // For explicability

			clearPlannedTrajectory();
			clearTargetCable();

			fixed_reference = setZeroVelocity(veh_state);

			set_point = fixed_reference;

			state_ = hovering;

		} else {

			rejectPendingRequest();

			target_cable_cnt = updateTargetCablePose(cable_id) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = loadTargetCableState();

			setTrajectoryTarget(fixed_reference);

			set_point = stepCableLandingMPC(veh_state, fixed_reference, true);

		}

		break;

	case on_cable_armed:

		if (!offboard || !armed) {

			rejectPendingRequest();

			state_ = init;

		} else {

			rejectPendingRequest();

			set_point = fixed_reference;

		}

		break;
	
	}

	publishControlState();
	publishPlannedTrajectory();

	publishOffboardControlMode();
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
		msg->angular_velocity[0],
		msg->angular_velocity[1],
		msg->angular_velocity[2]
	);

	vector_t pos(
		msg->position[0],
		msg->position[1],
		msg->position[2]
	);

	vector_t vel(
		msg->velocity[0],
		msg->velocity[1],
		msg->velocity[2]
	);

	odometry_mutex_.lock(); {

		odom_q_ = q;
		odom_ang_vel_ = ang_vel;
		odom_pos_ = pos;
		odom_vel_ = vel;

	} odometry_mutex_.unlock();

}

void CableLandingController::powerlineCallback(iii_interfaces::msg::Powerline::SharedPtr msg) {

	powerline_mutex_.lock(); {

		powerline_ = *msg;

	} powerline_mutex_.unlock();

}

bool CableLandingController::isOffboard() {

	return nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

}

bool CableLandingController::isArmed() {

	return arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

}

void CableLandingController::setModeOffboard() {

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

}

void CableLandingController::land() {

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

}

/**
 * @brief Send a command to Arm the vehicle
 */
void CableLandingController::arm() {

	//armed = true;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_DEBUG(this->get_logger(), "Arm command send");

}


/**
 * @brief Send a command to Disarm the vehicle
 */
void CableLandingController::disarm() {

	//armed = false;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_DEBUG(this->get_logger(), "Disarm command send");

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

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void CableLandingController::publishOffboardControlMode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;	
	offboard_control_mode_pub_->publish(msg);
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

	case arming:

		msg.state = msg.CONTROL_STATE_ARMING;
		break;

	case setting_offboard:

		msg.state = msg.CONTROL_STATE_SETTING_OFFBOARD;
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

	vector_t acc(
		set_point(8),
		set_point(9),
		set_point(10)
	);

	acc = R_NED_to_body_frame.transpose() * acc;

	float yaw = -set_point(3);

	float yaw_rate = -set_point(7);

	//float yaw_acc = -set_point(11);

	px4_msgs::msg::TrajectorySetpoint msg{};

	msg.timestamp = timestamp_.load();
	for (int i = 0; i < 3; i++) {

		msg.position[i] = pos(i);
		msg.velocity[i] = vel(i);
		msg.acceleration[i] = acc(i);
		msg.jerk[i] = NAN; //jerk(i);

	}

	msg.yaw = yaw;
	msg.yawspeed = yaw_rate;

	RCLCPP_DEBUG(this->get_logger(),  "\n Publishing trajectory setpoint: \n x: %f, y: %f, z: %f, yaw: %f, vx: %f, vy: %f, vz: %f, yawspeed: %f",
		pos(0), pos(1), pos(2), yaw, vel(0), vel(1), vel(2), yaw_rate);
	//msg.acceleration	// in meters/sec^2
	//msg.jerk			// in meters/sec^3
	//msg.thrust		// normalized thrust vector in NED

	trajectory_setpoint_pub_->publish(msg);

}

void CableLandingController::publishPlannedTrajectory() {

	nav_msgs::msg::Path path = loadPlannedPath();
	nav_msgs::msg::Path macro_path = loadPlannedMacroPath();

	geometry_msgs::msg::PoseStamped target;

	if (path.poses.size() == 0) {

		target = loadVehiclePose();
		path.poses.push_back(target);

	} else {

		target = loadPlannedTarget();

	}

	macro_path.poses.push_back(target);

	planned_traj_pub_->publish(path);
	planned_macro_traj_pub_->publish(macro_path);
	planned_target_pub_->publish(target);

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

	state4_t vehicle_state;
	vehicle_state(0) = pos(0);
	vehicle_state(1) = pos(1);
	vehicle_state(2) = pos(2);
	vehicle_state(3) = -eul(2);
	vehicle_state(4) = vel(0);
	vehicle_state(5) = vel(1);
	vehicle_state(6) = vel(2);
	vehicle_state(7) = -ang_vel(2);

	return vehicle_state;

}

geometry_msgs::msg::PoseStamped CableLandingController::loadVehiclePose() {

	state4_t veh_state = loadVehicleState();

	geometry_msgs::msg::PoseStamped pose;

	pose.header.frame_id = "world";
	pose.header.stamp = this->get_clock()->now();

	pose.pose.position.x = veh_state(0);
	pose.pose.position.y = veh_state(1);
	pose.pose.position.z = veh_state(2);

	orientation_t eul(0,0,veh_state(3));
	quat_t quat = eulToQuat(eul);

	pose.pose.orientation.w = quat(0);
	pose.pose.orientation.x = quat(1);
	pose.pose.orientation.y = quat(2);
	pose.pose.orientation.z = quat(3);

	return pose;

}

nav_msgs::msg::Path CableLandingController::loadPlannedPath() {

	std::vector<state4_t> trajectory_vec;

	planned_trajectory_mutex_.lock(); {

		trajectory_vec = planned_trajectory_;

	} planned_trajectory_mutex_.unlock();

	nav_msgs::msg::Path path;

	auto stamp = this->get_clock()->now();

	path.header.frame_id = "world";
	path.header.stamp = stamp;

	std::vector<geometry_msgs::msg::PoseStamped> poses_vec((std::size_t)trajectory_vec.size());

	for (int i = 0; i < trajectory_vec.size(); i++) {

		geometry_msgs::msg::PoseStamped pose;
		pose.header.frame_id = "world";
		pose.header.stamp = stamp;

		state4_t state = trajectory_vec[i];

		pose.pose.position.x = state(0);
		pose.pose.position.y = state(1);
		pose.pose.position.z = state(2);

		orientation_t eul(0,0,state(3));
		quat_t quat = eulToQuat(eul);

		pose.pose.orientation.w = quat(0);
		pose.pose.orientation.x = quat(1);
		pose.pose.orientation.y = quat(2);
		pose.pose.orientation.z = quat(3);

		poses_vec[i] = pose;

	}

	path.poses = poses_vec;

	return path;

}

nav_msgs::msg::Path CableLandingController::loadPlannedMacroPath() {

	std::vector<state4_t> trajectory_vec;

	planned_trajectory_mutex_.lock(); {

		trajectory_vec = planned_macro_trajectory_;

	} planned_trajectory_mutex_.unlock();

	nav_msgs::msg::Path path;

	auto stamp = this->get_clock()->now();

	path.header.frame_id = "world";
	path.header.stamp = stamp;

	std::vector<geometry_msgs::msg::PoseStamped> poses_vec((std::size_t)trajectory_vec.size());

	for (int i = 0; i < trajectory_vec.size(); i++) {

		geometry_msgs::msg::PoseStamped pose;
		pose.header.frame_id = "world";
		pose.header.stamp = stamp;

		state4_t state = trajectory_vec[i];

		pose.pose.position.x = state(0);
		pose.pose.position.y = state(1);
		pose.pose.position.z = state(2);

		orientation_t eul(0,0,state(3));
		quat_t quat = eulToQuat(eul);

		pose.pose.orientation.w = quat(0);
		pose.pose.orientation.x = quat(1);
		pose.pose.orientation.y = quat(2);
		pose.pose.orientation.z = quat(3);

		poses_vec[i] = pose;

	}

	path.poses = poses_vec;

	return path;

}

geometry_msgs::msg::PoseStamped CableLandingController::loadPlannedTarget() {

	state4_t target_cp;

	planned_trajectory_mutex_.lock(); {

		target_cp = trajectory_target_;

	} planned_trajectory_mutex_.unlock();

	geometry_msgs::msg::PoseStamped target_pose;

	target_pose.header.stamp = this->get_clock()->now();
	target_pose.header.frame_id = "world";

	target_pose.pose.position.x = target_cp(0);
	target_pose.pose.position.y = target_cp(1);
	target_pose.pose.position.z = target_cp(2);

	orientation_t eul(0,0,target_cp(3));
	quat_t quat = eulToQuat(eul);

	target_pose.pose.orientation.w = quat(0);
	target_pose.pose.orientation.x = quat(1);
	target_pose.pose.orientation.y = quat(2);
	target_pose.pose.orientation.z = quat(3);

	return target_pose;

}

state4_t CableLandingController::loadTargetCableState() {

	state4_t cable_state;

	geometry_msgs::msg::Pose cable_pose;

	powerline_mutex_.lock(); {

		cable_pose = target_cable_pose_.pose;

	} powerline_mutex_.unlock();

	quat_t quat(
		cable_pose.orientation.w,
		cable_pose.orientation.x,
		cable_pose.orientation.y,
		cable_pose.orientation.z
	);

	orientation_t eul = quatToEul(quat);

	cable_state(0) = cable_pose.position.x;
	cable_state(1) = cable_pose.position.x;
	cable_state(2) = cable_pose.position.x;
	cable_state(3) = eul(2);
	cable_state(4) = 0;
	cable_state(5) = 0;
	cable_state(6) = 0;
	cable_state(7) = 0;

	return cable_state;

}

state4_t CableLandingController::stepPositionMPC(state4_t vehicle_state, state4_t target_state, bool reset) {

		//LOG_INFO("3");

	// Variables:

	static bool first = true;

	static double dt;

	static double planned_traj[120];
	static double x[6];
	static double u[3];

	// static double prev_x[6];

	static double target[3];

	static int reset_target;
	static int reset_trajectory;
	static int reset_bounds;
	static int reset_weights;

	static state4_t prev_vehicle_state = vehicle_state;

	if (first || reset) {

		prev_vehicle_state = vehicle_state;

	}

	// Initialization:

	if (reset) {

		for (int i = 0; i < 3; i++) target[i] = target_state(i);

		reset_target = 1;
		reset_trajectory = 1;
		reset_bounds = 1;
		reset_weights = 1;

	} else {

		reset_target = 0;
		reset_trajectory = 0;
		reset_bounds = 0;
		reset_weights = 0;

	}

	// Step:

	auto resetTraj = [&]() -> void {

		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < 3; j++) {

				planned_traj[i*6+j] = vehicle_state(j);

			}

			for (int j = 3; j < 6; j++) {

				planned_traj[i*6+j] = 0;

			}
		}

		for (int i = 0; i < 6; i++) x[i] = planned_traj[i];
		// for (int i = 0; i < 6; i++) prev_x[i] = planned_traj[i];

	};


	if (first) {

		resetTraj();

	} else {

		MPC_thread_.join();

		if (reset) {

			resetTraj();

		} else {

			// for (int i = 0; i < 6; i++) prev_x[i] = x[i];

			for (int i = 0; i < 120; i++) planned_traj[i] = MPC_planned_traj_[i];
			for (int i = 0; i < 6; i++) x[i] = MPC_x_[i];
			for (int i = 0; i < 3; i++) u[i] = MPC_u_[i];

		}
	}

	for (int i = 0; i < 3; i++) MPC_x_[i] = vehicle_state(i);
	for (int i = 3; i < 6; i++) MPC_x_[i] = vehicle_state(i+1);

	prev_vehicle_state = vehicle_state;

	MPC_thread_ = std::thread( &CableLandingController::threadFunctionPositionMPC, this,
					MPC_x_, MPC_u_, MPC_planned_traj_, target, reset_target, reset_trajectory, reset_bounds, reset_weights);


	if (reset_trajectory) {

		this->get_parameter("position_MPC_dt", dt);

	}

	// Output:

	state4_t set_point;

	planned_trajectory_mutex_.lock(); {

		//LOG_INFO("5");

		if (first || reset) {

		//LOG_INFO("6");

			planned_trajectory_.clear();
			planned_trajectory_.resize(20);

			first = false;

		//LOG_INFO("7");

		}

		planned_trajectory_.resize(0);

		for (int i = 0; i < 20; i++) {

			state4_t tmp_state;
			for (int j = 0; j < 3; j++) tmp_state(j) = planned_traj[i*6+j];

		// std::cout << " \ttraj = [";
		// for (int j = 0; j < 3; j++) std::cout << " \t" << std::to_string(MPC_planned_traj_[i*6+j]);
		// std::cout << "\t]\n";

			for (int j = 3; j < 6; j++) tmp_state(j+1) = planned_traj[i*6+j];
			//for (int j = 6; j < 9; j++) tmp_state(j+2) = planned_traj[i*9+j];

			tmp_state(3) = target_state(3);
			tmp_state(7) = 0;
			tmp_state(11) = 0;


			//planned_trajectory_[i] = tmp_state;

			planned_trajectory_.push_back(tmp_state);

		}

    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";


		//set_point = planned_trajectory_[0];

		for (int i = 0; i < 3; i++) set_point(i) = x[i];
		for (int i = 0; i < 3; i++) set_point(i+4) = x[i+3];
		for (int i = 0; i < 3; i++) set_point(i+8) = u[i];

	} planned_trajectory_mutex_.unlock();

	set_point(7) = NAN;
	set_point(11) = NAN;

	return set_point;

}

void CableLandingController::threadFunctionPositionMPC(double *x, double *u, double *planned_traj, double *target, 
		int reset_target, int reset_trajectory, int reset_bounds, int reset_weights) {

	static double dt;

	static double vx_max;
	static double vy_max;
	static double vz_max;

	static double ax_max;
	static double ay_max;
	static double az_max;

	static double wx;
	static double wy;
	static double wz;

	static double wvx;
	static double wvy;
	static double wvz;

	static double wax;
	static double way;
	static double waz;

	static double wjx;
	static double wjy;
	static double wjz;

	const int N = 20;

	static pos_MPC::struct10_T Info;
	static pos_MPC::struct4_T mpcmovestate;
	static pos_MPC::struct5_T mpconlinedata;

	if (reset_trajectory) {

		this->get_parameter("position_MPC_dt", dt);

	}

	if (reset_bounds) {

		this->get_parameter("position_MPC_vx_max", vx_max);
		this->get_parameter("position_MPC_vy_max", vy_max);
		this->get_parameter("position_MPC_vz_max", vz_max);

		this->get_parameter("position_MPC_ax_max", ax_max);
		this->get_parameter("position_MPC_ay_max", ay_max);
		this->get_parameter("position_MPC_az_max", az_max);

	}

	if (reset_weights) {

		this->get_parameter("position_MPC_wx", wx);
		this->get_parameter("position_MPC_wy", wy);
		this->get_parameter("position_MPC_wz", wz);

		this->get_parameter("position_MPC_wvx", wvx);
		this->get_parameter("position_MPC_wvy", wvy);
		this->get_parameter("position_MPC_wvz", wvz);

		this->get_parameter("position_MPC_wax", wax);
		this->get_parameter("position_MPC_way", way);
		this->get_parameter("position_MPC_waz", waz);

		this->get_parameter("position_MPC_wjx", wjx);
		this->get_parameter("position_MPC_wjy", wjy);
		this->get_parameter("position_MPC_wjz", wjz);

	}

	if (reset_trajectory) {

		for(int i = 0; i < 81; i++) mpcmovestate.Covariance[i] = 0;
		for(int i=0; i < 3; i++) mpcmovestate.Disturbance[i] = 0;
		for(int i = 0; i < 228; i++) mpcmovestate.iA[i] = 0;
		for(int i=0; i < 3; i++) mpcmovestate.LastMove[i] = 0;
		for(int i=0; i < 6; i++) mpcmovestate.Plant[i] = x[i];

	}

	if (reset_bounds) {

		mpconlinedata.limits.MVMax[0] = ax_max;
		mpconlinedata.limits.MVMax[1] = ay_max;
		mpconlinedata.limits.MVMax[2] = az_max;

		mpconlinedata.limits.MVMin[0] = -ax_max;
		mpconlinedata.limits.MVMin[1] = -ay_max;
		mpconlinedata.limits.MVMin[2] = -az_max;

		mpconlinedata.limits.OutputMax[0] = 100000;
		mpconlinedata.limits.OutputMax[1] = 100000;
		mpconlinedata.limits.OutputMax[2] = 100000;
		mpconlinedata.limits.OutputMax[3] = vx_max;
		mpconlinedata.limits.OutputMax[4] = vy_max;
		mpconlinedata.limits.OutputMax[5] = vz_max;

		mpconlinedata.limits.OutputMin[0] = -100000;
		mpconlinedata.limits.OutputMin[1] = -100000;
		mpconlinedata.limits.OutputMin[2] = -100000;
		mpconlinedata.limits.OutputMin[3] = -vx_max;
		mpconlinedata.limits.OutputMin[4] = -vy_max;
		mpconlinedata.limits.OutputMin[5] = -vz_max;

	}

	if (reset_target) {

		for (int i = 0; i < 3; i++) {

			mpconlinedata.signals.ref[i] = target[i];
			mpconlinedata.signals.ref[i+3] = 0;

		}
	}

	if (reset_weights) {

		mpconlinedata.weights.du[0] = wjx;
		mpconlinedata.weights.du[1] = wjy;
		mpconlinedata.weights.du[2] = wjz;

		mpconlinedata.weights.u[0] = wax;
		mpconlinedata.weights.u[1] = way;
		mpconlinedata.weights.u[2] = waz;

		mpconlinedata.weights.y[0] = wx;
		mpconlinedata.weights.y[1] = wy;
		mpconlinedata.weights.y[2] = wz;
		mpconlinedata.weights.y[3] = wvx;
		mpconlinedata.weights.y[4] = wvy;
		mpconlinedata.weights.y[5] = wvz;

	}

	for(int i = 0; i < 6; i++) mpconlinedata.signals.ym[i] = x[i];

    //std::cout << " \tx = [";
    //for (int i = 0; i < 6; i++) std::cout << " \t" << std::to_string(x[i]);
    //std::cout << "\t]\n";

	pos_MPC::coder::mpcmoveCodeGeneration(&mpcmovestate, &mpconlinedata, u, &Info);

    //std::cout << " \tu = [";
    //for (int i = 0; i < 3; i++) std::cout << " \t" << std::to_string(u[i]);
    //std::cout << "\t]\n";

    //std::cout << " \ttraj = [";
    //for (int i = 0; i < 6; i++) std::cout << " \t" << std::to_string(Info.Yopt[i]);
    //std::cout << "\t]\n";
    //std::cout << "\n";
    //std::cout << "\n";

	for (int j = 0; j < 6; j++) x[j] = Info.Yopt[j*(N+1)+1];
	for (int i = 0; i < N; i++) {

		for (int j = 0; j < 6; j++) planned_traj[i*6+j] = Info.Yopt[j*(N+1)+(i+1)];

	} 

    for (int i = 0; i < N; i++) {
      std::cout << "[";
      for (int j = 0; j < 6; j++) std::cout << " \t" << std::to_string(Info.Yopt[j*(N+1)+(i+1)]);
      std::cout << "\t]\n";
    }

    std::cout << "\n\n\n";

    // for (int i = 0; i < 20; i++) {

	// 	std::cout << " \ttrajaaaa = [";
	// 	for (int j = 0; j < 6; j++) std::cout << " \t" << std::to_string(Info.Yopt[i*6+j]);
	// 	std::cout << "\t]\n";

	// }
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";

    // for (int i = 0; i < 20; i++) {

	// 	std::cout << " \ttrajhhh = [";
	// 	for (int j = 0; j < 6; j++) std::cout << " \t" << std::to_string(planned_traj[i*6+j]);
	// 	std::cout << "\t]\n";

	// }
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "\n";

}

// void CableLandingController::threadFunctionPositionMPC(double *x, double *planned_traj, double *target, 
// 		int reset_target, int reset_trajectory, int reset_bounds, int reset_weights) {

// 	static double dt;

// 	static double vx_max;
// 	static double vy_max;
// 	static double vz_max;

// 	static double ax_max;
// 	static double ay_max;
// 	static double az_max;

// 	static double wx;
// 	static double wy;
// 	static double wz;

// 	static double wvx;
// 	static double wvy;
// 	static double wvz;

// 	static double wax;
// 	static double way;
// 	static double waz;

// 	static double wx_last;
// 	static double wy_last;
// 	static double wz_last;

// 	static double wvx_last;
// 	static double wvy_last;
// 	static double wvz_last;

// 	static double wax_last;
// 	static double way_last;
// 	static double waz_last;

// 	if (reset_trajectory) {

// 		this->get_parameter("position_MPC_dt", dt);

// 	}

// 	if (reset_bounds) {

// 		this->get_parameter("position_MPC_vx_max", vx_max);
// 		this->get_parameter("position_MPC_vy_max", vy_max);
// 		this->get_parameter("position_MPC_vz_max", vz_max);

// 		this->get_parameter("position_MPC_ax_max", ax_max);
// 		this->get_parameter("position_MPC_ay_max", ay_max);
// 		this->get_parameter("position_MPC_az_max", az_max);

// 	}

// 	if (reset_weights) {

// 		this->get_parameter("position_MPC_wx", wx);
// 		this->get_parameter("position_MPC_wy", wy);
// 		this->get_parameter("position_MPC_wz", wz);

// 		this->get_parameter("position_MPC_wvx", wvx);
// 		this->get_parameter("position_MPC_wvy", wvy);
// 		this->get_parameter("position_MPC_wvz", wvz);

// 		this->get_parameter("position_MPC_wax", wax);
// 		this->get_parameter("position_MPC_way", way);
// 		this->get_parameter("position_MPC_waz", waz);

// 		this->get_parameter("position_MPC_wx_last", wx_last);
// 		this->get_parameter("position_MPC_wy_last", wy_last);
// 		this->get_parameter("position_MPC_wz_last", wz_last);

// 		this->get_parameter("position_MPC_wvx_last", wvx_last);
// 		this->get_parameter("position_MPC_wvy_last", wvy_last);
// 		this->get_parameter("position_MPC_wvz_last", wvz_last);

// 		this->get_parameter("position_MPC_wax_last", wax_last);
// 		this->get_parameter("position_MPC_way_last", way_last);
// 		this->get_parameter("position_MPC_waz_last", waz_last);

// 	}

// 	pos_MPC::MPCStepFunction(
// 		x, target, dt, 
// 		vx_max, vy_max, vz_max, ax_max, ay_max, az_max, 
// 		wx, wy, wz, wvx, wvy, wvz, wax,way,waz,
// 		wx_last, wy_last, wz_last,  wvx_last, wvy_last, wvz_last, wax_last,way_last,waz_last, 
// 		reset_target, reset_trajectory, reset_bounds, reset_weights, 
// 		planned_traj);

// }

state4_t CableLandingController::stepCableLandingMPC(state4_t vehicle_state, state4_t target_state, bool reset) {

	// Static variables:

	static bool first = true;

	static double planned_traj[160];

	static double x[8];
	static double target[4];

	static double dt;

	static double vx_max;
	static double vy_max;
	static double vz_max;
	static double vpsi_max;

	static double ax_max;
	static double ay_max;
	static double az_max;
	static double apsi_max;

	static double wx;
	static double wy;
	static double wz;
	static double wpsi;

	static double wvx;
	static double wvy;
	static double wvz;
	static double wvpsi;

	static double wx_last;
	static double wy_last;
	static double wz_last;
	static double wpsi_last;

	static double wvx_last;
	static double wvy_last;
	static double wvz_last;
	static double wvpsi_last;

	static int reset_target;
	static int reset_trajectory;
	static int reset_bounds;
	static int reset_weights;

	// Parameters:

	if (reset) {

		this->get_parameter("cable_landing_MPC_dt", dt);

		this->get_parameter("cable_landing_MPC_vx_max", vx_max);
		this->get_parameter("cable_landing_MPC_vy_max", vy_max);
		this->get_parameter("cable_landing_MPC_vz_max", vz_max);
		this->get_parameter("cable_landing_MPC_vpsi_max", vpsi_max);

		this->get_parameter("cable_landing_MPC_ax_max", ax_max);
		this->get_parameter("cable_landing_MPC_ay_max", ay_max);
		this->get_parameter("cable_landing_MPC_az_max", az_max);
		this->get_parameter("cable_landing_MPC_apsi_max", apsi_max);

		this->get_parameter("cable_landing_MPC_wx", wx);
		this->get_parameter("cable_landing_MPC_wy", wy);
		this->get_parameter("cable_landing_MPC_wz", wz);
		this->get_parameter("cable_landing_MPC_wpsi", wpsi);

		this->get_parameter("cable_landing_MPC_wvx", wvx);
		this->get_parameter("cable_landing_MPC_wvy", wvy);
		this->get_parameter("cable_landing_MPC_wvz", wvz);
		this->get_parameter("cable_landing_MPC_wvpsi", wvpsi);

		this->get_parameter("cable_landing_MPC_wx_last", wx_last);
		this->get_parameter("cable_landing_MPC_wy_last", wy_last);
		this->get_parameter("cable_landing_MPC_wz_last", wz_last);
		this->get_parameter("cable_landing_MPC_wpsi_last", wpsi_last);

		this->get_parameter("cable_landing_MPC_wvx_last", wvx_last);
		this->get_parameter("cable_landing_MPC_wvy_last", wvy_last);
		this->get_parameter("cable_landing_MPC_wvz_last", wvz_last);
		this->get_parameter("cable_landing_MPC_wvpsi_last", wvpsi_last);

	}

	// Initialization:

	for (int i = 0; i < 8; i++) x[i] = vehicle_state(i);

	if (reset) {

		for (int i = 0; i < 4; i++) target[i] = target_state(i);

	}

	if (reset) {

		reset_target = 1;
		reset_trajectory = 1;
		reset_bounds = 1;
		reset_weights = 1;

	} else {

		reset_target = 1;
		reset_trajectory = 0;
		reset_bounds = 0;
		reset_weights = 0;

	}

	// Step:

	//pos_MPC::PositionMPCStepFunction(
	//	x, target, dt, 
	//	vx_max, vy_max, vz_max, vpsi_max, ax_max, ay_max, az_max, apsi_max,
	//	wx, wy, wz, wpsi, wvx, wvy, wvz, wvpsi, 
	//	wx_last, wy_last, wz_last, wpsi_last, wvx_last, wvy_last, wvz_last, wvpsi_last,
	//	reset_target, reset_trajectory, reset_bounds, reset_weights, 
	//	planned_traj);

	// Output:

	state4_t set_point;

	planned_trajectory_mutex_.lock(); {

		if (first || reset) {

			planned_trajectory_.clear();
			planned_trajectory_.resize(20);

			first = false;

		}

		for (int i = 0; i < 20; i++) {

			state4_t tmp_state;
			for (int j = 0; j < 8; j++) tmp_state(j) = planned_traj[i*8+j];

			planned_trajectory_[i] = tmp_state;

		}

		set_point = planned_trajectory_[1];

	} planned_trajectory_mutex_.unlock();

	return set_point;

}

void CableLandingController::clearPlannedTrajectory() {

	planned_trajectory_mutex_.lock(); {

		planned_trajectory_.clear();
		planned_trajectory_.resize(0);

		planned_macro_trajectory_.clear();
		planned_macro_trajectory_.resize(0);

		for (int i = 0; i < 8; i++) trajectory_target_(i) = NAN;

	} planned_trajectory_mutex_.unlock();

}

void CableLandingController::setTrajectoryTarget(state4_t target) {

	planned_trajectory_mutex_.lock(); {

		trajectory_target_ = target;

	} planned_trajectory_mutex_.unlock();

}

bool CableLandingController::updateTargetCablePose(int new_id) {

	bool success = false;

	powerline_mutex_.lock(); {

		if (new_id > -1) {

			target_cable_id_ = new_id;

		}

		for (int i = 0; i < powerline_.count; i++) {

			if (powerline_.ids[i] == target_cable_id_) {

				target_cable_pose_ = tf_buffer_->transform(powerline_.poses[i], "world");

				success = true;

				break;
				
			}
		}

	} powerline_mutex_.unlock();

	return success;

}

void CableLandingController::clearTargetCable() {

	powerline_mutex_.lock(); {

		target_cable_id_ = -1;

	} powerline_mutex_.unlock();

}

int main(int argc, char* argv[]) {
	std::cout << "Starting cable landing controller node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CableLandingController>());

	rclcpp::shutdown();
	return 0;
}