#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <sensor_msgs/image_encodings.h>

#include "geometry.h"

#include <cstdlib>
#include <stdlib.h> 
#include <iostream>   
#include <vector>
#include <string>
#include <chrono>
#include <fstream>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "iii_interfaces/msg/control_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class IIILoggerNode : public rclcpp::Node
{
	public:
		IIILoggerNode(std::string logfiles_dir) : Node("iii_logger") {

			logfiles_dir_ = logfiles_dir;

			std::stringstream odom_ss;
			odom_ss << logfiles_dir_ << "/odom.txt";
			odom_logfile_ = odom_ss.str();
			odom_ofs_ = new std::ofstream(odom_logfile_, std::ofstream::out);
			*odom_ofs_ << "t,state,x,y,z" << std::endl;

			std::stringstream target_ss;
			target_ss << logfiles_dir_ << "/target.txt";
			target_logsfile_ = target_ss.str();
			target_ofs_ = new std::ofstream(target_logsfile_, std::ofstream::out);
			*target_ofs_ << "t,state,x,y,z" << std::endl;

			std::stringstream path_ss;
			path_ss << logfiles_dir_ << "/path.txt";
			path_logsfile_ = path_ss.str();
			path_ofs_ = new std::ofstream(path_logsfile_, std::ofstream::out);
			*path_ofs_ << "t,state,x,y,z" << std::endl;


			state_sub_ = this->create_subscription<std_msgs::msg::String>(
				"/double_cable_lander/state",	10,
				std::bind(&IIILoggerNode::onStateMsg, this, std::placeholders::_1));

			target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"/trajectory_controller/planned_target",	10,
				std::bind(&IIILoggerNode::onTargetMsg, this, std::placeholders::_1));

			path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
				"/trajectory_controller/planned_trajectory",	10,
				std::bind(&IIILoggerNode::onPathMsg, this, std::placeholders::_1));

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			timer = this->create_wall_timer(
			100ms, std::bind(&IIILoggerNode::timerCallback, this));
		}

	private:
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
		rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
		
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		rclcpp::TimerBase::SharedPtr timer{nullptr};

		std::string logfiles_dir_;
		std::string odom_logfile_;
		std::string target_logsfile_;
		std::string path_logsfile_;
		

		std::ofstream *odom_ofs_;
		std::ofstream *target_ofs_;
		std::ofstream *path_ofs_;

		std::string state_;
		geometry_msgs::msg::PoseStamped target_;
		nav_msgs::msg::Path path_;

		void onStateMsg(const std_msgs::msg::String::SharedPtr msg) {
			std::cout << "STATE CALLBACK" << std::endl;
			state_ = msg->data;
		}

		void onTargetMsg(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			std::cout << "TARGET CALLBACK" << std::endl;
			target_ = *msg;
			
		}

		void  onPathMsg(const nav_msgs::msg::Path::SharedPtr msg) {
			std::cout << "PATH CALLBACK" << std::endl;
			path_ = *msg;
		}

		void timerCallback() {

			unsigned int time = this->get_clock()->now().nanoseconds();

			int state = 0;

			if (state_ == "idle")
				state = 0;
			if (state_ == "fly under first cable")
				state = 1;
			if (state_ == "land on first cable")
				state = 2;
			if (state_ == "leave first cable")
				state = 3;
			if (state_ == "fly under second cable")
				state = 4;
			if (state_ == "land on second cable")
				state = 5;
			if (state_ == "on second cable")
				state = 6;

			geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

			point_t drone_pos(
				tf.transform.translation.x,
				tf.transform.translation.y,
				tf.transform.translation.z
			);

			*odom_ofs_ << time << "," << state << "," << drone_pos(0) << "," << drone_pos(1) << "," << drone_pos(2) << std::endl;

			*target_ofs_ << time << "," << state << "," << target_.pose.position.x << "," << target_.pose.position.y << "," << target_.pose.position.z << std::endl;

			for (int i = 0; i < path_.poses.size(); i++) {

				point_t path_point(
					path_.poses[i].pose.position.x,
					path_.poses[i].pose.position.y,
					path_.poses[i].pose.position.z
				);

				*path_ofs_ << time << "," << state << "," << path_point(0) << "," << path_point(1) << "," << path_point(2) << std::endl;

			}
			
		}
};

int main(int argc, char *argv[])
{

	if (argc != 2) {
		std::cout << "Please provide the logging directory as a single command line argument, full path" << std::endl;

		return 1;
	}

	std::cout << "Starting IIILoggerNode..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IIILoggerNode>(argv[1]));

	rclcpp::shutdown();
	return 0;
}
