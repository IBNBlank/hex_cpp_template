/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#include "hex_cpp_template/data_interface/ros2_interface.h"

#include <memory>
#include <string>
#include <vector>

namespace hex {
namespace cpp_template {

void DataInterface::Log(HexLogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    RCLCPP_FATAL(nh_ptr_->get_logger(), "### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case HexLogLevel::kDebug: {
      RCLCPP_DEBUG(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case HexLogLevel::kInfo: {
      RCLCPP_INFO(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case HexLogLevel::kWarn: {
      RCLCPP_WARN(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case HexLogLevel::kError: {
      RCLCPP_ERROR(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case HexLogLevel::kFatal: {
      RCLCPP_FATAL(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    default: {
      RCLCPP_FATAL(nh_ptr_->get_logger(), "### Wrong Log Level ###");
      RCLCPP_FATAL(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
  }

  free(buffer);
}

bool DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  rclcpp::init(argc, argv);
  nh_ptr_ = std::make_shared<rclcpp::Node>(name);

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(HexLogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
  return true;
}

bool DataInterface::Deinit() {
  // timer
  timer_.reset();

  // pub
  string_out_pub_.reset();
  odom_pub_.reset();

  // sub
  string_in_sub_.reset();

  // node
  nh_ptr_.reset();

  // shutdown
  Shutdown();

  return true;
}

void DataInterface::ParameterInit() {
  // string
  nh_ptr_->declare_parameter<std::string>("string_prefix", "hex");
  nh_ptr_->get_parameter("string_prefix", kparam_string_.prefix);

  // odom
  nh_ptr_->declare_parameter<std::string>("odom_frame", "odom");
  nh_ptr_->declare_parameter<std::string>("odom_child_frame", "base_link");
  nh_ptr_->declare_parameter<std::vector<double>>(
      "odom_vel_lin", std::vector<double>{0.0, 0.0, 0.0});
  nh_ptr_->declare_parameter<std::vector<double>>(
      "odom_vel_ang", std::vector<double>{0.0, 0.0, 0.0});
  nh_ptr_->declare_parameter<std::vector<double>>(
      "odom_child_in_frame",
      std::vector<double>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
  std::vector<double> param_odom_vel_lin;
  std::vector<double> param_odom_vel_ang;
  std::vector<double> param_odom_child_in_frame;
  nh_ptr_->get_parameter("odom_frame", kparam_odom_.frame);
  nh_ptr_->get_parameter("odom_child_frame", kparam_odom_.child_frame);
  nh_ptr_->get_parameter("odom_vel_lin", param_odom_vel_lin);
  nh_ptr_->get_parameter("odom_vel_ang", param_odom_vel_ang);
  nh_ptr_->get_parameter("odom_child_in_frame", param_odom_child_in_frame);
  kparam_odom_.vel_lin = Eigen::Vector3d(
      param_odom_vel_lin[0], param_odom_vel_lin[1], param_odom_vel_lin[2]);
  kparam_odom_.vel_ang = Eigen::Vector3d(
      param_odom_vel_ang[0], param_odom_vel_ang[1], param_odom_vel_ang[2]);
  kparam_odom_.child_in_frame.matrix().block<3, 1>(0, 3) = Eigen::Vector3d(
      param_odom_child_in_frame[0], param_odom_child_in_frame[1],
      param_odom_child_in_frame[2]);
  kparam_odom_.child_in_frame.matrix().block<3, 3>(0, 0) =
      Eigen::Quaterniond(
          param_odom_child_in_frame[3], param_odom_child_in_frame[4],
          param_odom_child_in_frame[5], param_odom_child_in_frame[6])
          .toRotationMatrix();
}

void DataInterface::VariableInit() {
  string_in_flag_ = false;
  string_in_buffer_.clear();
}

void DataInterface::PublisherInit() {
  string_out_pub_ =
      nh_ptr_->create_publisher<std_msgs::msg::String>("string_out", 10);
  odom_pub_ = nh_ptr_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

void DataInterface::SubscriberInit() {
  string_in_sub_ = nh_ptr_->create_subscription<std_msgs::msg::String>(
      "string_in", 10,
      std::bind(&DataInterface::StringInHandle, this, std::placeholders::_1));
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  timer_handle_ = handle;
  timer_ = nh_ptr_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(period)),
      std::bind(&DataInterface::TimerHandle, this));
}

void DataInterface::PubStringOut(const std::string& out_string) {
  std_msgs::msg::String::SharedPtr string_out_ptr(new std_msgs::msg::String);
  string_out_ptr->data = out_string;
  string_out_pub_->publish(*string_out_ptr);
}

void DataInterface::PubOdom(const HexOdom& odom) {
  nav_msgs::msg::Odometry::SharedPtr odom_ptr(new nav_msgs::msg::Odometry);
  odom_ptr->header.stamp = rclcpp::Time(odom.stamp.sec, odom.stamp.nsec);
  odom_ptr->header.frame_id = kparam_odom_.frame;
  odom_ptr->child_frame_id = kparam_odom_.child_frame;
  odom_ptr->twist.twist.linear.x = odom.vel_lin.x();
  odom_ptr->twist.twist.linear.y = odom.vel_lin.y();
  odom_ptr->twist.twist.linear.z = odom.vel_lin.z();
  odom_ptr->twist.twist.angular.x = odom.vel_ang.x();
  odom_ptr->twist.twist.angular.y = odom.vel_ang.y();
  odom_ptr->twist.twist.angular.z = odom.vel_ang.z();

  // translation
  Eigen::Vector3d translation = odom.base_in_odom.translation();
  odom_ptr->pose.pose.position.x = translation.x();
  odom_ptr->pose.pose.position.y = translation.y();
  odom_ptr->pose.pose.position.z = translation.z();

  // rotation
  Eigen::Quaterniond quaternion(odom.base_in_odom.rotation());
  odom_ptr->pose.pose.orientation.w = quaternion.w();
  odom_ptr->pose.pose.orientation.x = quaternion.x();
  odom_ptr->pose.pose.orientation.y = quaternion.y();
  odom_ptr->pose.pose.orientation.z = quaternion.z();

  // publish
  odom_pub_->publish(*odom_ptr);
}

void DataInterface::StringInHandle(std_msgs::msg::String::SharedPtr data) {
  if (!string_in_flag_) {
    string_in_buffer_.push_back(data->data);
    string_in_flag_ = true;
  }
}

}  // namespace cpp_template
}  // namespace hex
