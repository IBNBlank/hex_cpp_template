/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#include "hex_cpp_template/data_interface/ros1_interface.h"

namespace hex {
namespace cpp_template {

void DataInterface::Log(HexLogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case HexLogLevel::kDebug: {
      ROS_DEBUG("%s", buffer);
      break;
    }
    case HexLogLevel::kInfo: {
      ROS_INFO("%s", buffer);
      break;
    }
    case HexLogLevel::kWarn: {
      ROS_WARN("%s", buffer);
      break;
    }
    case HexLogLevel::kError: {
      ROS_ERROR("%s", buffer);
      break;
    }
    case HexLogLevel::kFatal: {
      ROS_FATAL("%s", buffer);
      break;
    }
    default: {
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
    }
  }

  free(buffer);
}

bool DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  ros::init(argc, argv, name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  nh_ptr_ = &nh;
  nh_local_ptr_ = &nh_local;

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
  spinner_->stop();
  ros::shutdown();
  spinner_.reset();
  timer_.reset();
  return true;
}

void DataInterface::ParameterInit() {
  // string
  nh_local_ptr_->param<std::string>("string_prefix", kparam_string_.prefix,
                                    "hex");

  // odom
  std::vector<double> param_odom_vel_lin;
  std::vector<double> param_odom_vel_ang;
  std::vector<double> param_odom_child_in_odom;
  nh_local_ptr_->param<std::string>("odom_frame", kparam_odom_.frame, "odom");
  nh_local_ptr_->param<std::string>("odom_child_frame",
                                    kparam_odom_.child_frame, "base_link");
  nh_local_ptr_->param<std::vector<double>>(
      "odom_vel_lin", param_odom_vel_lin, std::vector<double>({0.0, 0.0, 0.0}));
  nh_local_ptr_->param<std::vector<double>>(
      "odom_vel_ang", param_odom_vel_ang, std::vector<double>({0.0, 0.0, 0.0}));
  nh_local_ptr_->param<std::vector<double>>(
      "odom_child_in_frame", param_odom_child_in_odom,
      std::vector<double>({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}));
  kparam_odom_.vel_lin = Eigen::Vector3d(
      param_odom_vel_lin[0], param_odom_vel_lin[1], param_odom_vel_lin[2]);
  kparam_odom_.vel_ang = Eigen::Vector3d(
      param_odom_vel_ang[0], param_odom_vel_ang[1], param_odom_vel_ang[2]);
  kparam_odom_.child_in_frame.matrix().block<3, 1>(0, 3) =
      Eigen::Vector3d(param_odom_child_in_odom[0], param_odom_child_in_odom[1],
                      param_odom_child_in_odom[2]);
  kparam_odom_.child_in_frame.matrix().block<3, 3>(0, 0) =
      Eigen::Quaterniond(
          param_odom_child_in_odom[3], param_odom_child_in_odom[4],
          param_odom_child_in_odom[5], param_odom_child_in_odom[6])
          .toRotationMatrix();
}

void DataInterface::VariableInit() {
  string_in_flag_ = false;
  string_in_buffer_.clear();
}

void DataInterface::PublisherInit() {
  string_out_pub_ = nh_ptr_->advertise<std_msgs::String>("string_out", 10);
  odom_pub_ = nh_ptr_->advertise<nav_msgs::Odometry>("odom", 10);
}

void DataInterface::SubscriberInit() {
  string_in_sub_ =
      nh_ptr_->subscribe("string_in", 10, &DataInterface::StringInHandle, this);
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  // spin thread
  spinner_ = std::make_unique<ros::AsyncSpinner>(1);
  spinner_->start();

  // work thread
  timer_handle_ = handle;
  timer_ = std::make_unique<ros::Rate>(1000.0 / period);
}

void DataInterface::PubStringOut(const std::string& string_out) {
  std_msgs::StringPtr string_out_ptr(new std_msgs::String);
  string_out_ptr->data = string_out;
  string_out_pub_.publish(string_out_ptr);
}

void DataInterface::PubOdom(const HexOdom& odom) {
  nav_msgs::OdometryPtr odom_ptr(new nav_msgs::Odometry);
  odom_ptr->header.stamp = ros::Time(odom.stamp.sec, odom.stamp.nsec);
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

  odom_pub_.publish(odom_ptr);
}

void DataInterface::StringInHandle(const std_msgs::StringPtr& data) {
  std::lock_guard<std::mutex> lock(string_in_mutex_);
  string_in_buffer_.push_back(data->data);
  string_in_flag_ = true;
}

}  // namespace cpp_template
}  // namespace hex
