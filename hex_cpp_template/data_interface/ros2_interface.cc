/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#include "hex_example/data_interface/ros2_interface.h"

namespace hex {
namespace example {

void DataInterface::Log(LogLevel level, const char* format, ...) {
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
    case LogLevel::kDebug: {
      RCLCPP_DEBUG(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case LogLevel::kInfo: {
      RCLCPP_INFO(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case LogLevel::kWarn: {
      RCLCPP_WARN(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case LogLevel::kError: {
      RCLCPP_ERROR(nh_ptr_->get_logger(), "%s", buffer);
      break;
    }
    case LogLevel::kFatal: {
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

void DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  rclcpp::init(argc, argv);
  nh_ptr_ = std::make_shared<rclcpp::Node>(name);

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(LogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
}

void DataInterface::Deinit() {
  // timer
  timer_.reset();

  // pub
  out_string_pub_.reset();

  // sub
  in_string_sub_.reset();

  // node
  nh_ptr_.reset();

  // shutdown
  Shutdown();
}

void DataInterface::ParameterInit() {
  nh_ptr_->declare_parameter<std::string>("out_string", "hello");
  nh_ptr_->declare_parameter<int32_t>("max_count", 10);

  nh_ptr_->get_parameter("out_string", kout_string_);
  nh_ptr_->get_parameter("max_count", kmax_count_);
}

void DataInterface::VariableInit() {
  in_string_flag_ = false;
  in_string_ = "";
}

void DataInterface::PublisherInit() {
  out_string_pub_ =
      nh_ptr_->create_publisher<std_msgs::msg::String>("out_string", 1);
}

void DataInterface::SubscriberInit() {
  in_string_sub_ = nh_ptr_->create_subscription<std_msgs::msg::String>(
      "in_string", 1,
      std::bind(&DataInterface::InStringHandle, this, std::placeholders::_1));
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  timer_handle_ = handle;
  timer_ = nh_ptr_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(period)),
      std::bind(&DataInterface::TimerHandle, this));
}

void DataInterface::PublishOutString(const std::string& out_string) {
  std_msgs::msg::String::SharedPtr out_string_ptr(new std_msgs::msg::String);

  out_string_ptr->data = out_string;

  out_string_pub_->publish(*out_string_ptr);
}

void DataInterface::InStringHandle(std_msgs::msg::String::SharedPtr data) {
  if (!in_string_flag_) {
    in_string_ = data->data;
    in_string_flag_ = true;
  }
}

}  // namespace example
}  // namespace hex
