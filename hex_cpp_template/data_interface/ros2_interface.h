/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#ifndef HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS2_INTERFACE_H_
#define HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS2_INTERFACE_H_

#include <hex_cpp_utils/interfaces.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "hex_cpp_template/data_interface/base_interface.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

namespace hex {
namespace cpp_template {

class DataInterface : public BaseInterface {
 public:
  static DataInterface& GetSingleton() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(HexLogLevel, const char*, ...) override;
  inline void Shutdown() override { rclcpp::shutdown(); }
  inline bool Ok() override { return rclcpp::ok(); }
  inline hex_utils::HexStamp GetTime() override {
    rclcpp::Time time = nh_ptr_->now();
    return hex_utils::HexStamp(time.seconds(), time.nanoseconds());
  }
  inline void Work() override { rclcpp::spin(nh_ptr_); }

  // Initialization Handle
  bool Init(int, char*[], std::string, double, void (*)()) override;
  bool Deinit() override;

  // Publisher Handle
  void PubStringOut(const std::string&) override;
  void PubOdom(const HexOdom&) override;

 protected:
  // Timer Handle
  inline void TimerHandle() { timer_handle_(); }

  // Subscriber Handle
  void StringInHandle(const std_msgs::msg::String::SharedPtr);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit() override;
  void VariableInit() override;
  void PublisherInit() override;
  void SubscriberInit() override;
  void TimerInit(double, void (*)()) override;

  // Node Handle
  std::shared_ptr<rclcpp::Node> nh_ptr_;

  // Timer Handle
  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher Handle
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_out_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Subscriber Handle
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_in_sub_;
};

}  // namespace cpp_template
}  // namespace hex

#endif  // HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS2_INTERFACE_H_
