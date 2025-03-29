/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#ifndef HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS2_INTERFACE_H_
#define HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS2_INTERFACE_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace hex {
namespace example {

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class DataInterface {
 public:
  static DataInterface& GetDataInterface() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(LogLevel, const char*, ...);
  inline void Work() { rclcpp::spin(nh_ptr_); }
  inline void Shutdown() { rclcpp::shutdown(); }
  inline bool Ok() { return rclcpp::ok(); }
  inline double GetTime() { return nh_ptr_->now().seconds(); }

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)());
  void Deinit();

  // Parameter Handle
  inline const std::string& GetOutString() { return kout_string_; }
  inline int32_t GetMaxCount() const { return kmax_count_; }

  // Publisher Handle
  void PublishOutString(const std::string&);

  // Subscriber Handle
  inline bool GetInStringFlag() { return in_string_flag_; }
  inline void ResetInStringFlag() { in_string_flag_ = false; }
  inline const std::string& GetInString() { return in_string_; }

 protected:
  // Timer Handle
  inline void TimerHandle() { timer_handle_(); }

  // Subscriber Handle
  void InStringHandle(const std_msgs::msg::String::SharedPtr);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit();
  void VariableInit();
  void PublisherInit();
  void SubscriberInit();
  void TimerInit(double, void (*)());

  // Node Handle
  std::shared_ptr<rclcpp::Node> nh_ptr_;

  // Timer Handle
  rclcpp::TimerBase::SharedPtr timer_;
  void (*timer_handle_)();

  // Publisher Handle
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr out_string_pub_;

  // Subscriber Handle
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr in_string_sub_;

  // Parameters Handle
  std::string kout_string_;
  int32_t kmax_count_;

  // Variable Handle
  bool in_string_flag_;
  std::string in_string_;
};

}  // namespace example
}  // namespace hex

#endif  // HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS2_INTERFACE_H_
