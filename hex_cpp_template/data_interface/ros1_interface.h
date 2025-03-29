/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#ifndef HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS1_INTERFACE_H_
#define HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS1_INTERFACE_H_

#include <hex_cpp_utils/interfaces.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include "hex_cpp_template/data_interface/base_interface.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

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
  inline void Shutdown() override { ros::shutdown(); }
  inline bool Ok() override { return ros::ok(); }
  inline HexStamp GetTime() override {
    ros::Time ros_time = ros::Time::now();
    return hex_utils::HexStamp(ros_time.sec, ros_time.nsec);
  }
  inline void Work() override {
    timer_->reset();
    while (ros::ok()) {
      timer_handle_();
      timer_->sleep();
    }
  }
  // Initialization Handle
  bool Init(int, char*[], std::string, double, void (*)()) override;
  bool Deinit();

  // Publisher Handle
  void PubStringOut(const std::string&) override;
  void PubOdom(const HexOdom&) override;

 protected:
  // Subscriber Handle
  void StringInHandle(const std_msgs::StringPtr&);

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
  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;

  // Timer Handle
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::unique_ptr<ros::Rate> timer_;

  // Publisher Handle
  ros::Publisher string_out_pub_;
  ros::Publisher odom_pub_;

  // Subscriber Handle
  ros::Subscriber string_in_sub_;
};

}  // namespace cpp_template
}  // namespace hex

#endif  // HEX_CPP_TEMPLATE_DATA_INTERFACE_ROS1_INTERFACE_H_
