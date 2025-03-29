/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#ifndef HEX_CPP_TEMPLATE_DATA_INTERFACE_BASE_INTERFACE_H_
#define HEX_CPP_TEMPLATE_DATA_INTERFACE_BASE_INTERFACE_H_

#include <hex_cpp_utils/interfaces.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <string>
#include <vector>

using hex_utils::HexOdom;
using hex_utils::HexStamp;

namespace hex {
namespace cpp_template {

enum class HexLogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

struct HexParamString {
  std::string prefix;
};

struct HexParamOdom {
  std::string frame;
  std::string child_frame;
  Eigen::Vector3d vel_lin;
  Eigen::Vector3d vel_ang;
  Eigen::Affine3d child_in_frame;
};

class BaseInterface {
 public:
  BaseInterface() : string_in_flag_(false) {}
  virtual ~BaseInterface() {};
  BaseInterface(const BaseInterface&) = delete;
  BaseInterface& operator=(const BaseInterface&) = delete;

  // Interface Handle
  virtual void Log(HexLogLevel, const char*, ...) = 0;
  virtual void Shutdown() = 0;
  virtual bool Ok() = 0;
  virtual HexStamp GetTime() = 0;
  virtual void Work() = 0;

  // Initialization Handle
  virtual bool Init(int, char*[], std::string, double, void (*)()) = 0;
  virtual bool Deinit() = 0;

  // Publisher Handle
  virtual void PubStringOut(const std::string&) = 0;
  virtual void PubOdom(const HexOdom&) = 0;

  // Subscriber Handle
  inline bool GetStringInFlag() const { return string_in_flag_; }
  inline void ResetStringInFlag() { string_in_flag_ = false; }
  inline const std::vector<std::string>& GetStringInBuffer() {
    static std::vector<std::string> string_in_buffer;
    std::lock_guard<std::mutex> lock(string_in_mutex_);
    string_in_buffer = std::vector<std::string>(string_in_buffer_);
    string_in_buffer_.clear();
    return string_in_buffer;
  }

  // Parameters Handle
  inline const HexParamString& GetParamString() const { return kparam_string_; }
  inline const HexParamOdom& GetParamOdom() const { return kparam_odom_; }

 protected:
  // Initialization Handle
  virtual void ParameterInit() = 0;
  virtual void VariableInit() = 0;
  virtual void PublisherInit() = 0;
  virtual void SubscriberInit() = 0;
  virtual void TimerInit(double, void (*)()) = 0;

  // Timer Handle
  void (*timer_handle_)();

  // Parameters
  HexParamString kparam_string_;
  HexParamOdom kparam_odom_;

  // Variables
  std::atomic<bool> string_in_flag_;
  mutable std::mutex string_in_mutex_;
  std::vector<std::string> string_in_buffer_;
};

}  // namespace cpp_template
}  // namespace hex

#endif  // HEX_CPP_TEMPLATE_DATA_INTERFACE_BASE_INTERFACE_H_
