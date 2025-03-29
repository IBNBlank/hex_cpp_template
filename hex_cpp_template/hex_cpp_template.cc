/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#include "hex_cpp_template/hex_cpp_template.h"

#include "hex_cpp_template/data_interface/data_interface.h"

namespace hex {
namespace cpp_template {

bool HexCppTemplate::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // Parameter
  kparam_string_ = data_interface.GetParamString();
  kparam_odom_ = data_interface.GetParamOdom();

  // Variable
  string_out_ = "";
  curr_odom_.stamp = hex_utils::HexStamp(0, 0);
  curr_odom_.vel_lin = kparam_odom_.vel_lin;
  curr_odom_.vel_ang = kparam_odom_.vel_ang;
  curr_odom_.base_in_odom = kparam_odom_.child_in_frame;

  return true;
}

bool HexCppTemplate::Work() {
  SubMessage();
  CreateMessage();
  PubMessage();

  return true;
}

void HexCppTemplate::SubMessage() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // string in
  if (data_interface.GetStringInFlag()) {
    std::vector<std::string> string_in_buffer =
        data_interface.GetStringInBuffer();
    data_interface.ResetStringInFlag();

    string_in_queue_.insert(string_in_queue_.end(), string_in_buffer.begin(),
                            string_in_buffer.end());
  }
}

void HexCppTemplate::CreateMessage() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // string out
  if (string_in_queue_.empty()) {
    string_out_ = "";
  } else {
    string_out_ = kparam_string_.prefix + ": ";
    while (!string_in_queue_.empty()) {
      string_out_ += string_in_queue_.front() + ";";
      string_in_queue_.pop_front();
    }
    data_interface.Log(HexLogLevel::kInfo, "string_out = %s", string_out_.c_str());
  }

  // odom
  curr_odom_.stamp = data_interface.GetTime();
}

void HexCppTemplate::PubMessage() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // string out
  if (!string_out_.empty()) {
    data_interface.PubStringOut(string_out_);
  }

  // odom
  data_interface.PubOdom(curr_odom_);
}

}  // namespace cpp_template
}  // namespace hex
