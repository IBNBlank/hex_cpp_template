/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#ifndef HEX_CPP_TEMPLATE_CPP_TEMPLATE_H_
#define HEX_CPP_TEMPLATE_CPP_TEMPLATE_H_

#include <hex_cpp_utils/interfaces.h>

#include <deque>
#include <string>

#include "hex_cpp_template/data_interface/base_interface.h"

namespace hex {
namespace cpp_template {

class HexCppTemplate {
 public:
  static HexCppTemplate& GetSingleton() {
    static HexCppTemplate singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  HexCppTemplate() = default;
  virtual ~HexCppTemplate() = default;

  // Work Handle
  void SubMessage();
  void CreateMessage();
  void PubMessage();

  // Parameters Handle
  HexParamString kparam_string_;
  HexParamOdom kparam_odom_;

  // Variable Handle
  std::deque<std::string> string_in_queue_;
  std::string string_out_;
  HexOdom curr_odom_;
};

}  // namespace cpp_template
}  // namespace hex

#endif  // HEX_CPP_TEMPLATE_CPP_TEMPLATE_H_
