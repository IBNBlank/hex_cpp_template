/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-28
 ****************************************************************/

#include "hex_cpp_template/data_interface/data_interface.h"
#include "hex_cpp_template/hex_cpp_template.h"

using hex::cpp_template::DataInterface;
using hex::cpp_template::HexCppTemplate;
using hex::cpp_template::HexLogLevel;

const char kNodeName[] = "hex_cpp_template";

void TimeHandle() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static HexCppTemplate& hex_cpp_template = HexCppTemplate::GetSingleton();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (hex_cpp_template.Init()) {
        data_interface.Log(HexLogLevel::kInfo, "%s : Init Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Init Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (hex_cpp_template.Work()) {
        // data_interface.Log(HexLogLevel::kInfo, "%s : Work Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Work Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(HexLogLevel::kError, "%s : Unknown State", kNodeName);
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

int main(int argc, char** argv) {
  DataInterface& data_interface = DataInterface::GetSingleton();
  data_interface.Init(argc, argv, kNodeName, 20.0, TimeHandle);

  data_interface.Work();

  data_interface.Deinit();
  return 0;
}
