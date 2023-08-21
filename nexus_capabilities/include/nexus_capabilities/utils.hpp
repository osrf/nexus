/*
 * Copyright (C) 2022 Johnson & Johnson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef NEXUS_CAPABILITIES__UTILS_HPP
#define NEXUS_CAPABILITIES__UTILS_HPP

#include <behaviortree_cpp_v3/basic_types.h>

//TODO(YV): Add namespace
//==============================================================================
inline std::string bt_status_to_string(BT::NodeStatus status)
{
  switch (status)
  {
    case BT::NodeStatus::SUCCESS:
      return "SUCCESS";
    case BT::NodeStatus::FAILURE:
      return "FAILURE";
    case BT::NodeStatus::IDLE:
      return "IDLE";
    case BT::NodeStatus::RUNNING:
      return "RUNNING";
  }
}

//==============================================================================
struct WeakPtrHelpers
{
  /**
   * Tries to lock a weak pointer, calls std::terminate() if it fails.
   */
  template<typename T>
  static std::shared_ptr<T> lock_or_panic(std::weak_ptr<T> w_ptr)
  {
    auto sp = w_ptr.lock();
    if (!sp)
    {
      std::cerr <<
        "FATAL ERROR!!! OBJECT IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!"
                << std::endl;
      std::terminate();
    }
    return sp;
  }
};

#endif // NEXUS_CAPABILITIES__UTILS_HPP
