// Copyright 2022 Johnson & Johnson
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mock_detection.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto g_node = std::make_shared<DetectorNode>(rclcpp::NodeOptions());
  executor->add_node(g_node->get_node_base_interface());
  executor->spin();
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
