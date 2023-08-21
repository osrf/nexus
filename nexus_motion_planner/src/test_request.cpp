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

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <string_view>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// NEXUS messages
#include <nexus_endpoints.hpp>

namespace motion_planner_server {
using namespace std::placeholders;
using namespace std::literals;

class GetMotionPlanClient : public rclcpp::Node
{
public:
  using GetMotionPlanService =
    nexus::endpoints::GetMotionPlanService::ServiceType;

  /**
   * @brief Constructor to GetMotionPlanClient
   *
   * @param get_motion_plan_service_name Endpoint for service server to get motion plan
   */
  explicit GetMotionPlanClient(
    const std::string& get_motion_plan_service_name)
  : Node("get_motion_plan_client"),
    get_motion_plan_service_name_(get_motion_plan_service_name)
  {
    this->get_motion_plan_client_ =
      this->create_client<GetMotionPlanService>(get_motion_plan_service_name_);
  }

  /**
   * @brief Process command line arguments to populate the action goal message for motion planner server
   *
   * @param argc number of arguments
   * @param argv arguments
   */
  void process_cmd_line_args(int argc, char** argv)
  {
    // Map argument flag to functions
    const std::unordered_map<std::string,
      std::function<void(const std::string& arg)>> arg_to_func{

      {"-name", [this](const std::string& arg)
        {
          RCLCPP_INFO(this->get_logger(), "Robot name specified as '%s'",
            arg.c_str());
          get_motion_plan_req_->robot_name = arg;
        }},

      {"-goal_type", [this](const std::string& arg)
        {
          RCLCPP_INFO(this->get_logger(), "Goal Type: %s ", arg.c_str());
          get_motion_plan_req_->goal_type = std::atoi(arg.c_str());
        }},

      {"-frame_id", [this](const std::string& arg)
        {
          RCLCPP_INFO(this->get_logger(), "Pose frame specified as '%s'",
            arg.c_str());
          get_motion_plan_req_->start_pose.header.frame_id = arg;
          get_motion_plan_req_->goal_pose.header.frame_id = arg;
        }},

      {"-t", [this](const std::string& arg)
        {
          RCLCPP_INFO(this->get_logger(), "Target pose specified");
          if (!parse_pose_string(arg, get_motion_plan_req_->goal_pose))
          {
            std::exit(2);
          }
        }},

      {"-s", [this](const std::string& arg)
        {

          RCLCPP_INFO(this->get_logger(), "Start pose specified");
          if (!parse_pose_string(arg, get_motion_plan_req_->start_pose))
          {
            std::exit(2);
          }

          get_motion_plan_req_->start_type =
            get_motion_plan_req_->START_TYPE_POSE;
        }},

      {"-tj", [this](const std::string& arg)
        {
          std::vector<double> goal_joint_values;

          RCLCPP_INFO(this->get_logger(), "Target joint values specified");
          if (!parse_joint_value_string(arg, goal_joint_values))
          {
            std::exit(2);
          }
          print_joint_values("target", goal_joint_values);

          for (unsigned int i = 0; i < goal_joint_values.size(); i++)
          {
            moveit_msgs::msg::JointConstraint goal_joint;
            // NOTE: 'joint_name' is not populated here, as it's value will be filled
            // on the server side with the assumption that the default planning group
            // is being used
            goal_joint.position = goal_joint_values[i];
            goal_joint.tolerance_above = 0.01;
            goal_joint.tolerance_below = 0.01;

            goal_joint.weight = 1.0;

            get_motion_plan_req_->goal_joints.push_back(goal_joint);
          }
        }},

      {"-sj", [this](const std::string& arg)
        {
          std::vector<double> start_joint_values;

          RCLCPP_INFO(this->get_logger(), "Start joint values specified");
          if (!parse_joint_value_string(arg, start_joint_values))
          {
            std::exit(2);
          }
          print_joint_values("start", start_joint_values);

          for (unsigned int i = 0; i < start_joint_values.size(); i++)
          {
            moveit_msgs::msg::JointConstraint start_joint;
            // NOTE: 'joint_name' is not populated here, as it's value will be filled
            // on the server side with the assumption that the default planning group
            // is being used
            start_joint.position = start_joint_values[i];
            start_joint.tolerance_above = 0.01;
            start_joint.tolerance_below = 0.01;

            start_joint.weight = 1.0;

            get_motion_plan_req_->start_joints.push_back(start_joint);
          }

          get_motion_plan_req_->start_type =
            get_motion_plan_req_->START_TYPE_JOINTS;
        }}
    };

    get_motion_plan_req_.reset(new GetMotionPlanService::Request());

    get_motion_plan_req_->start_type = get_motion_plan_req_->START_TYPE_CURRENT;

    for (unsigned int i = 1u; i < (unsigned int)argc; i += 2)
    {
      auto it = arg_to_func.find(argv[i]);
      if (it != arg_to_func.end())
      {
        if (argv[i+1] == NULL)
        {
          RCLCPP_WARN(this->get_logger(),
            "No value specified for flag: %s", argv[i]);
          exit(2);
        }
        else
        {
          it->second(std::string(argv[i+1]));
        }
      }
      else
      {
        if (strcmp(argv[i], "-send_traj") == 0)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Toggled on sending of controller trajectory message");
          send_controller_traj_ = true;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),
            "Unused argument flag: %s", argv[i]);
        }
      }
    }
  }

  /**
   * @brief Parses a pose string into a geometry_msgs::Pose
   *
   * @param pose_str String representing a pose with values separated by commas
   * @param pose geometry_msgs::Pose message
   * @return true If parsing succeeded
   * @return false If parsing succeeded
   */
  bool parse_pose_string(
    const std::string& pose_str,
    geometry_msgs::msg::PoseStamped& pose)
  {
    // Separate pose string by comma
    std::istringstream pose_str_ss(pose_str);
    std::vector<std::string> pose_val_vect;

    for (std::string pose_val; std::getline(pose_str_ss, pose_val, ','); )
    {
      pose_val_vect.push_back(pose_val);
    }

    if (pose_val_vect.size() != 7)
    {
      std::cout <<
        "Make sure there are 7 values in your target/start pose string! In the form of 'X,Y,Z,QX,QY,QZ,QW'\n";
      return false;
    }
    pose.header.stamp = this->get_clock()->now();

    pose.pose.position.x = std::atof(pose_val_vect[0].c_str());
    pose.pose.position.y = std::atof(pose_val_vect[1].c_str());
    pose.pose.position.z = std::atof(pose_val_vect[2].c_str());
    pose.pose.orientation.x = std::atof(pose_val_vect[3].c_str());
    pose.pose.orientation.y = std::atof(pose_val_vect[4].c_str());
    pose.pose.orientation.z = std::atof(pose_val_vect[5].c_str());
    pose.pose.orientation.w = std::atof(pose_val_vect[6].c_str());

    RCLCPP_INFO(this->get_logger(), "   Pose specified: "
      "(%f, %f, %f), (%f, %f, %f, %f)",
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.pose.orientation.x, pose.pose.orientation.y,
      pose.pose.orientation.z, pose.pose.orientation.w);

    return true;
  }

  /**
   * @brief Parses a target string into a std::vector of joint values
   *
   * @param pose_str String representing joint values separated by commas
   * @param joint_values Joint values
   * @return true If parsing succeeded
   * @return false If parsing succeeded
   */
  bool parse_joint_value_string(const std::string& pose_str,
    std::vector<double>& joint_values)
  {
    // Separate pose string by comma
    std::istringstream pose_str_ss(pose_str);

    for (std::string pose_val; std::getline(pose_str_ss, pose_val, ','); )
    {
      joint_values.push_back(std::atof(pose_val.c_str()));
    }

    if (joint_values.size() < 1)
    {
      RCLCPP_ERROR(this->get_logger(), "joint values are empty");
      return false;
    }

    return true;
  }

  /**
   * @brief Print joint values
   *
   * @param joint_group_name
   * @param joint_values
   */
  void print_joint_values(const std::string& joint_group_name,
    const std::vector<double>& joint_values)
  {
    std::ostringstream ss;
    ss << "(";
    for (double joint : joint_values)
    {
      ss << std::to_string(joint) << ", ";
    }
    ss << ")";

    RCLCPP_INFO(this->get_logger(), " %s, joint values: %s",
      joint_group_name.c_str(), ss.str().c_str());
  }

  /**
   * @brief Send an action goal to the motion planner server
   *
   */
  void send_goal_motion_planner()
  {
    if (!this->get_motion_plan_client_->wait_for_service(std::chrono::
      seconds(10)))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Get Motion Plan Server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto result = get_motion_plan_client_->async_send_request(
      get_motion_plan_req_);

    //Wait for the result
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto motion_plan_resp = result.get();

      RCLCPP_INFO(this->get_logger(),
        "MOTION PLANNING RESULTS\n"
        "Moveit Error Code = %d\n"
        "Size of trajectory = %zu\n",
        motion_plan_resp->result.error_code.val,
        motion_plan_resp->result.trajectory.joint_trajectory.points.size());

      motion_planning_succeeded_ = true;

      if (send_controller_traj_)
      {
        publish_joint_trajectory(
          get_motion_plan_req_->robot_name,
          motion_plan_resp->result.trajectory.joint_trajectory);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call %s service",
        get_motion_plan_service_name_.c_str());
    }
  }

  /**
   * @brief Returns a bool indicating that the most recent motion planning has succeeded
   *
   * @return true motion planning succeeded
   * @return false motion planning failed
   */
  bool check_motion_planning_succeeded() const
  {
    return motion_planning_succeeded_;
  }

  /**
   * @brief Publish joint trajectory message
   *
   * @param robot_name Name of robot to plan for
   * @param joint_traj joint trajectory message, which is the motion plan to be executed
   */
  void publish_joint_trajectory(
    const std::string& robot_name,
    const trajectory_msgs::msg::JointTrajectory& joint_traj)
  {
    RCLCPP_INFO(this->get_logger(),
      "Sending trajectory_msgs::msg::JointTrajectory on topic %s",
      (robot_name+"/joint_trajectory_controller/joint_trajectory").c_str());
    this->joint_traj_publisher_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      robot_name+"/joint_trajectory_controller/joint_trajectory", 10);

    RCLCPP_INFO(this->get_logger(), "Publishing joint trajectory");
    joint_traj_publisher_->publish(joint_traj);
  }

private:
  rclcpp::Client<GetMotionPlanService>::SharedPtr get_motion_plan_client_;
  std::shared_ptr<GetMotionPlanService::Request> get_motion_plan_req_;

  // Params
  std::string get_motion_plan_service_name_;

  //Stored values
  bool motion_planning_succeeded_{false};
  bool send_controller_traj_{false};

  // Publishers
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>>
  joint_traj_publisher_;

};  // class GetMotionPlanClient

}  // namespace motion_planner_server

void display_help()
{
  std::cout <<
    "This is an action client for requesting motion plans from "
    "the motion planner server.\n";

  std::cout <<
    "Usage: ros2 run nexus_motion_planner motion_planner_task_client "
    "[-name] ROBOT_NAME [-goal_type] GOAL_TYPE\n"
    "[-t] X,Y,Z,QX,QY,QZ,QW [-s] X,Y,Z,QX,QY,QZ,QW\n"
    "[-tj] VAL1,VAL2,... [-sj] VAL1,VAL2,...\n"
    "[-send_traj]\n"
    "[-frame_id] FRAME_ID\n";

  std::cout << "options:" << '\n' <<
    std::setw(20) << "-name " << "    " << "Name of robot to plan for" <<
    '\n' <<
    std::setw(20) << "-goal_type " << "    " <<
    "Either 0 (Pose) or 1 (joint values)" << '\n' <<
    std::setw(20) << "-t "  << "    " << "Target end-effector pose values" <<
    '\n' <<
    std::setw(20) << "-s "  << "    "<< "Start end-effector pose values" <<
    '\n' <<
    std::setw(20) << "-tj " << "    " << "Target joint values" << '\n' <<
    std::setw(20) << "-sj " << "    " << "Start joint values" << '\n' <<
    std::setw(20) << "-frame_id " <<
    "Frame ID of start/goal pose. Only used if goal_type is 0 (Pose)" << '\n' <<
    std::setw(20) << "-send_traj "
            << "When used, sends a trajectory joint message to the "
            << "controller (debugging purposes)" << '\n';
}

int main(int argc, char** argv)
{
  using namespace std::literals;

  if (argc < 2 ||
    strcmp(argv[1], "-h") == 0 ||
    strcmp(argv[1], "--help") == 0)
  {
    display_help();

    return 1;
  }

  rclcpp::init(argc, argv);

  auto get_motion_plan_client =
    std::make_shared<motion_planner_server::GetMotionPlanClient>(
    nexus::endpoints::GetMotionPlanService::service_name());

  get_motion_plan_client->process_cmd_line_args(argc, argv);

  get_motion_plan_client->send_goal_motion_planner();

  if (!get_motion_plan_client->check_motion_planning_succeeded())
  {
    std::cerr << "Motion Planning failed\n";
    exit(1);
  }

  rclcpp::shutdown();

  return 0;
}
