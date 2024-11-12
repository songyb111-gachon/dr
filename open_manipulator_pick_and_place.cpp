/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_pick_and_place/open_manipulator_pick_and_place.h"

OpenManipulatorPickandPlace::OpenManipulatorPickandPlace()
: node_handle_(""),
  priv_node_handle_("~"),
  mode_state_(0),
  demo_count_(0),
  pick_ar_id_(0)
{
  present_joint_angle_.resize(NUM_OF_JOINT_AND_TOOL, 0.0);
  present_kinematic_position_.resize(3, 0.0);

  joint_name_ = {"joint1", "joint2", "joint3", "joint4"};

  initServiceClient();
  initSubscribe();
}

OpenManipulatorPickandPlace::~OpenManipulatorPickandPlace()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void OpenManipulatorPickandPlace::initServiceClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
}

void OpenManipulatorPickandPlace::initSubscribe()
{
  open_manipulator_states_sub_ = node_handle_.subscribe("states", 10, &OpenManipulatorPickandPlace::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorPickandPlace::jointStatesCallback, this);
  open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("gripper/kinematics_pose", 10, &OpenManipulatorPickandPlace::kinematicsPoseCallback, this);
  ar_pose_marker_sub_ = node_handle_.subscribe("/ar_pose_marker", 10, &OpenManipulatorPickandPlace::arPoseMarkerCallback, this);
}

bool OpenManipulatorPickandPlace::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorPickandPlace::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorPickandPlace::setTaskSpacePath(std::vector<double> kinematics_pose, std::vector<double> kinematics_orientation, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_orientation.at(0);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_orientation.at(1);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_orientation.at(2);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_orientation.at(3);

  srv.request.path_time = path_time;

  if (goal_task_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorPickandPlace::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  open_manipulator_is_moving_ = (msg->open_manipulator_moving_state == msg->IS_MOVING);
}

void OpenManipulatorPickandPlace::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle(NUM_OF_JOINT_AND_TOOL, 0.0);
  for (int i = 0; i < msg->name.size(); i++)
  {
    if (msg->name.at(i) == "joint1") temp_angle[0] = msg->position.at(i);
    else if (msg->name.at(i) == "joint2") temp_angle[1] = msg->position.at(i);
    else if (msg->name.at(i) == "joint3") temp_angle[2] = msg->position.at(i);
    else if (msg->name.at(i) == "joint4") temp_angle[3] = msg->position.at(i);
    else if (msg->name.at(i) == "gripper") temp_angle[4] = msg->position.at(i);
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorPickandPlace::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  present_kinematic_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

void OpenManipulatorPickandPlace::arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  ar_marker_pose.clear();
  for (const auto &marker : msg->markers)
  {
    ArMarker temp;
    temp.id = marker.id;
    temp.position = {marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z};
    ar_marker_pose.push_back(temp);
  }
}

void OpenManipulatorPickandPlace::publishCallback(const ros::TimerEvent&)
{
  printText();
  if (kbhit()) setModeState(std::getchar());

  if (mode_state_ == HOME_POSE)
  {
    setJointSpacePath(joint_name_, {0.00, -1.05, 0.35, 0.70}, 2.0);
    setToolControl({0.0});
    mode_state_ = 0;
  }
  else if (mode_state_ == DEMO_START && !open_manipulator_is_moving_)
  {
    demoSequence();
  }
}

void OpenManipulatorPickandPlace::setModeState(char ch)
{
  if (ch == '1') mode_state_ = HOME_POSE;
  else if (ch == '2') { mode_state_ = DEMO_START; demo_count_ = 0; }
  else if (ch == '3') mode_state_ = DEMO_STOP;
}

void OpenManipulatorPickandPlace::demoSequence()
{
  std::vector<double> joint_angle, kinematics_position, kinematics_orientation, gripper_value;

  switch (demo_count_)
  {
    case 0:
      setJointSpacePath(joint_name_, {0.00, -1.05, 0.35, 0.70}, 1.5);
      demo_count_++;
      break;

    case 1:
      setJointSpacePath(joint_name_, {0.01, -0.80, 0.00, 1.90}, 1.0);
      demo_count_++;
      break;

    case 2:
      setToolControl({0.010});
      demo_count_++;
      break;

    case 3:
      for (const auto &marker : ar_marker_pose)
      {
        if (marker.id == pick_ar_id_)
        {
          setTaskSpacePath({marker.position[0], marker.position[1], 0.05}, {0.74, 0.00, 0.66, 0.00}, 2.0);
          demo_count_++;
          return;
        }
      }
      demo_count_ = 2; // Retry detection
      break;

    case 4:
      setToolControl({-0.002});
      demo_count_++;
      break;

    case 5:
      setJointSpacePath(joint_name_, {0.01, -0.80, 0.00, 1.90}, 1.0);
      demo_count_++;
      break;

    case 6:
      setJointSpacePath(joint_name_, {1.57, -0.21, -0.15, 1.89}, 1.0);
      demo_count_++;
      break;

    case 7:
      setTaskSpacePath(
          {present_kinematic_position_[0], present_kinematic_position_[1], present_kinematic_position_[2] - 0.05},
          {0.74, 0.00, 0.66, 0.00}, 2.0);
      demo_count_++;
      break;

    case 8:
      setToolControl({0.010});
      demo_count_++;
      break;

    case 9:
      setTaskSpacePath({present_kinematic_position_[0], present_kinematic_position_[1], 0.135}, {0.74, 0.00, 0.66, 0.00}, 2.0);
      demo_count_++;
      break;

    case 10:
      setJointSpacePath(joint_name_, {0.00, -1.05, 0.35, 0.70}, 1.5);
      pick_ar_id_ = (pick_ar_id_ + 1) % 3;
      demo_count_ = (pick_ar_id_ == 0) ? 0 : 1;
      if (pick_ar_id_ == 0) mode_state_ = DEMO_STOP;
      break;
  }
}

void OpenManipulatorPickandPlace::printText()
{
  system("clear");
  printf("Pick and Place demonstration!\n");
  printf("1 : Home pose\n2 : Pick and Place demo start\n3 : Stop\n");

  printf("Present Joint Angles: J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         present_joint_angle_[0], present_joint_angle_[1], present_joint_angle_[2], present_joint_angle_[3]);
  printf("Present Tool Position: %.3lf\n", present_joint_angle_[4]);
  printf("Present Kinematics Position: X: %.3lf Y: %.3lf Z: %.3lf\n",
         present_kinematic_position_[0], present_kinematic_position_[1], present_kinematic_position_[2]);

  for (const auto &marker : ar_marker_pose)
    printf("AR Marker ID: %d --> X: %.3lf Y: %.3lf Z: %.3lf\n", marker.id, marker.position[0], marker.position[1], marker.position[2]);
}

bool OpenManipulatorPickandPlace::kbhit()
{
  termios term, term2;
  tcgetattr(0, &term);
  term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_pick_and_place");
  ros::NodeHandle node_handle("");

  OpenManipulatorPickandPlace open_manipulator_pick_and_place;

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.100), &OpenManipulatorPickandPlace::publishCallback, &open_manipulator_pick_and_place);

  ros::spin();
  return 0;
}
