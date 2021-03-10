/*!********************************************************************************
 * \brief     Take off behavior implementation 
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef MOVE_TO_POINT_H
#define MOVE_TO_POINT_H

// System
#include <string>
// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>

//behavior_execution_manager
#include <BehaviorExecutionManager.h>

// behavior_execution_manager msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>

class BehaviorMoveToPoint : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorMoveToPoint();
  ~BehaviorMoveToPoint();
  int main(int argc, char** argv);

private:
  ros::NodeHandle nh;
  std::string nspace;

  ros::Subscriber pose_sub;
  ros::Publisher vel_pub;

  turtlesim::Pose turtlesim_pose;
  turtlesim::Pose goal_pose;

  double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
  const double PI = 3.14159265359;


private:
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

public: // Callbacks
  void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
  double getDistance(double x1, double y1, double x2, double y2);
};

#endif
