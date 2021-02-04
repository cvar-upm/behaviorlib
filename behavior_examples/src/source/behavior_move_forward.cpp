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

#include "../include/behavior_move_forward.h"

namespace behavior_examples
{
  BehaviorMoveForward::BehaviorMoveForward() : BehaviorExecutionController() { 
    setName("move_forward"); 
    setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
  }

  BehaviorMoveForward::~BehaviorMoveForward() {}

  void BehaviorMoveForward::onConfigure(){
    nh = getNodeHandle();
    nspace = getNamespace();
  }

  void BehaviorMoveForward::onActivate(){
    std::cout<<"Behavior move_forward Activated"<<std::endl;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/" + nspace + "/cmd_vel", 1, true);
  }

  void BehaviorMoveForward::onDeactivate(){
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    vel_pub.shutdown();
    std::cout<<"Behavior move_forward Deactivated"<<std::endl;
  }

  void BehaviorMoveForward::onExecute()
  {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 1;
    vel_pub.publish(vel_msg);
  }

  bool BehaviorMoveForward::checkSituation(){
    
  }

  void BehaviorMoveForward::checkGoal(){

  }

  void BehaviorMoveForward::checkProgress() {
    
  }

  void BehaviorMoveForward::checkProcesses() {

  }

}
PLUGINLIB_EXPORT_CLASS(behavior_examples::BehaviorMoveForward, nodelet::Nodelet)