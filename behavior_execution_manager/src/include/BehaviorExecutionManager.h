/*!*********************************************************************************
 *  \file       behavior_execution_manager.h
 *  \brief      BehaviorExecutionManager definition file.
 *  \authors    Pablo Santamaria, Martin Molina, Abraham Carrera
 *  \copyright  Copyright (c) 2021 Universidad Politecnica de Madrid
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
 ********************************************************************************/

#ifndef BEHAVIORTEST_H
#define BEHAVIORTEST_H

/* GNU/Linux */

#include <map>
#include <string>
#include <chrono>

/* ROS */

#include <ros/ros.h>

/* Aerostack Messages */

#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <behavior_execution_manager_msgs/ActivateBehavior.h>
#include <behavior_execution_manager_msgs/DeactivateBehavior.h>
#include <behavior_execution_manager_msgs/CheckSituation.h>
#include <behavior_execution_manager_msgs/CheckActivation.h>



/* Class definition */
class BehaviorExecutionManager
{

public:

  /* Constructor */
  
  BehaviorExecutionManager();

  /*Destructor*/

  ~BehaviorExecutionManager();

protected:

 enum class ExecutionGoals
  {
    KEEP_RUNNING,
    ACHIEVE_GOAL
  };
  
private:

  /* Class variables */
  double EXECUTION_FREQUENCY;
  
  ros::NodeHandle node_handle;

  std::string name;
  std::string nspace;
  std::string parameters;
  std::string behavior_system;
  std::string error_message;

  std::string activate_behavior_str;
  std::string deactivate_behavior_str;
  std::string check_activation_conditions_str;
  std::string check_activation_str;
  std::string activation_finished_str;
  
  int maximum_execution_time; /* in seconds */
  
  behavior_execution_manager_msgs::BehaviorActivationFinished::_termination_cause_type termination_cause;


  enum class States
  {
    UNCONFIGURED,
    INACTIVE,
    ACTIVE
  };

  States state;
  ExecutionGoals execution_goal;

  ros::ServiceServer activate_behavior_server_srv;
  ros::ServiceServer deactivate_behavior_server_srv;
  ros::ServiceServer check_situation_server_srv;
  ros::ServiceServer check_activation_srv;

  ros::Publisher activation_finished_pub;

  ros::Timer timer;
  ros::Timer execution_timer;



private:

  /* Functions to override */
  virtual void onConfigure() = 0;
  virtual void onActivate() = 0;
  virtual void onDeactivate() = 0;
  virtual void onExecute() = 0;
  virtual bool checkSituation() = 0;
  virtual void checkGoal() = 0;
  virtual void checkProgress() = 0;
  virtual void checkProcesses() = 0;


public:
  void configure();
  void execute();
  void publishBehaviorActivationFinished();
  void start();

  /* Callbacks */

  bool activateServiceCallback(behavior_execution_manager_msgs::ActivateBehavior::Request &,
                                behavior_execution_manager_msgs::ActivateBehavior::Response &);
  bool deactivateServiceCallback(behavior_execution_manager_msgs::DeactivateBehavior::Request &,
                                  behavior_execution_manager_msgs::DeactivateBehavior::Response &);
  bool checkSituationServiceCallback(behavior_execution_manager_msgs::CheckSituation::Request &,
                                         behavior_execution_manager_msgs::CheckSituation::Response &);
  void notifyTimeout(const ros::TimerEvent &);

  /* Functions to inherit */
  bool checkActivation(behavior_execution_manager_msgs::CheckActivation::Request &,
                                         behavior_execution_manager_msgs::CheckActivation::Response &);

protected:

  /* Getters */

  std::string getName();
  std::string getParameters();
  std::string getNamespace();
  int getMaximumExecutionTime();
  behavior_execution_manager_msgs::BehaviorActivationFinished::_termination_cause_type getTerminationCause();
  ros::NodeHandle getNodeHandle();

  /* Setters */

  void setName(std::string name);
  void setParameters(std::string parameters);
  void setErrorMessage(std::string error_message);
  void setMaximumExecutionTime(int maximum_execution_time);
  void setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::_termination_cause_type);
  void setExecutionGoal(ExecutionGoals execution_goal);

};

#endif

