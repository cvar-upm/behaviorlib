/*!*********************************************************************************
 *  \file       behaviorlib.h
 *  \brief      BehaviorExecutionController definition file.
 *  \authors    Abraham Carrera Groba
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
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

#ifndef BEHAVIORLIB_H
#define BEHAVIORLIB_H

/* GNU/Linux */

#include <map>
#include <string>

/* ROS */

#include <nodelet/nodelet.h>
#include <ros/ros.h>

/* Aerostack Messages */

#include <behaviorlib_msg/BehaviorActivationFinished.h>
#include <behaviorlib_msg/ActivateBehavior.h>
#include <behaviorlib_msg/DeactivateBehavior.h>
#include <behaviorlib_msg/CheckSituation.h>



/* Class definition */

class BehaviorExecutionController : public nodelet::Nodelet
{

public:

  /* Constructor */

  BehaviorExecutionController();

  /*Destructor*/

  ~BehaviorExecutionController();

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
  std::string activation_finished_str;
  
  int maximum_execution_time; /* in seconds */
  
  behaviorlib_msg::BehaviorActivationFinished::_termination_cause_type termination_cause;


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

  /* Class functions */



public:
  void configure();
  void execute(const ros::TimerEvent &);
  void publishBehaviorActivationFinished();


  /* Nodelet functions */

  void onInit();

  /* Callbacks */

  bool activateServiceCallback(behaviorlib_msg::ActivateBehavior::Request &,
                                behaviorlib_msg::ActivateBehavior::Response &);
  bool deactivateServiceCallback(behaviorlib_msg::DeactivateBehavior::Request &,
                                  behaviorlib_msg::DeactivateBehavior::Response &);
  bool checkSituationServiceCallback(behaviorlib_msg::CheckSituation::Request &,
                                         behaviorlib_msg::CheckSituation::Response &);
  void notifyTimeout(const ros::TimerEvent &);


protected:

  /* Getters */

  std::string getName();
  std::string getParameters();
  std::string getNamespace();
  int getMaximumExecutionTime();
  behaviorlib_msg::BehaviorActivationFinished::_termination_cause_type getTerminationCause();
  ros::NodeHandle getNodeHandle();

  /* Setters */

  void setName(std::string name);
  void setParameters(std::string parameters);
  void setErrorMessage(std::string error_message);
  void setMaximumExecutionTime(int maximum_execution_time);
  void setTerminationCause(behaviorlib_msg::BehaviorActivationFinished::_termination_cause_type);
  void setExecutionGoal(ExecutionGoals execution_goal);

};

#endif

