/*!*********************************************************************************
 *  \file       behaviorlib.cpp
 *  \brief      This file implements the BehaviorExecutionController class
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

#include <behaviorlib.h>

/* Constructor */
BehaviorExecutionController::BehaviorExecutionController()
{
  execution_goal = ExecutionGoals::ACHIEVE_GOAL;
  state = States::UNCONFIGURED;
}

/* Destructor */
BehaviorExecutionController::~BehaviorExecutionController() {}

/* Class functions */

void BehaviorExecutionController::configure()
{
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("namespace", nspace, "drone1");
  private_nh.param<double>("frecuency", EXECUTION_FREQUENCY, 100.0);
  private_nh.param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  private_nh.param<std::string>("deactivate_behavior_srv", deactivate_behavior_str, "deactivate_behavior");
  private_nh.param<std::string>("check_activation_conditions_srv", check_activation_conditions_str,
                                "check_activation_conditions");
  private_nh.param<std::string>("activation_finished_topic", activation_finished_str, "behavior_activation_finished");
  private_nh.getParam("behavior_system", behavior_system);

  onConfigure();
  if (behavior_system!="") {
  activate_behavior_server_srv =
      node_handle.advertiseService("/"+nspace+"/"+behavior_system+"/behavior_"+getName() + "/" + activate_behavior_str,
                                   &BehaviorExecutionController::activateServiceCallback, this);
  deactivate_behavior_server_srv =
      node_handle.advertiseService("/"+nspace+"/"+behavior_system+"/behavior_"+getName() + "/" + deactivate_behavior_str,
                                   &BehaviorExecutionController::deactivateServiceCallback, this);
  check_situation_server_srv =
      node_handle.advertiseService("/"+nspace+"/"+behavior_system+"/behavior_"+getName() + "/" + check_activation_conditions_str,
                                   &BehaviorExecutionController::checkSituationServiceCallback, this);
  }
  else {
   
    activate_behavior_server_srv =
      node_handle.advertiseService("/"+nspace+"/behavior_"+getName() + "/" + activate_behavior_str,
                                   &BehaviorExecutionController::activateServiceCallback, this);
    deactivate_behavior_server_srv =
      node_handle.advertiseService("/"+nspace+"/behavior_"+getName() + "/" + deactivate_behavior_str,
                                   &BehaviorExecutionController::deactivateServiceCallback, this);
    check_situation_server_srv =
      node_handle.advertiseService("/"+nspace+"/behavior_"+getName() + "/" + check_activation_conditions_str,
                                   &BehaviorExecutionController::checkSituationServiceCallback, this);

  }
  activation_finished_pub = node_handle.advertise<behaviorlib_msg::BehaviorActivationFinished>(
      "/" + nspace + "/" + activation_finished_str, 100);
  state = States::INACTIVE;
}

void BehaviorExecutionController::execute(const ros::TimerEvent &)
{ 
  //std::cout<<"behavior execution controller entra en execute() de "<<ros::this_node::getName()<<std::endl;
  if (state == States::ACTIVE)
  {
    if (termination_cause != behaviorlib_msg::BehaviorActivationFinished::TIME_OUT)
    {
      if (execution_goal == ExecutionGoals::ACHIEVE_GOAL)
      {
        checkGoal();
      }
      if (termination_cause != behaviorlib_msg::BehaviorActivationFinished::GOAL_ACHIEVED)
      {
        checkProgress();
      }
      else if (termination_cause != behaviorlib_msg::BehaviorActivationFinished::WRONG_PROGRESS)
           {
               checkProcesses();
           } 
    }

   
   if (termination_cause == behaviorlib_msg::BehaviorActivationFinished::TIME_OUT  || termination_cause == behaviorlib_msg::BehaviorActivationFinished::GOAL_ACHIEVED || termination_cause == behaviorlib_msg::BehaviorActivationFinished::WRONG_PROGRESS || termination_cause == behaviorlib_msg::BehaviorActivationFinished::PROCESS_FAILURE)
    {

      publishBehaviorActivationFinished();

      onDeactivate();

      state = States::INACTIVE;
          
      termination_cause = 0;
      timer.stop(); 


    }

   else 
   {
      onExecute();
   }
  
 }

}

void BehaviorExecutionController::publishBehaviorActivationFinished()
{ 
  behaviorlib_msg::BehaviorActivationFinished activation_finished_msg;
  std::string behavior_name = getName();
  std::transform(behavior_name.begin(), behavior_name.end(), behavior_name.begin(), ::toupper);
  activation_finished_msg.header.stamp = ros::Time::now();
  activation_finished_msg.name = behavior_name;
  activation_finished_msg.termination_cause = termination_cause;
  activation_finished_msg.error_message = error_message;
  activation_finished_pub.publish(activation_finished_msg);
}

/* Nodelet functions */

void BehaviorExecutionController::onInit()
{

  configure();

  execution_timer = node_handle.createTimer(ros::Duration((double)(1.0/EXECUTION_FREQUENCY)), &BehaviorExecutionController::execute, this);

}    

/* Getters */

behaviorlib_msg::BehaviorActivationFinished::_termination_cause_type BehaviorExecutionController::getTerminationCause()
{
  return termination_cause;
}

std::string BehaviorExecutionController::getName() { 
  return name; 
}

std::string BehaviorExecutionController::getParameters() { 
  return parameters; 
}

std::string BehaviorExecutionController::getNamespace() { 
  return nspace; 
}


int BehaviorExecutionController::getMaximumExecutionTime() { 
  return  maximum_execution_time;
}

ros::NodeHandle BehaviorExecutionController::getNodeHandle() { 
  return node_handle; 
}

/* Setters */

void BehaviorExecutionController::setTerminationCause(behaviorlib_msg::BehaviorActivationFinished::_termination_cause_type termination_cause)
{
  this->termination_cause = termination_cause;
}

void BehaviorExecutionController::setName(std::string name) 
{ 
  this->name = name; 
}

void BehaviorExecutionController::setParameters(std::string parameters) 
{ 
  this->parameters = parameters; 
}

void BehaviorExecutionController::setErrorMessage(std::string error_message) 
{ 
  this->error_message = error_message; 
}

void BehaviorExecutionController::setMaximumExecutionTime(int maximum_execution_time) 
{ 
  this->maximum_execution_time = maximum_execution_time; 
}

void BehaviorExecutionController::setExecutionGoal(ExecutionGoals execution_goal) 
{ 
  this->execution_goal = execution_goal; 
}

/* Callbacks */

bool BehaviorExecutionController::activateServiceCallback(behaviorlib_msg::ActivateBehavior::Request &req,
                                                           behaviorlib_msg::ActivateBehavior::Response &resp)
{  //unsigned long init=ros::Time::now().toNSec();

    if (state == States::INACTIVE)
    { 
      state = States::ACTIVE;
      setParameters(req.arguments);
      setMaximumExecutionTime(req.timeout);
 
      onActivate();
      

      if (execution_goal == ExecutionGoals::ACHIEVE_GOAL)
      { 
        timer = node_handle.createTimer(ros::Duration(getMaximumExecutionTime()), &BehaviorExecutionController::notifyTimeout, this);
      }
     
      resp.ack = true;
      
    }

   else 
   {
    ROS_WARN("Node %s received an activation call when it was already active", ros::this_node::getName().c_str());

    resp.ack = false;
    resp.error_message = error_message;
   }

 //unsigned long end=ros::Time::now().toNSec();
//std::cout<<"BEHAVIOR: "<<getName()<<", activateServiceCallback , TIME: "<< end-init<<std::endl;

  return resp.ack;
}

bool BehaviorExecutionController::deactivateServiceCallback(behaviorlib_msg::DeactivateBehavior::Request &req,
                                                             behaviorlib_msg::DeactivateBehavior::Response &resp)
{ 

  if (state == States::ACTIVE)
  {
    state = States::INACTIVE;
    termination_cause = behaviorlib_msg::BehaviorActivationFinished::INTERRUPTED;
    publishBehaviorActivationFinished();
    onDeactivate();
    resp.ack = true;
  }
  else
  {
  ROS_WARN("Node %s received a deactivation call when it was already deactivated", ros::this_node::getName().c_str());
  resp.ack = false;
  error_message = "Behavior [" + ros::this_node::getName() + "] is not active";
  resp.error_message = error_message;
  }


  return resp.ack;
}

bool BehaviorExecutionController::checkSituationServiceCallback(behaviorlib_msg::CheckSituation::Request &req, behaviorlib_msg::CheckSituation::Response &resp)
{
  resp.situation_occurs = checkSituation();
  resp.error_message = error_message;
  return resp.situation_occurs;
}

void BehaviorExecutionController::notifyTimeout(const ros::TimerEvent &timer_event) 
{  if (state == States::ACTIVE)
  {
  termination_cause = behaviorlib_msg::BehaviorActivationFinished::TIME_OUT;
  }
}
