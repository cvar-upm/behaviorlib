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

#include "../include/behavior_move_to_point.h"

namespace behavior_examples
{
  BehaviorMoveToPoint::BehaviorMoveToPoint() : BehaviorExecutionController() 
  { 
    setName("move_to_point"); 
    setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
  }

  BehaviorMoveToPoint::~BehaviorMoveToPoint() {}

  void BehaviorMoveToPoint::onConfigure()
  {
    nh = getNodeHandle();
    nspace = getNamespace();
  }

  void BehaviorMoveToPoint::onActivate()
  {
    std::cout<<"Behavior move_to_point Activated"<<std::endl;
    pose_sub = nh.subscribe("/" + nspace + "/pose", 1, &BehaviorMoveToPoint::poseCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/" + nspace + "/cmd_vel", 1, true);
    
    std::cout<<"Parameters: \"";
    std::string arguments = getParameters();
    std::cout<<arguments<<"\""<<std::endl;

    YAML::Node config_file = YAML::Load(arguments);
    if(!config_file["destination"].IsDefined()){
      std::cout<< "Null point" <<std::endl;
    }
    else{
      std::vector<double> points = config_file["destination"].as<std::vector<double>>();
      goal_pose.x = points[0];
      goal_pose.y = points[1];
      //moveGoal(goal_pose, 0.01);
    }
  }

  void BehaviorMoveToPoint::onDeactivate()
  {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    vel_pub.shutdown();
    pose_sub.shutdown();
    std::cout<<"Behavior move_to_point Deactivated"<<std::endl;
  }

  void BehaviorMoveToPoint::onExecute()
  {
    //std::cout<<"["<<turtlesim_pose.x<<", "<<turtlesim_pose.y<<"]"<<std::endl;
    //We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
    geometry_msgs::Twist vel_msg;
    //angular velocity
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

    if(abs(vel_msg.angular.z)<0.2){
      //linear velocity 
      vel_msg.linear.x = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
      vel_msg.linear.y = 0;
      vel_msg.linear.z = 0;
    }
    vel_pub.publish(vel_msg);
  }

  bool BehaviorMoveToPoint::checkSituation(){
    return true;
  }

  void BehaviorMoveToPoint::checkGoal()
  {
    // Check achievement
    //std::cout<<getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)<<std::endl;
    if (!(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>0.1)){
      geometry_msgs::Twist vel_msg;
      vel_msg.linear.x = 0;
      vel_msg.angular.z = 0;
      vel_pub.publish(vel_msg);
      BehaviorExecutionController::setTerminationCause(behaviorlib_msg::BehaviorActivationFinished::GOAL_ACHIEVED);
      std::cout<<"Behavior move_to_point Finished"<<std::endl;
    }
  }

  void BehaviorMoveToPoint::checkProgress() {}

  void BehaviorMoveToPoint::checkProcesses() {}

  void BehaviorMoveToPoint::poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
    turtlesim_pose.x=pose_message->x;
    turtlesim_pose.y=pose_message->y;
    turtlesim_pose.theta=pose_message->theta;
  }

  void BehaviorMoveToPoint::rotate (double angular_speed, double relative_angle, bool clockwise){

    geometry_msgs::Twist vel_msg;
      //set a random linear velocity in the x-axis
      vel_msg.linear.x =0;
      vel_msg.linear.y =0;
      vel_msg.linear.z =0;
      //set a random angular velocity in the y-axis
      vel_msg.angular.x = 0;
      vel_msg.angular.y = 0;
      if (clockwise)
        vel_msg.angular.z =-abs(angular_speed);
      else
        vel_msg.angular.z =abs(angular_speed);

      double t0 = ros::Time::now().toSec();
      double current_angle = 0.0;
      ros::Rate loop_rate(1000);
      do{
        vel_pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
        //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
      }while(current_angle<relative_angle);
      vel_msg.angular.z =0;
      vel_pub.publish(vel_msg);
  }

  /**
   *  converts angles from degree to radians  
   */

  double BehaviorMoveToPoint::degrees2radians(double angle_in_degrees){
    return angle_in_degrees *PI /180.0;
  }

  /**
   *  turns the robot to a desried absolute angle  
   */
  double BehaviorMoveToPoint::setDesiredOrientation(double desired_angle_radians)
  {	
    double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
    //if we want to turn at a perticular orientation, we subtract the current orientation from it
    bool clockwise = ((relative_angle_radians<0)?true:false);
    //cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
    rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
  }

  void BehaviorMoveToPoint::moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
    //We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
    geometry_msgs::Twist vel_msg;

    ros::Rate loop_rate(10);
    do{
      //linear velocity 
      vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
      vel_msg.linear.y = 0;
      vel_msg.linear.z = 0;
      //angular velocity
      vel_msg.angular.x = 0;
      vel_msg.angular.y = 0;
      vel_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

      vel_pub.publish(vel_msg);

      ros::spinOnce();
      loop_rate.sleep();

    }while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
    std::cout<<"end move goal"<<std::endl;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
  }

  double BehaviorMoveToPoint::getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
  }

}
PLUGINLIB_EXPORT_CLASS(behavior_examples::BehaviorMoveToPoint, nodelet::Nodelet)