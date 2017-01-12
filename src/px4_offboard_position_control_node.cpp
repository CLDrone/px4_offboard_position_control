/****************************************************************************
 *
 *   Copyright (c) 2015 Crossline Drone Project Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name CLDrone or Crossline Drone nor the names of its c
 *    ontributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <array>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <keyboard/Key.h>
#include <math.h>



#include <stdio.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>




geometry_msgs::PoseStamped oriPos,ps,currentPos,nextPos,initCirclePos;
ros::Publisher localPositionPublisher;
float value;
bool hasSet;
Eigen::Vector3d current;
int angle,angleStep;
bool isFlyCircle,hasInitFlyCircle;
int radius;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;



Eigen::Vector3d circle_shape(int angle){
    double r = 5.0f;  // 5 meters radius

    return Eigen::Vector3d(r * cos(angles::from_degrees(angle)),
        r * sin(angles::from_degrees(angle)),
        1.0f);
  }


void flyCircleWithRadius(double r)
{
  if(!hasInitFlyCircle){
    initCirclePos = currentPos;
    nextPos = initCirclePos;
    angle = angle + angleStep;
    nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
    ps.pose = nextPos.pose;
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    ROS_INFO_STREAM("next angle:" << angle);
    hasInitFlyCircle = true;
  } 
  
  // judge whether is reached
  bool isReached = false;
  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
  double threshold = 0.2;
  if (distance < threshold)
  {
    isReached = true;
  }

  if (isReached)
  {
    // send next pos
    angle = angle + angleStep;
    if(angle > 360) angle = angle - 360;
    nextPos = initCirclePos;
    nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
    ps.pose = nextPos.pose;
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    
  } else {
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    ROS_INFO_STREAM("next angle:" << angle);
  }


}

void flyHeartWithRadius(double r)
{
  if(!hasInitFlyCircle){
    initCirclePos = currentPos;
    nextPos = initCirclePos;
    angle = angle + angleStep;
    nextPos.pose.position.x = r *(2*cos(angles::from_degrees(angle)) - cos(angles::from_degrees(2*angle)))  - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r *(2*sin(angles::from_degrees(angle)) - sin(angles::from_degrees(2*angle)))  + initCirclePos.pose.position.y;
    ps.pose = nextPos.pose;
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    ROS_INFO_STREAM("next angle:" << angle);
    hasInitFlyCircle = true;
  } 

  bool isReached = false;
  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
  double threshold = 0.2;
  if (distance < threshold)
  {
    isReached = true;
  }

  if (isReached)
  {
    // send next pos
    angle = angle + angleStep;
    if(angle > 360) angle = angle - 360;
    nextPos = initCirclePos;
    nextPos.pose.position.x = r *(2*cos(angles::from_degrees(angle)) - cos(angles::from_degrees(2*angle)))  - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r *(2*sin(angles::from_degrees(angle)) - sin(angles::from_degrees(2*angle)))  + initCirclePos.pose.position.y;
    ps.pose = nextPos.pose;
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    
  } else {
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    ROS_INFO_STREAM("next angle:" << angle);
  }
}

void flyPeachHeartWithRadius(double r)
{
  if(!hasInitFlyCircle){
    initCirclePos = currentPos;
    nextPos = initCirclePos;
    angle = angle + angleStep;
    nextPos.pose.position.x = 16 * sin(angles::from_degrees(angle))*
                                 sin(angles::from_degrees(angle))*
                                 sin(angles::from_degrees(angle))* - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = 13*cos(angles::from_degrees(angle))-
                               5*cos(angles::from_degrees(2*angle))-
                               2*cos(angles::from_degrees(3*angle)) - 
                               cos(angles::from_degrees(4*angle))+ initCirclePos.pose.position.y;
    ps.pose = nextPos.pose;
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    ROS_INFO_STREAM("next angle:" << angle);
    hasInitFlyCircle = true;
  } 

  bool isReached = false;
  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
  double threshold = 0.1;
  if (distance < threshold)
  {
    isReached = true;
  }

  if (isReached)
  {
    // send next pos
    angle = angle + angleStep;
    if(angle > 360) angle = angle - 360;
    nextPos = initCirclePos;
    nextPos.pose.position.x = 16 * sin(angles::from_degrees(angle))*
                                 sin(angles::from_degrees(angle))*
                                 sin(angles::from_degrees(angle))* - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = 13*cos(angles::from_degrees(angle))-
                               5*cos(angles::from_degrees(2*angle))-
                               2*cos(angles::from_degrees(3*angle)) - 
                               cos(angles::from_degrees(4*angle))+ initCirclePos.pose.position.y;
    ps.pose = nextPos.pose;
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    
  } else {
    ps.header.stamp = ros::Time::now();
    localPositionPublisher.publish(ps);
    ROS_INFO_STREAM("next angle:" << angle);
  }
}



void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg){
    currentPos = *msg;
    if(!hasSet){
	     ps = *msg;
       oriPos = ps;
	     hasSet = true;
    }	
}


void sendCommand(const keyboard::Key &key)
{

  switch(key.code)
  {
      case 'i':
      {
        // Forward
        ps.pose.position.x += value;
        ROS_INFO_STREAM("Forward: " << value);
        break;
      }
      case ',':
      {
        // Backward
        ps.pose.position.x -= value;
        ROS_INFO_STREAM("Backward: " << value);
        break;
      }
      case 'k':
      {
        // Hold
        ps.pose.position.x += 0;
        ps.pose.position.y += 0;
        ps.pose.position.z += 0;
        ROS_INFO_STREAM("Hold: " << value);
        break;
      }
      case 'j':
      {
        // left
        ps.pose.position.y += value;
        ROS_INFO_STREAM("Left");
        break;
      }
      case 'l':
      {
        // right
        ps.pose.position.y -= value;
        ROS_INFO_STREAM("Right: " << value);
        break;
      }
      case 'u':
      {
        // turn left
        ps.pose.orientation.z += value/2;
        ROS_INFO_STREAM("Turn Left:" << value/2*180/3.14 << "degree");
        break;
      }
      case 'o':
      {
        // turn right
        ps.pose.orientation.z -= value/2;
        ROS_INFO_STREAM("Turn Right" << value/2*180/3.14 << "degree");
        break;
      }
      case 'w':
      {
        // Up
        ps.pose.position.z += value;
        
        ROS_INFO_STREAM("Up: " << value);
        break;
      }
      case 's':
      {
        // Down
        ps.pose.position.z -= value;
        if (ps.pose.position.z < 0.2)
        {
          ps.pose.position.z = 0.2;
        }
        ROS_INFO_STREAM("Down: "<< value);
        break;
      }
      case 'a':
      {
        // Increase value
        value += 0.1f;
        ROS_INFO_STREAM("Increase value:" << value);
        break;
      }
      case 'd':
      {
        // decrease value
        value -= 0.1f;
        if (value == 0.0f)
        {
          value = 0.1f;
        }
        ROS_INFO_STREAM("Decrease value:" << value);
        break;
      }
      case 'x':
      {
        // turn to origin position
        ps = oriPos;
        ROS_INFO_STREAM("Turn to original position");
        break;
      }
      case 'y':
      {
        // fly circle
        isFlyCircle = true;
        ROS_INFO_STREAM("Fly Circle Mode");
        break;
      }
      case 'h':
      {
        // turn to manual mode
        isFlyCircle = false;
        hasInitFlyCircle = false;
        ROS_INFO_STREAM("Manual Mode");
        break;
      }
      // case 't':
      // {
      //   // increase radius
      //   radius++;
      //   ROS_INFO_STREAM("increase radius" << radius);
      //   break;
      // }
      // case 'g':
      // {
      //   // increase radius
      //   radius--;
      //   if (radius < 1)
      //   {
      //     radius = 1;
      //   }
      //   ROS_INFO_STREAM("decrease radius" << radius);
      //   break;
      // }
      // case 'b':
      // {
      //   angleStep++;
      //   ROS_INFO_STREAM("angle step:" << angleStep);
      //   break;
      // }
      case 't':
      {
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);

        if (offb_set_mode.response.success)
          ROS_WARN_STREAM("Offboard enabled");
        break;
      }
      case 'g':
      {
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);
        if (arm_cmd.response.success)
          ROS_WARN_STREAM("Vehicle armed");
        break;
      }
      case 'b':
      {
        arm_cmd.request.value = false;
        arming_client.call(arm_cmd);
        if (arm_cmd.response.success)
          ROS_WARN_STREAM("Vehicle disarmed");
        break;
      }
      case 'n':
      {
        hasSet = false;
        ROS_INFO_STREAM("Local Current Position");
        break;
      }

      default:
      {

      }
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_offboard_position_control_node");
  ros::NodeHandle nodeHandle("~");

  int uav_id;
  nodeHandle.getParam("uav_id", uav_id);
  // ROS_INFO("uav_id was %d", uav_id);

  std::string rotors_topic = "/rotors";
  std::string mavros_setpoint_position_topic = rotors_topic.append(std::to_string(uav_id));
  mavros_setpoint_position_topic = mavros_setpoint_position_topic.append("/mavros/setpoint_position/local");
  localPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(mavros_setpoint_position_topic,10);

  std::string mavros_local_position_topic;
  mavros_local_position_topic = rotors_topic;
  mavros_local_position_topic = mavros_local_position_topic.append("/mavros/local_position/pose");
  ros::Subscriber localPositionSubsciber = nodeHandle.subscribe(mavros_local_position_topic, 10, localPositionReceived);

  std::string keyboard_topic;
  keyboard_topic = rotors_topic;
  keyboard_topic = keyboard_topic.append("/keyboard/keydown");
  ros::Subscriber commandSubscriber = nodeHandle.subscribe(keyboard_topic,1,sendCommand);
  
  value = 0.1f;
  hasSet = false;

  // fly circle parameters
  isFlyCircle = false;
  hasInitFlyCircle = false;
  angle = 0;
  radius = 5;
  angleStep = 5;

  std::string arming_service;
  arming_service = rotors_topic;
  arming_service = arming_service.append("/mavros/cmd/arming");
  arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>(arming_service);

  std::string set_mode_service;
  set_mode_service = set_mode_service.append("/mavros/set_mode");
  set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>(set_mode_service);



  ros::Rate loopRate(10.0);

  while(ros::ok())
  {

    if(hasSet) {
      ps.header.seq++;

      if(!isFlyCircle){
        ps.header.stamp = ros::Time::now();
        //ROS_INFO_STREAM("send ps" << ps);
        localPositionPublisher.publish(ps);
      } else {
        flyCircleWithRadius(radius);
        //flyHeartWithRadius(radius);
        //flyPeachHeartWithRadius(radius);
      }
      
    }

    ros::spinOnce();

    loopRate.sleep();
  }

}
