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


geometry_msgs::PoseStamped oriPos1, ps1, currentPos1, nextPos1, initCirclePos1;
geometry_msgs::PoseStamped oriPos2, ps2, currentPos2, nextPos2, initCirclePos2;

ros::Publisher localPositionPublisher1;
ros::Publisher localPositionPublisher2;

float value;
bool hasSet1;
bool hasSet2;
Eigen::Vector3d current1;
Eigen::Vector3d current2;

int angle,angleStep;
bool isFlyCircle,hasInitFlyCircle;
int radius;

ros::ServiceClient arming_client1;
ros::ServiceClient arming_client2;

ros::ServiceClient set_mode_client1;
ros::ServiceClient set_mode_client2;

mavros_msgs::SetMode offb_set_mode1;
mavros_msgs::SetMode offb_set_mode2;

mavros_msgs::State current_state1;
mavros_msgs::State current_state2;


mavros_msgs::CommandBool arm_cmd1;
mavros_msgs::CommandBool arm_cmd2;

int currentUAV;



// Eigen::Vector3d circle_shape(int angle){
//   double r = 5.0f;  // 5 meters radius

//   return Eigen::Vector3d(r * cos(angles::from_degrees(angle)),
//       r * sin(angles::from_degrees(angle)),
//       1.0f);
// }


// void flyCircleWithRadius(double r)
// {
//   if(!hasInitFlyCircle){
//     initCirclePos = currentPos;
//     nextPos = initCirclePos;
//     angle = angle + angleStep;
//     nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
//                               initCirclePos.pose.position.x;
//     nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
//     ps.pose = nextPos.pose;
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
//     ROS_INFO_STREAM("next angle:" << angle);
//     hasInitFlyCircle = true;
//   } 
  
//   // judge whether is reached
//   bool isReached = false;
//   double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
//                        (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
//   double threshold = 0.2;
//   if (distance < threshold)
//   {
//     isReached = true;
//   }

//   if (isReached)
//   {
//     // send next pos
//     angle = angle + angleStep;
//     if(angle > 360) angle = angle - 360;
//     nextPos = initCirclePos;
//     nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
//                               initCirclePos.pose.position.x;
//     nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
//     ps.pose = nextPos.pose;
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
    
//   } else {
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
//     ROS_INFO_STREAM("next angle:" << angle);
//   }


// }

// void flyHeartWithRadius(double r)
// {
//   if(!hasInitFlyCircle){
//     initCirclePos = currentPos;
//     nextPos = initCirclePos;
//     angle = angle + angleStep;
//     nextPos.pose.position.x = r *(2*cos(angles::from_degrees(angle)) - cos(angles::from_degrees(2*angle)))  - r + 
//                               initCirclePos.pose.position.x;
//     nextPos.pose.position.y = r *(2*sin(angles::from_degrees(angle)) - sin(angles::from_degrees(2*angle)))  + initCirclePos.pose.position.y;
//     ps.pose = nextPos.pose;
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
//     ROS_INFO_STREAM("next angle:" << angle);
//     hasInitFlyCircle = true;
//   } 

//   bool isReached = false;
//   double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
//                        (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
//   double threshold = 0.2;
//   if (distance < threshold)
//   {
//     isReached = true;
//   }

//   if (isReached)
//   {
//     // send next pos
//     angle = angle + angleStep;
//     if(angle > 360) angle = angle - 360;
//     nextPos = initCirclePos;
//     nextPos.pose.position.x = r *(2*cos(angles::from_degrees(angle)) - cos(angles::from_degrees(2*angle)))  - r + 
//                               initCirclePos.pose.position.x;
//     nextPos.pose.position.y = r *(2*sin(angles::from_degrees(angle)) - sin(angles::from_degrees(2*angle)))  + initCirclePos.pose.position.y;
//     ps.pose = nextPos.pose;
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
    
//   } else {
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
//     ROS_INFO_STREAM("next angle:" << angle);
//   }
// }

// void flyPeachHeartWithRadius(double r)
// {
//   if(!hasInitFlyCircle){
//     initCirclePos = currentPos;
//     nextPos = initCirclePos;
//     angle = angle + angleStep;
//     nextPos.pose.position.x = 16 * sin(angles::from_degrees(angle))*
//                                  sin(angles::from_degrees(angle))*
//                                  sin(angles::from_degrees(angle))* - r + 
//                               initCirclePos.pose.position.x;
//     nextPos.pose.position.y = 13*cos(angles::from_degrees(angle))-
//                                5*cos(angles::from_degrees(2*angle))-
//                                2*cos(angles::from_degrees(3*angle)) - 
//                                cos(angles::from_degrees(4*angle))+ initCirclePos.pose.position.y;
//     ps.pose = nextPos.pose;
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
//     ROS_INFO_STREAM("next angle:" << angle);
//     hasInitFlyCircle = true;
//   } 

//   bool isReached = false;
//   double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
//                        (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
//   double threshold = 0.1;
//   if (distance < threshold)
//   {
//     isReached = true;
//   }

//   if (isReached)
//   {
//     // send next pos
//     angle = angle + angleStep;
//     if(angle > 360) angle = angle - 360;
//     nextPos = initCirclePos;
//     nextPos.pose.position.x = 16 * sin(angles::from_degrees(angle))*
//                                  sin(angles::from_degrees(angle))*
//                                  sin(angles::from_degrees(angle))* - r + 
//                               initCirclePos.pose.position.x;
//     nextPos.pose.position.y = 13*cos(angles::from_degrees(angle))-
//                                5*cos(angles::from_degrees(2*angle))-
//                                2*cos(angles::from_degrees(3*angle)) - 
//                                cos(angles::from_degrees(4*angle))+ initCirclePos.pose.position.y;
//     ps.pose = nextPos.pose;
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
    
//   } else {
//     ps.header.stamp = ros::Time::now();
//     localPositionPublisher.publish(ps);
//     ROS_INFO_STREAM("next angle:" << angle);
//   }
// }



void localPositionReceived1(const geometry_msgs::PoseStampedConstPtr& msg){
    currentPos1 = *msg;
    if(!hasSet1){
	     ps1 = *msg;
       oriPos1 = ps1;
	     hasSet1 = true;
    }	
}

void localPositionReceived2(const geometry_msgs::PoseStampedConstPtr& msg){
    currentPos2 = *msg;
    if(!hasSet2){
       ps2 = *msg;
       oriPos2 = ps2;
       hasSet2 = true;
    } 
}



void sendCommand(const keyboard::Key &key)
{

  switch(key.code)
  {
      case 'i':
      {
        // Forward
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.position.x += value;
              ROS_INFO_STREAM("Rotors1 Forward: " << value);
              break;
            }  
            case 2:
            {
              ps2.pose.position.x += value;
              ROS_INFO_STREAM("Rotors2 Forward: " << value);
              break; 
            }
        }       
        break;
      }
      case ',':
      {
        // Backward 
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.position.x -= value;
              ROS_INFO_STREAM("Rotors1 Backward: " << value);
              break;
            }  
            case 2:
            {
              ps2.pose.position.x -= value;
              ROS_INFO_STREAM("Rotors2 Backward: " << value);
              break; 
            }
        }       
        break;        
      }
      case 'k':
      {
        // Hold        
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.position.x += 0;
              ps1.pose.position.y += 0;
              ps1.pose.position.z += 0;
              ROS_INFO_STREAM("Rotors1 Hold: " << value);
              break;
            }  
            case 2:
            {
              ps2.pose.position.x += 0;
              ps2.pose.position.y += 0;
              ps2.pose.position.z += 0;
              ROS_INFO_STREAM("Rotors2 Hold: " << value);
              break; 
            }
        }
        break;
      }
      case 'j':
      {
        // left        
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.position.y += value;
              ROS_INFO_STREAM("Rotors1 Left: " << value);
              break;
            }  
            case 2:
            {
              ps2.pose.position.y += value;
              ROS_INFO_STREAM("Rotors2 Left: " << value);
              break; 
            }
        }        
        break;
      }
      case 'l':
      {
        // right
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.position.y -= value;
              ROS_INFO_STREAM("Rotors1 Right: " << value);
              break;
            }  
            case 2:
            {
              ps2.pose.position.y -= value;
              ROS_INFO_STREAM("Rotors2 Right: " << value);
              break; 
            }
        }        
        break;
      }
      case 'u':
      {
        // turn left        
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.orientation.z += value/2;
              ROS_INFO_STREAM("Rotors1 Turn Left: " << value/2*180/3.14 << "degree");
              break;
            }  
            case 2:
            {
              ps2.pose.orientation.z += value/2;
              ROS_INFO_STREAM("Rotors2 Turn Left: " << value/2*180/3.14 << "degree");
              break; 
            }
        }
        break;
      }
      case 'o':
      {
        // turn right
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.orientation.z -= value/2;
              ROS_INFO_STREAM("Rotors1 Turn Right: " << value/2*180/3.14 << "degree");
              break;
            }  
            case 2:
            {
              ps2.pose.orientation.z -= value/2;
              ROS_INFO_STREAM("Rotors2 Turn Right: " << value/2*180/3.14 << "degree");
              break; 
            }
        }        
        break;
      }
      case 'w':
      {
        // Up 
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.position.z += value;
              ROS_INFO_STREAM("Rotors1 Up: " << value);
              break;
            }  
            case 2:
            {
              ps2.pose.position.z += value;
              ROS_INFO_STREAM("Rotors2 Up: " << value);
              break; 
            }
        }
        break;
      }
      case 's':
      {
        // Down         
        switch(currentUAV)
        {
            case 1:
            {
              ps1.pose.position.z -= value;
              if (ps1.pose.position.z < 0.2)
              {
                  ps1.pose.position.z = 0.2;
              }
              ROS_INFO_STREAM("Rotors1 Down: " << value);
              break;
            }  
            case 2:
            {
              ps2.pose.position.z -= value;
              if (ps2.pose.position.z < 0.2)
              {
                  ps2.pose.position.z = 0.2;
              }
              ROS_INFO_STREAM("Rotors2 Down: " << value);
              break; 
            }
        }        
        break;
      }
      case 'q':
      {
        // Increase value
        value += 0.1f;
        ROS_INFO_STREAM("Increase value:" << value);
        break;
      }
      case 'a':
      {
         value = 0.1f;
         ROS_INFO_STREAM("Increase value:" << value);
         break;
      }
      case 'z':
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
        switch(currentUAV)
        {
            case 1:
            {
              ps1 = oriPos1;
              ROS_INFO_STREAM("Rotors1 Turn to original position");
              break;
            }  
            case 2:
            {
              ps2 = oriPos2;
              ROS_INFO_STREAM("Rotors2 Turn to original position");
              break; 
            }
        }     
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
        switch(currentUAV)
        {
            ROS_WARN_STREAM("Switch to Offboard");
            case 1:
            {
              offb_set_mode1.request.custom_mode = "OFFBOARD";
              set_mode_client1.call(offb_set_mode1);
              if (offb_set_mode1.response.success)
                ROS_WARN_STREAM("Rotors1 Offboard enabled");
              break;
            }  
            case 2:
            {
              offb_set_mode2.request.custom_mode = "OFFBOARD";
              set_mode_client2.call(offb_set_mode2);
              if (offb_set_mode2.response.success)
                ROS_WARN_STREAM("Rotors2 Offboard enabled");
              break; 
            }
        } 
        break;
      }
      case 'g':
      {   
        switch(currentUAV)
        {
            
            case 1:
            {
              ROS_WARN_STREAM("Rotors1 ARM");
              arm_cmd1.request.value = true;
              arming_client1.call(arm_cmd1);
              if (arm_cmd1.response.success)
                ROS_WARN_STREAM("Rotors1 Vehicle armed");
              break;
            }  
            case 2:
            {
              ROS_WARN_STREAM("Rotors2 ARM");
              arm_cmd2.request.value = true;
              arming_client2.call(arm_cmd2);
              if (arm_cmd2.response.success)
                ROS_WARN_STREAM("Rotors2 Vehicle armed");
              break; 
            }
        }
        break;
      }
      case 'b':
      {   
        switch(currentUAV)
        {
            case 1:
            {
              ROS_WARN_STREAM("Rotors1 DISARM");
              arm_cmd1.request.value = false;
              arming_client1.call(arm_cmd1);
              if (arm_cmd1.response.success)
                ROS_WARN_STREAM("Rotors1 Vehicle disarmed");
              break;
            }  
            case 2:
            {
              ROS_WARN_STREAM("Rotors2 DISARM");
              arm_cmd2.request.value = false;
              arming_client2.call(arm_cmd2);
              if (arm_cmd2.response.success)
                ROS_WARN_STREAM("Rotors2 Vehicle disarmed");
              break; 
            }
        }
        break;
      }
      case 'n':
      {                    
        switch(currentUAV)
        {
            case 1:
            {
              hasSet1 = false;
              arm_cmd1.request.value = false;
              arming_client1.call(arm_cmd1);
              if (arm_cmd1.response.success)
                ROS_WARN_STREAM("Rotors1 Vehicle disarmed");
              break;
            }  
            case 2:
            {
              hasSet2 = false;
              arm_cmd2.request.value = false;
              arming_client2.call(arm_cmd2);
              if (arm_cmd2.response.success)
                ROS_WARN_STREAM("Rotors2 Vehicle disarmed");
              break; 
            }
        }
        break;
      }
      case '1':
      {
        currentUAV = 1;
        break;
      }
      case '2':
      {
        currentUAV = 2;
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

  int uav_num;
  nodeHandle.getParam("uav_num", uav_num);
  ROS_INFO("uav_num was %d", uav_num);
  ros::Subscriber localPositionSubsciber1;
  ros::Subscriber localPositionSubsciber2;
  ros::Subscriber commandSubscriber1;
  ros::Subscriber commandSubscriber2;

  value = 0.1f;
  hasSet1 = false;
  hasSet2 = false;

  switch(uav_num)
  {
      case 1:
      {
        currentUAV = 1;
        localPositionPublisher1 = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
        localPositionSubsciber1 = nodeHandle.subscribe("/mavros/local_position/pose", 10, localPositionReceived1);
        commandSubscriber1 = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);
         
        break;
      }
      case 2:
      {
        currentUAV = 1;
        localPositionPublisher1 = nodeHandle.advertise<geometry_msgs::PoseStamped>("/rotors1/mavros/setpoint_position/local",10);
        localPositionSubsciber1 = nodeHandle.subscribe("/rotors1/mavros/local_position/pose", 10, localPositionReceived1);
        commandSubscriber1 = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);

        localPositionPublisher2 = nodeHandle.advertise<geometry_msgs::PoseStamped>("/rotors2/mavros/setpoint_position/local",10);
        localPositionSubsciber2 = nodeHandle.subscribe("/rotors2/mavros/local_position/pose", 10, localPositionReceived2);
        commandSubscriber2 = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);

        break; 
      }
      default:
      {

      }
  }   


  // std::string rotors_topic = "/rotors";
  // std::string mavros_setpoint_position_topic = rotors_topic.append(std::to_string(uav_num));
  // mavros_setpoint_position_topic = mavros_setpoint_position_topic.append("/mavros/setpoint_position/local");
  // localPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(mavros_setpoint_position_topic,10);

  // std::string mavros_local_position_topic;
  // mavros_local_position_topic = rotors_topic;
  // mavros_local_position_topic = mavros_local_position_topic.append("/mavros/local_position/pose");
  // ros::Subscriber localPositionSubsciber = nodeHandle.subscribe(mavros_local_position_topic, 10, localPositionReceived);

  // std::string keyboard_topic;
  // keyboard_topic = rotors_topic;
  // keyboard_topic = keyboard_topic.append("/keyboard/keydown");
  // ros::Subscriber commandSubscriber = nodeHandle.subscribe(keyboard_topic,1,sendCommand);
  


  // fly circle parameters
  isFlyCircle = false;
  hasInitFlyCircle = false;
  angle = 0;
  radius = 5;
  angleStep = 5;

  // std::string arming_service1;
  // std::string arming_service2;
  // arming_service = rotors_topic;
  // arming_service = arming_service.append("/mavros/cmd/arming");
  

  // std::string set_mode_service1;
  // std::string set_mode_service2;
  // set_mode_service = set_mode_service.append("/mavros/set_mode");
  switch(uav_num)
  {
      case 1:
      {
        arming_client1 = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client1 = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        break;
      }
      case 2:
      {
         
        arming_client1 = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/rotors1/mavros/cmd/arming");
        arming_client2 = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/rotors2/mavros/cmd/arming");
        set_mode_client1 = nodeHandle.serviceClient<mavros_msgs::SetMode>("/rotors1/mavros/set_mode");
        set_mode_client2 = nodeHandle.serviceClient<mavros_msgs::SetMode>("/rotors2/mavros/set_mode");

        break; 
      }
      default:
      {

      }
  }   





  ros::Rate loopRate(10.0);
  while(ros::ok())
  {

    if(true) {
      ps1.header.seq++;
      ps2.header.seq++;

      if(!isFlyCircle)
      {        
        //ROS_INFO_STREAM("send ps" << ps);
        switch(uav_num)
        {
            case 1:
            {
                ps1.header.stamp = ros::Time::now();
                localPositionPublisher1.publish(ps1);
                // ROS_INFO_STREAM("send ps" << ps1);
                break;                
            }
            case 2:
            {
                ps1.header.stamp = ros::Time::now();
                ps2.header.stamp = ros::Time::now();
                localPositionPublisher1.publish(ps1);
                localPositionPublisher2.publish(ps2);
                break;              
            }
        }
      } 
      else 
      {
        //flyCircleWithRadius(radius);
        //flyHeartWithRadius(radius);
        //flyPeachHeartWithRadius(radius);
      }      
    }
    ros::spinOnce();
    loopRate.sleep();
  }

}
