/*
 * Copyright (c) 2014, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Oct 24, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include "trs_msgs/MotorCmdSet.h"
#include "trs_control/switchNode.h"
#include "trs_control/getMotorCmdSet.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE	HEART_BEAT //50

/*** Variables ***/
bool switch_node = false; //disable by default
bool object_detected = false;
bool skeleton_detected = false;
bool face_near_detected = false;

/*** Callback functions ***/
void anglesArmsDescription_cb(const sensor_msgs::JointStateConstPtr& js)
{
	if(switch_node)
	{
		if((js->name.size() != 0))
		{
			skeleton_detected = true;
		}
		else
		{
			skeleton_detected = false;
		}
	}
}

void objectCoord_cb(const std_msgs::Int32MultiArrayConstPtr& coords)
{
	if(switch_node)
	{
		//for left Arm, following 1st object
		if((coords->data[0] != 0) && (coords->data[1] != 0) && (coords->data[2] < 200))
		{
			object_detected = true;
		}
		else
		{
			object_detected = false;
		}
	}
}

void headCoords_cb(const std_msgs::Int16MultiArrayConstPtr& headCoords)
{
	if(switch_node)
	{
		if((headCoords->data[0] != -1) && (headCoords->data[1] != -1)) //if head detected
		{
			if((headCoords->data[2]>200) && (headCoords->data[3]>200)) face_near_detected = true;
		}
		else
		{
			face_near_detected = false;
		}
	}
}

/*** Services ***/
bool switchNode(trs_control::switchNode::Request  &req, trs_control::switchNode::Response &res)
{
	switch_node = req.state;
	return true;
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
    ros::init(argc, argv, "trs_conductor_client_node");
    ros::NodeHandle nh;

    ros::Rate r(LOOP_RATE);

    //Publishers
    ros::Publisher pub_setArmsCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setArmsCommands", 100);
    ros::Publisher pub_setHandsCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setHandsCommands", 100);
    ros::Publisher pub_setHeadCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setHeadCommands", 100);
    ros::Publisher pub_setLegsCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setLegsCommands", 100);

    //Subscribers
    ros::Subscriber sub_anglesArmsDescription = nh.subscribe ("/anglesArmsDescription", 10, anglesArmsDescription_cb);
    ros::Subscriber sub_objectCoord = nh.subscribe ("/objectCoord", 10, objectCoord_cb);

    //Services
    ros::ServiceServer srv_switchNode = nh.advertiseService("switch_conductor", switchNode);
    ros::ServiceClient srvClt_getRandomPosturePlayerCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_random_posture_player_cmd");
    ros::ServiceClient srvClt_getKdTreeAnglesCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_kd_tree_angles_cmd");
    ros::ServiceClient srvClt_getKdTreeObjectCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_kd_tree_object_cmd");
    ros::ServiceClient srvClt_getKdTreeHeadCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_kd_tree_head_cmd");
    ros::ServiceClient srvClt_getFaceTrackingCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_face_tracking_cmd");
    ros::ServiceClient srvClt_getObjectTrackingCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_object_tracking_cmd");
    ros::ServiceClient srvClt_getHandshakeCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_handshake_cmd");
    ros::ServiceClient srvClt_getBalanceLegsCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_balance_legs_cmd");
    ros::ServiceClient srvClt_getRandomLegsPlayerCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_random_legs_player_cmd");

    trs_control::getMotorCmdSet srv_getMotorCmdSet;

    int counter = 0;
    int counterLegs = 0;

	while(ros::ok())
	{
		//ROS_INFO("avant spinOnce");
		ros::spinOnce();
		//ROS_INFO("après spinOnce");

		if(switch_node)
		{
			if(object_detected) //ROS_INFO("object_detected");
			{
				if(srvClt_getKdTreeObjectCmd.call(srv_getMotorCmdSet))
				{
					//publish to the commandBuilder node
					pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
				}
				else
				{
					ROS_ERROR("Failed to call service get_kd_tree_object_cmd");
				}
			}
			else
			{
				if(counter == 15) //publish at 1 Hz in a 25 Hz loop
				{
					if(srvClt_getKdTreeAnglesCmd.call(srv_getMotorCmdSet))
					{
						//publish to the commandBuilder node
						pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
					}
					else
					{
						ROS_ERROR("Failed to call service get_kdtree_angles_cmd");
					}

					counter = 0;
					//ROS_INFO("counter == 15");
				}
				else
				{
					counter++;
				}
			}

			//play handshake
			if(srvClt_getHandshakeCmd.call(srv_getMotorCmdSet))
			{
				//publish to the commandBuilder node
				pub_setHandsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
			}
			else
			{
				ROS_ERROR("Failed to call service get_handshake_cmd");
			}

			if(srvClt_getFaceTrackingCmd.call(srv_getMotorCmdSet))
			{
				//publish to the commandBuilder node
				pub_setHeadCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
			}
			else
			{
				ROS_ERROR("Failed to call service get_face_tracking_cmd");
			}

			if(counterLegs == 150) //publish at 0.1 Hz in a 25 Hz loop
			{
				//in all cases play some random legs postures
				if(srvClt_getRandomLegsPlayerCmd.call(srv_getMotorCmdSet))
				{
					//publish to the commandBuilder node
					pub_setLegsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
				}
				else
				{
					ROS_ERROR("Failed to call service get_random_legs_player_cmd");
				}

				counterLegs = 0;
				//ROS_INFO("counterLegs == 15");
			}
			else
			{
				counterLegs++;
			}

/*
			if(object_detected) //priority on object
			{
				if(srvClt_getKdTreeObjectCmd.call(srv_getMotorCmdSet))
				{
					//publish to the commandBuilder node
					pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
				}
				else
				{
					ROS_ERROR("Failed to call service get_kdtree_object_cmd");
				}

				if(srvClt_getObjectTrackingCmd.call(srv_getMotorCmdSet))
				{
					//publish to the commandBuilder node
					pub_setHeadCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
				}
				else
				{
					ROS_ERROR("Failed to call service get_object_tracking_cmd");
				}
			}
			else
			{
				if(face_near_detected) //2nd priority on face near for handshake
				{
					if(srvClt_getKdTreeHeadCmd.call(srv_getMotorCmdSet))
					{
						//publish to the commandBuilder node
						pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
					}
					else
					{
						ROS_ERROR("Failed to call service get_kdtree_head_cmd");
					}

					//play handshake
					if(srvClt_getHandshakeCmd.call(srv_getMotorCmdSet))
					{
						//publish to the commandBuilder node
						pub_setHandsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
					}
					else
					{
						ROS_ERROR("Failed to call service get_handshake_cmd");
					}
				}
				else
				{
					if(skeleton_detected) //3rd priority on skeleton copying
					{
						if(srvClt_getKdTreeAnglesCmd.call(srv_getMotorCmdSet))
						{
							//publish to the commandBuilder node
							pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
						}
						else
						{
							ROS_ERROR("Failed to call service get_kdtree_angles_cmd");
						}
					}
					else
					{
						//if nobody and no object detected, play random postures
						if(srvClt_getRandomPosturePlayerCmd.call(srv_getMotorCmdSet))
						{
							//publish to the commandBuilder node
							pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
							pub_setHandsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
							//pub_setHeadCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
							//pub_setLegsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
						}
						else
						{
							ROS_ERROR("Failed to call service get_random_posture_player_cmd");
						}
					}
				}

				//in all cases do face tracking
				if(srvClt_getFaceTrackingCmd.call(srv_getMotorCmdSet))
				{
					//publish to the commandBuilder node
					pub_setHeadCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
				}
				else
				{
					ROS_ERROR("Failed to call service get_face_tracking_cmd");
				}
			}*/
/*
			if(srvClt_getObjectTrackingCmd.call(srv_getMotorCmdSet))
			{
				//publish to the commandBuilder node
				pub_setHeadCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
			}
			else
			{
				ROS_ERROR("Failed to call service get_object_tracking_cmd");
			}
			*/
/*
			//in all cases play some random legs postures
			if(srvClt_getRandomLegsPlayerCmd.call(srv_getMotorCmdSet))
			{
				//publish to the commandBuilder node
				pub_setLegsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
			}
			else
			{
				ROS_ERROR("Failed to call service get_random_legs_player_cmd");
			}


			//in all cases balance the robot legs
			if(srvClt_getBalanceLegsCmd.call(srv_getMotorCmdSet))
			{
				//publish to the commandBuilder node
				pub_setLegsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
			}
			else
			{
				ROS_ERROR("Failed to call service get_balance_legs_cmd");
			}*/
		}
		else
		{
			//ROS_INFO("Conductor OFF");
		}

		r.sleep();
	}

	return 0;
}
