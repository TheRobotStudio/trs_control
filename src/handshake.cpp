/*
 * Copyright (c) 2013, The Robot Studio
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
 *  Created on: Apr 4, 2013
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include "trs_msgs/MotorCmdSet.h"
#include <trs_msgs/MotorDataSet.h>
#include "trs_control/switchNode.h"
#include "trs_control/getMotorCmdSet.h"
#include <std_msgs/Bool.h>
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE					HEART_BEAT
#define ARMS_IDS_RANGE				30
#define CURRENT_POSITION_THRESHOLD	40 //400
#define FORCE_CURRENT_RATIO			3
#define SPIKE_FILTER				3000
#define OPEN_HAND_POSITION			58000
//#define CLOSE_HAND_POSITION			60000


/*** Variables ***/
bool switch_node = false; //disable by default
trs_msgs::MotorCmdSet motorCmdSet;
int force = 0;
int current = 0;
int position = 0 + OPEN_HAND_POSITION;

/*** Callback functions ***/
void motorDataSet_cb(const trs_msgs::MotorDataSetConstPtr& data)
{
	if(switch_node)
	{
		force = data->motorData[RIGHT_ARM_HAND_ID-1].force;

		if(force <= SPIKE_FILTER)
		{
			current = force/FORCE_CURRENT_RATIO;

			motorCmdSet.motorCmd[RIGHT_ARM_HAND_ID-1].nodeID = RIGHT_ARM_HAND_ID;

			if(current < CURRENT_POSITION_THRESHOLD)
			{
				motorCmdSet.motorCmd[RIGHT_ARM_HAND_ID-1].mode = POSITION_MODE;
				motorCmdSet.motorCmd[RIGHT_ARM_HAND_ID-1].value = OPEN_HAND_POSITION;
			}
			else
			{
				motorCmdSet.motorCmd[RIGHT_ARM_HAND_ID-1].mode = CURRENT_MODE;//POSITION_MODE; //CURRENT_MODE;
				motorCmdSet.motorCmd[RIGHT_ARM_HAND_ID-1].value = 150 + current;
			}
		}
		//ROS_INFO("current = %d", current);
	}
}

/*** Services ***/
bool switchNode(trs_control::switchNode::Request  &req, trs_control::switchNode::Response &res)
{
	switch_node = req.state;
	return true;
}

bool getMotorCmdSet(trs_control::getMotorCmdSet::Request  &req, trs_control::getMotorCmdSet::Response &res)
{
	//send the cmd set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motorCmdSet = motorCmdSet;
		//ROS_INFO("service called");
		return true;
	}
	else
	{
		return false;
	}
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
    ros::init(argc, argv, "trs_handshake_server_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Subscribers
    ros::Subscriber sub_motorDataSet = nh.subscribe("/motorDataSet", 1, motorDataSet_cb);

    //Services
    ros::ServiceServer srv_switchNode = nh.advertiseService("switch_handshake", switchNode);
    ros::ServiceServer srv_getMotorCmdSet = nh.advertiseService("get_handshake_cmd", getMotorCmdSet);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

