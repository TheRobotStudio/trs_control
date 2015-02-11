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
 *  Created on: Oct 30, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include "trs_msgs/MotorCmdSet.h"
#include "trs_msgs/MotorDataSet.h"
#include <std_msgs/Int32MultiArray.h>
#include "trs_control/switchNode.h"
#include "trs_control/getMotorCmdSet.h"
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE						25
#define EYEBALL_CMD_OFFSET				70
#define HEAD_PITCH_OFFSET 				74
#define HEAD_YAW_OFFSET 				87
#define HEAD_PITCH_UP_LIMIT				74
#define HEAD_PITCH_DOWN_LIMIT			140
#define HEAD_YAW_LEFT_LIMIT				153
#define HEAD_YAW_RIGHT_LIMIT			87

/*** Variables ***/
bool switch_node = false; //disable by default
trs_msgs::MotorCmdSet motorCmdSet;

int currPitch = 120;
int currYaw = 120;

/*** Functions ***/
void initCmdSet()
{
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = NO_MODE;
		motorCmdSet.motorCmd[i].value = 120;
	}
}

void setHeadPitch(int pitch) //up/down
{
	int neck7_pos = 120;
	int neck8_pos = 120;
	int neck9_pos = 120;
	int neck10_pos = 120;

	//int pitch = sl_headPitch.getValue();
	int deltaPitch = 0;

	int yaw = currYaw;
	int deltaYaw = 0;

	if(pitch > HEAD_PITCH_DOWN_LIMIT) pitch = HEAD_PITCH_DOWN_LIMIT;
	if(pitch < HEAD_PITCH_UP_LIMIT) pitch = HEAD_PITCH_UP_LIMIT;

	if(yaw > 120)  //right
	{
		deltaYaw = yaw - 120;

		if(pitch > 120) //up
		{
			deltaPitch = pitch - 120;
			neck7_pos = 120 - deltaPitch - deltaYaw;
			neck8_pos = 120 - deltaPitch + deltaYaw;

			//neck9_pos = 120 + deltaPitch + deltaYaw;
			//neck10_pos = 120 + deltaPitch;
		}
		else if(pitch <= 120) //down
		{
			deltaPitch = 120 - pitch;
			neck7_pos = 120 + deltaPitch - deltaYaw;
			neck8_pos = 120 + deltaPitch + deltaYaw;

			//neck9_pos = 120 - deltaPitch;
			//neck10_pos = 120 - deltaPitch;
		}
	}
	else //left
	{
		deltaYaw = 120 - yaw;

		if(pitch > 120) //up
		{
			deltaPitch = pitch - 120;
			neck7_pos = 120 - deltaPitch + deltaYaw;
			neck8_pos = 120 - deltaPitch - deltaYaw;

			//neck9_pos += deltaPitch;
			//neck10_pos += deltaPitch;
		}
		else if(pitch <= 120) //down
		{
			deltaPitch = 120 - pitch;
			neck7_pos = 120 + deltaPitch + deltaYaw;
			neck8_pos = 120 + deltaPitch - deltaYaw;

			//neck9_pos -= deltaPitch;
			//neck10_pos -= deltaPitch;
		}

	}

	currPitch = pitch;

	motorCmdSet.motorCmd[HEAD_NECK7_ID-1].value = neck7_pos;
	motorCmdSet.motorCmd[HEAD_NECK8_ID-1].value = neck8_pos;
}

void setHeadYaw(int yaw) //left/right
{
	int neck7_pos = 120;
	int neck8_pos = 120;
	int neck9_pos = 120;
	int neck10_pos = 120;

	int deltaYaw = 0;

	int pitch = currPitch;
	int deltaPitch = 0;

	if(yaw > HEAD_YAW_LEFT_LIMIT) yaw = HEAD_YAW_LEFT_LIMIT;
	if(yaw < HEAD_YAW_RIGHT_LIMIT) yaw = HEAD_YAW_RIGHT_LIMIT;

	if(pitch > 120)  //down
	{
		deltaPitch = pitch - 120;

		if(yaw > 120) //left
		{
			deltaYaw = yaw - 120;
			neck7_pos = 120 - deltaPitch - deltaYaw;
			neck8_pos = 120 - deltaPitch + deltaYaw;
			//neck9_pos = 130;
			//neck10_pos = 120;
		}
		else if(yaw <= 120) //right
		{
			deltaYaw = 120 - yaw;
			neck7_pos = 120 - deltaPitch + deltaYaw;
			neck8_pos = 120 - deltaPitch - deltaYaw;
			//neck9_pos = 120;
			//neck10_pos = 130;
		}
	}
	else //up
	{
		deltaPitch = 120 - pitch;

		if(yaw > 120) //left
		{
			deltaYaw = yaw - 120;
			neck7_pos = 120 + deltaPitch - deltaYaw;
			neck8_pos = 120 + deltaPitch + deltaYaw;
			//neck9_pos = 130;
			//neck10_pos = 120;
		}
		else if(yaw <= 120) //right
		{
			deltaYaw = 120 - yaw;
			neck7_pos = 120 + deltaPitch + deltaYaw;
			neck8_pos = 120 + deltaPitch - deltaYaw;

			//neck9_pos = 120;
			//neck10_pos = 130;
		}
	}

	currYaw = yaw;

	motorCmdSet.motorCmd[HEAD_NECK7_ID-1].value = neck7_pos;
	motorCmdSet.motorCmd[HEAD_NECK8_ID-1].value = neck8_pos;
}

/*** Callback functions ***/
void objectCoord_cb(const std_msgs::Int32MultiArrayConstPtr& oc)
{
	if(switch_node)
	{
		if((oc->data[0] != 0) && (oc->data[1] != 0) && (oc->data[2] < 200)) //if object detected
		{
			//prepare motorCmdSet array with mode
			for(int i=FIRST_SERVO_ID-1; i<3*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
			{
				motorCmdSet.motorCmd[i].mode = SERVO_POS_MODE;
			}

			float headYaw_f = (oc->data[0]/450)*40;
			float headPitch_f = (oc->data[1]/450)*60;
			int headYaw = (int)headYaw_f + 100;
			int headPitch = 120;//(int)headPitch_f + 90;
			int eyeYaw = 120;
			int eyePitch = 120;
			int eyeRoll = 120;
			float eyeIris_f = ((oc->data[2]-100)/100)*120;
			int eyeIris = eyeIris_f + 60;

			setHeadYaw(headYaw);
			setHeadPitch(headPitch);

			motorCmdSet.motorCmd[HEAD_EYE_YAW_ID-1].value = eyeYaw;
			motorCmdSet.motorCmd[HEAD_EYE_PITCH_ID-1].value = eyePitch;
			motorCmdSet.motorCmd[HEAD_EYE_ROLL_ID-1].value = eyeRoll;
			motorCmdSet.motorCmd[HEAD_EYE_IRIS_ID-1].value = eyeIris;
		}
		else
		{

		}
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
	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motorCmdSet = motorCmdSet;
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
    ros::init(argc, argv, "trs_lookAtObject_server_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Subscribers
    ros::Subscriber sub_objectCoord = nh.subscribe("/objectCoord", 1, objectCoord_cb);

    //Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_object_tracking", switchNode);
	ros::ServiceServer srv_getMotorCmdSet = nh.advertiseService("get_object_tracking_cmd", getMotorCmdSet);

    initCmdSet();

    while(ros::ok())
	{
    	ros::spinOnce();

    	r.sleep();
	}
}
