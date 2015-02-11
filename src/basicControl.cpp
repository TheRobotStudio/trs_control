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
#include "trs_msgs/MotorDataSet.h"
#include <std_msgs/Bool.h>
#include "trs_control/getMotorCmdSet.h"
#include "trs_control/getSlaveCurrentCmd.h"
#include "trs_control/getHeadPositionCmd.h"
#include "trs_control/getSlaveLockCmd.h"
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				50
#define ARMS_IDS_RANGE			2*NUMBER_MAX_EPOS2_PER_SLAVE
#define RE25_ARM_CURRENT		400
#define HEAD_PITCH_UP_LIMIT		74
#define HEAD_PITCH_DOWN_LIMIT	140
#define HEAD_YAW_LEFT_LIMIT		153
#define HEAD_YAW_RIGHT_LIMIT	87

/*** Variables ***/
trs_msgs::MotorCmdSet motorCmdSet;
trs_msgs::MotorDataSet motorDataSet;

int currPitch = 120;
int currYaw = 120;

/*** Functions ***/
//Initialize the commands to NO_MODE
void initCmdSet()
{
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = NO_MODE;
		motorCmdSet.motorCmd[i].value = 0;
	}
}

void recenterHead()
{
	for(int i=FIRST_SERVO_ID-1; i<NUMBER_OF_MOTORS; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = SERVO_POS_MODE;
		motorCmdSet.motorCmd[i].value = 120;
	}

	currPitch = 120;
	currYaw = 120;
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
void motorDataSet_cb(const trs_msgs::MotorDataSetConstPtr& data)
{
	motorDataSet = *data;
}

/*** Services ***/
//services for the whole robot
bool getAllZeroPositionCmd(trs_control::getMotorCmdSet::Request  &req, trs_control::getMotorCmdSet::Response &res)
{
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = POSITION_MODE;
		motorCmdSet.motorCmd[i].value = 0;
	}

	res.motorCmdSet = motorCmdSet;

	return true;
}

bool getSlaveZeroPositionCmd(trs_control::getSlaveLockCmd::Request  &req, trs_control::getSlaveLockCmd::Response &res)
{
	initCmdSet();

	for(int i=(req.slaveNb-1)*NUMBER_MAX_EPOS2_PER_SLAVE; i<req.slaveNb*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = POSITION_MODE;
		motorCmdSet.motorCmd[i].value = 0;
	}

	res.motorCmdSet = motorCmdSet;

	return true;
}

//services for slave board
bool getSlaveCurrentCmd(trs_control::getSlaveCurrentCmd::Request  &req, trs_control::getSlaveCurrentCmd::Response &res)
{
	initCmdSet();

	for(int i=(req.slaveNb-1)*NUMBER_MAX_EPOS2_PER_SLAVE; i<req.slaveNb*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = CURRENT_MODE;
		motorCmdSet.motorCmd[i].value = req.current;
	}

	res.motorCmdSet = motorCmdSet;

	return true;
}

bool getSlaveLockCmd(trs_control::getSlaveLockCmd::Request  &req, trs_control::getSlaveLockCmd::Response &res)
{
	initCmdSet();

	for(int i=(req.slaveNb-1)*NUMBER_MAX_EPOS2_PER_SLAVE; i<req.slaveNb*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = POSITION_MODE;
		motorCmdSet.motorCmd[i].value = motorDataSet.motorData[i].encPosition;
	}

	res.motorCmdSet = motorCmdSet;

	return true;
}

//services for the head
bool getHeadPositionCmd(trs_control::getHeadPositionCmd::Request  &req, trs_control::getHeadPositionCmd::Response &res)
{
	initCmdSet();

	//prepare motorCmdSet array with mode
	//for(int i=2*NUMBER_MAX_EPOS2_PER_SLAVE-1; i<3*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	for(int i=FIRST_SERVO_ID-1; i<3*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motorCmdSet.motorCmd[i].mode = SERVO_POS_MODE;
	}

	if(req.recenter == true)
	{
		recenterHead();
	}
	else
	{
		setHeadYaw(req.headYaw);
		setHeadPitch(req.headPitch);
		motorCmdSet.motorCmd[HEAD_EYE_PITCH_ID-1].value = req.eyePitch;
		motorCmdSet.motorCmd[HEAD_EYE_YAW_ID-1].value = req.eyeYaw;
		motorCmdSet.motorCmd[HEAD_EYE_ROLL_ID-1].value = req.eyeRoll;
		motorCmdSet.motorCmd[HEAD_EYE_IRIS_ID-1].value = req.eyeIris;
	}

	res.motorCmdSet = motorCmdSet;

	return true;
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
    ros::init(argc, argv, "trs_basicControl_server_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Services
    ros::ServiceServer srv_getAllZeroPositionCmd = nh.advertiseService("get_all_zero_position_cmd", getAllZeroPositionCmd);
    ros::ServiceServer srv_getSlaveZeroPositionCmd = nh.advertiseService("get_slave_zero_position_cmd", getSlaveZeroPositionCmd);
    ros::ServiceServer srv_getSlaveCurrentCmd = nh.advertiseService("get_slave_current_cmd", getSlaveCurrentCmd);
    ros::ServiceServer srv_getSlaveLockCmd = nh.advertiseService("get_slave_lock_cmd", getSlaveLockCmd);
    ros::ServiceServer srv_getHeadPositionCmd = nh.advertiseService("get_head_position_cmd", getHeadPositionCmd);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
