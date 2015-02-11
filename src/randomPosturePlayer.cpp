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
 *  Created on: Sep 11, 2013
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include "trs_control/getMotorCmdSet.h"
#include "trs_control/switchNode.h"
#include <stdio.h>
#include <cstdlib>
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE	1 //0.5
#define ALPHA_HI	2.8
#define ALPHA_LO	0
#define BETA_HI		1.5
#define BETA_LO		0.2
#define THETA_HI	0
#define THETA_LO	-1.5
#define PHI_HI		1.2
#define PHI_LO		0

/*** Variables ***/
bool switch_node = false;  //disable by default
trs_msgs::MotorCmdSet motorCmdSet;

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
int main (int argc, char** argv)
{
	//Initialize ROS
	ros::init (argc, argv, "trs_randomPosturePlayer_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);
	
	//Services
    ros::ServiceServer srv_switchNode = nh.advertiseService("switch_random_posture_player", switchNode);
    ros::ServiceServer srv_getMotorCmdSet = nh.advertiseService("get_random_posture_player_cmd", getMotorCmdSet);

	while(ros::ok())
	{
		if(switch_node) ros::spinOnce();
		r.sleep();
	}

	return 0;
}
