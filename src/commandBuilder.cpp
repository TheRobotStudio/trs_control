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
 *  Created on: Mar 15, 2013
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include "trs_msgs/MotorCmdSet.h"
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT

/*** Variables ***/
trs_msgs::MotorCmdSet motorCmdSet;

/*** Functions ***/
void initCmdSet()
{
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		motorCmdSet.motorCmd[i].nodeID = i+1;
		motorCmdSet.motorCmd[i].mode = NO_MODE;
		motorCmdSet.motorCmd[i].value = 0;
	}
}

/*** Callback functions ***/
void setArmsCommands_cb(const trs_msgs::MotorCmdSetConstPtr& cmds)
{
	#ifdef TRS_DEBUG
	ROS_INFO("Arms command received");
	#endif

	//update motorCmdSet for slave 1 and 2
	for(int i=0; i<2*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		if((i != LEFT_ARM_HAND_ID-1 ) && (i != RIGHT_ARM_HAND_ID-1 ) && (i != LEFT_ARM_ROTATOR_ID-1 ) && (i != RIGHT_ARM_ROTATOR_ID-1 ))
		{
			motorCmdSet.motorCmd[i] = cmds->motorCmd[i];
		}
	}

	//test
	motorCmdSet.motorCmd[26].mode = POSITION_MODE;
	motorCmdSet.motorCmd[26].value = 5000;

	if(motorCmdSet.motorCmd[RIGHT_ARM_BRACHI_ID-1].mode == POSITION_MODE)
	motorCmdSet.motorCmd[RIGHT_ARM_BRACHI_ID-1].value += -16000;

	//motorCmdSet.motorCmd[RIGHT_ARM_INFRA_ID-1].mode = CURRENT_MODE;
	//motorCmdSet.motorCmd[RIGHT_ARM_INFRA_ID-1].value = 150;
	/*
	motorCmdSet.motorCmd[RIGHT_ARM_TMIN_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_ARM_TMIN_ID-1].value = 150;
	motorCmdSet.motorCmd[RIGHT_ARM_LATDELT_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_ARM_LATDELT_ID-1].value = 100;
	motorCmdSet.motorCmd[RIGHT_ARM_POSTDELT_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_ARM_POSTDELT_ID-1].value = 100;
	motorCmdSet.motorCmd[RIGHT_ARM_ANTDELT_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_ARM_ANTDELT_ID-1].value = 100;
	motorCmdSet.motorCmd[RIGHT_ARM_TRICEPS_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_ARM_TRICEPS_ID-1].value = 150;
	motorCmdSet.motorCmd[RIGHT_ARM_SUPRA_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_ARM_SUPRA_ID-1].value = 80;
*/
	//motorCmdSet.motorCmd[].mode = CURRENT_MODE;
	//motorCmdSet.motorCmd[].value = 100;
}

void setHandsCommands_cb(const trs_msgs::MotorCmdSetConstPtr& cmds)
{
	#ifdef TRS_DEBUG
	ROS_INFO("Hands command received");	
	#endif

	//Hand
	motorCmdSet.motorCmd[LEFT_ARM_HAND_ID-1] = cmds->motorCmd[LEFT_ARM_HAND_ID-1];
	motorCmdSet.motorCmd[RIGHT_ARM_HAND_ID-1] = cmds->motorCmd[RIGHT_ARM_HAND_ID-1];
	//Rotator
	motorCmdSet.motorCmd[LEFT_ARM_ROTATOR_ID-1] = cmds->motorCmd[LEFT_ARM_ROTATOR_ID-1];
	motorCmdSet.motorCmd[RIGHT_ARM_ROTATOR_ID-1] = cmds->motorCmd[RIGHT_ARM_ROTATOR_ID-1];
}

void setHeadCommands_cb(const trs_msgs::MotorCmdSetConstPtr& cmds)
{
	#ifdef TRS_DEBUG
	ROS_INFO("Head command received");
	#endif

	//update motorCmdSet for slave 3
	for(int i=2*NUMBER_MAX_EPOS2_PER_SLAVE; i<3*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motorCmdSet.motorCmd[i] = cmds->motorCmd[i];
	}
}

void setLegsCommands_cb(const trs_msgs::MotorCmdSetConstPtr& cmds)
{
	#ifdef TRS_DEBUG
	ROS_INFO("Legs command received");
	#endif

	//update motorCmdSet for slave 4 and 5
	for(int i=3*NUMBER_MAX_EPOS2_PER_SLAVE; i<5*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motorCmdSet.motorCmd[i] = cmds->motorCmd[i];
	}

	//for test
	/*
	motorCmdSet.motorCmd[61].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[61].value = 100;

	motorCmdSet.motorCmd[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].value = 100;
	*/
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
    ros::init(argc, argv, "trs_commandBuilder_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Subscribers
	ros::Subscriber sub_setArmsCommands = nh.subscribe("/setArmsCommands", 1, setArmsCommands_cb);
	ros::Subscriber sub_setHeadCommands = nh.subscribe("/setHeadCommands", 1, setHeadCommands_cb);
	ros::Subscriber sub_setHandsCommands = nh.subscribe("/setHandsCommands", 1, setHandsCommands_cb);
	ros::Subscriber sub_setLegsCommands = nh.subscribe("/setLegsCommands", 1, setLegsCommands_cb);

	//Publishers
	ros::Publisher pub_sendCmdSet = nh.advertise<trs_msgs::MotorCmdSet>("/sendCmdSet", 1);

	while(ros::ok())
	{
		initCmdSet(); //reset cmd set
		ros::spinOnce(); //grab msg and update cmd
		pub_sendCmdSet.publish(motorCmdSet); //publish it

		r.sleep();
	}

	return 0;
}



