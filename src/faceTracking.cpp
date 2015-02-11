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
 *  Created on: Mar 26, 2013
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include "trs_msgs/MotorCmdSet.h"
#include "trs_msgs/MotorDataSet.h"
#include <std_msgs/Int16MultiArray.h>
#include "trs_control/switchNode.h"
#include "trs_control/getMotorCmdSet.h"
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE						15
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
int facePosCenterRelative[2];
bool headHasMoved = false;

//previous values of the eyeball
int previousXPitch = 0;
int previousYYaw = 0;
int previousIris = 0;

int neckWaveGoal[8] = {0};
int neckWaveCounter = 0;

int currPitch = 120;
int currYaw = 120;

int noHeadDetectedCounter = 0;

bool manualMode = true;

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

void turnHeadLeft(int speed)
{
	currYaw += speed;
	setHeadYaw(currYaw);
}

void turnHeadRight(int speed)
{
	currYaw -= speed;
	setHeadYaw(currYaw);
}

void turnHeadUp(int speed)
{
	currPitch -= speed;
	setHeadPitch(currPitch);
}

void turnHeadDown(int speed)
{
	currPitch += speed;
	setHeadPitch(currPitch);
}

void computeHeadPosition()
{
	/* this is hard coded, may cause errors if objects does not exist */
	//boolean for the head
	bool turnRight = false;
	bool turnLeft = false;
	bool goUp = false;
	bool goDown = false;

	//move the iris randomly
	int generator = rand() % 10;
	int irisRandPos = generator - 5;
	int iris_pos = previousIris + irisRandPos;

	if(iris_pos < 65) iris_pos = 65;
	if(iris_pos > 175) iris_pos = 175;

	//update iris position
	motorCmdSet.motorCmd[HEAD_EYE_IRIS_ID-1].value = iris_pos;

	int x = facePosCenterRelative[0];
	int y = facePosCenterRelative[1];

	//ROS_INFO("facePosCenterRelative[%d, %d]", x, y);

	int pitch_pos;
	int yaw_pos;

	if(x > 0)
	{
		yaw_pos = previousYYaw - (x*2)/10;

		if(yaw_pos < 70)
		{
			//neckDistX = yaw_pos - 70; //inverted sign, negative sum is look left
			yaw_pos = 70;
			turnLeft = true;
			turnHeadRight(3);
		}
	}
	else
	{
		yaw_pos = previousYYaw - (x*2)/10;

		if(yaw_pos > 170)
		{
			//neckDistX = yaw_pos - 170;
			yaw_pos = 170;
			turnRight = true;
			turnHeadLeft(3);
		}
	}

	if(y > 0)
	{
		pitch_pos = previousXPitch + (y*2)/10;

		if(pitch_pos > 170)
		{
			//neckDistY = 170 - pitch_pos; //inverted sign, negative sum is look down
			pitch_pos = 170;
			goDown = true;
			turnHeadDown(3);
		}
	}
	else
	{
		pitch_pos = previousXPitch + (y*2)/10;

		if(pitch_pos < 70)
		{
			//neckDistY = 70 - pitch_pos;
			pitch_pos = 70;
			goUp = true;
			turnHeadUp(3);
		}
	}

	//call the eyeball commands
	//pitch
	motorCmdSet.motorCmd[HEAD_EYE_PITCH_ID-1].value = pitch_pos;
	//yaw
	motorCmdSet.motorCmd[HEAD_EYE_YAW_ID-1].value = yaw_pos;

	previousXPitch = pitch_pos;
	previousYYaw = yaw_pos;
	previousIris = iris_pos;

	//ROS_INFO("Head = EyeBall(pitch %d, yaw %d, roll %d, iris %d) - Neck(LB %d, RB %d, LM %d, RM %d)", motorCmdSet.motorCmd[HEAD_EYE_PITCH_ID-1].value, motorCmdSet.motorCmd[YAW_ID-1].value, motorCmdSet.motorCmd[ROLL_ID-1].value, motorCmdSet.motorCmd[HEAD_EYE_IRIS_ID-1].value, motorCmdSet.motorCmd[HEAD_NECK7_ID-1].value, motorCmdSet.motorCmd[HEAD_NECK8_ID-1].value, motorCmdSet.motorCmd[HEAD_NECK9_ID-1].value, motorCmdSet.motorCmd[HEAD_NECK10_ID-1].value);
}

/*** Callback functions ***/
void headCoords_cb(const std_msgs::Int16MultiArrayConstPtr& headCoords)
{
	if(switch_node)
	{
		if((headCoords->data[0] != -1) && (headCoords->data[1] != -1)) //if head detected
		{
			//prepare motorCmdSet array with mode
			//for(int i=2*NUMBER_MAX_EPOS2_PER_SLAVE; i<3*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
			for(int i=FIRST_SERVO_ID-1; i<3*NUMBER_MAX_EPOS2_PER_SLAVE; i++)
			{
				motorCmdSet.motorCmd[i].mode = SERVO_POS_MODE;
			}

			//update head coords
			facePosCenterRelative[0] = 320-headCoords->data[0]; //230-headCoords->data[0];
			facePosCenterRelative[1] = headCoords->data[1]-240;

			//call function that will compute new servo values for the head
			computeHeadPosition();

			noHeadDetectedCounter = 0;
		}
		else
		{
			noHeadDetectedCounter ++;
		}
	}
}

/*
void tabletCmd_cb(const std_msgs::Int16MultiArrayConstPtr& tablet_cmd)
{
	if(tablet_cmd->data[0] == 1) manualMode = false;
	else
	{
		int eyeball_pitch = 0;
		int eyeball_yaw = 0;
		int eyeball_roll = 0;
		int eyeball_iris = 0;

		int head_pitch = 0;
		int head_yaw = 0;

		manualMode = true;

		//prepare motorCmdSet array with node id an)d mode
		for(int i=FIRST_SERVO_ID-1; i<NUMBER_OF_MOTORS; i++)
		{
			motorCmdSet.motorCmd[i].mode = SERVO_POS_MODE;
		}

		eyeball_pitch = tablet_cmd->data[3] + EYEBALL_CMD_OFFSET;
		eyeball_yaw = tablet_cmd->data[4] + EYEBALL_CMD_OFFSET;
		eyeball_roll = tablet_cmd->data[1] + EYEBALL_CMD_OFFSET;
		eyeball_iris = tablet_cmd->data[2] + EYEBALL_CMD_OFFSET;

		head_pitch = (tablet_cmd->data[6]/3) + HEAD_PITCH_OFFSET;
		head_yaw = (tablet_cmd->data[5]/3) + HEAD_YAW_OFFSET;

		motorCmdSet.motorCmd[HEAD_EYE_PITCH_ID-1].value = eyeball_pitch;
		motorCmdSet.motorCmd[HEAD_EYE_YAW_ID-1].value = eyeball_yaw;
		motorCmdSet.motorCmd[HEAD_EYE_ROLL_ID-1].value = eyeball_roll;
		motorCmdSet.motorCmd[HEAD_EYE_IRIS_ID-1].value = eyeball_iris;

		setHeadPitch(head_pitch);
		setHeadYaw(head_yaw);
	}
}
*/

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
    ros::init(argc, argv, "trs_faceTracking_server_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Subscribers
    ros::Subscriber sub_headCoords = nh.subscribe("/headCoords", 1, headCoords_cb);

    //Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_face_tracking", switchNode);
	ros::ServiceServer srv_getMotorCmdSet = nh.advertiseService("get_face_tracking_cmd", getMotorCmdSet);

    facePosCenterRelative[0] = 0;
    facePosCenterRelative[1] = 0;

    initCmdSet();

    while(ros::ok())
	{
    	ros::spinOnce();

    	if(switch_node)
		{
    		//re-center head after 2 seconds without face detected
			if(noHeadDetectedCounter > 30)
			{
				//recenterHead();
				setHeadPitch(110);
				setHeadYaw(120);
				noHeadDetectedCounter = 0;
			}
		}

    	r.sleep();
	}
}
