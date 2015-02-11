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
 *  Created on: Jan 28, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include "trs_msgs/MotorCmdSet.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <trs_msgs/MotorDataSet.h>
#include "razor_imu_9dof/RazorImu.h"
#include <math.h>
#include "trs_control/getMotorCmdSet.h"
#include "trs_control/switchNode.h"
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT
#define DCX_LEGS_CURRENT		100
#define GAST_CURRENT_FACTOR		20

#define THRESHOLD_Y				-0.1
#define THRESHOLD_TOTAL_GRF		300
#define NOMINAL_PITCH			0.100
#define INNER_PITCH_CIRCLE		0.005

/*** Variables ***/
bool switch_node = false; //disable by default

trs_msgs::MotorCmdSet motorCmdSet;
std_msgs::Int32MultiArray calAdcVal;

std_msgs::Int32MultiArray avgAdcCalib;
std_msgs::Int32MultiArray offsetAdc;
std_msgs::Int32MultiArray current_array;

std_msgs::Float32MultiArray pitch_array;

trs_msgs::MotorDataSet motorData;
trs_msgs::MotorDataSet initialMotorData;
trs_msgs::MotorDataSet prevMotorData;

bool calibDone = false;
int counterCalib = 0;

float pitch = 0;
float nominalPitch = 0;
float prevPitch = 0;
float diffPitch = 0;

bool recoveringFallBackward = false;
bool recoveredFallBackward = false;
bool recoveringFallForward = false;
bool recoveredFallForward = false;
bool postureArrived = false;

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

void currentPulseLeftCALF(int currPulse)
{
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_INNER_GAST_ID-1].mode = CURRENT_MODE;

	//compare greatest one?

	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].current + currPulse;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].current + currPulse;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].current + currPulse;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_INNER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_LATERAL_INNER_GAST_ID-1].current + currPulse;
}

void currentPulseRightCALF(int currPulse)
{
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].mode = CURRENT_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].mode = CURRENT_MODE;

	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].current + currPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].current + currPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].current + currPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].current + currPulse;
}

void positionPulseLeftCALF(int posPulse)
{
	motorCmdSet.motorCmd[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_INNER_GAST_ID-1].mode = POSITION_MODE;

	float val_f = posPulse*0.8;
	int val_i = (int)val_f;

	motorCmdSet.motorCmd[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].value = motorData.motorData[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].encPosition - val_i;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_INNER_GAST_ID-1].value = motorData.motorData[LEFT_LEG_LATERAL_INNER_GAST_ID-1].encPosition + posPulse;

}

void positionPulseRightCALF(int posPulse)
{
	motorCmdSet.motorCmd[RIGHT_LEG_TIBALIS_ANTERIOR_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].mode = POSITION_MODE;

	float val_f = posPulse*0.8;
	int val_i = (int)val_f;

	motorCmdSet.motorCmd[RIGHT_LEG_TIBALIS_ANTERIOR_ID-1].value = motorData.motorData[RIGHT_LEG_TIBALIS_ANTERIOR_ID-1].encPosition - val_i;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].value = motorData.motorData[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].encPosition + posPulse;
}

void prevPosPulseLeftCALF(int posPulse)
{
	motorCmdSet.motorCmd[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_INNER_GAST_ID-1].mode = POSITION_MODE;

	float val_f = posPulse*0.8;
	int val_i = (int)val_f;

	motorCmdSet.motorCmd[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].value = prevMotorData.motorData[LEFT_LEG_TIBALIS_ANTERIOR_ID-1].encPosition - val_i;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].value = prevMotorData.motorData[LEFT_LEG_LATERAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].value = prevMotorData.motorData[LEFT_LEG_MEDIAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].value = prevMotorData.motorData[LEFT_LEG_MEDIAL_INNER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[LEFT_LEG_LATERAL_INNER_GAST_ID-1].value = prevMotorData.motorData[LEFT_LEG_LATERAL_INNER_GAST_ID-1].encPosition + posPulse;
}

void prevPosPulseRightCALF(int posPulse)
{
	motorCmdSet.motorCmd[RIGHT_LEG_TIBALIS_ANTERIOR_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].mode = POSITION_MODE;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].mode = POSITION_MODE;

	float val_f = posPulse*0.8;
	int val_i = (int)val_f;

	motorCmdSet.motorCmd[RIGHT_LEG_TIBALIS_ANTERIOR_ID-1].value = prevMotorData.motorData[RIGHT_LEG_TIBALIS_ANTERIOR_ID-1].encPosition - val_i;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].value = prevMotorData.motorData[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].value = prevMotorData.motorData[RIGHT_LEG_MEDIAL_OUTER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].value = prevMotorData.motorData[RIGHT_LEG_MEDIAL_INNER_GAST_ID-1].encPosition + posPulse;
	motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].value = prevMotorData.motorData[RIGHT_LEG_LATERAL_INNER_GAST_ID-1].encPosition + posPulse;
}

/*** Callback functions ***/
void adc_cb(const std_msgs::Int32MultiArrayPtr& adc)
{
	//ROS_INFO("%d \t %d \t %d \t %d \t %d \t %d\n", adc->data[0]-offsetAdc.data[0], adc->data[1]-offsetAdc.data[1], adc->data[2]-offsetAdc.data[2], adc->data[3]-offsetAdc.data[3], adc->data[4]-offsetAdc.data[4], adc->data[5]-offsetAdc.data[5]);

	if(!calibDone)
	{
		//prevMotorData = motorData;
		//initialMotorData = motorData;

		for(int i=0; i<6; i++)
		{
			if(counterCalib>=10) avgAdcCalib.data[i] += adc->data[i];
		}
		counterCalib++;
		if(counterCalib == 20)
		{
			for(int i=0; i<6; i++)
			{
				offsetAdc.data[i] = avgAdcCalib.data[i]/10;
			}
			calibDone = true;
			prevMotorData = motorData;
			initialMotorData = motorData;
			ROS_INFO("CALIB DONE : %d \t %d \t %d \t %d \t %d \t %d\n", adc->data[0]-offsetAdc.data[0], adc->data[1]-offsetAdc.data[1], adc->data[2]-offsetAdc.data[2], adc->data[3]-offsetAdc.data[3], adc->data[4]-offsetAdc.data[4], adc->data[5]-offsetAdc.data[5]);
			//ROS_INFO("CALIB motorData RIGHT_LEG_LATERAL_OUTER_GAST = %d", motorData.motorData[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].encPosition);
			//ROS_INFO("CALIB initialMotorData RIGHT_LEG_LATERAL_OUTER_GAST = %d", initialMotorData.motorData[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].encPosition);
		}
	}
	else
	{
		for(int i=0; i<6; i++)
		{
			calAdcVal.data[i] = adc->data[i] - offsetAdc.data[i];
		}
	}
}

void motorDataSet_cb(const trs_msgs::MotorDataSetConstPtr& data)
{
	motorData = *data;
	postureArrived = true;
}

void imuRaw_cb(const razor_imu_9dof::RazorImuConstPtr& imuData)
{
	pitch = imuData->pitch - nominalPitch;
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
    ros::init(argc, argv, "trs_balanceLegs_server_node");
    ros::NodeHandle nh;
    ros::Rate r(LOOP_RATE);

    //Subscribers
	ros::Subscriber sub_adc = nh.subscribe("/adc", 1, adc_cb);
	ros::Subscriber sub_motorDataSet = nh.subscribe ("/motorDataSet", 10, motorDataSet_cb);
	ros::Subscriber sub_imuRaw = nh.subscribe ("/imuRaw", 10, imuRaw_cb);

	//Publishers
	ros::Publisher markerFeet_pub = nh.advertise<visualization_msgs::Marker>("/feetMarker", 1);
	ros::Publisher pitch_pub = nh.advertise<std_msgs::Float32MultiArray>("/pitchArray", 1);
	ros::Publisher current_pub = nh.advertise<std_msgs::Int32MultiArray>("/currentArray", 1);

	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_balance_legs", switchNode);
	ros::ServiceServer srv_getMotorCmdSet = nh.advertiseService("get_balance_legs_cmd", getMotorCmdSet);

	pitch_array.data.clear();
	pitch_array.data.push_back(0);
	pitch_array.data.push_back(0);

	current_array.data.clear();
	current_array.data.push_back(0);
	current_array.data.push_back(0);
	current_array.data.push_back(0);
	current_array.data.push_back(0);

	avgAdcCalib.data.clear();
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);

	offsetAdc.data.clear();
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);

	calAdcVal.data.clear();
	calAdcVal.data.push_back(0);
	calAdcVal.data.push_back(0);
	calAdcVal.data.push_back(0);
	calAdcVal.data.push_back(0);
	calAdcVal.data.push_back(0);
	calAdcVal.data.push_back(0);

	visualization_msgs::Marker feet_marker;
	feet_marker.header.frame_id = "/origin";
	feet_marker.ns = "marker_feet";
	feet_marker.id = 7;
	feet_marker.type = visualization_msgs::Marker::SPHERE;
	feet_marker.action = visualization_msgs::Marker::ADD;
	feet_marker.pose.orientation.x = 0.0;
	feet_marker.pose.orientation.y = 0.0;
	feet_marker.pose.orientation.z = 0.0;
	feet_marker.pose.orientation.w = 1.0;
	feet_marker.pose.position.x = 0.1;
	feet_marker.pose.position.y = 0.1;
	feet_marker.scale.x = 0.05;
	feet_marker.scale.y = 0.05;
	feet_marker.scale.z = 0.05;
	feet_marker.color.r = 1.0;
	feet_marker.color.g = 0.0;
	feet_marker.color.b = 0.0;
	feet_marker.color.a = 1.0;

	int forwardThresh_cnt = 0;
	int backwardThresh_cnt = 0;
	int leftThresh_cnt = 0;
	int rightThresh_cnt = 0;

	int currPulseLeftCalfDone = 0;

	int startup = 1;

	float rawPitchSum = 0;

	int state = 0;
	int state_delay = 0;
	int stateCtr2 = 0;
	int stateLatch2 = 0;
	int stateCtr3 = 0;
	int stateLatch3 = 0;

	float COP_y_20Gauss[20];
	int COP_y_20GaussCtr = 0;
	float COP_y_20GaussArea = 0;
	float COP_y_20GaussAreaSum[10];
	float COP_y_20GaussAreaSum10 = 0;
	int COP_y_20GaussAreaSum10Ctr = 0;

	while(ros::ok())
	{
		if(switch_node)
		{
			initCmdSet(); //reset motorCmdSet
			ros::spinOnce(); //grab msg and update motorCmdSet

			diffPitch = pitch - prevPitch;

			pitch_array.data[0] = pitch;
			pitch_array.data[1] = diffPitch;

			pitch_pub.publish(pitch_array);

			//sum of all feet sensors
			int sum = calAdcVal.data[0]+calAdcVal.data[1]+calAdcVal.data[2]+calAdcVal.data[3]+calAdcVal.data[4]+calAdcVal.data[5];
			int sumToes = calAdcVal.data[1]+calAdcVal.data[2]+calAdcVal.data[4]+calAdcVal.data[5];
			int sumHeels = calAdcVal.data[0]+calAdcVal.data[3];


			if(calibDone && (sum > THRESHOLD_TOTAL_GRF))// && (sumToes > 100) && (sumHeels > 100))
			{
				//publish COP dot
				//feet marker, center of gravity
				feet_marker.pose.position.x = ((calAdcVal.data[0]+calAdcVal.data[1]+calAdcVal.data[2])-(calAdcVal.data[3]+calAdcVal.data[4]+calAdcVal.data[5]));///(calAdcVal.data[0]+calAdcVal.data[1]+calAdcVal.data[2]+calAdcVal.data[3]+calAdcVal.data[4]+calAdcVal.data[5]);
				feet_marker.pose.position.y = (-(calAdcVal.data[0]+calAdcVal.data[3])+(calAdcVal.data[1]+calAdcVal.data[2]+calAdcVal.data[4]+calAdcVal.data[5]));///(calAdcVal.data[0]+calAdcVal.data[1]+calAdcVal.data[2]+calAdcVal.data[3]+calAdcVal.data[4]+calAdcVal.data[5]);

				feet_marker.pose.position.x /= -sum;
				feet_marker.pose.position.y /= sum;

				double scale_f = sum;
				scale_f /= 10000;

				//ROS_INFO("scale_f = %f", scale_f);

				feet_marker.scale.x = scale_f;
				feet_marker.scale.y = scale_f;
				feet_marker.scale.z = scale_f;

				markerFeet_pub.publish(feet_marker);

				//ROS_INFO("Y = %f", feet_marker.pose.position.y);
				double COP_y = feet_marker.pose.position.y;
				double COP_x = feet_marker.pose.position.x;

				//Rolling filters
				//generate 20 frame moving window filter of movements in COP y position

				COP_y_20Gauss[COP_y_20GaussCtr] = COP_y;		//load latest value

				COP_y_20GaussCtr ++;
				if(COP_y_20GaussCtr > 20) COP_y_20GaussCtr = 0;	//and increment to oldest value

				COP_y_20GaussArea = 0;

				int i;
				for(i=0; i<=20; i++)
				{
					COP_y_20GaussArea += COP_y_20Gauss[i] - COP_y_20Gauss[COP_y_20GaussCtr];
				}
				//ROS_INFO("Pitch = %f, DiffPitch = %f,  COPGauss = %f", pitch, diffPitch, COP_y_20GaussArea);

				//Sum over 10 frames for threshold

				COP_y_20GaussAreaSum10Ctr++;
				if(COP_y_20GaussAreaSum10Ctr >= 10) COP_y_20GaussAreaSum10Ctr = 0;	//and increment to oldest value

				COP_y_20GaussAreaSum[COP_y_20GaussAreaSum10Ctr] = COP_y_20GaussArea;

				COP_y_20GaussAreaSum10 = COP_y_20GaussAreaSum[0] + COP_y_20GaussAreaSum[1] + COP_y_20GaussAreaSum[2] + COP_y_20GaussAreaSum[3] + COP_y_20GaussAreaSum[4]
									   + COP_y_20GaussAreaSum[5] + COP_y_20GaussAreaSum[6] + COP_y_20GaussAreaSum[7] + COP_y_20GaussAreaSum[8] + COP_y_20GaussAreaSum[9];


				//need the robot to be stable for a while after calibration
				if(startup > 0)
				{
					//deadband to COP
					if((COP_y <= -0.1) && (COP_y >= -0.5) && (COP_x <= 0.2) && (COP_x >= -0.2))
					{
						ROS_INFO("%d Startup Balancing", startup);

						startup++;

						if(startup >= 200)
						{
							rawPitchSum += pitch;

						}

						if(startup == 250)
						{
							startup = 0;

							nominalPitch = rawPitchSum/50;

							ROS_INFO("*********************************************************");
							ROS_INFO("%d Startup Finished Pitch = %f Nominal Pitch = %f", startup, pitch, nominalPitch);

							int i;
							for(i=0; i<=20; i++)
							{
							ROS_INFO("%f", COP_y_20Gauss[i]);
							state = 1;
							}
						}
					}
					else
					{
						ROS_INFO("%d Startup Not Balanced", startup);

						if(startup > 1)
							{
								startup = 1;
								nominalPitch = 0;
								//COP_y_20gauss = 0;
							}
					}
				}

				//freshly balanced so detect start of fall and correct
				if(state == 1)
				{
					ROS_INFO("%d, Pitch = %f, COPGauss20 = %f, COPGaussSum = %f", state, pitch, COP_y_20GaussArea, COP_y_20GaussAreaSum10);

					//Detect forward fall
					if(COP_y_20GaussAreaSum10 > 7 && COP_y > -0.2 || COP_y > -0.1)
					{
						(state = 2);
					}

					//Detect backward fall
					if(COP_y_20GaussAreaSum10 < -7 && COP_y < -0.4 || COP_y < -0.5)
					{
						(state = 3);
					}

				}
				//fall forward limit
				if(state == 2)
				{
					ROS_INFO("% d, Pitch = %f, COPGauss20 = %f, COPGaussSum = %f", state, pitch, COP_y_20GaussArea, COP_y_20GaussAreaSum10);

					if(stateCtr2 == 0)
					{
						//Twitch the calves
						int incPos_COPGauss20 = 15 + COP_y_20GaussArea * 35;

						positionPulseLeftCALF(incPos_COPGauss20);
						positionPulseRightCALF(incPos_COPGauss20);

						stateLatch2 = 10 + (int)abs(COP_y_20GaussArea * 15);
						if(stateLatch2 > 50) stateLatch2 = 50;

						ROS_INFO("%d", stateLatch2);

						stateCtr2 = 1;
					}
					else if(stateCtr2 > 0)		//Wait for reaction, todo: actively
					{
						stateCtr2++;

						if(stateCtr2 == stateLatch2)
						{
							state = 1;
							stateCtr2 = 0; //Exit State2
						}
					}

				}
				//fall backward limit
				if(state == 3)
				{
					ROS_INFO("%d, Pitch = %f, COPGauss20 = %f, COPGaussSum = %f", state, pitch, COP_y_20GaussArea, COP_y_20GaussAreaSum10);

						if(stateCtr3 == 0)
						{
							//Twitch the calves
							int incPos_COPGauss20 = -15 + COP_y_20GaussArea * 35;

							positionPulseLeftCALF(incPos_COPGauss20);
							positionPulseRightCALF(incPos_COPGauss20);

							stateLatch3 = 10 + (int)abs(COP_y_20GaussArea * 15);

							if(stateLatch3 > 50) stateLatch3 = 50;

							ROS_INFO("%d", stateLatch3);

							stateCtr3 = 1;
						}
						else if(stateCtr3 > 0)		//Wait for reaction, todo: actively
						{
							stateCtr3++;

							if(stateCtr3 == stateLatch3)
							{
								state = 1;
								stateCtr3 = 0; //Exit State3
							}
						}
				}

				if(state == 4000)
				{
					//below sensor noise
					//but can use to tune via COP
					if((pitch <= INNER_PITCH_CIRCLE) && (pitch >= -INNER_PITCH_CIRCLE))
					{
						//TODO generate shifting best estimate of balanced position

						// can come here direct
						if (recoveringFallForward) recoveringFallForward = false;
						if (recoveringFallBackward) recoveringFallBackward = false;

						int incPos_balance = diffPitch * 25000;

						ROS_INFO("Pitch = %f, DiffPitch = %f, Balanced, incPos = %d", pitch, diffPitch, incPos_balance);
					}

					//resets to initial posture
					else if(recoveredFallForward)
					{
						ROS_INFO("Pitch = %f, DiffPitch = %f, Position restored", pitch, diffPitch);
						recoveredFallForward = false;
						prevMotorData = initialMotorData;
						prevPosPulseLeftCALF(0);
						prevPosPulseRightCALF(0);
					}

					//forward case
					else if((pitch > INNER_PITCH_CIRCLE) && (diffPitch > 0))
					{
						//float incPos_f = diffPitch*fabs(sin(pitch))*1500000;
						//int incPos_i = (int)incPos_f + 100;

						int incPos_i = diffPitch*50000;

						positionPulseLeftCALF(incPos_i);
						positionPulseRightCALF(incPos_i);

						state_delay = diffPitch*400;
						if(state_delay > 15) state_delay = 15;


						ROS_INFO("Pitch = %f, DiffPitch = %f, Falling forward : incPos = %d state_delay = %d", pitch, diffPitch, incPos_i, state_delay);
					}
					//sign change of diffPitch approx velocity
					else if((pitch > INNER_PITCH_CIRCLE) && (diffPitch < 0))
					{
						ROS_INFO("Pitch = %f, DiffPitch = %f, Recovering fall forward", pitch, diffPitch);
						recoveringFallForward = true;
					}
					//back in inner circle
					else if((pitch < INNER_PITCH_CIRCLE) && (diffPitch < 0) && recoveringFallForward)
					{
						ROS_INFO("Pitch = %f, DiffPitch = %f, Recovered fall forward", pitch, diffPitch);
						//recoveredFall = true;
					}
					//backward case
					else if((pitch < -INNER_PITCH_CIRCLE) && (diffPitch < 0))
						//float incPos_f = diffPitch*fabs(sin(pitch))*1500000;
					{
						//int incPos_i = (int)incPos_f;

						int incPos_i = diffPitch*100000;

						positionPulseLeftCALF(incPos_i);
						positionPulseRightCALF(incPos_i);

						state_delay = -diffPitch*400;
						if(state_delay > 15) state_delay = 15;

						ROS_INFO("Pitch = %f, DiffPitch = %f, Falling backward : incPos = %d state_delay = %d", pitch, diffPitch, incPos_i, state_delay);
					}
					// sign change
					else if((pitch < -INNER_PITCH_CIRCLE) && (diffPitch > 0))
					{
						ROS_INFO("Pitch = %f, DiffPitch = %f, Recovering fall backward", pitch, diffPitch);
						recoveringFallBackward = true;
					}
					else if((pitch > -INNER_PITCH_CIRCLE) && (diffPitch > 0) && recoveringFallBackward)
					{
						ROS_INFO("Pitch = %f, DiffPitch = %f, Recovered fall backward", pitch, diffPitch);
						//recoveredFall = true;
					}
					else
					{
						ROS_INFO("Pitch = %f, DiffPitch = %f, Unclassified", pitch, diffPitch, diffPitch);
					}
				}//else if(state == 4000)
			}//the big if standing on the ground loop
			else
			{
				//return to start posture
				if(calibDone)
				{
					prevMotorData = initialMotorData;

					prevPosPulseLeftCALF(0);
					prevPosPulseRightCALF(0);
				}

				//ROS_INFO("motorData RIGHT_LEG_LATERAL_OUTER_GAST = %d", motorData.motorData[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].encPosition);
			}

			//ROS_INFO("RIGHT_LEG_LATERAL_OUTER_GAST = %d", motorCmdSet.motorCmd[RIGHT_LEG_LATERAL_OUTER_GAST_ID-1].value);

			//ROS_INFO("Commands");

			prevPitch = pitch;

			postureArrived = false;

		}//end if(switch_node)

		r.sleep();
	}//while(ros::ok())
}

