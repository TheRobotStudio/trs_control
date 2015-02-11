/*
 * Copyright (c) 2012, The Robot Studio
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
 *  Created on: Oct 19, 2012
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/recorder.h>
#include <trs_msgs/MotorDataSet.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>

/*** Variables ***/
trs_msgs::MotorDataSet motorDataSet;
sensor_msgs::JointState anglesArmsDescription;

//to wait for the msg from both posture and anglesArmDescription topics
bool motorDataSet_arrived = false;
bool anglesArmsDescription_arrived = false;

/*** Callback functions ***/
void motorDataSet_cb(const trs_msgs::MotorDataSetConstPtr& data)
{
	motorDataSet = *data;
	motorDataSet_arrived = true;
}

void anglesArmsDescription_cb(const sensor_msgs::JointStateConstPtr& js)
{
	anglesArmsDescription = *js;
	anglesArmsDescription_arrived = true;
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "trs_bagRecorderArmsAndAngles");
	ros::NodeHandle nh;
	bool run = true;

	rosbag::Bag bag;
	bag.open("/home/dcx/catkin_ws/src/trs_control/bag/anglesToPosture_1.bag", rosbag::bagmode::Write);

	//Subscribers
	ros::Subscriber sub_motorDataSet = nh.subscribe ("/motorDataSet", 10, motorDataSet_cb);
	ros::Subscriber sub_anglesArmsDescription = nh.subscribe ("/anglesArmsDescription", 10, anglesArmsDescription_cb);

	char inputKey = '0';
	int loopNb = 0;

	while(run)
	{
		//wait for user keyboard input
		printf("Enter 'r' to record a posture, or 'q' to exit :\n");
		inputKey = getchar();

		if(inputKey == 'r')
		{
			while(!motorDataSet_arrived || !anglesArmsDescription_arrived)
			{
				ros::spinOnce(); //listen to topics
			}
			//get time
			ros::Time time = ros::Time::now();
			//write data to the bag
			bag.write("/motorDataSet", time, motorDataSet);
			bag.write("/anglesArmsDescription", time, anglesArmsDescription);

			//but this back to false for the next record
			motorDataSet_arrived = false;
			anglesArmsDescription_arrived = false;

			printf("Posture number %d recorded OK !\n", loopNb);
		}
		else if(inputKey == 'q')
		{
			bag.close(); //close the bag
			run = false; //stop the while loop, quit the program
			printf("Bag closed ok\n");
		}
		getchar(); //to grab carriage return

		loopNb++;
	}

	return 0;
}
