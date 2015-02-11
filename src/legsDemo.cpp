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
 *  Created on: Jan 24, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/recorder.h>
#include <trs_msgs/MotorDataSet.h>
#include "trs_msgs/MotorCmdSet.h"
#include <stdio.h>
#include <boost/foreach.hpp>
#include <pthread.h>
#include "robotDefines.h"

/*** Defines ***/
//#define LOOP_RATE						40
//#define ARMS_IDS_RANGE				30
//#define NUMBER_OF_SLAVES				4
//#define MAX_NUMBER_OF_MOTORS_PER_SLAVE	15
//#define MAX_NUMBER_OF_MOTORS			MAX_NUMBER_OF_MOTORS_PER_SLAVE*NUMBER_OF_SLAVES
//#define CURRENT_MODE					1
//#define POSITION_MODE					0
#define DCX_LEGS_CURRENT				100

/*** Variables ***/
ros::Publisher legs_cmd_pub;
ros::Publisher arms_cmd_pub;
trs_msgs::MotorCmdSet cmd;
trs_msgs::MotorDataSet posture;
trs_msgs::MotorDataSet postureArray[100];


//to wait for the msg from both posture and joint_states_kinect topics
bool postureArrived = false;

/*
//thread for getting ADC values
void* getADCVal(void* name)
{
    return NULL;
}
*/

/*** Functions ***/
void initCmdSet()
{
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		cmd.motorCmd[i].nodeID = i+1;
		cmd.motorCmd[i].mode = CURRENT_MODE;
		cmd.motorCmd[i].value = DCX_LEGS_CURRENT;
	}
}

void setCurrentCmd(int curr)
{
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		cmd.motorCmd[i].nodeID = i+1;
		cmd.motorCmd[i].mode = CURRENT_MODE;
		cmd.motorCmd[i].value = curr;
	}
}

void setPositionCmd(trs_msgs::MotorDataSet post)
{
	for(int i=45; i<NUMBER_OF_MOTORS; i++)
	{
		cmd.motorCmd[i].nodeID = i+1;
		cmd.motorCmd[i].mode = POSITION_MODE;
		cmd.motorCmd[i].value = post.motorData[i].encPosition;
	}
}

/*** Callback functions ***/
void posture_cb(const trs_msgs::MotorDataSetConstPtr& data)
{
	posture = *data;
	postureArrived = true;
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "trs_legsDemo");
	ros::NodeHandle nh;

	ros::Subscriber postureSub = nh.subscribe ("/motorDataSet", 10, posture_cb);
	legs_cmd_pub = nh.advertise<trs_msgs::MotorCmdSet>("/setLegsCommands", 100, true);
	arms_cmd_pub = nh.advertise<trs_msgs::MotorCmdSet>("/setArmsCommands", 100, true);

/*	//thread
	pthread_t filsA;

	if (pthread_create(&filsA, NULL, getADCVal, "AA"))
	{
		perror("pthread_create");
		exit(EXIT_FAILURE);
	}
*/
	char inputKey = '0';
	postureArrived = false;

	bool run = true;
	int nbPostToPlay = 0;

	while(run)
	{
		//init cmd
		initCmdSet();

		printf("\n***** LEGS DEMO *****\n");
		//wait for user keyboard input
		printf("Enter 'c' to set legs to current mode\n");
		printf("Enter 'r' to record new postures\n");
		printf("Enter 'o' to open a bag\n");
		printf("Enter 's' to save recorded postures in a bag\n");
		printf("Enter 'p' to play postures\n");
		//printf("Enter 'b' to balance\n");
		printf("Enter 'q' to exit\n");

		inputKey = getchar();

		if(inputKey == 'c') //set current
		{
			//init cmd
			initCmdSet();

			int currentAmp = 100;
			printf("Enter a value in mA (0 to 500) :\n");
			scanf("%d", &currentAmp);
			if(currentAmp < 0) currentAmp = 0;
			if(currentAmp > 500) currentAmp = 500;

			setCurrentCmd(currentAmp);

			legs_cmd_pub.publish(cmd);
			ros::Duration(0.1).sleep();
			legs_cmd_pub.publish(cmd);
			ros::Duration(0.1).sleep();

			for(int i=0; i<30; i++)
			{
				cmd.motorCmd[i].nodeID = i+1;
				cmd.motorCmd[i].mode = CURRENT_MODE;
				cmd.motorCmd[i].value = 100;
			}

			arms_cmd_pub.publish(cmd);
			ros::Duration(0.1).sleep();
			arms_cmd_pub.publish(cmd);

			printf("Current set to %d mA\n", currentAmp);
		}
		else if(inputKey == 'r')
		{
			//getchar();

			int postureNb = 0;
			printf("How many postures do you want to grab ?\n");
			scanf("%d", &postureNb);

			printf("%d postures will be recorded !\n", postureNb);

			nbPostToPlay = postureNb;

			//empty keyboard buffer
			std::cin.clear();
			std::cin.ignore(INT_MAX, '\n');

			for(int i=0; i<postureNb; i++)
			{
				printf("Recording of posture Nb %d\n", i+1);
				printf("Press Enter when ready !\n");
				std::cin.get();

				printf("Waiting for posture data to arrive...");

				while(!postureArrived)
				{
					ros::spinOnce(); //listen to topics
				}

				printf("...OK\n");
				//but this back to false for the next record
				postureArrived = false;

				//save posture in array
				postureArray[i] = posture;
			}
		}
		else if(inputKey == 'o')
		{
			rosbag::Bag bag;

			char filename[256];
			printf("Enter a file name to open :\n");
			scanf("%s", filename);
			//printf("file : %s\n", filename);

			char *path = "/home/dcx/catkin_hydro_ws/src/trs_control/bag/legs/";
			char filePath[512];
			strcpy (filePath, path);
			strcat (filePath, filename);

			printf("file : %s\n", filePath);

			bag.open(filePath, rosbag::bagmode::Read);

			rosbag::View view_data(bag, rosbag::TopicQuery("/motorDataSet"));
			int dataset_dim = view_data.size();
			//int line = 0;
			nbPostToPlay = 0;
			BOOST_FOREACH(rosbag::MessageInstance const m, view_data)
			{
				trs_msgs::MotorDataSet::Ptr i = m.instantiate<trs_msgs::MotorDataSet>();

				if(i != NULL)
				{
					postureArray[nbPostToPlay] = *i;
				}
				nbPostToPlay++;
			}

			bag.close(); //close the bag
			printf("Bag opened ok\n");
		}
		else if(inputKey == 's')
		{
			rosbag::Bag bag;

			char filename[256];
			printf("Enter a file name :\n");
			scanf("%s", filename);
			//printf("file : %s\n", filename);

			char *path = "/home/dcx/catkin_hydro_ws/src/trs_control/bag/legs/";
			char filePath[512];
			strcpy (filePath, path);
			strcat (filePath, filename);

			printf("file : %s\n", filePath);

			bag.open(filePath, rosbag::bagmode::Write);

			for(int i=0; i<nbPostToPlay ;i++)
			{
				//get time
				ros::Time time = ros::Time::now();
				//write data to the bag
				bag.write("/motorDataSet", time, postureArray[i]);
				ros::Duration(0.1).sleep();
			}

			bag.close(); //close the bag
			printf("Bag saved ok\n");
		}
		else if(inputKey == 'p')
		{
			if(nbPostToPlay != 0)
			{
				printf("There are %d postures to be played\n", nbPostToPlay);
				int postNbToPlay = 0;
				printf("Which posture do you want to play ? (enter 0 to exit Player mode)\n");

				bool play = true;

				while(play)
				{
					scanf("%d", &postNbToPlay);
					if((postNbToPlay >= 1) && (postNbToPlay <= nbPostToPlay))
					{
						setPositionCmd(postureArray[postNbToPlay-1]);

						printf("val = %d\n", postureArray[postNbToPlay-1].motorData[45].encPosition);

						legs_cmd_pub.publish(cmd);
						ros::Duration(0.1).sleep();
						legs_cmd_pub.publish(cmd);
					}
					else
					{
						play = false;
						printf("Exit Player mode\n");
					}
				}
			}
			else
			{
				printf("There are no postures to be played !\n");
			}
		}/*
		else if(inputKey == 'b')
		{
			printf("Run balance algorithm\n");
		}*/
		else if(inputKey == 'q')
		{
			run = false; //stop the while loop, quit the program
			printf("Exit Legs Demo\n");
		}
		else
		{
			printf("Wrong menu input !\n");
		}

		//getchar(); //to grab carriage return
		//empty keyboard buffer
		std::cin.clear();
		std::cin.ignore(INT_MAX, '\n');

		//loopNb++;
	}

	return 0;
}

