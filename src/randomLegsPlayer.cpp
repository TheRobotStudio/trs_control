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
 *  Created on: Oct 27, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <boost/foreach.hpp>
#include "trs_control/getMotorCmdSet.h"
#include "trs_control/switchNode.h"
#include <trs_msgs/MotorCmdSet.h>
#include <trs_msgs/MotorDataSet.h>
#include <stdio.h>
#include <cstdlib>
#include <flann/flann.hpp> //used for the kdtree search
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE	HEART_BEAT
#define ALPHA_HI	2.8
#define ALPHA_LO	0
#define BETA_HI		1.5
#define BETA_LO		0.2
#define THETA_HI	0
#define THETA_LO	-1.5
#define PHI_HI		1.2
#define PHI_LO		0
#define DOF_DIM 	30

/*** Namespaces ***/
using namespace flann;

/*** Variables ***/
bool switch_node = false;  //disable by default
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
	ros::init (argc, argv, "trs_randomPlayer_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);
	
	//Services
    ros::ServiceServer srv_switchNode = nh.advertiseService("switch_random_legs_player", switchNode);
    ros::ServiceServer srv_getMotorCmdSet = nh.advertiseService("get_random_legs_player_cmd", getMotorCmdSet);

    //read a bag to generate 2 datasets, one for each arm
	rosbag::Bag bag(ros::package::getPath("trs_control") + "/bag/legsPostures_1.bag"); //change this bag
	rosbag::View view_posture(bag, rosbag::TopicQuery("/motorDataSet")); //motor data position info

	//create the dataset_positions matrix
	int dataset_post_dim = view_posture.size(); //set the dataset dim equal to the number of lines in the bag file

	while(ros::ok())
	{
		if(switch_node)
		{
			//update
			initCmdSet();
			int generator = rand() % dataset_post_dim;

			int line = 0;
				BOOST_FOREACH(rosbag::MessageInstance const m, view_posture) //error compiles ok
				{
					trs_msgs::MotorDataSet::Ptr i = m.instantiate<trs_msgs::MotorDataSet>();

					if(i != NULL)
					{
						if(line == generator)
						{
							for(int j=3*NUMBER_MAX_EPOS2_PER_SLAVE; j<5*NUMBER_MAX_EPOS2_PER_SLAVE; j++)
							{
								motorCmdSet.motorCmd[j].mode = POSITION_MODE;
								motorCmdSet.motorCmd[j].value =i->motorData[j].encPosition;
							}
						}
					}
					line++;
				}
		}

		ros::spinOnce();
		r.sleep();
	}

	bag.close();

	return 0;
}
