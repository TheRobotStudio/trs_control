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
 *  Created on: Jul 25, 2012
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>
#include <trs_msgs/MotorCmdSet.h>
#include <trs_msgs/MotorDataSet.h>
#include <std_msgs/Bool.h>
#include "trs_control/getMotorCmdSet.h"
#include "trs_control/switchNode.h"
#include <flann/flann.hpp> //used for the kdtree search
#include <stdio.h>
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT	//1  //20
#define SPACE_DIM  				4  //4 angles : alpha, beta, theta and phi
#define REF_DATA_DIM 			1  //only one reference point in 4d space to find its nearest neighboor.
#define NUM_NN 					5  //number of nearest neighboors
#define DOF_DIM 				13 //degrees of freedom = number of motors //(no rotator and hand)

using namespace flann;

/*** Variables ***/
bool switch_node = false; //disable by default
trs_msgs::MotorCmdSet motorCmdSet;
int dataset_dim = 0; //this will depend on the size of the bag, number of lines of data

//left arm
Matrix<float> dataset_L_angles; //matrix for the angles
Matrix<int> dataset_L_positions; //matrix for the positions
Matrix<float> query_L(new float[SPACE_DIM*REF_DATA_DIM], REF_DATA_DIM, SPACE_DIM); //matrix for the query, just one line
Matrix<int> indices_L(new int[query_L.rows*SPACE_DIM], query_L.rows, NUM_NN);
Matrix<float> dists_L(new float[query_L.rows*SPACE_DIM], query_L.rows, NUM_NN);

//right arm
Matrix<float> dataset_R_angles; //matrix for the angles
Matrix<int> dataset_R_positions; //matrix for the positions
Matrix<float> query_R(new float[SPACE_DIM*REF_DATA_DIM], REF_DATA_DIM, SPACE_DIM); //matrix for the query, just one line
Matrix<int> indices_R(new int[query_R.rows*SPACE_DIM], query_R.rows, NUM_NN);
Matrix<float> dists_R(new float[query_R.rows*SPACE_DIM], query_R.rows, NUM_NN);

//ROS publisher
ros::Publisher pub_cmdSet; //this allows to send a cmd msg packet to the mbed and drive the motors

//debug function to display a matrix
void displayMatrixFloat(const Matrix<float> matrix, char* name)
{
	std::cout << name << " matrix :" << std::endl;

	int count = 0;
	for(int i=0; i<matrix.rows; i++)
	{
		std::cout << count << " | ";

		for(int j=0; j<matrix.cols; j++)
		{
			std::cout << matrix.ptr()[matrix.cols*i+j] << "\t";
		}

		count += 1;
		std::cerr << std::endl;
	}
}

void displayMatrixInt(const Matrix<int> matrix, char* name)
{
	std::cout << name << " matrix :" << std::endl;

	int count = 0;
	for(int i=0; i<matrix.rows; i++)
	{
		std::cout << count << " | ";

		for(int j=0; j<matrix.cols; j++)
		{
			std::cout << matrix.ptr()[matrix.cols*i+j] << "\t";
		}

		count += 1;
		std::cerr << std::endl;
	}
}

/*** Callback functions ***/
void tracking_cb(const sensor_msgs::JointStateConstPtr& js)
{
	if(switch_node)
	{
		if(js->name.size() != 0)
		{
			//update the query
			for(int i=0; i<query_R.cols; i++) //query cols should be 1
			{
				query_L.ptr()[i] = js->position[i];
				query_R.ptr()[i] = js->position[SPACE_DIM + i];
			}

			/*** Left ***/
			// construct a randomized kd-tree index using 4 kd-trees
			Index<L2<float> > data_L_index(dataset_L_angles, flann::KDTreeIndexParams(SPACE_DIM));
			data_L_index.buildIndex();

			// do a knn search, using 128 checks
			data_L_index.knnSearch(query_L, indices_L, dists_L, SPACE_DIM, flann::SearchParams(128));
			int data_L_idx = indices_L.ptr()[0];

			//ROS_INFO("data_L_idx = %d", data_L_idx);
			//displayMatrixFloat(query_L, "Query_L");

			/*** Right ***/
			// construct a randomized kd-tree index using 4 kd-trees
			Index<L2<float> > data_R_index(dataset_R_angles, flann::KDTreeIndexParams(SPACE_DIM));
			data_R_index.buildIndex();

			// do a knn search, using 128 checks
			data_R_index.knnSearch(query_R, indices_R, dists_R, SPACE_DIM, flann::SearchParams(128));
			int data_R_idx = indices_R.ptr()[0];

			//ROS_INFO("data_R_idx = %d", data_R_idx);
			//displayMatrixFloat(query_R, "Query_R");

			//init of the packet
			for(int i=0; i<DOF_DIM; i++)
			{
				//L
				motorCmdSet.motorCmd[i].nodeID = i+1;
				motorCmdSet.motorCmd[i].mode = 0;
				motorCmdSet.motorCmd[i].value = dataset_L_positions.ptr()[dataset_L_positions.cols*data_L_idx+i];

				//R
				motorCmdSet.motorCmd[NUMBER_MAX_EPOS2_PER_SLAVE+i].nodeID = NUMBER_MAX_EPOS2_PER_SLAVE+i+1;
				motorCmdSet.motorCmd[NUMBER_MAX_EPOS2_PER_SLAVE+i].mode = 0;
				motorCmdSet.motorCmd[NUMBER_MAX_EPOS2_PER_SLAVE+i].value = dataset_R_positions.ptr()[dataset_R_positions.cols*data_R_idx+i];
			}
		}
	}
}

/*** Services ***/
bool switchNode(trs_control::switchNode::Request  &req, trs_control::switchNode::Response &res)
{
	switch_node = req.state;
	return true;
}

bool getKdTreeAnglesCmd(trs_control::getMotorCmdSet::Request  &req, trs_control::getMotorCmdSet::Response &res)
{
	//send the cmd set by the callback function
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
	// Initialize ROS
	ros::init (argc, argv, "trs_kdtreeAnglesToArms_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	// Create a ROS subscriber for the input joint_states_kinect topic, given by the trs_kinectToAngles node
	//that will be the requests for the kd-trees
	ros::Subscriber sub = nh.subscribe ("/anglesArmsDescription", 10, tracking_cb);

	//Services
	ros::ServiceServer service_switchNode = nh.advertiseService("switch_kd_tree_angles", switchNode);
	ros::ServiceServer service_getKdTreeAnglesCmd = nh.advertiseService("get_kd_tree_angles_cmd", getKdTreeAnglesCmd);

	//Initialization of the kd tree
	ROS_INFO("Initialization of the 2 kd-tree :");

	//read a bag to generate 2 datasets, one for each arm
	rosbag::Bag bag(ros::package::getPath("trs_control") + "/bag/anglesToPosture_1.bag"); //change this bag
	rosbag::View view_joint(bag, rosbag::TopicQuery("/anglesArmsDescription")); //angle info
	rosbag::View view_posture(bag, rosbag::TopicQuery("/motorDataSet")); //motor data position info

	//create the 2 dataset_angle matrix
	dataset_dim = view_joint.size(); //set the dataset dim equal to the number of lines in the bag file
	Matrix<float> tempL1(new float[SPACE_DIM*dataset_dim], dataset_dim, SPACE_DIM);
	dataset_L_angles = tempL1;
	Matrix<float> tempR1(new float[SPACE_DIM*dataset_dim], dataset_dim, SPACE_DIM);
	dataset_R_angles = tempR1;

	ROS_INFO("Create angle matrix");
	int line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_joint)
	{
		sensor_msgs::JointState::Ptr i = m.instantiate<sensor_msgs::JointState>();

	    if(i != NULL)
	    {
	    	//Build the right dataset_angles matrix
	    	for(int j=0; j<SPACE_DIM; j++)
	    	{
	    		dataset_L_angles.ptr()[dataset_L_angles.cols*line+j] = i->position[j]; //offset of 4 for left
	    		dataset_R_angles.ptr()[dataset_R_angles.cols*line+j] = i->position[SPACE_DIM + j];
	    	}
	    }
		else
			std::cout << "null" << std::endl;

	    line++;
	}

	ROS_INFO("Create posture matrix");
	//create the dataset_positions matrix
	int dataset_post_dim = view_posture.size(); //set the dataset dim equal to the number of lines in the bag file
	Matrix<int> tempL2(new int[DOF_DIM*dataset_post_dim], dataset_post_dim, DOF_DIM);
	dataset_L_positions = tempL2;
	Matrix<int> tempR2(new int[DOF_DIM*dataset_post_dim], dataset_post_dim, DOF_DIM);
	dataset_R_positions = tempR2;

	line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_posture) //error compiles ok
	{
		trs_msgs::MotorDataSet::Ptr i = m.instantiate<trs_msgs::MotorDataSet>();

		if(i != NULL)
		{
			//i->header.stamp = ros::Time::now();

			//Build the dataset_positions matrix
			for(int j=0; j<DOF_DIM; j++)
			{
				dataset_L_positions.ptr()[dataset_L_positions.cols*line+j] = i->motorData[j].encPosition;
				dataset_R_positions.ptr()[dataset_R_positions.cols*line+j] = i->motorData[NUMBER_MAX_EPOS2_PER_SLAVE + j].encPosition;
			}
		}
		else
			std::cout << "null" << std::endl;

		line++;
	}

	bag.close();

	//#ifdef TRS_DEBUG
	displayMatrixFloat(dataset_L_angles, "dataset_L_angles");
	displayMatrixInt(dataset_L_positions, "dataset_L_positions");
	displayMatrixFloat(dataset_R_angles, "dataset_R_angles");
	displayMatrixInt(dataset_R_positions, "dataset_R_positions");
	//#endif

	while(ros::ok())
	{
		ros::spinOnce();
		//ROS_INFO("spinOnce");
		r.sleep();
	}

	//free matrix pointers from memory

	delete[] dataset_L_angles.ptr();
	delete[] dataset_L_positions.ptr();
	delete[] query_L.ptr();
	delete[] indices_L.ptr();
	delete[] dists_L.ptr();

	delete[] dataset_R_angles.ptr();
	delete[] dataset_R_positions.ptr();
	delete[] query_R.ptr();
	delete[] indices_R.ptr();
	delete[] dists_R.ptr();

	return 0;
}




