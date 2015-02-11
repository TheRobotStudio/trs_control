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
 *  Created on: Jun 13, 2013
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>
#include <trs_msgs/MotorDataSet.h>
#include <stdio.h>

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "trs_mergeBags");
	ros::NodeHandle nh;

	rosbag::Bag bag1, bag2, mergedBag;
	bag1.open(ros::package::getPath("trs_control") + "/bag/jointToPosture_bag1.bag", rosbag::bagmode::Read);
	bag2.open(ros::package::getPath("trs_control") + "/bag/jointToPosture_bag2.bag", rosbag::bagmode::Read);
	mergedBag.open(ros::package::getPath("trs_control") + "/bag/anglesToPosture_merged.bag", rosbag::bagmode::Write);

	rosbag::View view_joint_bag1(bag1, rosbag::TopicQuery("/anglesArmsDescription")); //angle info
	rosbag::View view_posture_bag1(bag1, rosbag::TopicQuery("/motorDataSet")); //motor data position info

	rosbag::View view_joint_bag2(bag2, rosbag::TopicQuery("/anglesArmsDescription")); //angle info
	rosbag::View view_posture_bag2(bag2, rosbag::TopicQuery("/motorDataSet")); //motor data position info


	int line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_joint_bag1)
	{
		sensor_msgs::JointState::Ptr i = m.instantiate<sensor_msgs::JointState>();
	    if(i != NULL)
	    {
			ros::Time time = ros::Time::now(); //time doesn't matter
			mergedBag.write("/anglesArmsDescription", time, *i);
	    }
	    line++;
	}

	line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_joint_bag2)
	{
		sensor_msgs::JointState::Ptr i = m.instantiate<sensor_msgs::JointState>();
		if(i != NULL)
		{
			ros::Time time = ros::Time::now(); //time doesn't matter
			mergedBag.write("/anglesArmsDescription", time, *i);
		}
		line++;
	}

	line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_posture_bag1)
	{
		trs_msgs::MotorDataSet::Ptr i = m.instantiate<trs_msgs::MotorDataSet>();
		if(i != NULL)
		{
			ros::Time time = ros::Time::now(); //time doesn't matter
			mergedBag.write("/motorDataSet", time, *i);
		}
		line++;
	}

	line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_posture_bag2)
	{
		trs_msgs::MotorDataSet::Ptr i = m.instantiate<trs_msgs::MotorDataSet>();
		if(i != NULL)
		{
			ros::Time time = ros::Time::now(); //time doesn't matter
			mergedBag.write("/motorDataSet", time, *i);
		}
		line++;
	}

	bag1.close();
	bag1.close();
	mergedBag.close();

	return 0;
}



