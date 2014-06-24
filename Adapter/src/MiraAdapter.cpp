/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file MiraAdapter.cpp
 *    An adapter for messages between ROS and MIRA.
 *
 * @author Dirk Steindorf
 * @date   2014/05/16
 */

#include <algorithm>
#include <ros/ros.h>
#include <fw/Framework.h>
#include <std_msgs/String.h>
#include <fw/ChannelReadWrite.h>

mira::Authority authority; 

// ROS publisher
ros::Publisher scitosToKuka1;
ros::Publisher scitosToKuka2;

// channels for publishing ROS messages to MIRA
mira::Channel<std::string> kuka1ToScitos;
mira::Channel<std::string> kuka2ToScitos;


//------------------------------------------------------------------------------
// callbacks for sending messages to the Kuka robots
void onDataForKuka1(mira::ChannelRead<std::string> data)
{
    std_msgs::String msg;
    msg.data = data->value();
    scitosToKuka1.publish(msg);
}

void onDataForKuka2(mira::ChannelRead<std::string> data)
{
    std_msgs::String msg;
    msg.data = data->value();
    scitosToKuka2.publish(msg);
}

//------------------------------------------------------------------------------
// callbacks for sending messages to the Scitos
void kuka1Callback(const std_msgs::String::ConstPtr& msg)
{
    std::string message(msg->data);
    kuka1ToScitos.post(message);
}

void kuka2Callback(const std_msgs::String::ConstPtr& msg)
{
    std::string message(msg->data);
    kuka2ToScitos.post(message);
}

int main(int argc, char **argv)
{
    //--------------------------------------------------------------------------
    // initialize ROS and MIRA

	// ros init
	ros::init(argc, argv, "Ros2Mira");

	// create and start the mira framework
	mira::Framework framework(argc, argv, true);

    //--------------------------------------------------------------------------
    // MIRA Channels

	// create mira authority and publish the channels
	authority.checkin("/", "Ros2Mira");
    authority.start();
	kuka1ToScitos = authority.publish<std::string>("kuka1ToScitos");
	kuka2ToScitos = authority.publish<std::string>("kuka2ToScitos");

    // subscribe to MIRA-Channel
    authority.subscribe<std::string>("scitosToKuka1", &onDataForKuka1);
    authority.subscribe<std::string>("scitosToKuka2", &onDataForKuka2);

    //--------------------------------------------------------------------------
    // ROS nodes
    
    // publisher
    ros::NodeHandle pubNode1;
    ros::NodeHandle pubNode2;
    
    scitosToKuka1 = pubNode1.advertise<std_msgs::String>("scitosChatter1", 1000);
    scitosToKuka2 = pubNode2.advertise<std_msgs::String>("scitosChatter2", 1000);

	// subscriber
	ros::NodeHandle node;
	ros::NodeHandle subNode1;
	ros::NodeHandle subNode2;
	
	ros::Subscriber subscriber1 = subNode1.subscribe("kuka1Chatter", 1000, kuka1Callback);
	ros::Subscriber subscriber2 = subNode2.subscribe("kuka2Chatter", 1000, kuka2Callback);


	// do the locomotion
	ros::spin();
	return 0;
}
