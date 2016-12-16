/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>
#include <stdlib.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

enum Sequence_Enum{
	ON_GROUND, TAKEOFF, GO_TO_INITIAL_TARGET_ALT, GO_TO_ZONE, APPROACH_DROP_ZONE, ENTER_DROP_ZONE, GO_TO_SAFE_ALT, RETURN, APPROACH_GROUND, LAND
};

int main(int argc, char** argv){
	ros::init(argc, argv, "hovering_example");
	ros::NodeHandle nh;
	ros::Publisher trajectory_pub = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
	ROS_INFO("Started hovering example.");
	std_srvs::Empty srv;
	bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
	unsigned int i = 0;
	// Trying to unpause Gazebo for 10 seconds.
	while(i <= 10 && !unpaused){
		ROS_INFO("Wait for 1 second before trying to unpause Gazebo again");
		std::this_thread::sleep_for(std::chrono::seconds(1));
		unpaused = ros::service::call("/gazebo/unpause_physics", srv);
		++i;
	}

	Sequence_Enum sequence = ON_GROUND;

	if(!unpaused){
		ROS_FATAL("Could not wake up Gazebo.");
		return -1;
	}else{
		ROS_INFO("Unpaused the Gazebo simulation.");
	}
	float desired_x = 0;
	float desired_y = 0;
	float desired_z = 0.5;
	std::string nameofuav = argv[1];
//		ros::Duration(5.0).sleep();

	if(nameofuav == "b"){
		popen("rosrun rotors_gazebo spawn_models.py", "r");
	}

	// Motion sequence of the first UAV.
	while(1){

		switch(sequence){
			case ON_GROUND:
				ros::Duration(5).sleep();
				desired_x = 0.0;
				desired_y = 0.0;
				desired_z = 0.5;
				sequence = TAKEOFF;
				break;

			case TAKEOFF:
				ros::Duration(5).sleep();
				desired_x = 0.0;
				desired_y = 0.0;
				desired_z = 2.0;
				sequence = GO_TO_INITIAL_TARGET_ALT;
				break;

			case GO_TO_INITIAL_TARGET_ALT:
				ros::Duration(5).sleep();
				desired_x = 0.0;
				desired_y = 0.0;
				desired_z = 8.0;
				sequence = GO_TO_ZONE;
				break;

			case GO_TO_ZONE:
				ros::Duration(10).sleep();
				desired_x = 2.0;
				desired_y = 7.0;
				desired_z = 5.0;
				sequence = APPROACH_DROP_ZONE;
				break;

			case APPROACH_DROP_ZONE:
				ros::Duration(10).sleep();
				desired_x = 2.0;
				desired_y = 7.0;
				desired_z = 2.0;
				sequence = ENTER_DROP_ZONE;
				break;

			case ENTER_DROP_ZONE:
				ros::Duration(5).sleep();
				desired_x = 2.0;
				desired_y = 7.0;
				desired_z = 0.0;
				sequence = GO_TO_SAFE_ALT;
				break;

			case GO_TO_SAFE_ALT:
				ros::Duration(3).sleep();
				desired_x = 2.0;
				desired_y = 7.0;
				desired_z = 5.0;
				sequence = RETURN;
				break;

			case RETURN:
				ros::Duration(5).sleep();
				desired_x = 0.0;
				desired_y = 0.0;
				desired_z = 6.0;
				sequence = APPROACH_GROUND;
				break;

			case APPROACH_GROUND:
				ros::Duration(10).sleep();
				desired_x = 0.0;
				desired_y = 0.0;
				desired_z = 2.0;
				sequence = LAND;
				break;

			case LAND:
				ros::Duration(5).sleep();
				desired_x = 0.0;
				desired_y = 0.0;
				desired_z = 0.0;
				sequence = ON_GROUND;
				break;

			default:
				ROS_WARN("Unknown Sequence!");
		}

		trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		trajectory_msg.header.stamp = ros::Time::now();
		Eigen::Vector3d desired_position(desired_x, desired_y, desired_z);
		double desired_yaw = 0.0;
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		ROS_INFO("Publishing waypoint on namespace  %s: [%f, %f, %f].", nh.getNamespace().c_str(), desired_position.x(), desired_position.y(),
				desired_position.z());
		trajectory_pub.publish(trajectory_msg);
	}
	ros::spin();
}
