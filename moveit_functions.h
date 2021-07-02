#ifndef MOVEIT_FUNCTIONS
#define MOVEIT_FUNCTIONS

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

namespace CustomMoveItFuncs{

void updateJointStates(moveit::planning_interface::MoveGroupInterface &group,moveit::planning_interface::MoveGroupInterface::Plan &myPlan,std::vector<double> encoder_list);
	
void ExecuteCartesianPath(std::vector<float> deltaXYZ,moveit::planning_interface::MoveGroupInterface &group, std::vector<geometry_msgs::Pose> &waypoints, geometry_msgs::Pose &target_pose);

void ExecuteRPYGoal(std::vector<float> deltaRPY,moveit::planning_interface::MoveGroupInterface &group, geometry_msgs::Pose &target_pose);

std_msgs::Float64MultiArray returnJointAngles(moveit::planning_interface::MoveGroupInterface &group, int mode);
}

#endif
