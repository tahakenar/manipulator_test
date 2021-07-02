#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include "moveit_functions.h"


void CustomMoveItFuncs::updateJointStates(moveit::planning_interface::MoveGroupInterface &group,moveit::planning_interface::MoveGroupInterface::Plan &myPlan,std::vector<double> encoder_list){

    bool success;
    group.setJointValueTarget(encoder_list);
    success = (group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    group.move();
	}
	
void CustomMoveItFuncs::ExecuteCartesianPath(std::vector<float> deltaXYZ,moveit::planning_interface::MoveGroupInterface &group, std::vector<geometry_msgs::Pose> &waypoints,geometry_msgs::Pose &target_pose){
		target_pose.position.x += deltaXYZ[0];
		target_pose.position.y += deltaXYZ[1];
		target_pose.position.z += deltaXYZ[2];

        waypoints.push_back(target_pose);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if(fraction == 1){
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            group.execute(plan);
        } 
}

void CustomMoveItFuncs::ExecuteRPYGoal(std::vector<float> deltaRPY, moveit::planning_interface::MoveGroupInterface &group, geometry_msgs::Pose &target_pose){
	
    tf::Quaternion q_orig, q_rot, q_new;
    double frac = 2*0.17453;
    double roll = frac*deltaRPY[0], pitch = frac*deltaRPY[1], yaw = frac*deltaRPY[2];
    q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
    quaternionMsgToTF(target_pose.orientation , q_orig);
    q_new = q_rot*q_orig;
    q_new.normalize();
    quaternionTFToMsg(q_new, target_pose.orientation);
    group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    group.move();

    // I took the euler-queternion transformation from ITU rover team 2019 repository
}

std_msgs::Float64MultiArray CustomMoveItFuncs::returnJointAngles(moveit::planning_interface::MoveGroupInterface &group, int mode){

    std::vector<double> joint_values = group.getCurrentJointValues();
    std_msgs::Float64MultiArray array_of_angles;
    for (int i = 0; i < 6; i++)
    {
        array_of_angles.data.push_back(joint_values[i]);
    }
    array_of_angles.data.push_back(mode);

    return array_of_angles;
}



