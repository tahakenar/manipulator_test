#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

static const std::string PLANNING_GROUP = "manipulator";
float deltaXYZ[3] = {0.0,0.0,0.0};
float deltaRPY[3] = {0.0,0.0,0.0};

bool checkJoyDeadband(float data){
    if (abs(data) > 0.45)
        return true;
    else 
        return false;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    float frac = 0.04;

    // cartesian point position control
    float x = joy->axes[1];
    if (checkJoyDeadband(x))
        deltaXYZ[0] = frac*x;
    else
        deltaXYZ[0] = 0;
    
    float y = joy->axes[0];
    if (checkJoyDeadband(y))
        deltaXYZ[1] = frac*y;
    else
        deltaXYZ[1] = 0;

    float z = joy->axes[7];
    if (checkJoyDeadband(z))
        deltaXYZ[2] = frac*z;
    else
        deltaXYZ[2] = 0;

    // euler angles orientation control
    float roll = joy->axes[6];
    if (checkJoyDeadband(roll))
        deltaRPY[0] = frac*roll*4;
    else
        deltaRPY[0] = 0;

    float pitch = joy->axes[4];
    if (checkJoyDeadband(pitch))
        deltaRPY[1] = frac*pitch*4;
    else
        deltaRPY[1] = 0;

    float yaw = joy->axes[3];
    if (checkJoyDeadband(yaw))
        deltaRPY[2] = frac*yaw*4;
    else
        deltaRPY[2] = 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(150);
    ros::Subscriber joy_sub = nh.subscribe("joy",1,joyCallback);

    ros::AsyncSpinner spinner(2); 
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    while (ros::ok())
    {
    ros::spinOnce();
    ROS_INFO("%f %f %f ----\n %f %f %f",deltaXYZ[0],deltaXYZ[1],deltaXYZ[2],deltaRPY[0],deltaRPY[1],deltaRPY[2]);
    //std::vector<double> joint_values = move_group.getCurrentJointValues();
    //std::cout << joint_values[0] << std::endl;

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose;
    current_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(current_pose);

    if(deltaXYZ[2] != 0 && deltaXYZ[1] == 0 && deltaXYZ[0] == 0) // z axis
        {
            geometry_msgs::Pose target_pose = current_pose;

            target_pose.position.z += deltaXYZ[2];
            waypoints.push_back(target_pose);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            if(fraction == 1){
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group.execute(plan);
            }
        }

    else if (deltaXYZ[0] != 0 && deltaXYZ[1] == 0 && deltaXYZ[2] == 0) // x axis
        {
            geometry_msgs::Pose target_pose = current_pose;

            target_pose.position.x += deltaXYZ[0];
            waypoints.push_back(target_pose);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            if(fraction == 1){
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group.execute(plan);
            }
        }

    else if (deltaXYZ[1] != 0 && deltaXYZ[0] == 0 && deltaXYZ[2] == 0) // y axis
        {
            geometry_msgs::Pose target_pose = current_pose;

            target_pose.position.y += deltaXYZ[1];
            waypoints.push_back(target_pose);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            if(fraction == 1){
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group.execute(plan);
            }
        }

    // I took the euler-quaternion transformation from ITU rover team 2019 repository
    // TODO: learn and implement euler-quaternion transform. 

    else if (deltaRPY[0] != 0 && deltaRPY[1] == 0 && deltaRPY[2] == 0) // roll 
        {
            geometry_msgs::Pose target_pose = current_pose;
            tf::Quaternion q_orig, q_rot, q_new;
            double roll = 2*0.17453*deltaRPY[0], yaw = 0, pitch = 0;
            q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
            quaternionMsgToTF(target_pose.orientation , q_orig);
            q_new = q_rot*q_orig;
            q_new.normalize();
            quaternionTFToMsg(q_new, target_pose.orientation);
            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            move_group.move();

            }

    else if (deltaRPY[1] != 0 && deltaRPY[0] == 0 && deltaRPY[2] == 0) // pitch 
        {
            geometry_msgs::Pose target_pose = current_pose;
            tf::Quaternion q_orig, q_rot, q_new;
            double pitch = 2*0.17453*deltaRPY[1], yaw = 0, roll = 0;
            q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
            quaternionMsgToTF(target_pose.orientation , q_orig);
            q_new = q_rot*q_orig;
            q_new.normalize();
            quaternionTFToMsg(q_new, target_pose.orientation);
            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            move_group.move();

            }

    else if (deltaRPY[2] != 0 && deltaRPY[0] == 0 && deltaRPY[1] == 0) // yaw 
        {
            geometry_msgs::Pose target_pose = current_pose;
            tf::Quaternion q_orig, q_rot, q_new;
            double yaw = 2*0.17453*deltaRPY[2], pitch = 0, roll = 0;
            q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
            quaternionMsgToTF(target_pose.orientation , q_orig);
            q_new = q_rot*q_orig;
            q_new.normalize();
            quaternionTFToMsg(q_new, target_pose.orientation);
            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            move_group.move();

            }

    loop_rate.sleep();
    
    }  
    return 0;
}
