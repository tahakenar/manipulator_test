#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>

// TODO: Forward kinematics mode / control using joint space
// TODO: Update according to the last encoder data -> EZ, partly ready
// TODO: Reading data from encoder database
// TODO: Quaternion conversion check
// TODO: Signs of directions for each joint will be determined
// TODO: Joint space constraints will be determined -> can be solved from joint_limits.yaml file

static const std::string PLANNING_GROUP = "manipulator";

float deltaXYZ[3] = {0.0,0.0,0.0};
float deltaRPY[3] = {0.0,0.0,0.0};
// global variables to control end effector position and orientation

std::vector<double> encoder{0.0,0.0,0.0,0.0,0.0,0.0};
bool first_run = true;
// global variable to store last encoder data

int mode = 0; 
int switch_btn_prev = 0;
float frac = 0.002;
bool update_joints = false;
// global variables to achieve a mode switch algorithm, they are being used in joyCallback function

bool checkJoyDeadband(float data){
    if (abs(data) > 0.45)
        return true;
    else 
        return false;
}


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    int switch_btn_current = joy->buttons[7];
    if (switch_btn_current == 1 && switch_btn_prev == 0){
        mode += 1;
        if (mode == 4)
            mode = 0;
        if (mode == 0){
            frac = 0.002;
            ROS_INFO("Control mode: SLOW \n");
        }
        else if (mode == 1){
            frac = 0.01;
            ROS_INFO("Control mode: MID \n");
        }
        else if (mode == 2){
            frac = 0.04;
            ROS_INFO("Control mode: FAST /tikkatli olun\n");
        } 
        else if (mode == 3){
            ROS_INFO("Control mode: FORWARD KINEMATICS\n");
            // Forward kinematics mode will be added
        } 
    }
    switch_btn_prev = switch_btn_current;

    if (mode == 0 || mode == 1 || mode == 2){
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
            deltaRPY[0] = frac*roll*7;
        else
            deltaRPY[0] = 0;

        float pitch = joy->axes[4];
        if (checkJoyDeadband(pitch))
            deltaRPY[1] = -frac*pitch*7;
        else
            deltaRPY[1] = 0;

        float yaw = joy->axes[3];
        if (checkJoyDeadband(yaw))
            deltaRPY[2] = frac*yaw*7;
        else
            deltaRPY[2] = 0;
    } 
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(150);
    ros::Subscriber joy_sub = nh.subscribe("joy",1,joyCallback);
    
    ros::Publisher joint_values_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_states/send", 1000);

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

    ros::AsyncSpinner spinner(2); 
    spinner.start();

    
    ROS_INFO("Control mode: SLOW \n");
    while (ros::ok())
    {

    ros::spinOnce();
    ROS_INFO("%f %f %f ----\n %f %f %f",deltaXYZ[0],deltaXYZ[1],deltaXYZ[2],deltaRPY[0],deltaRPY[1],deltaRPY[2]);
    
    std::vector<double> joint_values = move_group.getCurrentJointValues();
    std_msgs::Float64MultiArray array_of_angles;
    for (int i = 0; i < 6; i++)
        {
            array_of_angles.data.push_back(joint_values[i]);
        }
    array_of_angles.data.push_back(mode);
    joint_values_pub.publish(array_of_angles);
    //TODO: migth be solved in a better way

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose;
    current_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(current_pose);

////////////////////////////////// UPDATING     
    if (update_joints || first_run){
        move_group.setJointValueTarget(encoder);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
        move_group.move();
        first_run = false;
    }
/////////////////////////////////


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

    // I took the euler-queternion transformation from ITU rover team 2019 repository

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
    update_joints = false;
    
    }  
    return 0;
}
