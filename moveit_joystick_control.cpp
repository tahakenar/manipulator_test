#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
    float frac = 0.02;

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

    float z = joy->axes[4];
    if (checkJoyDeadband(z))
        deltaXYZ[2] = frac*z;
    else
        deltaXYZ[2] = 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(150);
    ros::Subscriber joy_sub = nh.subscribe("joy",1,joyCallback);

    ros::AsyncSpinner spinner(2); //do we need these
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    while (ros::ok())
    {
    ros::spinOnce();
    ROS_INFO("%f %f %f",deltaXYZ[0],deltaXYZ[1],deltaXYZ[2]);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose;
    current_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(current_pose);

    if(deltaXYZ[2] != 0 && deltaXYZ[1] == 0 && deltaXYZ[0] == 0) // z axis
        {
            geometry_msgs::Pose target_pose = current_pose;

            target_pose.position.z += deltaXYZ[3];
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
    
    loop_rate.sleep();
    
    }  
    return 0;
}