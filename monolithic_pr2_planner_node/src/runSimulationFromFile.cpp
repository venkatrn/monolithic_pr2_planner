#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <vector>
#include <string>
#include <cstdio>
#include <geometry_msgs/PoseStamped.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/StatsWriter.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <angles/angles.h>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace monolithic_pr2_planner;

int main(int argc, char** argv){
    ros::init(argc, argv, "runSimulationFromFile");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path");
    monolithic_pr2_planner_node::GetMobileArmPlan srv;

    std::vector<double> right_arm_start(7,0), left_arm_start(7,0), body_start(4,0);
    std::vector<double> right_arm_goal(7,0), left_arm_goal(7,0), body_goal(4,0);

    std::string input_env = string(argv[1]);
    FILE* path = fopen(input_env.c_str(), "r");
    ROS_INFO("Reading from file: %s", input_env.c_str());
    fscanf(path, "%lf %lf %lf %lf\n",
                       &body_start[0],
                       &body_start[1],
                       &body_start[2],
                       &body_start[3]);
    ROS_INFO(" Base start : %lf %lf %lf %lf",
                       body_start[0],
                       body_start[1],
                       body_start[2],
                       body_start[3]);

    fscanf(path, "%lf %lf %lf %lf %lf %lf %lf\n",
                    &left_arm_start[0],
                    &left_arm_start[1],
                    &left_arm_start[2],
                    &left_arm_start[3],
                    &left_arm_start[4],
                    &left_arm_start[5],
                    &left_arm_start[6]);
    ROS_INFO("Left arm start: %lf %lf %lf %lf %lf %lf %lf",
                    left_arm_start[0],
                    left_arm_start[1],
                    left_arm_start[2],
                    left_arm_start[3],
                    left_arm_start[4],
                    left_arm_start[5],
                    left_arm_start[6]);
    fscanf(path, "%lf %lf %lf %lf %lf %lf %lf\n", 
                    &right_arm_start[0],
                    &right_arm_start[1],
                    &right_arm_start[2],
                    &right_arm_start[3],
                    &right_arm_start[4],
                    &right_arm_start[5],
                    &right_arm_start[6]);
    ROS_INFO("Right arm start : %lf %lf %lf %lf %lf %lf %lf", 
                    right_arm_start[0],
                    right_arm_start[1],
                    right_arm_start[2],
                    right_arm_start[3],
                    right_arm_start[4],
                    right_arm_start[5],
                    right_arm_start[6]);
    fscanf(path, "%lf %lf %lf %lf\n",
                       &body_goal[0],
                       &body_goal[1],
                       &body_goal[2],
                       &body_goal[3]);
    ROS_INFO("Base goal : %lf %lf %lf %lf",
                       body_goal[0],
                       body_goal[1],
                       body_goal[2],
                       body_goal[3]);
    fscanf(path, "%lf %lf %lf %lf %lf %lf %lf\n",
                    &left_arm_goal[0],
                    &left_arm_goal[1],
                    &left_arm_goal[2],
                    &left_arm_goal[3],
                    &left_arm_goal[4],
                    &left_arm_goal[5],
                    &left_arm_goal[6]);
    ROS_INFO("Left arm goal : %lf %lf %lf %lf %lf %lf %lf",
                    left_arm_goal[0],
                    left_arm_goal[1],
                    left_arm_goal[2],
                    left_arm_goal[3],
                    left_arm_goal[4],
                    left_arm_goal[5],
                    left_arm_goal[6]);
    fscanf(path, "%lf %lf %lf %lf %lf %lf %lf\n", 
                    &right_arm_goal[0],
                    &right_arm_goal[1],
                    &right_arm_goal[2],
                    &right_arm_goal[3],
                    &right_arm_goal[4],
                    &right_arm_goal[5],
                    &right_arm_goal[6]);
    ROS_INFO("Right arm goal: %lf %lf %lf %lf %lf %lf %lf", 
                    right_arm_goal[0],
                    right_arm_goal[1],
                    right_arm_goal[2],
                    right_arm_goal[3],
                    right_arm_goal[4],
                    right_arm_goal[5],
                    right_arm_goal[6]);
    fclose(path);

    // RightContArmState rarm_goal(right_arm_goal);
    // LeftContArmState larm_goal(left_arm_goal);
    // ContBaseState base_goal(body_goal);

    // RobotState::setPlanningMode(monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE);


    srv.request.rarm_start = right_arm_start;
    srv.request.larm_start = left_arm_start;
    srv.request.body_start = body_start;
    srv.request.rarm_goal = right_arm_goal;
    srv.request.larm_goal = left_arm_goal;
    srv.request.body_goal = body_goal;

    geometry_msgs::PoseStamped pose;

    geometry_msgs::PoseStamped rarm_offset;
    rarm_offset.pose.position.x = 0;
    rarm_offset.pose.position.y = 0;
    rarm_offset.pose.orientation.z = 0;
    rarm_offset.pose.orientation.x = 0;
    rarm_offset.pose.orientation.y = 0;
    rarm_offset.pose.orientation.z = 0;
    rarm_offset.pose.orientation.w = 1;

    geometry_msgs::PoseStamped larm_offset;
    larm_offset.pose.position.x = 0;
    larm_offset.pose.position.y = 0;
    larm_offset.pose.orientation.z = 0;
    larm_offset.pose.orientation.x = 0;
    larm_offset.pose.orientation.y = 0;
    larm_offset.pose.orientation.z = 0;
    larm_offset.pose.orientation.w = 1;
    srv.request.rarm_object = rarm_offset;
    srv.request.larm_object = larm_offset;
    // larm_goal.setObjectOffset(larm_offset);
    // rarm_goal.setObjectOffset(rarm_offset);
    
    // RobotPosePtr goal_robot = boost::make_shared<RobotState>(base_goal,
        // rarm_goal, larm_goal);

    // ContObjectState obj_goal = goal_robot->getObjectStateRelMap();

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    
    KDL::Rotation rot =
    KDL::Rotation::RPY(0,0,M_PI/2);
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;

    srv.request.goal = pose;
    srv.request.initial_eps = 100;
    srv.request.final_eps = 100;
    srv.request.dec_eps = .1;
    srv.request.xyz_tolerance = .02;
    srv.request.roll_tolerance = .1;
    srv.request.pitch_tolerance = .1;
    srv.request.yaw_tolerance = .1;

    srv.request.allocated_planning_time = 60;

    srv.request.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;

    ROS_INFO("Sending request at : %s",
        boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::local_time()).c_str());
    if (client.call(srv))
    {
        ROS_INFO("called service");
        for (size_t i=0; i < srv.response.stats_field_names.size(); i++){
            ROS_INFO("%s: %f", srv.response.stats_field_names[i].c_str(),
                               srv.response.stats[i]);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}
