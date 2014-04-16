#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <angles/angles.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "simulation");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/run_simulation");
    monolithic_pr2_planner_node::GetMobileArmPlan srv;

    std::vector<double> right_arm_start(7, 0), left_arm_start(7, 0),
     body_start(4, 0);

    // right_arm_start[0] = -0.852858395281043;
    // right_arm_start[1] = 0.075369128335531;
    // right_arm_start[2] = 0.569623788333581;
    // right_arm_start[3] = -0.54373199879478;
    // right_arm_start[4] = angles::normalize_angle(-22.4372417947492);
    // right_arm_start[5] = -1.86517790099345;
    // right_arm_start[6] = angles::normalize_angle(8.571527760711906);

    // left_arm_start[0] = (0.8202582499433417);
    // left_arm_start[1] = (0.8174119480040183);
    // left_arm_start[2] = (1.175226343713942);
    // left_arm_start[3] = (-0.9897705605674373);
    // left_arm_start[4] = angles::normalize_angle(-4.586757274289091);
    // left_arm_start[5] = (-1.2633349113604524);
    // left_arm_start[6] = angles::normalize_angle(48.100199487910714);

    // works
    right_arm_start[0] = -0.034127;
    right_arm_start[1] = 0.309261;
    right_arm_start[2] = 0.000000;
    right_arm_start[3] = -1.614009;
    right_arm_start[4] = 2.987015;
    right_arm_start[5] = -1.413143;
    right_arm_start[6] = 2.889659;

    left_arm_start[0] = 0.137274;
    left_arm_start[1] = 0.314918;
    left_arm_start[2] = 0.185035;
    left_arm_start[3] = -1.662954;
    left_arm_start[4] = 2.923877;
    left_arm_start[5] = -1.305254;
    left_arm_start[6] = -0.370584;

    body_start[0] = 3.5;
    body_start[1] = 1;
    body_start[2] = .1;
    body_start[3] = 0;

    srv.request.rarm_start = right_arm_start;
    srv.request.larm_start = left_arm_start;
    srv.request.body_start = body_start;

    KDL::Rotation rot = KDL::Rotation::RPY(0,0,M_PI/2);
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 7;
    pose.pose.position.y = 4.3;
    pose.pose.position.z = 1.0;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;

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

    srv.request.goal = pose;
    srv.request.initial_eps = 50;
    srv.request.final_eps = 50;
    srv.request.dec_eps = .1;
    srv.request.xyz_tolerance = .02;
    srv.request.roll_tolerance = .1;
    srv.request.pitch_tolerance = .1;
    srv.request.yaw_tolerance = .1;

    srv.request.allocated_planning_time = 60;

    srv.request.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;

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
