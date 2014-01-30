#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <angles/angles.h>
#include <boost/date_time/posix_time/posix_time.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "testPlanningRequest");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path");
    monolithic_pr2_planner_node::GetMobileArmPlan srv;

    std::vector<double> right_arm_start(7,0), left_arm_start(7,0), body_start(4,0);

    //right_arm_start[0] = -0.852858395281043;
    //right_arm_start[1] = 0.075369128335531;
    //right_arm_start[2] = 0.569623788333581;
    //right_arm_start[3] = -0.54373199879478;
    //right_arm_start[4] = angles::normalize_angle(-22.4372417947492);
    //right_arm_start[5] = -1.86517790099345;
    //right_arm_start[6] = angles::normalize_angle(8.571527760711906);

    //left_arm_start[0] = (0.8202582499433417);
    //left_arm_start[1] = (0.8174119480040183);
    //left_arm_start[2] = (1.175226343713942);
    //left_arm_start[3] = (-0.9897705605674373);
    //left_arm_start[4] = angles::normalize_angle(-4.586757274289091);
    //left_arm_start[5] = (-1.2633349113604524);
    //left_arm_start[6] = angles::normalize_angle(48.100199487910714);

    //works
    right_arm_start[0] = 0.337747;
    right_arm_start[1] = 0.361561;
    right_arm_start[2] = -2.034540;
    right_arm_start[3] = -1.317021;
    right_arm_start[4] = 2.922495;
    right_arm_start[5] = -1.046692;
    right_arm_start[6] = -1.136617;


    left_arm_start[0] = 0.038946;
    left_arm_start[1] = 1.214670;
    left_arm_start[2] = 1.396356;
    left_arm_start[3] = -1.197227;
    left_arm_start[4] = -4.616317;
    left_arm_start[5] = -0.988727;
    left_arm_start[6] = 1.175568;

    // Config1
    // body_start[0] = 1.5;
    // body_start[1] = 2;
    // body_start[2] = .1;
    // body_start[3] = -M_PI;
    
    // Config2 - cupboard
    // body_start[0] = 1.5;
    // body_start[1] = 3.0;
    // body_start[2] = 0.1;
    // body_start[3] = -M_PI;

 
    body_start[0] = 1.340000;
    body_start[1] = 3.740000;
    body_start[2] = 0.260000;
    body_start[3] = 4.319690;

    srv.request.rarm_start = right_arm_start;
    srv.request.larm_start = left_arm_start;
    srv.request.body_start = body_start;

    // Config1
    // KDL::Rotation rot = KDL::Rotation::RPY(0,0,-M_PI);

    // Config2 - cupboard
    // KDL::Rotation rot = KDL::Rotation::RPY(0,0,M_PI/2);


    KDL::Rotation rot = KDL::Rotation::RPY(0,1.276272,1.668971);
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);

    geometry_msgs::PoseStamped pose;

    // Config1
    // pose.pose.position.x = 3.2;
    // pose.pose.position.y = 1.9;
    // pose.pose.position.z = 1.1;

    // Config2 - cupboard
    // pose.pose.position.x = 3.0;
    // pose.pose.position.y = 4.9;
    // pose.pose.position.z = 1.0;

    pose.pose.position.x = 4.840000;
    pose.pose.position.y = 1.860000;
    pose.pose.position.z = 1.160000;
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
    srv.request.initial_eps = 100;
    srv.request.final_eps = 100;
    srv.request.dec_eps = .1;
    srv.request.xyz_tolerance = .02;
    srv.request.roll_tolerance = .1;
    srv.request.pitch_tolerance = .1;
    srv.request.yaw_tolerance = .1;

    srv.request.allocated_planning_time = 30;

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
