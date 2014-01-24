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

    // Config1
    // body_start[0] = 1.5;
    // body_start[1] = 2;
    // body_start[2] = .1;
    // body_start[3] = -M_PI;

    // Potential bug!
    body_start[0] = 5.0;
    body_start[1] = 1.0;
    body_start[2] = 0.1;
    body_start[3] = -M_PI;

    // body_start[0] = 5.0;
    // body_start[1] = 3.0;
    // body_start[2] = 0.1;
    // body_start[3] = -M_PI;

    srv.request.rarm_start = right_arm_start;
    srv.request.larm_start = left_arm_start;
    srv.request.body_start = body_start;

    // Config1
    // KDL::Rotation rot = KDL::Rotation::RPY(0,0,-M_PI);

    // Potential bug!
    KDL::Rotation rot = KDL::Rotation::RPY(0,0,-M_PI);


    // KDL::Rotation rot = KDL::Rotation::RPY(0,0,M_PI/2);
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);

    geometry_msgs::PoseStamped pose;

    // Config1
    // pose.pose.position.x = 3.2;
    // pose.pose.position.y = 1.9;
    // pose.pose.position.z = 1.1;

    // Potential bug!
    pose.pose.position.x = 2.7;
    pose.pose.position.y = 4.5;
    pose.pose.position.z = 0.7;

    // pose.pose.position.x = 2.7;
    // pose.pose.position.y = 4.8;
    // pose.pose.position.z = 0.7;
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
    srv.request.xyz_tolerance = .1;
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
