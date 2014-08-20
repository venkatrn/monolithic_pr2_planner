#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <angles/angles.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sbpl/planners/mha_planner.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "testPlanningRequest");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path");
    monolithic_pr2_planner_node::GetMobileArmPlan srv;

    std::vector<double> right_arm_start(7,0), left_arm_start(7,0), body_start(4,0);

    // right_arm_start[0] = -0.852858395281043;
    // right_arm_start[1] = 0.075369128335531;
    // right_arm_start[2] = 0.569623788333581;
    // right_arm_start[3] = -0.54373199879478;
    // right_arm_start[4] = angles::normalize_angle(-22.4372417947492);
    // right_arm_start[5] = -1.86517790099345;
    // right_arm_start[6] = angles::normalize_angle(8.571527760711906);

    // tucked in left arm
    left_arm_start[0] = 0.138946;
    left_arm_start[1] = 1.214670;
    left_arm_start[2] = 1.396356;
    left_arm_start[3] = -1.197227;
    left_arm_start[4] = -4.616317;
    left_arm_start[5] = -0.988727;
    left_arm_start[6] = 1.175568;

    // dual arm
    right_arm_start[0] =  0.00477255721399; // shoulder pan
    right_arm_start[1] = 0.623628824976; // shoulder lift
    right_arm_start[2] = -0.0655416789175; // upper arm roll
    right_arm_start[3] = -1.95828661972; // elbow flex
    right_arm_start[4] = angles::normalize_angle(-53.5614619989); // forearm
// roll
    right_arm_start[5] = -2.01798572931; // wrist flex
    right_arm_start[6] = angles::normalize_angle(-84.879228644); // wrist
// roll

    // left_arm_start[0] = 0.2105629859;
    // left_arm_start[1] = 0.452384854808;
    // left_arm_start[2] = 0.634028804716;
    // left_arm_start[3] = -1.6515174823;
    // left_arm_start[4] = -angles::normalize_angle(-22.6616580629);
    // left_arm_start[5] = -1.92314290347;
    // left_arm_start[6] = angles::normalize_angle(2.85636911006);
 
    // body_start[0] = 3.500000;
    // body_start[1] = 1.00000;
    // body_start[2] = 0.260000;
    // body_start[3] = 0;          // theta
    body_start[0] = 4.0;
    body_start[1] = 1.0;
    body_start[2] = 0.260000;
    body_start[3] = 0;          // theta

    // body_start[0] = 4.00000;
    // body_start[1] = 2.060000;
    // body_start[2] = 0.260000;
    // body_start[3] = 0;          // theta

    srv.request.rarm_start = right_arm_start;
    srv.request.larm_start = left_arm_start;
    srv.request.body_start = body_start;
/*
    srv.request.underspecified_start = false;//TODO
    geometry_msgs::PoseStamped start_object_pose_rel_body;
    {
        // Set the orientation
        KDL::Rotation rot = KDL::Rotation::RPY(1.178097,0,0);
        double qx, qy, qz, qw;
        rot.GetQuaternion(qx, qy, qz, qw);
        start_object_pose_rel_body.pose.orientation.x = qx;
        start_object_pose_rel_body.pose.orientation.y = qy;
        start_object_pose_rel_body.pose.orientation.z = qz;
        start_object_pose_rel_body.pose.orientation.w = qw;

        // set the position. Remember that this is with respect to the body.
        start_object_pose_rel_body.pose.position.x = 0.5;
        start_object_pose_rel_body.pose.position.y = 0;
        start_object_pose_rel_body.pose.position.z = 0.1;
    }
    srv.request.start = start_object_pose_rel_body;
  */

    // Config1
    // KDL::Rotation rot = KDL::Rotation::RPY(0,0,-M_PI);

    // Config2 - cupboard
    // KDL::Rotation rot = KDL::Rotation::RPY(0,0,M_PI/2);

    // Goal pose
    KDL::Rotation rot = KDL::Rotation::RPY(0,0.0,0);
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);

    geometry_msgs::PoseStamped goal_pose;

    //goal_pose.pose.position.x = 8.00000;
    //goal_pose.pose.position.y = 2.30000;
    //goal_pose.pose.position.z = 1.2000;
    goal_pose.pose.position.x = 4.56;
    goal_pose.pose.position.y = 1.6835;
    goal_pose.pose.position.z = 1.102;

    // pose.pose.position.x = 6.440000;
    // pose.pose.position.y = 0.460000;
    // pose.pose.position.z = 1.18000;
    goal_pose.pose.orientation.x = qx;
    goal_pose.pose.orientation.y = qy;
    goal_pose.pose.orientation.z = qz;
    goal_pose.pose.orientation.w = qw;

    geometry_msgs::PoseStamped rarm_offset;
    rarm_offset.pose.position.x = 0.0;
    rarm_offset.pose.position.y = 0.0;
    rarm_offset.pose.position.z = 0.0;
    // rarm_offset.pose.position.y = 0.0;
    // rarm_offset.pose.position.z = 0.15;

    rot = KDL::Rotation::RPY(0, 0, 0);
    rot.GetQuaternion(qx, qy, qz, qw);

    rarm_offset.pose.orientation.x = qx;
    rarm_offset.pose.orientation.y = qy;
    rarm_offset.pose.orientation.z = qz;
    rarm_offset.pose.orientation.w = qw;

    geometry_msgs::PoseStamped larm_offset;
    larm_offset.pose.position.x = 0.0;
    larm_offset.pose.position.y = 0.0;
    larm_offset.pose.position.z = 0.0;
    // larm_offset.pose.position.y = 0.0;
    // larm_offset.pose.position.z = -0.15;
    rot = KDL::Rotation::RPY(0, 0, 0);
    rot.GetQuaternion(qx, qy, qz, qw);
    larm_offset.pose.orientation.x = qx;
    larm_offset.pose.orientation.y = qy;
    larm_offset.pose.orientation.z = qz;
    larm_offset.pose.orientation.w = qw;
    srv.request.rarm_object = rarm_offset;
    srv.request.larm_object = larm_offset;

    srv.request.goal = goal_pose;
    srv.request.initial_eps = 2.0;
    srv.request.final_eps = 2.0;
    srv.request.dec_eps = .1;
    srv.request.xyz_tolerance = .02;
    srv.request.roll_tolerance = .1;
    srv.request.pitch_tolerance = .1;
    srv.request.yaw_tolerance = .1;

    srv.request.allocated_planning_time = 30;

    srv.request.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;
    srv.request.planner_type = mha_planner::PlannerType::SMHA;

     srv.request.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;
    //srv.request.planning_mode =
    //monolithic_pr2_planner::PlanningModes::DUAL_ARM_MOBILE;

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
