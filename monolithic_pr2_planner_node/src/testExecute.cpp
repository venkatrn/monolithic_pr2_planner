#include <monolithic_pr2_planner/Environment.h>
#include <kdl/frames.hpp>
#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Pose.h>

#include <log4cxx/logger.h>

using namespace monolithic_pr2_planner;
using namespace boost;

void changeLoggerLevel(std::string name, std::string level)
{
    std::string logger_name = name;
    log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);
    // Set the logger for this package to output all statements
    if (level.compare("debug") == 0)
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    else if (level.compare("warn") == 0)
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Warn]);
    else if (level.compare("info") == 0)
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
    else if (level.compare("fatal") == 0){
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Fatal]);
    }
}

// TODO clean this up
void setLoggersFromParamServer(ros::NodeHandle nh){
    std::string level;
    nh.param<std::string>("debug/logging/configuration",
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") +
                                  std::string(".") +
                                  std::string(CONFIG_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") +
                                  std::string(".") +
                                  std::string(CONFIG_LOG), level);
    ROS_INFO_NAMED(CONFIG_LOG, "configuration logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/initialization",
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") +
                                  std::string(".") +
                                  std::string(INIT_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") +
                                  std::string(".") +
                                  std::string(INIT_LOG), level);
    ROS_INFO_NAMED(CONFIG_LOG, "initialization logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/collision_space",
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") +
                                  std::string(".") +
                                  std::string(CSPACE_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") +
                                  std::string(".") +
                                  std::string(CSPACE_LOG), level);
    ROS_INFO_NAMED(CONFIG_LOG, "collision space logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/kinematics",
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") +
                                  std::string(".") +
                                  std::string(KIN_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") +
                                  std::string(".") +
                                  std::string(KIN_LOG), level);
    ROS_INFO_NAMED(CONFIG_LOG, "kinematics logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/hashmanager",
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") +
                                  std::string(".") +
                                  std::string(HASH_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") +
                                  std::string(".") +
                                  std::string(HASH_LOG), level);
    ROS_INFO_NAMED(CONFIG_LOG, "hashmanager logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/search",
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") +
                                  std::string(".") +
                                  std::string(SEARCH_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") +
                                  std::string(".") +
                                  std::string(SEARCH_LOG), level);
    ROS_INFO_NAMED(CONFIG_LOG, "search logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/motionprimitives",
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") +
                                  std::string(".") +
                                  std::string(MPRIM_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") +
                                  std::string(".") +
                                  std::string(MPRIM_LOG), level);
    ROS_INFO_NAMED(CONFIG_LOG, "search logging level set to %s", level.c_str());
}
int main(int argc, char** argv){
    ros::init(argc, argv, "testExecute");
    ros::NodeHandle nh("testExecute");
    setLoggersFromParamServer(nh);
    std::vector<double> right_arm_start(7,0), left_arm_start(7,0), body_start(4,0);

    right_arm_start[0] = 0.052858395281043;
    right_arm_start[1] = 0.075369128335531;
    right_arm_start[2] = 0.569623788333581;
    right_arm_start[3] = -0.54373199879478;
    right_arm_start[4] = -22.4372417947492;
    right_arm_start[5] = -1.86517790099345;
    right_arm_start[6] = 8.571527760711906;



    body_start[0] = 5;
    body_start[1] = 1;
    body_start[2] = .1;
    body_start[3] = 0;

    boost::shared_ptr<monolithic_pr2_planner::Environment> m_env(new Environment(nh));

    boost::shared_ptr<arm_navigation_msgs::CollisionMap const> cmap = ros::topic::waitForMessage<arm_navigation_msgs::CollisionMap>("/collision_map_out");
    m_env->getCollisionSpace()->updateMap(*cmap);


    std::unique_ptr<ARAPlanner> m_planner;
    bool forward_search = true;
    m_planner.reset(new ARAPlanner(m_env.get(), forward_search));
    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = 10;
    search_request->final_epsilon = 9;
    search_request->decrement_epsilon = .1;
    search_request->obj_goal= ContObjectState(6.2,.78,.9,0,0,0);
    // BROKEN TODO search_request->obj_goal= ContObjectState(5.9,.78,.9,0,0,0);
    search_request->base_start = ContBaseState(5,1,.1,0);
    search_request->left_arm_start = LeftContArmState(left_arm_start);
    search_request->right_arm_start = RightContArmState(right_arm_start);

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(-.18);
    rarm_offset.p.y(-1);
    rarm_offset.p.z(0);
    larm_offset.p.x(-.18);
    larm_offset.p.y(1);
    larm_offset.p.z(0);

    rarm_offset.M = KDL::Rotation::Quaternion(0, 0, 0, 1);
    larm_offset.M = KDL::Rotation::Quaternion(0, 0, 0, 1);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = .1;
    search_request->roll_tolerance = .1;
    search_request->pitch_tolerance = .1;
    search_request->yaw_tolerance = .1;

    int start_id, goal_id;
    bool retVal = m_env->configureRequest(search_request, start_id, goal_id);

    m_planner->set_initialsolution_eps(search_request->initial_epsilon);
    bool return_first_soln = false;
    m_planner->set_search_mode(return_first_soln);
    m_planner->set_start(start_id);
    ROS_INFO("setting goal id to %d", goal_id);
    m_planner->set_goal(goal_id);
    m_planner->force_planning_from_scratch();
    vector<int> soln;
    int soln_cost;
    bool success = m_planner->replan(20.0, &soln, &soln_cost);
    if (success)
        vector<FullBodyState> states =  m_env->reconstructPath(soln);
}
