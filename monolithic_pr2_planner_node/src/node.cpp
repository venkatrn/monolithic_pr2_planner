#include <monolithic_pr2_planner_node/node.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <log4cxx/logger.h>
#include <ros/console.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;

Node::Node(ros::NodeHandle nh) : m_env(new Environment(nh)), m_env_interface(m_env,
  nh){
    m_env_interface.bindPlanPathToEnv("/sbpl_planning/plan_path");
    m_env_interface.bindNavMapToTopic("/projected_map");
    // m_env_interface.bindNavMapToTopic("/map");
    m_env_interface.bindCollisionSpaceToTopic("collision_map_out");
    m_env_interface.bindExperimentToEnv("/sbpl_planning/run_simulation");
    m_env_interface.bindWriteExperimentToEnv("/sbpl_planning/generate_experiments_file");
    m_env_interface.bindDemoToEnv("/sbpl_planning/run_demo");
}


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
    ROS_DEBUG_NAMED(CONFIG_LOG, "configuration logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/initialization", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(INIT_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(INIT_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "initialization logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/collision_space", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(CSPACE_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(CSPACE_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "collision space logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/kinematics", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(KIN_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(KIN_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "kinematics logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/hashmanager", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(HASH_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(HASH_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "hashmanager logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/search", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(SEARCH_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(SEARCH_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "search logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/motionprimitives", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(MPRIM_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(MPRIM_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "search logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/heuristics", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(HEUR_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(HEUR_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "search logging level set to %s", level.c_str());

    nh.param<std::string>("debug/logging/pathpostprocessor", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(POSTPROCESSOR_LOG), level);
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner_node") + 
                                  std::string(".") + 
                                  std::string(POSTPROCESSOR_LOG), level);
    ROS_DEBUG_NAMED(CONFIG_LOG, "search logging level set to %s", level.c_str());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "monolithic_pr2_planner_node");
    ros::NodeHandle nh("~");
    setLoggersFromParamServer(nh);
    Node node(nh);
    ROS_INFO("Node is ready to receive planning requests.");
    ros::MultiThreadedSpinner spinner(4);//need 4 threads to catch a the interrupt (a thread for each blocking callback + the interrupt)
    spinner.spin();


    //ros::spin();
}
