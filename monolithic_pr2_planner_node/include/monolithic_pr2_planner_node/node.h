#include <ros/ros.h>
// #include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/EnvironmentESP.h>
#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace monolithic_pr2_planner_node {
    class Node {
        public:
            Node(ros::NodeHandle nh);
        private:
            boost::shared_ptr<monolithic_pr2_planner::Environment> m_env;
            EnvInterfaces m_env_interface;
    };
} 
