#pragma once
#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner_node/CollisionSpaceInterface.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <Eigen/Core>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sbpl/planners/araplanner.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sbpl/planners/planner.h>

namespace monolithic_pr2_planner_node {
    struct InterfaceParams {
        std::string ref_frame;
    };

    class EnvInterfaces {
        public:
            EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::Environment> env);
            void getParams();
            bool planPathCallback(GetMobileArmPlan::Request &req, 
                                  GetMobileArmPlan::Response &res);
            void bindPlanPathToEnv(std::string service_name);
            bool bindCollisionSpaceToTopic(std::string topic_name);
            void bindNavMapToTopic(std::string topic_name);
            void packageStats(std::vector<std::string>& stat_names,
                              std::vector<double>& stats,
                              int solution_cost,
                              size_t solution_size);

        private:
            void loadNavMap(const nav_msgs::OccupancyGridPtr& map);
            void crop2DMap(const nav_msgs::OccupancyGridPtr& map,
                           double new_origin_x, double new_origin_y,
                           double width, double height,
                           vector<signed char>& final_map);
            ros::NodeHandle m_nodehandle;
            InterfaceParams m_params;
            boost::shared_ptr<monolithic_pr2_planner::Environment> m_env;
            tf::TransformListener m_tf;
            CollisionSpaceInterface m_collision_space_interface;
            ros::ServiceServer m_plan_service;
            std::unique_ptr<SBPLPlanner> m_planner;
            ros::Subscriber m_nav_map;
            ros::Publisher m_heur_map_pub;
            
            // Doesn't have the need to store the Costmap2D object. Simply has
// to update the costmap of the heurMgr.
            std::unique_ptr<costmap_2d::Costmap2DROS> m_costmap_ros;
            std::unique_ptr<costmap_2d::Costmap2DPublisher> m_costmap_publisher;
            

    };
}
