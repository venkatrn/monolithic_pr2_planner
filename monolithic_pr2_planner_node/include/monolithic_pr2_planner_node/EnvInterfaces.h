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
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <monolithic_pr2_planner/StatsWriter.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>

#include <monolithic_pr2_planner_node/ompl_pr2_planner.h>
#include <monolithic_pr2_planner/ExperimentFramework/randomStartGoalGenerator.h>

#include <full_body_controller/ExecutePath.h>

namespace monolithic_pr2_planner_node {
    struct InterfaceParams {
        std::string ref_frame;
        std::string controller_service;
        bool run_trajectory;
    };

    class EnvInterfaces {
        public:
            EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::Environment> env,
                ros::NodeHandle nh);
            void getParams();
            bool planPathCallback(GetMobileArmPlan::Request &req, 
                                  GetMobileArmPlan::Response &res);
            void bindPlanPathToEnv(std::string service_name);
            void bindExperimentToEnv(std::string service_name);
            void bindDemoToEnv(std::string service_name);
            bool bindCollisionSpaceToTopic(std::string topic_name);
            void bindNavMapToTopic(std::string topic_name);
            void packageStats(std::vector<std::string>& stat_names,
                              std::vector<double>& stats,
                              int solution_cost,
                              size_t solution_size,
                              double total_planning_time);
            void packageMHAStats(std::vector<std::string>& stat_names,
                              std::vector<double>& stats,
                              int solution_cost,
                              size_t solution_size,
                              double total_planning_time);
            bool experimentCallback(GetMobileArmPlan::Request &req,
                                    GetMobileArmPlan::Response &res);
            bool demoCallback(GetMobileArmPlan::Request &req,
                                    GetMobileArmPlan::Response &res);

        private:
            void loadNavMap(const nav_msgs::OccupancyGridPtr& map);
            void crop2DMap(const nav_msgs::MapMetaData& map_info, const std::vector<signed char>& v,
                           double new_origin_x, double new_origin_y,
                           double width, double height);
            bool runMHAPlanner(int planner_type,
                  std::string planner_prefix,
                  GetMobileArmPlan::Request &req,
                  GetMobileArmPlan::Response &res,
                  monolithic_pr2_planner::SearchRequestParamsPtr search_request,
                  int counter);
            void getRobotState(tf::TransformListener &tf_, BodyPose &body_pos, std::vector<double> &rangles, std::vector<double>
              &langles);
            double getJointAngle(std::string name, sensor_msgs::JointStateConstPtr
              msg);
            void runTrajectory(std::vector<monolithic_pr2_planner::FullBodyState>& states);

            ros::NodeHandle m_nodehandle;
            InterfaceParams m_params;
            boost::shared_ptr<monolithic_pr2_planner::Environment> m_env;
            tf::TransformListener m_tf;
            std::unique_ptr<CollisionSpaceInterface> m_collision_space_interface;
            ros::ServiceServer m_plan_service;
            ros::ServiceServer m_experiment_service;
            ros::ServiceServer m_demo_service;
            std::unique_ptr<SBPLPlanner> m_ara_planner;
            std::unique_ptr<SBPLPlanner> m_mha_planner;
            ros::Subscriber m_nav_map;
            ros::Publisher m_costmap_pub;
            std::vector<signed char> m_final_map;
            
            // Doesn't have the need to store the Costmap2D object. Simply has
// to update the costmap of the heurMgr.
            std::unique_ptr<costmap_2d::Costmap2DROS> m_costmap_ros;
            std::unique_ptr<costmap_2d::Costmap2DPublisher> m_costmap_publisher;


            std::unique_ptr<StartGoalGenerator> m_generator;
            std::unique_ptr<OMPLPR2Planner> m_rrt;
            std::unique_ptr<OMPLPR2Planner> m_prm;
            std::unique_ptr<OMPLPR2Planner> m_rrtstar;
            std::unique_ptr<OMPLPR2Planner> m_rrtstar_first_sol;
            StatsWriter m_stats_writer;
    };
}
