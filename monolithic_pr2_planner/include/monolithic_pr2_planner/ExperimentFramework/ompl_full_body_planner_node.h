/*
 * Copyright (c) 2011, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Benjamin Cohen  */

#ifndef __SBPL_TWO_ARM_PLANNER_NODE_H_
#define __SBPL_TWO_ARM_PLANNER_NODE_H_

#define USE_LEARNING 0

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <angles/angles.h>
#include <ompl_full_body_planner_node/visualize_arm.h>
#include <sbpl_arm_planner/body_pose.h>
#include <sbpl_two_arm_planner_node/GetTwoArmPlan.h>
#include <sbpl_two_arm_planner/environment_dualrobarm3d.h>
#include <pviz/pviz.h>
//#include <olcg_full_body/olcg_full_body_optimized.h>

/** Messages **/
#include <geometry_msgs/Pose.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <kinematics_msgs/GetPositionFK.h>

#include <ros/console.h>
#include <log4cxx/logger.h>

//OMPL
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "ompl/base/goals/GoalState.h"

#define X_MIN 0
#define X_MAX 7.2
#define Y_MIN 0
#define Y_MAX 6.3
#define Z_MIN 0
#define Z_MAX 2


struct State {
    float torso;
    float arm_x;
    float arm_y;
    float arm_z;
    float arm_yaw;
    float free_angle_left;
    float free_angle_right;
    float base_x;
    float base_y;
    float base_yaw;

};

struct Region {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
};


//Define omplFullBodyCollisionChecker (the full body collision checker class), a StateValidityChecker
class omplFullBodyCollisionChecker : public ompl::base::StateValidityChecker
{
  public:
    omplFullBodyCollisionChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si){}

    PViz pviz_;
    vector<Region> regions;
    void initialize(sbpl_two_arm_planner::EnvironmentDUALROBARM3D* e, sbpl_two_arm_planner::SBPLDualCollisionSpace* c){
      env_ = e;
      cspace_ = c;
      num = new int[4];
    }

    void readFile(char filename[], std::vector<std::pair<State, State> >& pairs);
    void initializeRegions(std::string file);
    bool generateRandomValidState(State& s, vector<double>& arm_right, vector<double>& arm_left, int idx, int region_id=0);
    void generateRandomState(State& s, int region_id);
    inline double randomDouble(double min, double max);
    bool isValidVH(State state, vector<double>& arm_right, vector<double>& arm_left) {
      vector<double> wpose(12,0);

      wpose[0] = state.arm_x;
      wpose[1] = state.arm_y;
      wpose[2] = state.arm_z;
      wpose[3] = 0;
      wpose[4] = 0;
      wpose[5] = state.arm_yaw;
      wpose[6] = state.free_angle_right;
      wpose[7] = state.free_angle_left;
      wpose[8] = state.base_x;
      wpose[9] = state.base_y;
      wpose[10] = state.torso;
      wpose[11] = state.base_yaw;

      arm_right[2] = state.free_angle_right;
      arm_left[2] = state.free_angle_left;

      //num_calls++;
      if(env_->convertWorldPoseToAngles(wpose, arm_right, arm_left, false)){
        BodyPose body;
        body.x = state.base_x;
        body.y = state.base_y;
        body.theta = state.base_yaw;
        body.z = state.torso;
        //pviz_.visualizeRobot(arm_right, arm_left, body, 128, "waypoint", 0);
        //char shit;
        //cin >> shit;
        
        unsigned char dist_temp;
        int debug_code_;
        bool valid = cspace_->checkAllMotion(arm_left,arm_right,body,true,dist_temp,debug_code_);
        if(valid){
          //num_valid++;
          //ROS_INFO("state is valid");
        }
        else{
          //fail_collision++;
          //ROS_INFO("state is not valid");
        }
        return valid;
      }
      //fail_ik++;
      //ROS_INFO("IK is not valid");
      return false;
    }





    virtual bool isValid(const ompl::base::State *state) const
    {
      const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);
      
      vector<double> wpose(12,0);
      wpose[0] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1]; //arm x
      wpose[1] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2]; //arm y
      wpose[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3]; //arm z
      wpose[3] = 0; //roll
      wpose[4] = 0; //pitch
      wpose[5] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4]; //yaw
      wpose[6] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6]; //right arm free angle
      wpose[7] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5]; //left arm free angle
      wpose[8] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getX(); //base x
      wpose[9] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getY(); //base y
      wpose[10] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0]; //spine z
      wpose[11] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw(); //base yaw

      /*
      wpose[0] = 0.82; //(*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1];
      wpose[1] = 0.16; //(*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2];
      wpose[2] = -0.44; //(*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3];
      wpose[3] = 0;
      wpose[4] = 0;
      wpose[5] = 0; //(*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4];
      wpose[6] = 0; //(*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6];
      wpose[7] = 0.314159; //(*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5];
      wpose[8] = 0; //s->as<ompl::base::SE2StateSpace::StateType>(1)->getX();
      wpose[9] = 0; //s->as<ompl::base::SE2StateSpace::StateType>(1)->getY();
      wpose[10] = 0; //(*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0];
      wpose[11] = 0; //s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw();
      */

      vector<double> arm0(7,0); //right arm angles
      vector<double> arm1(7,0); //left arm angles
      /*
      arm0[0] = 0.408002;
      arm0[1] = 0.768743;
      arm0[2] = 0.000000;
      arm0[3] = -0.310828;
      arm0[4] = 0.774167;
      arm0[5] = -0.603510;
      arm0[6] = -0.677818;

      arm1[0] = 0.252906;
      arm1[1] = 0.984174;
      arm1[2] = 0.314159;
      arm1[3] = -0.728816;
      arm1[4] = -0.041016;
      arm1[5] = -0.276538;
      arm1[6] = -0.132454;
      */

      arm1[0] = 1.1993011559598821;
      arm1[1] = 0.3639994180419307;
      arm1[2] = 1.1930743691585055;
      arm1[3] = -1.673176609553904;
      arm1[4] = 1.9162929181216546;
      arm1[5] = -1.681316333341242;
      arm1[6] = -4.385533351016869;

      arm0[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6];
      arm1[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5];

      //num_calls++;
      num[0]++;
      if(env_->convertWorldPoseToAngles(wpose, arm0, arm1, false)){
        ROS_DEBUG("IK is valid");
        BodyPose body;
        body.x = s->as<ompl::base::SE2StateSpace::StateType>(1)->getX();
        body.y = s->as<ompl::base::SE2StateSpace::StateType>(1)->getY();
        body.theta = s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw();
        body.z = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0];
        
        unsigned char dist_temp;
        int debug_code_;
        bool valid = cspace_->checkAllMotion(arm1,arm0,body,true,dist_temp,debug_code_);
        if(valid){
          //num_valid++;
          num[3]++;
          //ROS_DEBUG("state is valid");
        }
        else{
          //fail_collision++;
          num[2]++;
          //ROS_WARN("state is not valid");
        }
        return valid;
      }
      //fail_ik++;
      num[1]++;
      //ROS_WARN("IK is not valid");
      return false;
    }

    void reset_count(){
      num[0] = 0;
      num[1] = 0;
      num[2] = 0;
      num[3] = 0;
      //num_calls = 0;
      //fail_ik = 0;
      //fail_collision = 0;
      //num_valid = 0;
    }

    void print_checks(){
      //printf("num_calls=%d failed_ik=%d failed_collision=%d num_valid=%d\n",num_calls,fail_ik,fail_collision,num_valid);
      printf("num_calls=%d failed_ik=%d failed_collision=%d num_valid=%d\n",num[0],num[1],num[2],num[3]);
    }

  private:
    int* num;
    //int num_calls;
    //int fail_ik;
    //int fail_collision;
    //int num_valid;
    sbpl_two_arm_planner::EnvironmentDUALROBARM3D* env_;
    sbpl_two_arm_planner::SBPLDualCollisionSpace* cspace_;
};

namespace sbpl_two_arm_planner {

class OMPLFullBodyPlannerNode
{
  public:
    OMPLFullBodyPlannerNode();
    ~OMPLFullBodyPlannerNode();
    
    bool init();
    
    int run();

    void changeLoggerLevel(std::string name, std::string level);

    bool setStart(geometry_msgs::Pose start, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object);

    bool setGoalPosition(geometry_msgs::Pose goal, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object, std::vector<double> &goal_tolerance);

    bool planToPosition(sbpl_two_arm_planner_node::GetTwoArmPlan::Request &req, sbpl_two_arm_planner_node::GetTwoArmPlan::Response &res);

    //void sendArmsToStart(geometry_msgs::Pose &object_pose);

  private:
    std::string planner_string_;
    int planner_id_;
    int filenum_;
    ompl::base::StateSpacePtr fullBodySpace;
    ompl::base::ProblemDefinition* pdef;
    ompl::base::Planner* planner;
    ompl::geometric::PathSimplifier* pathSimplifier;
    omplFullBodyCollisionChecker* ompl_checker;

    ros::NodeHandle node_handle_, root_handle_;
    ros::ServiceServer planning_service_;
    ros::Publisher pcl_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber collision_object_subscriber_;
    ros::Subscriber object_subscriber_;
    ros::Subscriber path_subscriber_;
    message_filters::Subscriber<arm_navigation_msgs::CollisionMap> collision_map_subscriber_;
    tf::MessageFilter<arm_navigation_msgs::CollisionMap> *collision_map_filter_;

    boost::mutex joint_states_mutex_;
    boost::mutex collision_map_mutex_;

    std::string left_fk_service_name_;
    std::string right_fk_service_name_;
    std::string left_ik_service_name_;
    std::string right_ik_service_name_;

    double allocated_time_;
    double waypoint_time_;
    double env_resolution_;

    clock_t totalPlanTime;

    /* params */
    bool generate_goals_;
    bool forward_search_;
    bool search_mode_;
    bool visualize_expanded_states_;
    bool visualize_heuristic_;
    bool visualize_goal_;
    bool visualize_end_effector_path_;
    bool planner_initialized_;
    bool print_path_;
    bool visualize_trajectory_;
    bool visualize_collision_model_trajectory_;
    bool use_first_solution_;
    bool use_collision_map_from_sensors_;
    bool use_shortened_path_;
    bool use_inner_circle_;
    bool visualize_heuristic_grid_;
    int throttle_;
    int num_joints_;
    double object_radius_;

    std::string stl_file_;
    std::string region_file_;
    std::string start_goal_file_;
    std::string collision_map_topic_;
    std::string robot_description_;
    std::string reference_frame_;
    std::string map_frame_;
    std::string planning_joint_;
    std::string arm_name_;
    std::string left_arm_description_filename_;
    std::string right_arm_description_filename_;
    std::string mprims_filename_;
    std::string base_mprims_filename_;
    std::vector<std::string> ljoint_names_;
    std::vector<std::string> rjoint_names_;
    std::vector<std::vector<double> > dpath_;
    std::vector<std::vector<double> > dpath0_;
    std::vector<std::vector<double> > dpath1_;
    std::vector<int> solution_state_ids_;
    std::vector<int> solution_state_ids_short_;
    std::vector<double> rangles_;
    std::vector<double> langles_;
    double torso_lift_;
    double head_pan_;
    double head_tilt_;
    BodyPose body_pos_;

    geometry_msgs::Pose object_start_;
    geometry_msgs::Pose rarm_object_offset_;
    geometry_msgs::Pose larm_object_offset_;

    /* planner & environment */
    MDPConfig mdp_cfg_;
#if USE_LEARNING
    OLCG_Body sbpl_arm_env_;
#else
    sbpl_two_arm_planner::EnvironmentDUALROBARM3D sbpl_arm_env_;
#endif
    SBPLPlanner *planner_;
    sbpl_two_arm_planner::SBPLDualCollisionSpace* cspace_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    sbpl_two_arm_planner::VisualizeArm* laviz_;
    sbpl_two_arm_planner::VisualizeArm* raviz_;

    /* transforms and kinematics */
    tf::TransformListener tf_;
    tf::StampedTransform transform_;
    tf::StampedTransform base_map_transform_;
    KDL::Frame kdl_transform_;

    //Arm* rarm_;
    //Arm* larm_;

    /* debug */
    double x_min_, x_max_, x_inc_;
    double y_min_, y_max_, y_inc_;
    double z_min_, z_max_, z_inc_;
    std::string succ_log_name_;
    std::string succ_log_level_;
    std::vector<std::string> stats_field_names_;
    std::vector<double> stats_;
    std::vector<std::string> debug_code_names_;
    PViz pviz_;

    /* attached objects */
    bool attached_object_;

    /* collision objects */
    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;

    /** callbacks **/
    void readCollisionMapFromFile(std::string file);
    void updateMapFromCollisionMap(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);

    void collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);

    void collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object);

    void attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object);

    //void addPathCallback(const full_body_demonstration::FullBodyTrajectoryConstPtr &path);

    /** planning **/
    bool initializePlannerAndEnvironment();

    bool plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    bool isGoalConstraintSatisfied(const std::vector<double> &rangles, const std::vector<double> &langles, const geometry_msgs::Pose &goal);

    void setArmToMapTransform(std::string &map_frame);
    
    /** kinematics **/
    bool computeFK(const std::vector<double> &jnt_pos, std::string arm_name, geometry_msgs::Pose &pose);

    void getRobotState(BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles);

    /** visualizations **/
    void visualizeExpansions();

    void visualizeUniqueExpansions();

    void displayShortestPath();

    void printPath(FILE* fOut, const std::vector<std::vector<double> > path);

    void printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    void visualizeGoal(geometry_msgs::Pose goal);

    void visualizeCollisionObjects();

    void visualizeGoalPosition(const arm_navigation_msgs::Constraints &goal);
    
    void visualizeEndEffectorPath();
    
    void visualizeHeuristicInfo();

    void visualizeExpansionsPerHValue();

    void visualizeHeuristicGrid();

    void visualizeAttachedObject();

    void visualizeCollisionObject(const arm_navigation_msgs::CollisionObject &object);

    void visualizeAttachedObjectPath();

    void visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    void printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text);

    void visualizeObjectPath();
};

}

#endif
