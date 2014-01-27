/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
/** \author Benjamin Cohen */

#include <ompl_full_body_planner_node/ompl_full_body_planner_node.h>
//#include <ompl_full_body_planner_node/planner_id_.h>
#include <LinearMath/btVector3.h>
#include <sbpl_geometry_utils/mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

void omplFullBodyCollisionChecker::readFile(char filename[], std::vector<std::pair<State, State> >& pairs){
    std::ifstream file(filename);
    std::string line;
    while(std::getline(file, line)){
        istringstream iss(line);
        std::pair<State, State> point;
        vector<float> pts;
        float value;
        while (iss >> value){
            pts.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
        point.first.torso = pts[0];
        point.first.arm_x = pts[1];
        point.first.arm_y = pts[2];
        point.first.arm_z = pts[3];
        point.first.arm_yaw = pts[4];
        point.first.free_angle_left = pts[5];
        point.first.free_angle_right = pts[6];
        point.first.base_x = pts[7];
        point.first.base_y = pts[8];
        point.first.base_yaw = pts[9];
        point.second.torso = pts[10];
        point.second.arm_x = pts[11];
        point.second.arm_y = pts[12];
        point.second.arm_z = pts[13];
        point.second.arm_yaw = pts[14];
        point.second.free_angle_left = pts[15];
        point.second.free_angle_right = pts[16];
        point.second.base_x = pts[17];
        point.second.base_y = pts[18];
        point.second.base_yaw = pts[19];
        pairs.push_back(point);
    }
}

void omplFullBodyCollisionChecker::initializeRegions(std::string file){
  FILE* fin = fopen(file.c_str(), "r");
  while(1){
    Region region;
    int ret_code = fscanf(fin, "%lf, %lf, %lf, %lf, %lf, %lf\n",&(region.x_min),
                                                                &(region.x_max),
                                                                &(region.y_min),
                                                                &(region.y_max),
                                                                &(region.z_min),
                                                                &(region.z_max));
    if (ret_code < 6){
        return;
    }
    regions.push_back(region);
  }
}

bool omplFullBodyCollisionChecker::generateRandomValidState(State& state, 
        vector<double>& arm_right, vector<double>& arm_left, int idx, int region_id){
  arm_right.resize(7);
  arm_left.resize(7);
  arm_left[0] = 1.1993011559598821;
  arm_left[1] = 0.3639994180419307;
  arm_left[2] = 1.1930743691585055;
  arm_left[3] = -1.673176609553904;
  arm_left[4] = 1.9162929181216546;
  arm_left[5] = -1.681316333341242;
  arm_left[6] = -4.385533351016869;

  int counter = 0;
  while(1){
    counter++;
    //if (counter % 1000 == 0)
    //    ROS_INFO("up to iteration %d while searching for valid state", counter);

    generateRandomState(state, region_id);
    BodyPose bp;
    BodyPose initial_body;
    BodyCell intermediate_body;
    BodyPose corrected_body;
    initial_body.x = state.base_x;
    initial_body.y = state.base_y;
    initial_body.z = state.torso;
    initial_body.theta = state.base_yaw;
    //ROS_INFO("initial unadjusted state is %f %f %f %f", 
    //        initial_body.x,
    //        initial_body.y,
    //        initial_body.z,
    //        initial_body.theta);
  
    env_->worldToDiscBody(initial_body, &intermediate_body);
    env_->discToWorldBody(intermediate_body, &bp);
    state.base_x = bp.x;
    state.base_y = bp.y;
    state.torso = bp.z;
    state.base_yaw = bp.theta;
    //ROS_INFO("adjusted state is %f %f %f %f", 
    //        bp.x,
    //        bp.y,
    //        bp.z,
    //        bp.theta);

    if(isValidVH(state, arm_right, arm_left)){
      short unsigned int x,y,z,yaw;
      env_->computeObjectPose(bp, arm_right, x, y, z, yaw);
      double obj_x, obj_y, obj_z;
      vector<double> ik_solution(7,0);

      env_->discToWorldXYZ(x,y,z,obj_x,obj_y,obj_z,true);

      //iterate through our desired regions
      //if the object location is in one of our regions then we found a valid state
      if (region_id != -1){
          if(obj_x >= regions[region_id].x_min && obj_x <= regions[region_id].x_max &&
             obj_y >= regions[region_id].y_min && obj_y <= regions[region_id].y_max &&
             obj_z >= regions[region_id].z_min && obj_z <= regions[region_id].z_max){
            
            //pviz_.visualizeRobot(arm_right, arm_left, bp, 128, "valid_state"+boost::lexical_cast<std::string>(idx),idx);
            return true;
          }
      } else {
          if(obj_x >= X_MIN && obj_x <= X_MAX &&
             obj_y >= Y_MIN && obj_y <= Y_MAX &&
             obj_z >= Z_MIN && obj_z <= Z_MAX){
              return true;
          }
      }
    }
  }
  // only reaches here if no state generated
  ROS_WARN("Couldn't find state for region %d", region_id);
  return false;
}

void omplFullBodyCollisionChecker::generateRandomState(State& s, int region_id){
  s.torso = randomDouble(0.0, 0.3);
  s.arm_x = randomDouble(0.35, 1.2);
  s.arm_y = randomDouble(-0.6, 0.6);
  s.arm_z = randomDouble(-0.6, 0.6);
  s.arm_yaw = randomDouble(-1.396, 1.396);
  s.free_angle_left = randomDouble(-.65, 3.75);
  s.free_angle_right = randomDouble(-3.75, .65);

  // if region_id = -1, then we do a random sample across the entire state space
  double x_lower_bound = 0;
  double x_upper_bound = X_MAX;
  double y_lower_bound = 0;
  double y_upper_bound = Y_MAX;

  if (region_id != -1){
      int padding = 1;
      x_lower_bound = (regions[region_id].x_min-padding < 0) ? 0 : regions[region_id].x_min-padding;
      x_upper_bound = (regions[region_id].x_max+padding > X_MAX) ? X_MAX : regions[region_id].x_max+padding;
      y_lower_bound = (regions[region_id].y_min-padding < 0) ? 0 : regions[region_id].y_min-padding;
      y_upper_bound = (regions[region_id].y_max+padding > Y_MAX) ? Y_MAX : regions[region_id].y_max+padding;
  }
  s.base_x = randomDouble(x_lower_bound, x_upper_bound);
  s.base_y = randomDouble(y_lower_bound, y_upper_bound);
  s.base_yaw = randomDouble(-M_PI, M_PI);
}

inline double omplFullBodyCollisionChecker::randomDouble(double min, double max){
  return min + (max-min) * ( double(rand()) / RAND_MAX );
}

namespace sbpl_two_arm_planner {

/** Initializers -------------------------------------------------------------*/
OMPLFullBodyPlannerNode::OMPLFullBodyPlannerNode() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1), collision_map_filter_(NULL),grid_(NULL),laviz_(NULL),raviz_(NULL)
{
  std::string filename = "/tmp/ompl_stats_" + planner_string_ + ".csv";
  FILE* stat_out = fopen(filename.c_str(),"a");
  fclose(stat_out);
  filenum_ = 0;
  planner_initialized_ = false;
  forward_search_ = true;
  planning_joint_ = "r_wrist_roll_link";
  allocated_time_ = 60.0;
  env_resolution_ = 0.02;

  langles_.resize(7,0);
  rangles_.resize(7,0);

  ljoint_names_.resize(7);
  rjoint_names_.resize(7);

  rarm_object_offset_.orientation.x = 0.0;
  rarm_object_offset_.orientation.y = 0.0;
  rarm_object_offset_.orientation.z = 0.0;
  rarm_object_offset_.orientation.w = 1.0;
  larm_object_offset_.orientation.x = 0.0;
  larm_object_offset_.orientation.y = 0.0;
  larm_object_offset_.orientation.z = 0.0;
  larm_object_offset_.orientation.w = 1.0;

  x_min_ = 0.0;
  x_max_ = 2.0;
  x_inc_ = 0.04;
  y_min_ = 0.0;
  y_max_ = 2.0;
  y_inc_ = 0.04;
  z_min_ = 0.7; 
  z_max_ = 1.1;
  z_inc_ = 0.04;

  use_inner_circle_ = true;
  succ_log_name_ = "succ";
  succ_log_level_ = "info";

  attached_object_ = false;
  //rarm_ = new Arm(std::string("right"));
  //larm_ = new Arm(std::string("left"));

  //TODO: These should be moved to be with the enum DebugCodes
  debug_code_names_.push_back("valid successor");
  debug_code_names_.push_back("collision between arms");
  debug_code_names_.push_back("right arm in collision");
  debug_code_names_.push_back("left arm in collision");
  debug_code_names_.push_back("attached object in collision"); 
  debug_code_names_.push_back("right ik fail ik search success");
  debug_code_names_.push_back("right ik fail ik search fail");
  debug_code_names_.push_back("left ik fail ik search success");
  debug_code_names_.push_back("left ik fail ik search fail");
  debug_code_names_.push_back("invalid right shoulder pan");
  debug_code_names_.push_back("invalid right shoulder pitch");
  debug_code_names_.push_back("invalid right upper arm roll");
  debug_code_names_.push_back("invalid right elbow flex");
  debug_code_names_.push_back("invalid right forearm roll");
  debug_code_names_.push_back("invalid right wrist pitch");
  debug_code_names_.push_back("invalid right wrist roll");
  debug_code_names_.push_back("invalid left shoulder pan");
  debug_code_names_.push_back("invalid left shoulder pitch");
  debug_code_names_.push_back("invalid left upper arm roll");
  debug_code_names_.push_back("invalid left elbow flex");
  debug_code_names_.push_back("invalid left forearm roll");
  debug_code_names_.push_back("invalid left wrist pitch");
  debug_code_names_.push_back("invalid left wrist roll");

}

OMPLFullBodyPlannerNode::~OMPLFullBodyPlannerNode()
{
  if(laviz_ != NULL)
    delete laviz_;
  if(raviz_ != NULL)
    delete raviz_;
  if(collision_map_filter_ != NULL)
    delete collision_map_filter_;
  if(planner_ != NULL)
    delete planner_;
/*
  if(rarm_ != NULL)
    delete rarm_;
  if(larm_ != NULL)
    delete larm_;
*/
}

bool OMPLFullBodyPlannerNode::init()
{
  //planner
  node_handle_.param("planner/search_mode", search_mode_, true); //true: stop after first solution
  node_handle_.param("planner/allocated_time", allocated_time_, 60.0);
  node_handle_.param("planner/object_radius", object_radius_, 0.10);
  node_handle_.param<std::string>("planner/left_arm_description_file", left_arm_description_filename_, "");
  node_handle_.param<std::string>("planner/right_arm_description_file", right_arm_description_filename_, "");
  node_handle_.param<std::string>("planner/motion_primitive_file", mprims_filename_, "");
  node_handle_.param<std::string>("planner/base_motion_primitive_file", base_mprims_filename_, "");
  node_handle_.param("planner/use_shortened_path", use_shortened_path_, false);
  node_handle_.param("planner/use_inner_circle_of_object", use_inner_circle_, true);
  node_handle_.param("debug/print_out_path", print_path_, true);
  node_handle_.param<std::string>("debug/succesors_logger_level", succ_log_level_, "info");
  node_handle_.param("robot/waypoint_time", waypoint_time_, 0.2);
  node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("map"));
  node_handle_.param<std::string>("left_fk_service_name", left_fk_service_name_, "pr2_left_arm_kinematics/get_fk");
  node_handle_.param<std::string>("left_ik_service_name", left_ik_service_name_, "pr2_left_arm_kinematics/get_ik");
  node_handle_.param<std::string>("right_fk_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_fk");
  node_handle_.param<std::string>("right_ik_service_name", right_ik_service_name_, "pr2_right_arm_kinematics/get_ik");

  node_handle_.param("generate_goals", generate_goals_, false);
  node_handle_.param("planner_id", planner_id_, 0);

  node_handle_.param ("use_collision_map_from_sensors", use_collision_map_from_sensors_, true);
  node_handle_.param ("right_arm_pose_on_object_x", rarm_object_offset_.position.x, -0.2);
  node_handle_.param ("right_arm_pose_on_object_y", rarm_object_offset_.position.y, -0.1);
  node_handle_.param ("right_arm_pose_on_object_z", rarm_object_offset_.position.z, 0.0);
  node_handle_.param ("left_arm_pose_on_object_x", larm_object_offset_.position.x, -0.2);
  node_handle_.param ("left_arm_pose_on_object_y", larm_object_offset_.position.y, 0.1);
  node_handle_.param ("left_arm_pose_on_object_z", larm_object_offset_.position.z, 0.0);


  //robot description
  node_handle_.param<std::string>("robot/arm_name", arm_name_, "right_arm");
  std::string robot_urdf_param;
  if(!node_handle_.searchParam("robot_description",robot_urdf_param))
  {
    ROS_ERROR("Unable to find robot description on param server (/robot_description is not set). Exiting");
    return false;
  }
  node_handle_.param<std::string>(robot_urdf_param, robot_description_, "robot_description");
  node_handle_.param ("robot/num_joints", num_joints_, 7);
  
  //pr2 specific
  ljoint_names_[0] = "l_shoulder_pan_joint";
  ljoint_names_[1] = "l_shoulder_lift_joint";
  ljoint_names_[2] = "l_upper_arm_roll_joint";
  ljoint_names_[3] = "l_elbow_flex_joint";
  ljoint_names_[4] = "l_forearm_roll_joint";
  ljoint_names_[5] = "l_wrist_flex_joint";
  ljoint_names_[6] = "l_wrist_roll_joint";

  rjoint_names_[0] = "r_shoulder_pan_joint";
  rjoint_names_[1] = "r_shoulder_lift_joint";
  rjoint_names_[2] = "r_upper_arm_roll_joint";
  rjoint_names_[3] = "r_elbow_flex_joint";
  rjoint_names_[4] = "r_forearm_roll_joint";
  rjoint_names_[5] = "r_wrist_flex_joint";
  rjoint_names_[6] = "r_wrist_roll_joint";
 
  //collision space
  node_handle_.param<std::string>("collision_space/collision_map_topic", collision_map_topic_, "collision_map_occ");

  node_handle_.param<std::string>("stl_file", stl_file_, "");
  node_handle_.param<std::string>("start_goal_file", start_goal_file_, "");
  node_handle_.param<std::string>("region_file", region_file_, "");

  //visualizations
  node_handle_.param ("visualizations/goal", visualize_goal_, true);
  node_handle_.param ("visualizations/expanded_states",visualize_expanded_states_,true);
  node_handle_.param ("visualizations/heuristic", visualize_heuristic_, true);
  node_handle_.param ("visualizations/voxel_size", env_resolution_, 0.02);
  node_handle_.param ("visualizations/trajectory", visualize_trajectory_, false);
  node_handle_.param ("visualizations/end_effector_path", visualize_end_effector_path_, false);
  node_handle_.param ("visualizations/collision_model_trajectory", visualize_collision_model_trajectory_, false);
  node_handle_.param ("visualizations/trajectory_throttle", throttle_, 4);
  node_handle_.param ("visualizations/heuristic_grid", visualize_heuristic_grid_, false);

  map_frame_ = "map";

  pcl_pub_ = root_handle_.advertise<sensor_msgs::PointCloud2>("kitchen_cloud", 3);

  //initialize planner
  if(!initializePlannerAndEnvironment())
    return false;


  marker_pub_ = root_handle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  collision_map_filter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&OMPLFullBodyPlannerNode::collisionMapCallback, this, _1));

  //joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &OMPLFullBodyPlannerNode::jointStatesCallback,this);

  collision_object_subscriber_ = root_handle_.subscribe("collision_object", 5, &OMPLFullBodyPlannerNode::collisionObjectCallback, this);

  object_subscriber_ = root_handle_.subscribe("sbpl_attached_collision_object", 3, &OMPLFullBodyPlannerNode::attachedObjectCallback,this);

  //path_subscriber_ = root_handle_.subscribe("demonstrated_path", 3, &OMPLFullBodyPlannerNode::addPathCallback,this);

  // main planning service
  planning_service_ = root_handle_.advertiseService("/sbpl_planning/plan_path", &OMPLFullBodyPlannerNode::planToPosition,this);
  
  planner_initialized_ = true;

  ROS_INFO("[node] The SBPL arm planner node initialized succesfully.");
  return true;
}

int OMPLFullBodyPlannerNode::run()
{
  ros::spin();
  return 0;
}


void writeHeader(FILE* file){
    fprintf(file, "goal_tolerance:\n");
    fprintf(file, "  xyz: 0.02 0.02 0.02\n");
    fprintf(file, "  yaw: 0.1\n");
    fprintf(file, "\n");
    fprintf(file, "object_pose_in_gripper:\n");
    fprintf(file, "  right:\n");
    fprintf(file, "    xyz: -0.16 0.0 0.0\n");
    fprintf(file, "    rpy: 0.0 0.0 0.0\n");
    fprintf(file, "  left:\n");
    fprintf(file, "    xyz: -0.16 0.0 0.0\n");
    fprintf(file, "    rpy: 0.0 0.0 0.0\n");
    fprintf(file, "\n");
    fprintf(file, "use_current_pose_as_start_state: false\n");
    fprintf(file, "start_state:\n");
    fprintf(file, "  right: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n");
    fprintf(file, "  left: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n");
    fprintf(file, "  base: 1.9 0.6 1.57\n");
    fprintf(file, "  spine: 0.0\n");
    fprintf(file, "\n");
    fprintf(file, "egraph_eps: 10.0\n");
    fprintf(file, "final_egraph_eps: 10.0\n");
    fprintf(file, "dec_egraph_eps: 0.5\n");
    fprintf(file, "initial_eps: 2.0\n");
    fprintf(file, "final_eps: 2.0\n");
    fprintf(file, "use_egraph: true\n");
    fprintf(file, "save_egraph: true\n");
    fprintf(file, "dec_eps: 0.2\n");
    fprintf(file, "final_delta: 10.0\n");
    fprintf(file, "inc_delta: 0.5\n");
    fprintf(file, "feedback_paths: true\n");
    fprintf(file, "experiments:\n");

}

void writeStartGoal(FILE* file, vector<double> arm_right_start, 
                    vector<double> arm_left_start, State start_state, 
                    vector<double> arm_right_goal, vector<double> arm_left_goal,
                    State goal_state, std::string tag){
    static int counter = 0;
    fprintf(file, "- name: %s_%d\n", tag.c_str(), counter);
    fprintf(file, "  goal: goal_%d\n", counter);
    fprintf(file, "  pre_action: attach\n");
    fprintf(file, "  post_action: detach\n");
    fprintf(file, "  sound_bite: \"how ya doin?\"\n");
    fprintf(file, "  start:\n");
    fprintf(file, "    right: %f %f %f %f %f %f %f\n", 
            arm_right_start[0], 
            arm_right_start[1],
            arm_right_start[2],
            arm_right_start[3],
            arm_right_start[4],
            arm_right_start[5],
            arm_right_start[6]);
    fprintf(file, "    left: %f %f %f %f %f %f %f\n",
            arm_left_start[0], 
            arm_left_start[1],
            arm_left_start[2],
            arm_left_start[3],
            arm_left_start[4],
            arm_left_start[5],
            arm_left_start[6]);

    fprintf(file, "    base: %f %f %f\n", start_state.base_x, start_state.base_y, start_state.base_yaw);
    fprintf(file, "    torso: %f\n", start_state.torso);
    fprintf(file, "  complete_goal:\n");
    fprintf(file, "    right: %f %f %f %f %f %f %f\n", 
            arm_right_goal[0], 
            arm_right_goal[1],
            arm_right_goal[2],
            arm_right_goal[3],
            arm_right_goal[4],
            arm_right_goal[5],
            arm_right_goal[6]);
    fprintf(file, "    left: %f %f %f %f %f %f %f\n",
            arm_left_goal[0], 
            arm_left_goal[1],
            arm_left_goal[2],
            arm_left_goal[3],
            arm_left_goal[4],
            arm_left_goal[5],
            arm_left_goal[6]);

    fprintf(file, "    base: %f %f %f\n", goal_state.base_x, goal_state.base_y, goal_state.base_yaw);
    fprintf(file, "    torso: %f\n", goal_state.torso);
    counter++;
}

bool OMPLFullBodyPlannerNode::initializePlannerAndEnvironment()
{
  planner_ = new ARAPlanner(&sbpl_arm_env_, forward_search_);

  if(robot_description_.empty())
  {
    ROS_ERROR("[node] Robot description file is empty. Exiting.");
    return false;
  }

  //initialize arm planner environment
  if(!sbpl_arm_env_.initEnvironment(right_arm_description_filename_,left_arm_description_filename_,mprims_filename_,base_mprims_filename_))
  {
    ROS_ERROR("[node] ERROR: initEnvironment failed");
    return false;
  }

  cspace_ = sbpl_arm_env_.getCollisionSpace();
  grid_ = sbpl_arm_env_.getOccupancyGrid();

  readCollisionMapFromFile(stl_file_);

  //set epsilon
#if USE_LEARNING
  printf("\n\nepsH=%f eps=%f\n\n",sbpl_arm_env_.getEpsilon()/sbpl_arm_env_.getEpsilon2(),sbpl_arm_env_.getEpsilon2());
  sbpl_arm_env_.setEpsH(sbpl_arm_env_.getEpsilon()/sbpl_arm_env_.getEpsilon2());
  sbpl_arm_env_.useCaching = true;
  sbpl_arm_env_.computeNormalHeuristic = false;
  planner_->set_initialsolution_eps(sbpl_arm_env_.getEpsilon2());
#else
  planner_->set_initialsolution_eps(sbpl_arm_env_.getEpsilon());
  sbpl_arm_env_.computeNormalHeuristic = true;
#endif

  //set search mode (true - settle with first solution)
  planner_->set_search_mode(search_mode_);

  laviz_ = new sbpl_two_arm_planner::VisualizeArm(std::string("left_arm"));
  raviz_ = new sbpl_two_arm_planner::VisualizeArm(std::string("right_arm"));
  laviz_->setReferenceFrame(reference_frame_);
  raviz_->setReferenceFrame(reference_frame_);

  ROS_INFO("[node] Initialized sbpl planning environment.");



  rangles_[0] = 0.256421;
  rangles_[1] = 0.827628;
  rangles_[2] = 0.000000;
  rangles_[3] = -0.426053;
  rangles_[4] = 0.893944;
  rangles_[5] = -0.595797;
  rangles_[6] = -0.800253;

  langles_[0] = 1.1993011559598821;
  langles_[1] = 0.3639994180419307;
  langles_[2] = 1.1930743691585055;
  langles_[3] = -1.673176609553904;
  langles_[4] = 1.9162929181216546;
  langles_[5] = -1.681316333341242;
  langles_[6] = -4.385533351016869;

  body_pos_.x = 1.840000;
  body_pos_.y = 1.380000;
  body_pos_.z = 0.000000;
  body_pos_.theta = 1.963495;
  btQuaternion btoffset;
  geometry_msgs::Pose rarm;
  rarm.position.x = -0.16;
  rarm.position.y = 0;
  rarm.position.z = 0.0;
  btoffset.setRPY(0.0,0.0,0.0);
  tf::quaternionTFToMsg(btoffset,rarm.orientation);
  geometry_msgs::Pose larm;
  larm.position.x = -0.16;
  larm.position.y = 0.0;
  larm.position.z = 0.0;
  btoffset.setRPY(0.0,0.0,0.0);
  tf::quaternionTFToMsg(btoffset,larm.orientation);
  geometry_msgs::Pose start;
  if(!setStart(start, rarm, larm)){
    ROS_ERROR("[node] Failed to set the starting configuration.");
    return false;
  }


  //create the StateSpace (defines the dimensions and their bounds)
  ompl::base::SE2StateSpace* se2 = new ompl::base::SE2StateSpace();
  ompl::base::RealVectorBounds base_bounds(2);
  base_bounds.setLow(0,0);
  base_bounds.setHigh(0,7.2);//3
  base_bounds.setLow(1,0);
  base_bounds.setHigh(1,6.2);//3
  se2->setBounds(base_bounds);
  ompl::base::RealVectorStateSpace* r7 = new ompl::base::RealVectorStateSpace(7);
  r7->setDimensionName(0,"torso");
  r7->setDimensionName(1,"arms_x");
  r7->setDimensionName(2,"arms_y");
  r7->setDimensionName(3,"arms_z");
  r7->setDimensionName(4,"arms_yaw");
  r7->setDimensionName(5,"free_angle_left");
  r7->setDimensionName(6,"free_angle_right");
  ompl::base::RealVectorBounds bounds(7);
  bounds.setLow(0,0);
  bounds.setHigh(0,0.30);
  bounds.setLow(1,0.35);//0
  bounds.setHigh(1,1.2);//1.5
  bounds.setLow(2,-0.6);//-1
  bounds.setHigh(2,0.6);//1
  bounds.setLow(3,-0.6);//-1
  bounds.setHigh(3,0.6);//1
  bounds.setLow(4,-1.396);//-1.57
  bounds.setHigh(4,1.396);//1.57
  bounds.setLow(5,-0.65);//-pi
  bounds.setHigh(5,3.75);//pi
  bounds.setLow(6,-3.75);//-pi
  bounds.setHigh(6,0.65);//pi
  r7->setBounds(bounds);
  ompl::base::StateSpacePtr se2_p(se2);
  ompl::base::StateSpacePtr r7_p(r7);
  fullBodySpace = r7_p + se2_p;

  //Define our SpaceInformation (combines the state space and collision checker)
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(fullBodySpace));
  //omplFullBodyCollisionChecker* ompl_checker = new omplFullBodyCollisionChecker(si);
  ompl_checker = new omplFullBodyCollisionChecker(si);



  ompl_checker->initialize(&sbpl_arm_env_, cspace_);
  ompl::base::StateValidityChecker* temp2 = ompl_checker;
  si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(temp2));
  si->setStateValidityCheckingResolution(0.0009); // 0.1%
  si->setup();
  if(generate_goals_){
    ompl_checker->initializeRegions(region_file_);
    FILE* valid_pts = fopen("/tmp/valid_pts.yaml", "w");
    writeHeader(valid_pts);
    State start_state;
    State goal_state;
    vector<double> arm_right_start(7,0);
    vector<double> arm_left_start(7,0);
    vector<double> arm_right_goal(7,0);
    vector<double> arm_left_goal(7,0);
    srand(time(NULL));
    FILE* picked_region_file = fopen("/tmp/picked_regions.data","w");

    // generate uniform random samples across entire state space
    int NUM_UNIFORM_SAMPLES = 100;
    ROS_INFO("generating %d uniform random samples", NUM_UNIFORM_SAMPLES);
    int counter = 0;
    for (int i=0; i < NUM_UNIFORM_SAMPLES; i++){
        if (counter % 1 == 0){
            ROS_INFO("generated %d uniform samples", counter);
        }
        counter++;

        int set_uniform_sampling = -1;
        ompl_checker->generateRandomValidState(start_state, 
                arm_right_start, arm_left_start, 2*i, set_uniform_sampling);
        ompl_checker->generateRandomValidState(goal_state, arm_right_goal, 
                arm_left_goal, 2*i+1, set_uniform_sampling);
        std::string tag = "a_uniform_samples";
        writeStartGoal(valid_pts, arm_right_start, arm_left_start, start_state, 
                arm_right_goal, arm_left_goal, goal_state, tag);
    }
    fflush(valid_pts);

    ROS_INFO("generating %d samples within the user defined regions", 1000);
    for(int i=0; i<1000; i++){
      bool generated_start = false;
      bool generated_goal = false;
      int sampled_region_id_2;
      int sampled_region_id_1;

      // caution - if there are no valid states in a particular region, this
      // will infinite loop
      while (!generated_start){
          sampled_region_id_1 = rand() % ompl_checker->regions.size();
          ROS_INFO("trying region %d", sampled_region_id_1);
          generated_start = ompl_checker->generateRandomValidState(start_state, 
                  arm_right_start, arm_left_start, 2*i, sampled_region_id_1);
      }
      while (!generated_goal){
          sampled_region_id_2 = rand() % ompl_checker->regions.size();
          
          // let's make sure the picked start/goals are in different regions
          while (ompl_checker->regions.size() > 1 && (sampled_region_id_2 == sampled_region_id_1)){
              sampled_region_id_2 = rand() % ompl_checker->regions.size();
          }

          ROS_INFO("trying region %d", sampled_region_id_2);
          generated_goal = ompl_checker->generateRandomValidState(goal_state, arm_right_goal, 
                                                 arm_left_goal, 2*i+1, sampled_region_id_2);
      }
      if (generated_start && generated_goal){
          ROS_INFO("generated pair in regions %d and %d", sampled_region_id_1, 
                   sampled_region_id_2);
          fprintf(picked_region_file, "regions %d,%d\n", sampled_region_id_1, 
                  sampled_region_id_2);
          fflush(picked_region_file);
          std::string tag = "goal";
          writeStartGoal(valid_pts, arm_right_start, arm_left_start, start_state, 
                                    arm_right_goal, arm_left_goal, goal_state, tag.c_str());
      }
    }
    fclose(valid_pts);
    exit(0);
  }

  //Define a ProblemDefinition (a start/goal pair)
  pdef = new ompl::base::ProblemDefinition(si);

  //Create the planner
  if(planner_id_==0){
    planner_string_ = "RRTConnect";
    planner = new ompl::geometric::RRTConnect(si);
  } else if(planner_id_==1){
    planner_string_ = "PRM";
    planner = new ompl::geometric::PRM(si);
  } else if(planner_id_==2 || planner_id_==3){
    planner_string_ = "RRTStar";
    planner = new ompl::geometric::RRTstar(si); 
  } else{
    ROS_ERROR("not a valid planner. exiting!");
    exit(1);
  }

  planner->setup();
  planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr(pdef));
  pathSimplifier = new ompl::geometric::PathSimplifier(si);

  return true;
}

/** Callbacks ----------------------------------------------------------------*/
void OMPLFullBodyPlannerNode::readCollisionMapFromFile(std::string file)
{
  ROS_WARN("reading stl file %s",file.c_str());
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  std::vector<std::vector<double> > voxels;
  leatherman::getVoxelsFromMesh(file, pose, voxels);

  std::vector<btVector3> points;
  double maxx = -1;
  double maxy = -1;
  double maxz = -1;
  double minx = 100000;
  double miny = 100000;
  double minz = 100000;
  for(unsigned int i=0; i<voxels.size(); i++){
    if(voxels[i][0]>maxx)
      maxx = voxels[i][0];
    if(voxels[i][1]>maxy)
      maxy = voxels[i][1];
    if(voxels[i][2]>maxz)
      maxz = voxels[i][2];
    if(voxels[i][0]<minx)
      minx = voxels[i][0];
    if(voxels[i][1]<miny)
      miny = voxels[i][1];
    if(voxels[i][2]<minz)
      minz = voxels[i][2];
  }
  for(unsigned int i=0; i<voxels.size(); i++){
    btVector3 p(voxels[i][0]-minx,
                voxels[i][1]-miny,
                voxels[i][2]-minz-0.32);
    points.push_back(p);
  }
  grid_->addPointsToField(points);
  ROS_WARN("added %d voxels to the occupancy grid",int(points.size()));
  ROS_WARN("bounds (%f, %f, %f) to (%f, %f, %f)",minx,miny,minz,maxx,maxy,maxz);

  //make a point cloud for visualization

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for(unsigned int i=0; i<points.size(); i++)
    pclCloud->push_back(pcl::PointXYZ(points[i].m_floats[0], points[i].m_floats[1], points[i].m_floats[2]));
  sensor_msgs::PointCloud2 pc;
  pcl::toROSMsg (*pclCloud, pc);
  pc.header.frame_id = "/map";
  pc.header.stamp = ros::Time::now();
  pcl_pub_.publish(pc);
  sleep(1);

  //grid_->visualize();
}

void OMPLFullBodyPlannerNode::collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  updateMapFromCollisionMap(collision_map);
}

void OMPLFullBodyPlannerNode::updateMapFromCollisionMap(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  ROS_DEBUG("collision map callback");
  if(collision_map->header.frame_id.compare(reference_frame_) != 0)
  {
    ROS_WARN("collision_map_occ is in %s not in %s", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
    ROS_DEBUG("the collision map has %i cubic obstacles", int(collision_map->boxes.size()));
  }

  // add collision map msg
  if(use_collision_map_from_sensors_)
    grid_->updateFromCollisionMap(*collision_map);

  map_frame_ = collision_map->header.frame_id; 
  setArmToMapTransform(map_frame_);

  ROS_WARN("collision_map_occ frame: %s", map_frame_.c_str());

  cspace_->putCollisionObjectsInGrid();
  grid_->visualize();
  return;
}

void OMPLFullBodyPlannerNode::collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object)
{
  // for some reason, it wasn't getting all of the 'all' messages...
  if(collision_object->id.compare("all") == 0)
    cspace_->removeAllCollisionObjects();

  // debug: have we seen this collision object before?
  if(object_map_.find(collision_object->id) != object_map_.end())
    ROS_DEBUG("[collisionObjectCallback] We have seen this object ('%s')  before.", collision_object->id.c_str());
  else
    ROS_DEBUG("[collisionObjectCallback] We have NOT seen this object ('%s') before.", collision_object->id.c_str());

  object_map_[collision_object->id] = (*collision_object);

  ROS_DEBUG("[collisionObjectCallback] %s", collision_object->id.c_str());
  cspace_->processCollisionObjectMsg((*collision_object));

  //visualize exact collision object
  visualizeCollisionObject(*collision_object);

  //visualize collision voxels
  //visualizeCollisionObjects();
  
  if(attached_object_)
    visualizeAttachedObject();
}

void OMPLFullBodyPlannerNode::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
  ROS_DEBUG("joint states callback");
  rangles_[0] = state->position[17];
  rangles_[1] = state->position[18];
  rangles_[2] = state->position[16];
  rangles_[3] = state->position[20];
  rangles_[4] = state->position[19];
  rangles_[5] = state->position[21];
  rangles_[6] = state->position[22];

  langles_[0] = state->position[29];
  langles_[1] = state->position[30];
  langles_[2] = state->position[28];
  langles_[3] = state->position[32];
  langles_[4] = state->position[31];
  langles_[5] = state->position[33];
  langles_[6] = state->position[34];

  body_pos_.z = state->position[12];
  head_pan_ = state->position[13];
  head_tilt_ = state->position[14];

  // get base position
  try
  {
    tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
    body_pos_.x = base_map_transform_.getOrigin().x();
    body_pos_.y = base_map_transform_.getOrigin().y();
    //body_pos_.theta = tf::getYaw(base_map_transform_.getRotation().getYaw());
    body_pos_.theta = 2 * atan2(base_map_transform_.getRotation().getZ(), base_map_transform_.getRotation().getW()); 
    ROS_DEBUG("Received transform from base_footprint to map (x: %f y: %f yaw: %f)", body_pos_.x, body_pos_.y, body_pos_.theta);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Is there a map? The map-robot transform failed. (%s)",ex.what());
  }

  if(attached_object_)
    visualizeAttachedObject();
}

void OMPLFullBodyPlannerNode::attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  // remove all objects
  if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    ROS_INFO("[node] Removing all attached objects.");
    attached_object_ = false;
    cspace_->removeAllAttachedObjects();
  }
  // add object
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    attached_object_ = true;
    ROS_INFO("[node] Received a message to ADD an object (%s) with %d shapes.", attached_object->object.id.c_str(), int(attached_object->object.shapes.size()));
    cspace_->addAttachedObject(attached_object->object);
  }
  else
    ROS_INFO("[node] We don't support this operation.");

  //get object radius
  if(attached_object->object.poses.size() > 0)
  {
    if(!use_inner_circle_)
    {
      sbpl_arm_env_.setObjectRadius(max(attached_object->object.poses[0].position.x, attached_object->object.poses[0].position.y)/2.0);
      ROS_INFO("[node] Setting the object radius to the OUTER radius of %0.3fm",max(attached_object->object.poses[0].position.x, attached_object->object.poses[0].position.y)/2.0);
    }
    else
    {
      ROS_INFO("[node] Setting the object radius to the INNER radius of %0.3fm",min(attached_object->object.poses[0].position.x, attached_object->object.poses[0].position.y)/2.0);
      sbpl_arm_env_.setObjectRadius(min(attached_object->object.poses[0].position.x, attached_object->object.poses[0].position.y)/2.0);
    }

    //temporary hack to get Z inflation of object
    ROS_INFO("[node] Object should be inflated by %0.3fm above it.",attached_object->object.poses[0].orientation.x);
    sbpl_arm_env_.setObjectZInflation(int(attached_object->object.poses[0].orientation.x/env_resolution_ + 0.5),0);
  }

  if(attached_object_)
    visualizeAttachedObject();
}

//void OMPLFullBodyPlannerNode::addPathCallback(const full_body_demonstration::FullBodyTrajectoryConstPtr &path){
//#if USE_LEARNING
//  ROS_INFO("[node] got a demonstrated path");
//  sbpl_arm_env_.addDemonstratedPath(path);
//  ROS_INFO("[node] done with demonstrated path");
//#endif
//}
 
/** Planner Interface  -------------------------------------------------------*/
bool OMPLFullBodyPlannerNode::setStart(geometry_msgs::Pose start, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object)
{
  int startid = -1;
  double roll,pitch,yaw;
  tf::Pose tstart;
  std::vector<double> vstart(6,0);;

  ROS_INFO("[node] start0: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", rangles_[0],rangles_[1],rangles_[2],rangles_[3],rangles_[4],rangles_[5],rangles_[6]);
  ROS_INFO("[node] start1: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", langles_[0],langles_[1],langles_[2],langles_[3],langles_[4],langles_[5],langles_[6]);

  vstart[0] = start.position.x;
  vstart[1] = start.position.y;
  vstart[2] = start.position.z;

  tf::poseMsgToTF(start, tstart);
  tstart.getBasis().getRPY(roll,pitch,yaw);
  vstart[3] = roll;
  vstart[4] = pitch;
  vstart[5] = yaw;

  KDL::Frame rarm_offset, larm_offset;
  rarm_offset.p.x(rarm_object.position.x);
  rarm_offset.p.y(rarm_object.position.y);
  rarm_offset.p.z(rarm_object.position.z);
  larm_offset.p.x(larm_object.position.x);
  larm_offset.p.y(larm_object.position.y);
  larm_offset.p.z(larm_object.position.z);
  rarm_offset.M = KDL::Rotation::Quaternion(rarm_object.orientation.x, rarm_object.orientation.y, rarm_object.orientation.z, rarm_object.orientation.w);
  larm_offset.M = KDL::Rotation::Quaternion(larm_object.orientation.x, larm_object.orientation.y, larm_object.orientation.z, larm_object.orientation.w);

  if((startid = sbpl_arm_env_.setStartConfiguration(rangles_, langles_, body_pos_, rarm_offset, larm_offset)) == -1)
  {
    ROS_ERROR("[node] Environment failed to set start state. Not Planning.");
    return false;
  }
  ROS_INFO("[node] Start stateid: %d", startid);

  if(planner_->set_start(startid) == 0)
  {
    ROS_ERROR("[node] Failed to set start state. Not Planning.");
    return false;
  }

  if(attached_object_)
    visualizeAttachedObject();

  ROS_INFO("[node] Visualizing start position."); 
  printRobotState(rangles_, langles_, body_pos_, "start state");
  pviz_.visualizeRobotWithTitle(rangles_, langles_, body_pos_, 30, "start", 0, "start");

  return true;
}

bool OMPLFullBodyPlannerNode::setGoalPosition(geometry_msgs::Pose goal, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object, std::vector<double> &goal_tolerance)
{
  int goalid = -1;
  double roll,pitch,yaw;
  geometry_msgs::Pose pgoal;
  tf::Pose tf_pose;
  std::vector <std::vector <double> > sbpl_goal(1, std::vector<double> (11,0));  //Changed to include Quaternion
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  //currently only supports one goal
  sbpl_goal[0][0] = goal.position.x;
  sbpl_goal[0][1] = goal.position.y;
  sbpl_goal[0][2] = goal.position.z;

  //perturb quaternion to prevent gimbal lock when using grasping pipeline
  pgoal = goal;
  pgoal.orientation.w += 0.005;

  tf::poseMsgToTF(pgoal, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  sbpl_goal[0][3] = roll;
  sbpl_goal[0][4] = pitch;
  sbpl_goal[0][5] = yaw;

  //6dof goal: true, 3dof: false 
  sbpl_goal[0][6] = true;
 
  //orientation constraint as a quaternion 
  sbpl_goal[0][7] = goal.orientation.x;
  sbpl_goal[0][8] = goal.orientation.y;
  sbpl_goal[0][9] = goal.orientation.z;
  sbpl_goal[0][10] = goal.orientation.w;

  //allowable tolerance from goal
  sbpl_tolerance[0] = goal_tolerance;

  ROS_INFO("[node] goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)", map_frame_.c_str(),sbpl_goal[0][0],sbpl_goal[0][1],sbpl_goal[0][2],sbpl_tolerance[0][0],sbpl_goal[0][3],sbpl_goal[0][4],sbpl_goal[0][5], sbpl_tolerance[0][1]);

  KDL::Frame rarm_offset, larm_offset;
  rarm_offset.p.x(rarm_object.position.x);
  rarm_offset.p.y(rarm_object.position.y);
  rarm_offset.p.z(rarm_object.position.z);
  larm_offset.p.x(larm_object.position.x);
  larm_offset.p.y(larm_object.position.y);
  larm_offset.p.z(larm_object.position.z);

  rarm_offset.M = KDL::Rotation::Quaternion(rarm_object.orientation.x, rarm_object.orientation.y, rarm_object.orientation.z, rarm_object.orientation.w);
  larm_offset.M = KDL::Rotation::Quaternion(larm_object.orientation.x, larm_object.orientation.y, larm_object.orientation.z, larm_object.orientation.w);

  //set object_radius if no object attached 
  if(!attached_object_)
    sbpl_arm_env_.setObjectRadius(object_radius_);

  ROS_INFO("[node] Setting goal position.");
  //set sbpl environment goal
  if((goalid = sbpl_arm_env_.setGoalPosition(sbpl_goal, sbpl_tolerance, rarm_offset, larm_offset, object_radius_)) == -1)
  {
    ROS_ERROR("[node] Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
    return false;
  }
#if USE_LEARNING
  sbpl_arm_env_.updateHeurToH();
#endif
  ROS_INFO("[node] Goal stateID: %d", goalid);

  //set planner goal	
  if(planner_->set_goal(goalid) == 0)
  {
    ROS_ERROR("[node] Failed to set goal state. Exiting.");
    return false;
  }

  return true;
}



bool OMPLFullBodyPlannerNode::planToPosition(sbpl_two_arm_planner_node::GetTwoArmPlan::Request &req, sbpl_two_arm_planner_node::GetTwoArmPlan::Response &res)
{
  std::vector<trajectory_msgs::JointTrajectoryPoint> lpath, rpath, bpath;

  starttime = clock();

  ROS_INFO("[node] About to set start configuration");

  //set start
  rangles_ = req.rarm_start;
  langles_ = req.larm_start;
  body_pos_.x = req.body_start[0];
  body_pos_.y = req.body_start[1];
  body_pos_.z = req.body_start[2];
  body_pos_.theta = req.body_start[3];
  if(!setStart(req.start.pose, req.rarm_object.pose, req.larm_object.pose))
  {
    ROS_ERROR("[node] Failed to set the starting configuration.");
    return false;
  }

  rarm_object_offset_ = req.rarm_object.pose;
  larm_object_offset_ = req.larm_object.pose;

  /*
  std::vector<double> solution(7,0);
  if(!larm_->computeIK(req.goal,langles_,solution))
    ROS_WARN("[node] Failed to compute an IK solution for the left arm.");
    
  if(!rarm_->computeIK(req.goal,rangles_,solution))
    ROS_WARN("[node] Failed to compute an IK solution for the right arm.");
  */

  totalPlanTime = clock();

  //set goal
  if(!setGoalPosition(req.goal.pose, req.rarm_object.pose, req.larm_object.pose, req.absolute_xyzrpy_tolerance))
  {
    ROS_ERROR("[node] Failed to set the goal pose.");
    return false;
  }

  ///////////////////////////////////////////
  //OMPL

  double cstate[10];
  BodyPose bp;

  double figit[3] = {0.0, 0.02, -0.02};

  // make start state
  bool valid = false;
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> ompl_start(fullBodySpace);
  for(int bx=0; bx<3 && !valid; bx++){
    for(int by=0; by<3 && !valid; by++){
      for(int i=1; i<3 && !valid; i++){
        for(int x=0; x<3 && !valid; x++){
          for(int y=0; y<3 && !valid; y++){
            for(int z=0; z<3 && !valid; z++){
              bp.x = req.body_start[0];
              bp.y = req.body_start[1];
              bp.z = req.body_start[2];
              bp.theta = req.body_start[3];
              sbpl_arm_env_.configurationToWState(req.rarm_start, req.larm_start, bp, cstate);

              ROS_INFO("testing fidgeted state (start) %f %f %f %f %f %f %f, %f %f %f",
                      cstate[8],
                      cstate[0]+figit[x]*i,
                      cstate[1]+figit[y]*i,
                      cstate[2]+figit[z]*i,
                      cstate[3],
                      cstate[5],
                      cstate[4],
                      cstate[6]+figit[bx],
                      cstate[7]+figit[by],
                      angles::normalize_angle(cstate[9]));

              //ompl::base::ScopedState<ompl::base::CompoundStateSpace>* ompl_start(fullBodySpace);
              (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0] = cstate[8];
              (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1] = cstate[0]+figit[x]*i;
              (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2] = cstate[1]+figit[y]*i;
              (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3] = cstate[2]+figit[z]*i;
              (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4] = cstate[3];
              (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5] = cstate[5];
              (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6] = cstate[4];
              ompl_start->as<ompl::base::SE2StateSpace::StateType>(1)->setXY(cstate[6]+figit[bx],cstate[7]+figit[by]);
              ompl_start->as<ompl::base::SE2StateSpace::StateType>(1)->setYaw(angles::normalize_angle(cstate[9]));

              if(planner->getSpaceInformation()->isValid(ompl_start.get())){
                valid = true;
              }
            }
          }
        }
      }
    }
  }
  if(!valid){
    ROS_ERROR("oh noes, invalid start!");
    return false;
  }
  pdef->clearStartStates();
  ROS_INFO("found a valid start");
  ROS_INFO("%f %f %f %f %f %f %f, %f %f %f",
        (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0],
        (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1],
        (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2],
        (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3],
        (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4],
        (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5],
        (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6],
        ompl_start->as<ompl::base::SE2StateSpace::StateType>(1)->getX(),
        ompl_start->as<ompl::base::SE2StateSpace::StateType>(1)->getY(),
        ompl_start->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw());


  // make goal state
  valid = false;
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> ompl_goal(fullBodySpace);
  for(int bx=0; bx<3 && !valid; bx++){
    for(int by=0; by<3 && !valid; by++){
      for(int x=0; x<3 && !valid; x++){
        for(int y=0; y<3 && !valid; y++){
          for(int z=0; z<3 && !valid; z++){
            bp.x = req.body_goal[0];
            bp.y = req.body_goal[1];
            bp.z = req.body_goal[2];
            bp.theta = req.body_goal[3];
            sbpl_arm_env_.configurationToWState(req.rarm_goal, req.larm_goal, bp, cstate);
            ROS_INFO("testing fidgeted state (goal) %f %f %f %f %f %f %f, %f %f %f",
                    cstate[8],
                    cstate[0]+figit[x],
                    cstate[1]+figit[y],
                    cstate[2]+figit[z],
                    cstate[3],
                    cstate[5],
                    cstate[4],
                    cstate[6]+figit[bx],
                    cstate[7]+figit[by],
                    angles::normalize_angle(cstate[9]));

            (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0] = cstate[8];
            (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1] = cstate[0]+figit[x];
            (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2] = cstate[1]+figit[y];
            (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3] = cstate[2]+figit[z];
            (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4] = cstate[3];
            (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5] = cstate[5];
            (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6] = cstate[4];
            ompl_goal->as<ompl::base::SE2StateSpace::StateType>(1)->setXY(cstate[6]+figit[bx],cstate[7]+figit[by]);
            ompl_goal->as<ompl::base::SE2StateSpace::StateType>(1)->setYaw(angles::normalize_angle(cstate[9]));

            if(planner->getSpaceInformation()->isValid(ompl_goal.get())){
              valid = true;
            }
          }
        }
      }
    }
  }
  if(!valid){
    ROS_ERROR("oh noes!");
    return false;
  }
  ROS_INFO("found a valid goal");
  ROS_INFO("%f %f %f %f %f %f %f, %f %f %f",
        (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0],
        (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1],
        (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2],
        (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3],
        (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4],
        (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5],
        (*(ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6],
        ompl_goal->as<ompl::base::SE2StateSpace::StateType>(1)->getX(),
        ompl_goal->as<ompl::base::SE2StateSpace::StateType>(1)->getY(),
        ompl_goal->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw());

  pdef->setStartAndGoalStates(ompl_start,ompl_goal);

  //for RRT*
  ompl::base::GoalState* temp_goal = new ompl::base::GoalState(planner->getSpaceInformation());
  temp_goal->setState(ompl_goal);
  //if(planner_id_==2)
  //  temp_goal->setMaximumPathLength(10000000.0);
  //else if(planner_id_==3)
  //  temp_goal->setMaximumPathLength(0.1);
  ompl::base::GoalPtr temp_goal2(temp_goal);
  if(planner_id_==2 || planner_id_==3)
    pdef->setGoal(temp_goal2);

  ompl_checker->reset_count();
  double t0 = ros::Time::now().toSec();
  printf("start clock\n");
  planner->solve(60.0);
  double t1 = ros::Time::now().toSec();
  printf("end clock\n");
  ROS_ERROR("\n\nOMPL time %f\n",t1-t0);
  ompl_checker->print_checks();
  ompl::base::PathPtr path = planner->getProblemDefinition()->getSolutionPath();
  if (path){
    ROS_ERROR("OMPL found a solution!");

    // do something with the solution
    fstream path_stream;
  std::string filename = "/tmp/ompl_stats_" + planner_string_ + ".txt";
    path_stream.open(filename.c_str(),fstream::out | fstream::app);
    path->print(path_stream);
    path_stream.close();
    
    ompl::geometric::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);
    double t2 = ros::Time::now().toSec();
    bool b1 = pathSimplifier->reduceVertices(geo_path);
    bool b2 = pathSimplifier->collapseCloseVertices(geo_path);
    ROS_ERROR("reduce:%d collapse:%d\n",b1,b2);
    //bool b3 = pathSimplifier->shortcutPath(geo_path);
    //ROS_ERROR("shortcut:%d\n",b3);
    double t3 = ros::Time::now().toSec();
  filename = "/tmp/ompl_stats_" + planner_string_ + ".csv";
  FILE* stat_out = fopen(filename.c_str(),"a");
    fprintf(stat_out,"%f, %f\n",t1-t0,t3-t2);
    fclose(stat_out);
    geo_path.interpolate();

    /*
    int color_inc = 240.0 / geo_path.getStateCount();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time(0);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.ns = "object path";
    marker.id = 0;
    marker.lifetime = ros::Duration(0);
    */
    char buf[64];
  filename = "/tmp/ompl_paths_" + planner_string_ + "/%.4d.csv";
  
    sprintf(buf,filename.c_str(),filenum_);
    filenum_++;
    FILE* traj_file = fopen(buf, "w");
    for(unsigned int i=0; i<geo_path.getStateCount(); i++){
      ompl::base::State* state = geo_path.getState(i);
      const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);
      
      vector<double> wpose(12,0);
      wpose[0] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1];
      wpose[1] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2];
      wpose[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3];
      wpose[3] = 0;
      wpose[4] = 0;
      wpose[5] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4];
      wpose[6] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6];
      wpose[7] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5];
      wpose[8] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getX();
      wpose[9] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getY();
      wpose[10] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0];
      wpose[11] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw();

      vector<double> arm0(7,0); //right arm angles
      vector<double> arm1(7,0); //left arm angles
      arm0[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6];
      arm1[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5];

      if(sbpl_arm_env_.convertWorldPoseToAngles(wpose, arm0, arm1, false)){
        ROS_DEBUG("IK is valid");
        BodyPose body;
        body.x = s->as<ompl::base::SE2StateSpace::StateType>(1)->getX();
        body.y = s->as<ompl::base::SE2StateSpace::StateType>(1)->getY();
        body.theta = s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw();
        body.z = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0];

        /*
        short unsigned int blah_x, blah_y, blah_z, blah_yaw;
        sbpl_arm_env_.computeObjectPose(body,arm0,blah_x,blah_y,blah_z,blah_yaw);
        geometry_msgs::Point p;
        sbpl_arm_env_.discToWorldXYZ(blah_x,blah_y,blah_z,p.x,p.y,p.z,true);
        marker.points.push_back(p);
        pviz_.visualizeRobot(arm0, arm1, body, i*color_inc, "waypoint_"+boost::lexical_cast<std::string>(i),i);
        usleep(3000);
        */

        fprintf(traj_file, "%0.4f, ", body.x);
        fprintf(traj_file, "%0.4f, ", body.y);
        fprintf(traj_file, "%0.4f, ", body.theta);
        fprintf(traj_file, "%0.4f, ", body.z);
        for(size_t j=0; j<arm0.size(); ++j)
          fprintf(traj_file, "%0.4f, ", arm0[j]);
        for(size_t j=0; j<arm1.size(); ++j)
          fprintf(traj_file, "%0.4f, ", arm1[j]);
        fprintf(traj_file,"\n");
      }
      else{
        ROS_ERROR("IK is not valid");
      }
    }
    fclose(traj_file);
    //marker_pub_.publish(marker);
    
    planner->getProblemDefinition()->clearSolutionPaths();

    if(planner_id_==0 || planner_id_==2 || planner_id_==3)
      planner->clear();
    else
      planner->as<ompl::geometric::PRM>()->clearQuery();
    return true;
  }
  else{
    ROS_ERROR("OMPL could not find a solution!");

  std::string filename = "/tmp/ompl_stats_" + planner_string_ + ".csv";
  FILE* stat_out = fopen(filename.c_str(),"a");
    fprintf(stat_out,"%f, -1.0\n",t1-t0);
    fclose(stat_out);

    filenum_++;
    planner->getProblemDefinition()->clearSolutionPaths();

    if(planner_id_==0 || planner_id_==2 || planner_id_==3)
      planner->clear();
    else
      planner->as<ompl::geometric::PRM>()->clearQuery();
    return false;
  }


  ///////////////////////////////////////////

  if(visualize_goal_)
    visualizeGoal(req.goal.pose);

  if(visualize_heuristic_grid_)
  {
    visualizeHeuristicGrid();
    return true;
  }

  //plan a path
  if(plan(rpath, lpath, bpath))
  {
    //fill the response message
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.header.frame_id = reference_frame_;
    res.trajectory.points.resize(rpath.size());
    res.trajectory.joint_names.resize(14);

    // added for base trajectory
    res.body_trajectory.points = bpath;

    for(size_t i = 0; i < 7; ++i)
    {
      res.trajectory.joint_names[i] = rjoint_names_[i];
      res.trajectory.joint_names[i+7] = ljoint_names_[i];
    }
    for(size_t i = 0; i < res.trajectory.points.size(); ++i)
    {
      res.trajectory.points[i].positions.resize(14);
      for(size_t j = 0; j < 7; ++j)
      {
        res.trajectory.points[i].positions[j] = rpath[i].positions[j];
        res.trajectory.points[i].positions[j+7] = lpath[i].positions[j];
      }
      if(use_shortened_path_)
        res.trajectory.points[i].time_from_start = ros::Duration((i+1)*waypoint_time_*3);
      else
        res.trajectory.points[i].time_from_start = ros::Duration((i+1)*waypoint_time_);


      if(i == res.trajectory.points.size()-1)
      {
        ROS_INFO("[node] Adding an additional second for the 'adaptive mprim' waypoint time.");
        res.trajectory.points[i].time_from_start += ros::Duration(1.0);
      }
    }

    res.stats_field_names = stats_field_names_;
    res.stats = stats_;

    if(print_path_)
      printPath(rpath, lpath, bpath);

    // visualizations
    if(visualize_expanded_states_)
      visualizeUniqueExpansions();
    
    if(visualize_heuristic_)
      //visualizeHeuristicInfo();
      displayShortestPath();

    if(visualize_trajectory_)
    {

      ROS_INFO("[node] Visualizing trajectory...");
      visualizeTrajectory(rpath,lpath,bpath);
      /*
      trajectory_msgs::JointTrajectory traj;
      traj.points = rpath;
      raviz_->visualizeJointTrajectoryMsg(traj,throttle_);
      traj.points.clear();
      traj.points = lpath;
      laviz_->visualizeJointTrajectoryMsg(traj,throttle_);
      visualizeAttachedObjectPath();
      */
    }

    if(visualize_end_effector_path_)
    {
      ROS_INFO("[node] Visualizing end effector path...");
      visualizeObjectPath();
      //visualizeEndEffectorPath();
      //visualizeExpansionsPerHValue();
    }
  }
  else
  {
    ROS_ERROR("[node] Failed to plan within alotted time frame (%0.2f seconds).", allocated_time_);
 
    res.stats_field_names = stats_field_names_;
    res.stats = stats_;

    if(visualize_expanded_states_)
      visualizeUniqueExpansions();
      //visualizeExpansions();
  
    if(visualize_heuristic_)
      //visualizeHeuristicInfo();
      displayShortestPath();

    if(!sbpl_arm_env_.checkExpandedStatesAreValid())
      ROS_WARN("[node] Invalid states were found.");
 
    std::vector<std::vector<double> > arm0, arm1;
    sbpl_arm_env_.getFinalArmConfigurations(arm0, arm1);
   
    /* 
    for(size_t i = 0; i < arm0.size(); ++i)
    {
      raviz_->visualizeArmConfiguration((i % 260), arm0[i]);
      laviz_->visualizeArmConfiguration((i % 260), arm1[i]);
    }
    */
    
  }

  return true;
  //return false;
}

bool OMPLFullBodyPlannerNode::plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath)
{
  bool b_ret=false;
  int solution_cost;
  std::vector<double> angles(num_joints_,0);
  trajectory_msgs::JointTrajectoryPoint traj_point, body_point;

  rpath.clear();
  lpath.clear();
  bpath.clear();

  ROS_INFO("\n[node] Calling planner");

  //reinitialize the search space
  planner_->force_planning_from_scratch();

  sbpl_arm_env_.recordDebugData(true);
  
  clock_t starttime = clock();
  
  //plan
  b_ret = planner_->replan(allocated_time_, &solution_state_ids_, &solution_cost);

  totalPlanTime = clock() - totalPlanTime;

  ROS_INFO("[node] Planning Time: %0.4fsec",(clock() - starttime) / (double)CLOCKS_PER_SEC);

  sbpl_arm_env_.recordDebugData(false);
  
  //check if an empty plan was received.
  if((b_ret && solution_state_ids_.size() <= 0) || !b_ret)
    ROS_WARN("[node] Planning failed.");
  else
    ROS_INFO("[node] Planning succeeded.");

  // if a path is returned, then pack it into msg form
  if(b_ret && (solution_state_ids_.size() > 0))
  {
#if USE_LEARNING
    vector<int> temp_ids;
    sbpl_arm_env_.postProcessPath(&solution_state_ids_,&temp_ids);
    solution_state_ids_.clear();
    for(unsigned int i=0; i<temp_ids.size(); i++)
      solution_state_ids_.push_back(temp_ids[i]);
    sbpl_arm_env_.updateH(solution_state_ids_);
#endif

    std::vector<std::vector<double> > angles_path, shortened_path;
    sbpl_arm_env_.convertStateIDPathToJointAnglesPath(solution_state_ids_,angles_path);
    sbpl_arm_env_.convertStateIDPathToShortenedJointAnglesPath(solution_state_ids_,shortened_path,solution_state_ids_short_);
#if USE_LEARNING
    sbpl_arm_env_.useGoalID_ = true;
#endif

    
    ROS_INFO("[node] A path was returned with %d waypoints. Shortened path has %d waypoints.", int(solution_state_ids_.size()), int(solution_state_ids_short_.size()));
    ROS_INFO("[node] Initial Epsilon: %0.3f  Final Epsilon: %0.3f Solution Cost: %d", planner_->get_initial_eps(),planner_->get_final_epsilon(), solution_cost);

    if(use_shortened_path_)
    {
      ROS_INFO("[node] Using shortened path.");
      angles_path = shortened_path;
    }
    else
      ROS_INFO("[node] Not using shortened path.");

    if(angles_path.size() == 0)
    {
      ROS_ERROR("[node] Returned path has at least 1 stateid but is empty after converting to joint angles.");
      return false;
    }

    if(angles_path.size() % 4 != 0)
    {
      ROS_ERROR("[node] Length of path received from environment is not a multiple of 4. (length: %d)", int(angles_path.size()));
      return false;
    }

    body_point.positions.resize(4);
    for(size_t i=0; i < angles_path.size(); ++i)
    {
      traj_point.positions.resize(angles_path.size());
      for (size_t p = 0; p < angles_path[i].size(); ++p)
        traj_point.positions[p] = angles_path[i][p];

      if(i % 4 == 0)
        rpath.push_back(traj_point);
      else if(i % 4 == 1)
        lpath.push_back(traj_point);
      else if(i % 4 == 2)
        body_point.positions[2] = traj_point.positions[0];
      else
      {
        body_point.positions[0] = traj_point.positions[0];
        body_point.positions[1] = traj_point.positions[1];
        body_point.positions[3] = traj_point.positions[2];
        bpath.push_back(body_point);
      }

      /*
      printf("[node] [%d] [stateid: %d] ", int(i), solution_state_ids_[i/4]);
      for(size_t q = 0; q < angles_path[i].size(); ++q)
        printf("% 1.4f ", angles_path[i][q]);
      printf("\n"); 
      */
    }
  }

  // planner stats
#if USE_LEARNING
  int num_planning_stats = 16;
#else
  int num_planning_stats = 12;
#endif
  int total_num_stats = num_planning_stats + sbpl_arm_planner::NUM_DEBUG_CODES;

  stats_field_names_.resize(total_num_stats);
  stats_field_names_[0] = "total plan time";
  stats_field_names_[1] = "initial solution planning time";
  stats_field_names_[2] = "initial epsilon";
  stats_field_names_[3] = "initial solution expansions";
  stats_field_names_[4] = "final epsilon planning time";
  stats_field_names_[5] = "final epsilon";
  stats_field_names_[6] = "solution epsilon";
  stats_field_names_[7] = "expansions";
  stats_field_names_[8] = "solution cost";
  stats_field_names_[9] = "path length";
  stats_field_names_[10] = "initial heuristic time";
  stats_field_names_[11] = "in search heuristic time";
#if USE_LEARNING
  stats_field_names_[12] = "percent path reuse";
  stats_field_names_[13] = "post-process time";
  stats_field_names_[14] = "H update time";
  stats_field_names_[15] = "size of H";
#endif

  stats_.resize(total_num_stats);
  stats_[0] = double(totalPlanTime)/CLOCKS_PER_SEC;
  stats_[1] = planner_->get_initial_eps_planning_time();
  stats_[2] = planner_->get_initial_eps();
  stats_[3] = planner_->get_n_expands_init_solution();
  stats_[4] = planner_->get_final_eps_planning_time();
  stats_[5] = planner_->get_final_epsilon();
  stats_[6] = planner_->get_solution_eps();
  stats_[7] = planner_->get_n_expands();
  stats_[8] = solution_cost;
  stats_[9] = solution_state_ids_.size();
  double initial_heuristic_time, in_search_heuristic_time;
  sbpl_arm_env_.getHeuristicTime(&initial_heuristic_time, &in_search_heuristic_time);
  stats_[10] = initial_heuristic_time;
  stats_[11] = in_search_heuristic_time;
#if USE_LEARNING
  double reuse,postProcessT,updateT;
  int sizeH;
  sbpl_arm_env_.getHStats(&reuse,&postProcessT,&updateT,&sizeH);
  stats_[12] = reuse;
  stats_[13] = postProcessT;
  stats_[14] = updateT;
  stats_[15] = sizeH;
#endif

  //environment stats
  std::vector<double> env_stats = sbpl_arm_env_.getPlanningStats();

  for(size_t i = 0; i < env_stats.size(); ++i)
  {
    stats_field_names_[i+num_planning_stats] = debug_code_names_[i];
    stats_[i+num_planning_stats] = env_stats[i];
  }

  return b_ret;
}

bool OMPLFullBodyPlannerNode::isGoalConstraintSatisfied(const std::vector<double> &rangles, const std::vector<double> &langles, const geometry_msgs::Pose &goal)
{
  bool satisfied = true;
  geometry_msgs::Pose rpose, lpose, lerr, rerr, rgoal, lgoal;
  tf::Pose tgoal, tright, tleft;


  tf::poseMsgToTF(goal,tgoal);
  tf::poseMsgToTF(rarm_object_offset_,tright);
  tf::poseMsgToTF(larm_object_offset_,tleft);

  tright = tgoal*tright;
  tleft = tgoal*tleft;

  tf::poseTFToMsg(tright, rgoal);
  tf::poseTFToMsg(tleft, lgoal);

  if(!computeFK(rangles,"right_arm",rpose))
  {
    ROS_ERROR("[node] Failed to check if goal constraint is satisfied because the right arm FK service failed.");
    return false;
  }

  if(!computeFK(langles,"left_arm", lpose))
  {
    ROS_ERROR("[node] Failed to check if goal constraint is satisfied because the left arm FK service failed.");
    return false;
  }

  lerr.position.x = fabs(lpose.position.x - lgoal.position.x);
  lerr.position.y = fabs(lpose.position.y - lgoal.position.y);
  lerr.position.z = fabs(lpose.position.z - lgoal.position.z);
  lerr.orientation.x = fabs(lpose.orientation.x - lgoal.orientation.x);
  lerr.orientation.y = fabs(lpose.orientation.y - lgoal.orientation.y);
  lerr.orientation.z = fabs(lpose.orientation.z - lgoal.orientation.z);
  lerr.orientation.w = fabs(lpose.orientation.w - lgoal.orientation.w);

  rerr.position.x = fabs(rpose.position.x - rgoal.position.x);
  rerr.position.y = fabs(rpose.position.y - rgoal.position.y);
  rerr.position.z = fabs(rpose.position.z - rgoal.position.z);
  rerr.orientation.x = fabs(rpose.orientation.x - rgoal.orientation.x);
  rerr.orientation.y = fabs(rpose.orientation.y - rgoal.orientation.y);
  rerr.orientation.z = fabs(rpose.orientation.z - rgoal.orientation.z);
  rerr.orientation.w = fabs(rpose.orientation.w - rgoal.orientation.w);


  ROS_INFO(" ");
  ROS_INFO("[node] -- Right Gripper --");
  ROS_INFO("[node]  Pose:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", rpose.position.x, rpose.position.y, rpose.position.z, rpose.orientation.x, rpose.orientation.y, rpose.orientation.z, rpose.orientation.w);
  ROS_INFO("[node]  Goal:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", rgoal.position.x, rgoal.position.y, rgoal.position.z, rgoal.orientation.x, rgoal.orientation.y, rgoal.orientation.z, rgoal.orientation.w);
  ROS_INFO("[node] Error:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", rerr.position.x, rerr.position.y, rerr.position.z, rerr.orientation.x, rerr.orientation.y, rerr.orientation.z, rerr.orientation.w);
  ROS_INFO(" ");
  ROS_INFO("[node] -- Left Gripper --");
  ROS_INFO("[node]  Pose:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", lpose.position.x, lpose.position.y, lpose.position.z, lpose.orientation.x, lpose.orientation.y, lpose.orientation.z, lpose.orientation.w);
  ROS_INFO("[node]  Goal:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", lgoal.position.x, lgoal.position.y, lgoal.position.z, lgoal.orientation.x, lgoal.orientation.y, lgoal.orientation.z, lgoal.orientation.w);
  ROS_INFO("[node] Error:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", lerr.position.x, lerr.position.y, lerr.position.z, lerr.orientation.x, lerr.orientation.y, lerr.orientation.z, lerr.orientation.w);
  ROS_INFO(" ");


  /*
  if(goal.position_constraints[0].constraint_region_shape.type == arm_navigation_msgs::Shape::BOX)
  {
    if(goal.position_constraints[0].constraint_region_shape.dimensions.size() < 3)
    { 
      ROS_WARN("[node] Goal constraint region shape is a BOX but fewer than 3 dimensions are defined.");
      return false;
    }
    if(err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
      satisfied = false;
    }
    if(err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[1])
    {
      ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]); 
      satisfied = false;
    }
    if(err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[2])
    {
      ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
      satisfied = false;
    }
  }
  else if(goal.position_constraints[0].constraint_region_shape.type == arm_navigation_msgs::Shape::SPHERE)
  {
    if(goal.position_constraints[0].constraint_region_shape.dimensions.size() < 1)
    { 
      ROS_WARN("Goal constraint region shape is a SPHERE but it has no dimensions...");
      return false;
    }
    if(err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
      satisfied = false;
    }
    if(err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]);
      satisfied = false;
    }
    if(err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
      satisfied = false;
    }
  }
  else
    ROS_WARN("Goal constraint region shape is of type %d.", goal.position_constraints[0].constraint_region_shape.type);
*/
  return satisfied;
}

/* Kinematics ----------------------------------------------------------------*/
bool OMPLFullBodyPlannerNode::computeFK(const std::vector<double> &jnt_pos, std::string arm_name, geometry_msgs::Pose &pose)
{
  kinematics_msgs::GetPositionFK::Request  request;
  kinematics_msgs::GetPositionFK::Response response;

  std::string fk_service;
  request.header.stamp = ros::Time();
  request.header.frame_id = reference_frame_;
  request.fk_link_names.resize(1);

  for(size_t j = 0 ; j < jnt_pos.size(); ++j)
    request.robot_state.joint_state.position.push_back(jnt_pos[j]);

  if(arm_name.compare("right_arm") == 0)
  {
    request.robot_state.joint_state.name = rjoint_names_;
    request.fk_link_names[0] = "r_wrist_roll_link";
    fk_service = right_fk_service_name_;
  }
  else if(arm_name.compare("left_arm") == 0)
  {
    request.robot_state.joint_state.name = ljoint_names_;
    request.fk_link_names[0] = "l_wrist_roll_link";
    fk_service = left_fk_service_name_;
  }
  else
  {
    ROS_ERROR("Invalid arm name. Forward kinematics only supports 'right_arm' and 'left_arm'. (%s)", arm_name.c_str());
    return false;
  }

  ROS_DEBUG("waiting for %s service", fk_service.c_str());
  ros::service::waitForService(fk_service);
  ros::ServiceClient client = root_handle_.serviceClient<kinematics_msgs::GetPositionFK>(fk_service);

  if(client.call(request, response))
  {
    if(response.error_code.val == response.error_code.SUCCESS)
    {
      pose = response.pose_stamped[0].pose;
      return true;
    }
    else
      return false;
  }
  else
  {
    ROS_ERROR("FK service failed");
    return false;
  }
}

void OMPLFullBodyPlannerNode::setArmToMapTransform(std::string &map_frame)
{
  std::string fk_root_frame;

  // frame that the sbpl_arm_model is working in
  sbpl_arm_env_.getArmChainRootLinkName(fk_root_frame);

  // get transform to frame that collision map is in
  try
  {
    tf_.lookupTransform(map_frame, fk_root_frame, ros::Time(0), transform_);

    ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)",fk_root_frame.c_str(),map_frame.c_str(), transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // convert transform to a KDL object
  tf::TransformTFToKDL(transform_,kdl_transform_);
  sbpl_arm_env_.setReferenceFrameTransform(kdl_transform_, map_frame);
}

/* Visualizations ------------------------------------------------------------*/
void OMPLFullBodyPlannerNode::visualizeExpansions()
{
  geometry_msgs::Pose pose;
  std::vector<std::vector<double> > expanded_states;
  std::vector<double> color(4,1);

  sbpl_arm_env_.getExpandedStates(expanded_states);

  if(!expanded_states.empty())
  {
    std::vector<std::vector<double> > detailed_color(2);
    detailed_color[0].resize(4,0.0);
    detailed_color[0][0] = 1;
    detailed_color[0][3] = 1;

    detailed_color[1].resize(4,0.0);
    detailed_color[1][1] = 1;
    detailed_color[1][3] = 1;

    laviz_->visualizeDetailedStates(expanded_states, detailed_color,"expansions",0.01);
 
    for(size_t i = 0; i < expanded_states.size(); ++i)
    {
      pose.position.x = expanded_states[i][0];
      pose.position.y = expanded_states[i][1]+0.01;
      pose.position.z = expanded_states[i][2]+0.01;
      raviz_->visualizeText(pose, boost::lexical_cast<std::string>(expanded_states[i][11]), "expansions-heuristic", i, color,0.01);
       usleep(4000);
    }
    ROS_INFO("[node] Displaying %d expanded states",int(expanded_states.size()));
  
    laviz_->visualizeSphere(expanded_states[0], 250, "expansions-start", 0.015);
  }
  else
    ROS_WARN("[node] No expanded states to display.");
}

void OMPLFullBodyPlannerNode::visualizeUniqueExpansions()
{
  geometry_msgs::Pose pose;
  double size = 0.15;
  std::vector<std::vector<double> > expanded_states;
  std::vector<double> color(4,1);
  color[0] = 0.0;

  sbpl_arm_env_.getUniqueExpandedStates(expanded_states);

  if(!expanded_states.empty())
  {
    std::vector<std::vector<double> > detailed_color(2);
    detailed_color[0].resize(4,0.0);
    detailed_color[0][0] = 1;
    detailed_color[0][3] = 1;

    detailed_color[1].resize(4,0.0);
    detailed_color[1][0] = 1;
    detailed_color[1][3] = 1;

    laviz_->visualizeDetailedStates(expanded_states, detailed_color,"expansions",0.01);
 
    for(size_t i = 0; i < expanded_states.size(); ++i)
    {
      pose.position.x = expanded_states[i][0];
      pose.position.y = expanded_states[i][1]+0.005;
      pose.position.z = expanded_states[i][2]+0.0075;
      if(expanded_states[i][3]>=100)
        size = 0.01;
      else
        size = 0.015;
      raviz_->visualizeText(pose, boost::lexical_cast<std::string>(expanded_states[i][3]), "expansions-heuristic", i, color,size);
       usleep(4000);
    }
    ROS_INFO("[node] Displaying %d expanded states",int(expanded_states.size()));
  
    //laviz_->visualizeSphere(expanded_states[0], 250, "expansions-start", 0.015);
  }
  else
    ROS_WARN("[node] No expanded states to display.");
}

void OMPLFullBodyPlannerNode::visualizeGoalPosition(const arm_navigation_msgs::Constraints &goal_pose)
{
  geometry_msgs::Pose pose;
  pose.position = goal_pose.position_constraints[0].position;
  pose.orientation = goal_pose.orientation_constraints[0].orientation;
  laviz_->visualizePose(pose, "goal_pose");
  ROS_DEBUG("[node] publishing goal marker visualizations.");
}

void OMPLFullBodyPlannerNode::displayShortestPath()
{
  // right arm path
  dpath_ = sbpl_arm_env_.getShortestPath(0);

  if(dpath_.empty())
  {
    ROS_INFO("The heuristic path has a length of 0");
    return;
  }
  else
    ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.",int(dpath_.size()));
 
  raviz_->visualizeSpheres(dpath_, 45, "right_heuristic_path", 0.03);

  // left arm path
  dpath_ = sbpl_arm_env_.getShortestPath(1);

  if(dpath_.empty())
  {
    ROS_INFO("The heuristic path has a length of 0");
    return;
  }
  else
    ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.",int(dpath_.size()));
 
  laviz_->visualizeSpheres(dpath_, 100, "left_heuristic_path", 0.03);

  // object path
  dpath_ = sbpl_arm_env_.getShortestPath(2);

  if(dpath_.empty())
  {
    ROS_INFO("The heuristic path has a length of 0");
    return;
  }
  else
    ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.",int(dpath_.size()));
 
  laviz_->visualizeSpheres(dpath_, 268, "object_heuristic_path", 0.03);
}

void OMPLFullBodyPlannerNode::visualizeEndEffectorPath()
{
  std::vector<geometry_msgs::Point> points;
  std::vector<std::vector<double> > path, path0, path1;

  sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

  points.resize(path.size());

  for(size_t i = 0; i < path.size(); ++i)
  {
    if(path[i][3] == 1)
      path1.push_back(path[i]);
    else
      path0.push_back(path[i]);

    points[i].x = path[i][0];
    points[i].y = path[i][1];
    points[i].z = path[i][2];
  }

  ROS_DEBUG("[viz] path0: %d, path1: %d", int(path0.size()), int(path1.size())); 
  laviz_->visualizeSpheres(path0, 230, "end_effector_path0", 0.01);
  laviz_->visualizeSpheres(path1, 30, "end_effector_path1", 0.01);
  laviz_->visualizeLine(points,"end_effector_path", 0, 120, 0.005);
}


void OMPLFullBodyPlannerNode::visualizeObjectPath()
{
  std::vector<geometry_msgs::Point> points;
  std::vector<std::vector<double> > path;

  sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

  points.resize(path.size());

  for(size_t i = 0; i < path.size(); ++i)
  {
    points[i].x = path[i][0];
    points[i].y = path[i][1];
    points[i].z = path[i][2];
  }

  ROS_INFO("[node] path length: %d", int(path.size())); 
  laviz_->visualizeSpheres(path, 0, "object_path", 0.015);
  usleep(50000);
  laviz_->visualizeLine(points,"object_path_line", 0, 120, 0.005);
}

void OMPLFullBodyPlannerNode::visualizeAttachedObjectPath()
{
  std::vector<double> pose, radius;
  std::vector<std::vector<double> > path, spheres;

  sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

  for(size_t i = 0; i < path.size(); ++i)
  {
    pose = path[i];
    pose.erase(pose.begin()+3);
  
    cspace_->getAttachedObjectInWorldFrame(pose,spheres);

    radius.resize(spheres.size());
    for(size_t j = 0; j < spheres.size(); ++j)
      radius[j] = spheres[j][3];
   
    laviz_->setReferenceFrame("base_footprint"); 
    laviz_->visualizeSpheres(spheres, 238, "attached_object_1" +  boost::lexical_cast<std::string>(i), radius);
    ROS_DEBUG("[node] [%d] Publishing %d spheres for the attached object. {pose: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f}",int(i),int(spheres.size()), pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);
    usleep(300);
    laviz_->setReferenceFrame("map"); 
  }
}

void OMPLFullBodyPlannerNode::visualizeExpansionsPerHValue()
{
  int last_dist=0, dist=0, total_num_hvals=0;
  std::vector<std::vector<double> > path;
  std::string text="";
  std::vector<double> color(4,1.0);
  std::vector<int> hvals, num_hvals;
  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

  sbpl_arm_env_.getHeuristicDebugStats(hvals, num_hvals);

  if(hvals.size() != num_hvals.size())
    ROS_WARN("Heuristic debugging information doesn't make sense.");
  for(size_t i = 0; i < hvals.size(); ++i)
    ROS_DEBUG("[%d] h-val: %d  # exp: %d", int(i), hvals[i], num_hvals[i]);

  for(size_t i = 0; i < path.size(); ++i)
  {
    dist = sbpl_arm_env_.getDijkstraDistance(path[i][0], path[i][1], path[i][2]);

    if(dist == last_dist)
    {
      ROS_DEBUG("[%d] dist: %d last_dist: %d CONTINUE",int(i),dist,last_dist);
      continue;
    }
    ROS_DEBUG("[%d] dist: %d last_dist: %d",int(i),dist,last_dist);

    //text = "";
    total_num_hvals = 0;
    for(size_t j = 0; j < (hvals.size()/2); ++j)
    {
      if(hvals[j] == dist)
      {
        total_num_hvals += num_hvals[j];
        /* 
        if(text == "")
          text = boost::lexical_cast<std::string>(num_hvals[j]);
        else
          text = text + "," + boost::lexical_cast<std::string>(num_hvals[j]);
        */
      }
    }

    if(total_num_hvals == 0)
      total_num_hvals = 1;

    text = boost::lexical_cast<std::string>(total_num_hvals);
    pose.position.x = path[i][0];
    pose.position.y = path[i][1] - 0.02;
    pose.position.z = path[i][2];
    
    raviz_->visualizeText(pose, text, "num_expansions_per_hval", i,color,0.03);
    last_dist = dist;
  }
}

void OMPLFullBodyPlannerNode::visualizeHeuristicInfo()
{
  int dist = 0;
  std::string text="";
  std::vector<double> color(4,1.0);
  std::vector<int> hvals, num_hvals;
  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  sbpl_arm_env_.getHeuristicDebugStats(hvals, num_hvals);

  if(hvals.size() != num_hvals.size())
    ROS_WARN("[node] Heuristic debugging information doesn't make sense.");

  ROS_DEBUG("\b Number of expansions per H-value");
  for(size_t i = 0; i < hvals.size(); ++i)
    ROS_DEBUG("[%d] h-val: %d  # exp: %d", int(i), hvals[i], num_hvals[i]);

  dpath0_.clear();
  dpath0_ = sbpl_arm_env_.getShortestPath(0);

  //check if the list is empty
  if(dpath0_.empty())
  {
    ROS_INFO("[node] The heuristic path has a length of 0");
    return;
  }
  else
    ROS_INFO("node] Visualizing right arm heuristic path from start to goal with %d waypoints.",int(dpath0_.size()));

  raviz_->visualizeSpheres(dpath0_, 45, "right_arm_heuristic_path", 0.04);

  ROS_DEBUG("[node] Right Arm Heuristic Visualization:");
  for(size_t i = 0; i < dpath0_.size(); ++i)
  {
    dist = sbpl_arm_env_.getDijkstraDistance(dpath0_[i][0], dpath0_[i][1], dpath0_[i][2]);
    text = "";
    for(size_t j = 0; j < (hvals.size()/2); ++j)
    {
      if(hvals[j] == dist)
      {
        if(text == "")
          text = boost::lexical_cast<std::string>(num_hvals[j]);
        else
          text = text + "," + boost::lexical_cast<std::string>(num_hvals[j]);
      }
    }
    pose.position.x = dpath0_[i][0];
    pose.position.y = dpath0_[i][1];
    pose.position.z = dpath0_[i][2];
    raviz_->visualizeText(pose, text, "right_arm_heuristic_text", i,color,0.03);
    ROS_DEBUG("[%d] Distance: %d  Text: %s  Pose: %0.3f %0.3f %0.3f", int(i), dist, text.c_str(), dpath0_[i][0], dpath0_[i][1], dpath0_[i][2]);
  }

  //get the heuristic for the left arm path
  dpath1_.clear();
  dpath1_ = sbpl_arm_env_.getShortestPath(1);

  //check if the list is empty
  if(dpath1_.empty())
  {
    ROS_INFO("The heuristic path has a length of 0");
    return;
  }
  else
    ROS_INFO("Visualizing left arm heuristic path from start to goal with %d waypoints.",int(dpath1_.size()));

  //visualize the path
  laviz_->visualizeSpheres(dpath1_, 100, "left_arm_heuristic_path", 0.04);

  //visualize the # expansions for each heuristic value
  ROS_DEBUG("Left Arm Heuristic Visualization:");
  for(size_t i = 0; i < dpath1_.size(); ++i)
  {
    dist = sbpl_arm_env_.getDijkstraDistance(dpath1_[i][0], dpath1_[i][1], dpath1_[i][2]);
    text = "";
    for(size_t j = 0; j < (hvals.size()/2); ++j)
    {
      if(hvals[j] == dist)
      {
        if(text == "")
          text = boost::lexical_cast<std::string>(num_hvals[j]);
        else
          text = text + "," + boost::lexical_cast<std::string>(num_hvals[j]);
      }
      if(dist == 1000000000)
        text = "impossible";
    }
    pose.position.x = dpath1_[i][0];
    pose.position.y = dpath1_[i][1];
    pose.position.z = dpath1_[i][2];
    laviz_->visualizeText(pose, text, "left_arm_heuristic_text", i,color,0.03);
    ROS_DEBUG("[%d] Distance: %d  Text: %s  Pose: %0.3f %0.3f %0.3f", int(i), dist, text.c_str(), dpath1_[i][0], dpath1_[i][1], dpath1_[i][2]);
  }
}

void OMPLFullBodyPlannerNode::visualizeGoal(geometry_msgs::Pose goal)
{
  /*
  tf::Pose tgoal, tright, tleft;
  geometry_msgs::Pose mright, mleft;

  tf::poseMsgToTF(goal,tgoal);
  tf::poseMsgToTF(rarm_object_offset_,tright);
  tf::poseMsgToTF(larm_object_offset_,tleft);

  tright = tgoal*tright;
  tleft = tgoal*tleft;

  tf::poseTFToMsg(tright, mright);
  tf::poseTFToMsg(tleft, mleft);
  laviz_->visualizePose(mright, "right_gripper");
  laviz_->visualizePose(mleft, "left_gripper");
  */

  laviz_->visualizePose(goal, "goal");
  goal.position.z +=0.05;
  laviz_->visualizeText(goal, "goal", "goal-title", 0, 360);
}

void OMPLFullBodyPlannerNode::visualizeCollisionObjects()
{
  std::vector<geometry_msgs::Pose> poses;
  std::vector<std::vector<double> > points(1,std::vector<double>(3,0));
  std::vector<double> color(4,1);
  color[2] = 0;

  cspace_->getCollisionObjectVoxelPoses(poses);

  points.resize(poses.size());
  for(size_t i = 0; i < poses.size(); ++i)
  {
    points[i].resize(3);
    points[i][0] = poses[i].position.x;
    points[i][1] = poses[i].position.y;
    points[i][2] = poses[i].position.z;
  }

  ROS_DEBUG("[node] Displaying %d known collision object voxels.", int(points.size()));
  laviz_->visualizeBasicStates(points, color, "known_objects", 0.01);
}

void OMPLFullBodyPlannerNode::visualizeAttachedObject()
{
  geometry_msgs::Pose wpose,owpose;
  tf::Pose tf_wpose, tf_opose, tf_owpose;
  std::vector<std::vector<double> > points;
  std::vector<double> color(4,1), pose(6,0);
  std::vector<double> radius;
  color[2] = 0;

  if(!computeFK(rangles_,"right_arm",wpose))
    ROS_ERROR("[node] FK service failed.");

  tf::poseMsgToTF(wpose,tf_wpose);
  tf::poseMsgToTF(rarm_object_offset_,tf_opose);

  tf_owpose= tf_wpose*tf_opose.inverse();
  tf::poseTFToMsg(tf_owpose,owpose);

  pose[0] = owpose.position.x;
  pose[1] = owpose.position.y;
  pose[2] = owpose.position.z;
  tf_owpose.getBasis().getRPY(pose[3],pose[4],pose[5]);
  
  ROS_DEBUG("[node] object pose: %f %f %f %f %f %f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

  cspace_->getAttachedObjectInWorldFrame(pose,points);

  ROS_DEBUG("[node] Displaying %d spheres for the attached object.", int(points.size()));
  radius.resize(points.size());
  for(size_t i = 0; i < points.size(); ++i)
    radius[i] = points[i][3];
 
  laviz_->setReferenceFrame("base_footprint"); 
  laviz_->visualizeSpheres(points, 163, "attached_object", radius);
  laviz_->setReferenceFrame("map"); 
}

void OMPLFullBodyPlannerNode::printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath)
{
  double roll,pitch,yaw;
  tf::Pose tf_pose;
  geometry_msgs::Pose pose;
  geometry_msgs::PoseStamped pose_in, pose_out;
  std::vector<double> jnt_pos(num_joints_,0);
  ROS_INFO("Right Arm Path:");
  for(size_t i = 0; i < rpath.size(); i++)
  {
    for(int j = 0; j < num_joints_; ++j)
      jnt_pos[j] = rpath[i].positions[j];

    computeFK(jnt_pos, "right_arm", pose);
    tf::poseMsgToTF(pose, tf_pose);
    tf_pose.getBasis().getRPY(roll,pitch,yaw);

    ROS_INFO("%3d: %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n   xyz: %2.3f %2.3f %2.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.2f %0.2f %0.2f %0.2f", int(i),rpath[i].positions[0],rpath[i].positions[1],rpath[i].positions[2],rpath[i].positions[3],rpath[i].positions[4],rpath[i].positions[5],rpath[i].positions[6],pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }

  ROS_INFO("Left Arm Path:");
  for(size_t i = 0; i < lpath.size(); i++)
  {
    for(int j = 0; j < num_joints_; ++j)
      jnt_pos[j] = lpath[i].positions[j];

    computeFK(jnt_pos, "left_arm", pose);
    tf::poseMsgToTF(pose, tf_pose);
    tf_pose.getBasis().getRPY(roll,pitch,yaw);

    ROS_INFO("%3d: %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n   xyz: %2.3f %2.3f %2.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.2f %0.2f %0.2f %0.2f", int(i),lpath[i].positions[0],lpath[i].positions[1],lpath[i].positions[2],lpath[i].positions[3],lpath[i].positions[4],lpath[i].positions[5],lpath[i].positions[6],pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }
  ROS_INFO("Body Path:");
  for(size_t i = 0; i < bpath.size(); i++)
  {
    ROS_INFO("%3d: x: %0.3f  y: %0.3f  torso: %2.3f  theta: %0.3f", int(i),bpath[i].positions[0],bpath[i].positions[1],bpath[i].positions[2],bpath[i].positions[3]);
  }
}

void OMPLFullBodyPlannerNode::printPath(FILE* fOut, const std::vector<std::vector<double> > path)
{
  time_t init_time;
  time(&init_time);
  std::string str_time(asctime (localtime(&init_time)));

  fprintf(fOut, "%s", str_time.c_str());
  for(unsigned int i = 0; i < path.size(); i++)
    fprintf(fOut, "state %3d: %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",i,path[i][0],path[i][1],path[i][2],path[i][3],path[i][4],path[i][5],path[i][6]);
  fprintf(fOut,"---------------------------------");
}

void OMPLFullBodyPlannerNode::visualizeHeuristicGrid()
{
  bool indent =  false, x_indent = false;;
  int dist=0,cntr=0;
  double y_left=0, x_left=0;
  std::vector<double> color(4), blue(4,0.0),white(4,1.0), red(4,0.0), orange(4,0.0);
  red[0] = 1.0;
  red[3] = 1.0;
  blue[2] = 0.5;
  blue[3] = 1.0;
  orange[0] = 0.5;
  orange[1] = 0.5;
  orange[3] = 1.0;
  std::vector<double> goal(3);
  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  for(double z = z_min_; z < z_max_; z+=z_inc_)
  {
    y_left = y_min_;
    //x_left = x_min_;
    if(indent)
    {
      y_left = y_min_ - (y_inc_/2.0);
      //x_left = x_min_ - (x_inc_/2.0);
      //ROS_INFO("Indent x: %f y: %f", x_left, y_left);
      color = blue;
      indent = false;
    }
    else
    {
      color = white;
      //ROS_INFO("Don't Indent");
      indent = true;
    }

    for(double y = y_left; y < y_max_; y+=y_inc_)
    {
      x_left = x_min_;
      if(x_indent)
      {
        x_left = x_min_ - (x_inc_/2.0);
        x_indent = false;
        color = orange;
      }
      else
      {
        x_indent = true;
        color = blue;
      }

      for(double x = x_left; x < x_max_; x+=x_inc_)
      {
        dist = sbpl_arm_env_.getDijkstraDistance(x, y, z);

        if(dist == 0)
        {
          goal[0] = x; goal[1] = y; goal[2] = z; 
          raviz_->visualizeSphere(goal, 160, "heuristic_grid_goal-"+boost::lexical_cast<std::string>(cntr), 0.015);
        }
        else if(dist < 10000)
        {
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;
          raviz_->visualizeText(pose, boost::lexical_cast<std::string>(dist), "heuristic_grid", cntr, color,0.015);
        } 
        else
        {
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;
          raviz_->visualizeText(pose, "x", "heuristic_grid", cntr, red,0.02);
        } 

        usleep(5000);
        cntr++;
      }
    }
  }
}

void OMPLFullBodyPlannerNode::visualizeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  geometry_msgs::PoseStamped pose;
  std::vector<double> dim,color(4,0.0);

  for(size_t i = 0; i < object.shapes.size(); ++i)
  {
    if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      pose.pose = object.poses[i];
      pose.header = object.header;
      dim.resize(object.shapes[i].dimensions.size());
      for(size_t j = 0; j < object.shapes[i].dimensions.size(); ++j)
        dim[j] = object.shapes[i].dimensions[j];      
 
      for(size_t j = 0; j < object.shapes[i].triangles.size(); ++j)
        color[j] = double(object.shapes[i].triangles[j]) / 255.0;      
/*
      //alpha shouldn't be divided by 255
      color[j] = double(object.shapes[i].triangles[j]);
*/
      ROS_INFO("[node] Visualizing %s", object.id.c_str());  
      raviz_->visualizeCube(pose,color,object.id,int(i),dim);
    }
    else
      ROS_WARN("[node] Collision objects of type %d are not yet supported.", object.shapes[i].type);
  }
}

void OMPLFullBodyPlannerNode::changeLoggerLevel(std::string name, std::string level)
{
  //ROSCONSOLE_AUTOINIT;
  
  std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;

  ROS_INFO("[node] Setting %s to %s level", logger_name.c_str(), level.c_str());

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

  // Set the logger for this package to output all statements
  if(level.compare("debug") == 0)
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  else
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
   
  ROS_DEBUG("This is a debug statement, and should print if you enabled debug.");
}

void OMPLFullBodyPlannerNode::getRobotState(BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  rangles[0] = state->position[17];
  rangles[1] = state->position[18];
  rangles[2] = state->position[16];
  rangles[3] = state->position[20];
  rangles[4] = state->position[19];
  rangles[5] = state->position[21];
  rangles[6] = state->position[22];

  langles[0] = state->position[29];
  langles[1] = state->position[30];
  langles[2] = state->position[28];
  langles[3] = state->position[32];
  langles[4] = state->position[31];
  langles[5] = state->position[33];
  langles[6] = state->position[34];
  body_pos_.z = state->position[12];

  try
  {
    tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
    body_pos.x = base_map_transform_.getOrigin().x();
    body_pos.y = base_map_transform_.getOrigin().y();
    body_pos_.theta = 2 * atan2(base_map_transform_.getRotation().getX(), base_map_transform_.getRotation().getW());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Is there a map? The map-robot transform failed. (%s)",ex.what());
  }
}

void OMPLFullBodyPlannerNode::visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath)
{
  int length = rpath.size();
  std::vector<double> rangles(7,0), langles(7,0);
  BodyPose body_pos;

  if(rpath.size() != lpath.size() || rpath.size() != bpath.size())
  {
    ROS_ERROR("[node] The right arm, left arm and body trajectories are of unequal lengths.");
    return;
  }

  int color_inc = 240.0 / (length/throttle_); // hue: red -> blue
  ROS_DEBUG("[node] length: %d color_inc: %d throttle: %d)", length, color_inc, throttle_);

  for(int i = 0; i < length; ++i)
  {
    for(int j = 0; j < 7; ++j)
    {
      rangles[j] = rpath[i].positions[j];
      langles[j] = lpath[i].positions[j];
    }
    body_pos.x = bpath[i].positions[0];
    body_pos.y = bpath[i].positions[1];
    body_pos.z = bpath[i].positions[2];
    body_pos.theta = bpath[i].positions[3];

    if((i != length-1) && (i % throttle_ != 0))
      continue;

    rangles_ = rangles;
    visualizeAttachedObject();

    ROS_DEBUG("[node] length: %d color_inc: %d throttle: %d", length, color_inc, throttle_);
    ROS_DEBUG("[node] Visualizing waypoint #%d (i mod color_inc: %d) with color: %d (color_inc: %d, throttle: %d)", i, (i/throttle_),(i/throttle_)*color_inc, color_inc, throttle_);

    pviz_.visualizeRobot(rangles, langles, body_pos, (i/throttle_)*color_inc, "waypoint_"+boost::lexical_cast<std::string>(i),i);
    usleep(3000);
  }
}

void OMPLFullBodyPlannerNode::printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
  ROS_INFO("robot state:  %s", text.c_str());
  ROS_INFO("     x: %0.3f  y: % 0.3f  z: %0.3f yaw: % 0.3f", body_pos.x, body_pos.y, body_pos.z, body_pos.theta);
  ROS_INFO(" right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", rangles[0], rangles[1], rangles[2], rangles[3], rangles[4], rangles[5], rangles[6]);
  ROS_INFO("  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", langles[0], langles[1], langles[2], langles[3], langles[4], langles[5], langles[6]);
}

}

/* Node --------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
  ROSCONSOLE_AUTOINIT;
  ros::init(argc, argv, "sbpl_two_arm_planner");
  sbpl_two_arm_planner::OMPLFullBodyPlannerNode arm_planner;
  if(!arm_planner.init())
  {
    ROS_ERROR("Failed to initialize arm planner node. Exiting.");
    return 0;
  }

  return arm_planner.run();
}

