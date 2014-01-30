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

#include <compute_stats/computePaths.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <dirent.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

bool vis = false;
bool volume_and_area = false;

/** Initializers -------------------------------------------------------------*/
PathParser::PathParser() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1), collision_map_filter_(NULL),grid_(NULL)
{
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

PathParser::~PathParser()
{
  if(planner_ != NULL)
    delete planner_;
}

bool PathParser::init()
{
  //planner
  node_handle_.param("planner/search_mode", search_mode_, false); //true: stop after first solution
  node_handle_.param("planner/allocated_time", allocated_time_, 1200.0);
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

  node_handle_.param ("use_collision_map_from_sensors", use_collision_map_from_sensors_, true);
  node_handle_.param ("right_arm_pose_on_object_x", rarm_object_offset_.position.x, 0.0);
  node_handle_.param ("right_arm_pose_on_object_y", rarm_object_offset_.position.y, -0.15);
  node_handle_.param ("right_arm_pose_on_object_z", rarm_object_offset_.position.z, 0.0);
  node_handle_.param ("left_arm_pose_on_object_x", larm_object_offset_.position.x, 0.0);
  node_handle_.param ("left_arm_pose_on_object_y", larm_object_offset_.position.y, 0.15);
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

  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  pointCloudPub = node_handle_.advertise<sensor_msgs::PointCloud2>("volume", 1);

  map_frame_ = "map";

  //initialize planner
  if(!initializePlannerAndEnvironment())
    return false;

  collision_object_subscriber_ = root_handle_.subscribe("collision_object", 5, &PathParser::collisionObjectCallback, this);

  planner_initialized_ = true;

  ROS_INFO("[node] The SBPL arm planner node initialized succesfully.");
  return true;
}

bool PathParser::initializePlannerAndEnvironment()
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

  ROS_INFO("[node] Initialized sbpl planning environment.");

  rangles_[0] = 0.256421;
  rangles_[1] = 0.827628;
  rangles_[2] = 0.000000;
  rangles_[3] = -0.426053;
  rangles_[4] = 0.893944;
  rangles_[5] = -0.595797;
  rangles_[6] = -0.800253;
  langles_[0] = 0.006191;
  langles_[1] = 0.688923;
  langles_[2] = 0.314159;
  langles_[3] = -0.153169;
  langles_[4] = -0.002098;
  langles_[5] = -0.560216;
  langles_[6] = -0.239085;
  body_pos_.x = 1.840000;
  body_pos_.y = 1.380000;
  body_pos_.z = 0.000000;
  body_pos_.theta = 1.963495;
  btQuaternion btoffset;
  geometry_msgs::Pose rarm;
  rarm.position.x = -0.20;
  rarm.position.y = -0.1;
  rarm.position.z = 0.0;
  btoffset.setRPY(0.0,0.0,0.0);
  tf::quaternionTFToMsg(btoffset,rarm.orientation);
  geometry_msgs::Pose larm;
  larm.position.x = -0.20;
  larm.position.y = 0.1;
  larm.position.z = 0.0;
  btoffset.setRPY(0.0,0.0,0.0);
  tf::quaternionTFToMsg(btoffset,larm.orientation);
  geometry_msgs::Pose start;
  if(!setStart(start, rarm, larm)){
    ROS_ERROR("[node] Failed to set the starting configuration.");
    return false;
  }

  return true;
}

void PathParser::collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object)
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

}

void PathParser::visualizeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
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
      //raviz_->visualizeCube(pose,color,object.id,int(i),dim);
    }
    else
      ROS_WARN("[node] Collision objects of type %d are not yet supported.", object.shapes[i].type);
  }
}

/** Planner Interface  -------------------------------------------------------*/
bool PathParser::setStart(geometry_msgs::Pose start, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object)
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

  return true;
}

void PathParser::visPath(vector<vector<vector<double> > > paths1, vector<vector<vector<double> > > bpaths1,
                         vector<vector<vector<double> > > paths2, vector<vector<vector<double> > > bpaths2){
  for(unsigned int i=0; i<paths1.size(); i++){
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time(0);
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.02;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.ns = "object path";
      marker.id = 0;
      marker.lifetime = ros::Duration(0);

      double length = 0;
      geometry_msgs::Point p;
      p.x = paths1[i][0][0];
      p.y = paths1[i][0][1];
      p.z = paths1[i][0][2];
      marker.points.push_back(p);
      for(unsigned int j=10; j<paths1[i].size(); j+=10){
        p.x = paths1[i][j][0];
        p.y = paths1[i][j][1];
        p.z = paths1[i][j][2];
        marker.points.push_back(p);
        double dx = paths1[i][j][0] - paths1[i][j-10][0];
        double dy = paths1[i][j][1] - paths1[i][j-10][1];
        double dz = paths1[i][j][2] - paths1[i][j-10][2];
        length += sqrt(dx*dx + dy*dy + dz*dz);
      }
      printf("e-graphs: %f\n",length);
      marker_pub_.publish(marker);

      visualization_msgs::Marker marker2;
      marker2.header.frame_id = "map";
      marker2.header.stamp = ros::Time(0);
      marker2.type = visualization_msgs::Marker::LINE_STRIP;
      marker2.action = visualization_msgs::Marker::ADD;
      marker2.scale.x = 0.02;
      marker2.color.r = 0.0;
      marker2.color.g = 0.0;
      marker2.color.b = 0.3;
      marker2.color.a = 1.0;
      marker2.ns = "base path";
      marker2.id = 0;
      marker2.lifetime = ros::Duration(0);

      length = 0;
      p.x = bpaths1[i][0][0];
      p.y = bpaths1[i][0][1];
      p.z = bpaths1[i][0][2];
      marker2.points.push_back(p);
      for(unsigned int j=1; j<bpaths1[i].size(); j++){
        p.x = bpaths1[i][j][0];
        p.y = bpaths1[i][j][1];
        p.z = bpaths1[i][j][2];
        marker2.points.push_back(p);
        double dx = bpaths1[i][j][0] - bpaths1[i][j-1][0];
        double dy = bpaths1[i][j][1] - bpaths1[i][j-1][1];
        double dz = bpaths1[i][j][2] - bpaths1[i][j-1][2];
        length += sqrt(dx*dx + dy*dy + dz*dz);
      }
      marker_pub_.publish(marker2);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time(0);
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.02;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.ns = "object path 2";
      marker.id = 0;
      marker.lifetime = ros::Duration(0);

      double length = 0;
      geometry_msgs::Point p;
      p.x = paths2[i][0][0];
      p.y = paths2[i][0][1];
      p.z = paths2[i][0][2];
      marker.points.push_back(p);
      for(unsigned int j=1; j<paths2[i].size(); j++){
        p.x = paths2[i][j][0];
        p.y = paths2[i][j][1];
        p.z = paths2[i][j][2];
        marker.points.push_back(p);
        double dx = paths2[i][j][0] - paths2[i][j-1][0];
        double dy = paths2[i][j][1] - paths2[i][j-1][1];
        double dz = paths2[i][j][2] - paths2[i][j-1][2];
        length += sqrt(dx*dx + dy*dy + dz*dz);
      }
      printf("rrt:      %f\n",length);
      marker_pub_.publish(marker);

      visualization_msgs::Marker marker2;
      marker2.header.frame_id = "map";
      marker2.header.stamp = ros::Time(0);
      marker2.type = visualization_msgs::Marker::LINE_STRIP;
      marker2.action = visualization_msgs::Marker::ADD;
      marker2.scale.x = 0.02;
      marker2.color.r = 0.0;
      marker2.color.g = 0.3;
      marker2.color.b = 0.0;
      marker2.color.a = 1.0;
      marker2.ns = "base path 2";
      marker2.id = 0;
      marker2.lifetime = ros::Duration(0);

      length = 0;
      p.x = bpaths2[i][0][0];
      p.y = bpaths2[i][0][1];
      p.z = bpaths2[i][0][2];
      marker2.points.push_back(p);
      for(unsigned int j=1; j<bpaths2[i].size(); j++){
        p.x = bpaths2[i][j][0];
        p.y = bpaths2[i][j][1];
        p.z = bpaths2[i][j][2];
        marker2.points.push_back(p);
        double dx = bpaths2[i][j][0] - bpaths2[i][j-1][0];
        double dy = bpaths2[i][j][1] - bpaths2[i][j-1][1];
        double dz = bpaths2[i][j][2] - bpaths2[i][j-1][2];
        length += sqrt(dx*dx + dy*dy + dz*dz);
      }
      marker_pub_.publish(marker2);
    }
    std::cin.get();
  }
}

bool PathParser::readTrajectories(char* path, vector<BodyPose>& btraj, vector<vector<double> >& rtraj, vector<vector<double> >& ltraj){
  btraj.clear();
  rtraj.clear();
  ltraj.clear();
  FILE* fin = fopen(path,"r");
  if(!fin){
    printf("%s doesn't exist!\n",path);
    return false;
  }
  BodyPose b;
  vector<double> r(7,0);
  vector<double> l(7,0);
  while(fscanf(fin, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, \n",&(b.x),&(b.y),&(b.theta),&(b.z),
                &(r[0]),&(r[1]),&(r[2]),&(r[3]),&(r[4]),&(r[5]),&(r[6]),
                &(l[0]),&(l[1]),&(l[2]),&(l[3]),&(l[4]),&(l[5]),&(l[6]))>0){
    btraj.push_back(b);
    rtraj.push_back(r);
    ltraj.push_back(l);
  }

  printf("%s\n",path);
  if(vis){
    int s = btraj.size();
    int step = 1;
    if(btraj.size() > 6){
      s = 6;
      step = ceil(double(btraj.size())/s);
    }
    int cnt = 0;
    int color_inc = 240.0 / s;
    for(unsigned int i=0; i<btraj.size(); i+=step){
      pviz_.visualizeRobot(rtraj[i], ltraj[i], btraj[i], cnt*color_inc, "waypoint_",0);
      cnt++;
      usleep(3000);
      std::cin.get();
    }
    pviz_.visualizeRobot(rtraj.back(), ltraj.back(), btraj.back(), 240.0, "waypoint_",0);
    usleep(3000);
    std::cin.get();
  }

  fclose(fin);
  return true;
}

void PathParser::trajectoriesToObjectPath(vector<BodyPose> btraj, vector<vector<double> > rtraj, vector<vector<double> > ltraj, double& pathLength, double& bpathLength){
  if(btraj.empty()){
    pathLength = 0;
    bpathLength = 0;
    return;
  }
  vector<vector<double> > path;
  vector<vector<double> > bpath;
  vector<double> pt(3,0);
  for(unsigned int i=0; i<btraj.size(); i++){
    short unsigned int xi,yi,zi,yawi;
    sbpl_arm_env_.computeObjectPose(btraj[i], rtraj[i], xi, yi, zi, yawi);
    double x,y,z;
    sbpl_arm_env_.discToWorldXYZ(xi, yi, zi, x, y, z, true);
    pt[0] = x;
    pt[1] = y;
    pt[2] = z;
    path.push_back(pt);
    pt[0] = btraj[i].x;
    pt[1] = btraj[i].y;
    pt[2] = 0;
    bpath.push_back(pt);
  }
  
  pathLength = computePathLength(path);
  bpathLength = computePathLength(bpath);
}

double PathParser::computePathLength(vector<vector<double> > path){
  double length = 0;
  for(unsigned int i=1; i<path.size(); i++){
    double dx = path[i][0] - path[i-1][0];
    double dy = path[i][1] - path[i-1][1];
    double dz = path[i][2] - path[i-1][2];
    length += sqrt(dx*dx + dy*dy + dz*dz);
  }
  return length;
}

void PathParser::getVolume(vector<BodyPose> btraj, vector<vector<double> > rtraj, vector<vector<double> > ltraj, double& volume, double& area){
  volume = 0;
  area = 0;
  if(!volume_and_area)
    return;
  double res = 0.02;
  if(btraj.empty())
    return;
  vector<vector<double> > spheres;
  vector<vector<double> > voxels;
  //for(unsigned int i=0; i<1; i++)
  for(unsigned int i=0; i<btraj.size(); i++)
    sbpl_arm_env_.getCollisionSpheres(btraj[i], rtraj[i], ltraj[i], spheres);
  for(unsigned int i=0; i<spheres.size(); i++){
    spheres[i][0] *= res;
    spheres[i][1] *= res;
    spheres[i][2] *= res;
    spheres[i][3] *= res;
  }
  /*
  for(unsigned int i=0; i<spheres.size(); i++){
    printf("weee %d\n",i);
    voxels.clear();
    vector<vector<double> > temp;
    temp.push_back(spheres[i]);
    printf("%d\n",temp.size());
    printf("%f %f %f %f\n",temp[0][0],temp[0][1],temp[0][2],temp[0][3]);
    sbpl::Voxelizer::voxelizeSphereListQAD(temp, 0.04, true, voxels, volume);
    printf("volume %f\n",volume);
  }
  */
  sbpl::Voxelizer::voxelizeSphereListQAD(spheres, res, true, voxels, volume);
  double minXc = 1000000000000.0;
  double maxXc = -1000000000000.0;
  double minYc = 1000000000000.0;
  double maxYc = -1000000000000.0;
  for(unsigned int i=0; i<voxels.size(); i++){
    if(voxels[i][0]<minXc)
      minXc = voxels[i][0];
    if(voxels[i][0]>maxXc)
      maxXc = voxels[i][0];
    if(voxels[i][1]<minYc)
      minYc = voxels[i][1];
    if(voxels[i][1]>maxYc)
      maxYc = voxels[i][1];
  }
  int sx = (maxXc - minXc)/res + 1;
  int sy = (maxYc - minYc)/res + 1;
  bool** grid = new bool*[sx];
  for(int x=0; x<sx; x++){
    grid[x] = new bool[sy];
    for(int y=0; y<sy; y++)
      grid[x][y] = false;
  }
  for(unsigned int i=0; i<voxels.size(); i++){
    int x = (voxels[i][0]-minXc)/res;
    int y = (voxels[i][1]-minYc)/res;
    grid[x][y] = true;
  }
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  for(int x=0; x<sx; x++){
    for(int y=0; y<sy; y++){
      if(grid[x][y]){
        area += res*res;
        //pclCloud.push_back(pcl::PointXYZ(x*res+minXc, y*res+minYc, 0));
      }
    }
  }
  for(int x=0; x<sx; x++)
    delete[] grid[x];
  delete[] grid;


  //pcl::PointCloud<pcl::PointXYZ> pclCloud;
  for(unsigned int i=0; i<voxels.size(); i++)
    pclCloud.push_back(pcl::PointXYZ(voxels[i][0], voxels[i][1], voxels[i][2]));
  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg (pclCloud, cloud);
  cloud.header.frame_id = "map";
  cloud.header.stamp = ros::Time::now();
  pointCloudPub.publish(cloud);

  printf("area=%f volume=%f\n",area,volume);
  if(vis)
    std::cin.get();
}

void PathParser::printStats(char* filename, vector<double> volumes, vector<double> areas, vector<double> objPathLen, vector<double> basePathLen){
  if(volumes.size()!=areas.size() || objPathLen.size()!=areas.size() || basePathLen.size()!=areas.size()){
    printf("metric sizes don't line up!\n");
    exit(-1);
  }
  char buf[256];
  sprintf(buf,"%s_metrics.csv",filename);
  FILE* fout = fopen(buf,"w");
  for(unsigned int i=0; i<volumes.size(); i++)
    fprintf(fout,"%f, %f, %f, %f\n",volumes[i],areas[i],objPathLen[i],basePathLen[i]);
  fclose(fout);
}

void PathParser::parse(char* folder_path,int numPaths){
  char file_path[512];
  char stats_path[512];

  //iterate over the directories
  DIR *dir = opendir(folder_path);
  struct dirent *entry = readdir(dir);
  while (entry != NULL){
    if (entry->d_type == DT_DIR && entry->d_name[0]!='.'){

      //read each path file and generate quality metrics
      vector<double> volumes;
      vector<double> areas;
      vector<double> objPathLengths;
      vector<double> basePathLengths;
      for(int i=0; i<numPaths; i++){
        vector<BodyPose> btraj;
        vector<vector<double> > rtraj;
        vector<vector<double> > ltraj;
        double volume;
        double area;
        double objPathLen;
        double basePathLen;

        sprintf(file_path,"%s/%s/%.4d.csv",folder_path,entry->d_name,i);
        readTrajectories(file_path,btraj,rtraj,ltraj);
        getVolume(btraj,rtraj,ltraj,volume,area);
        volumes.push_back(volume);
        areas.push_back(area);
        trajectoriesToObjectPath(btraj, rtraj, ltraj, objPathLen, basePathLen);
        objPathLengths.push_back(objPathLen);
        basePathLengths.push_back(basePathLen);
      }
      //write the stats for this method out to file
      sprintf(stats_path,"%s/%s",folder_path,entry->d_name);
      printStats(stats_path,volumes,areas,objPathLengths,basePathLengths);
    }
    entry = readdir(dir);
  }
  closedir(dir);
}


/* Node --------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_parser");
  PathParser parser;
  if(!parser.init()){
    ROS_ERROR("Failed to initialize arm planner node. Exiting.");
    return 1;
  }
  if(argc < 4){
    ROS_ERROR("Usage: ./compute_stats path_to_folder_with_methods number_of_trials 1_or_0_visualize");
    return 1;
  }
  vis = atoi(argv[3]);
  parser.parse(argv[1],atoi(argv[2]));
  return 0;
}

