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

#include <warehouse_stats4/consistency.h>
#include <sbpl_geometry_utils/PathSimilarityMeasurer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

bool vis = false;

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
  int s = btraj.size();
  int step = 1;
  if(btraj.size() > 6){
    s = 6;
    step = ceil(double(btraj.size())/s);
  }
  int cnt = 0;
  int color_inc = 240.0 / s;
  for(unsigned int i=0; i<btraj.size(); i+=step){
    pviz_.visualizeRobot(rtraj[i], ltraj[i], btraj[i], cnt*color_inc, "waypoint_"+boost::lexical_cast<std::string>(cnt),cnt);
    cnt++;
    usleep(3000);
  }
  pviz_.visualizeRobot(rtraj.back(), ltraj.back(), btraj.back(), 240.0, "waypoint_"+boost::lexical_cast<std::string>(cnt),cnt);
  usleep(3000);
  //std::cin.get();

  fclose(fin);
  return true;
}

void PathParser::visualize(vector<vector<double> > path){
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/map";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "path";
  marker.id = count;
  marker.scale.x = 0.01;
  if(count==0){
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  }
  else{
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
  }
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  for(int i=1; i<path.size(); i++){
    p1.x = path[i-1][0];
    p1.y = path[i-1][1];
    p1.z = path[i-1][2];
    p2.x = path[i][0];
    p2.y = path[i][1];
    p2.z = path[i][2];
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  marker_pub_.publish(marker);
  count++;
}



void PathParser::trajectoriesToObjectPath(vector<BodyPose> btraj, vector<vector<double> > rtraj, vector<vector<double> > ltraj, vector<vector<double> >& path){
  path.clear();
  if(btraj.empty())
    return;
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
  }
  visualize(path);
}

void PathParser::printStats(char* filename, vector<vector<vector<double> > > paths, int s){
  vector<const vector<geometry_msgs::Point>* > trajectories;
  for(unsigned int i=0; i<paths.size(); i++){
    if(paths[i].empty())
      continue;
    vector<geometry_msgs::Point>* traj = new vector<geometry_msgs::Point>();
    for(unsigned int j=0; j<paths[i].size(); j++){
      geometry_msgs::Point p;
      p.x = paths[i][j][0];
      p.y = paths[i][j][1];
      p.z = paths[i][j][2];
      traj->push_back(p);
    }
    trajectories.push_back(traj);
  }

  double sim1 = sbpl::PathSimilarityMeasurer::measure(trajectories,s);
  double sim2 = sbpl::PathSimilarityMeasurer::measureDTW(trajectories,s);

  char buf[256];
  sprintf(buf,"%s_consistency.csv",filename);
  //FILE* fout = fopen(buf,"w");
  printf("%f %f\n",sim1,sim2);
  //fclose(fout);
}

void PathParser::parse(char* folder_path,int numPaths){
  count = 0;
  char buf[256];
  vector<vector<vector<double> > > learningPaths;
  vector<vector<vector<double> > > waPaths;
  vector<vector<vector<double> > > rrtPaths;
  vector<vector<vector<double> > > prmPaths;
  vector<vector<vector<double> > > rrtstarPaths;
  vector<vector<vector<double> > > rrtstarfirstPaths;
  int s = 0;

  //for(int i=0; i<numPaths; i++){
  for(int i=1; i<3; i++){
    vector<BodyPose> btraj;
    vector<vector<double> > rtraj;
    vector<vector<double> > ltraj;
    vector<vector<double> > path;

    //learning
    sprintf(buf,"%s/learning_paths/%.4d.csv",folder_path,i);
    readTrajectories(buf,btraj,rtraj,ltraj);
    //trajectoriesToObjectPath(btraj, rtraj, ltraj, path);
    learningPaths.push_back(path);
    s = (path.size() > s ? path.size() : s);

    /*
    //Weighted A*
    sprintf(buf,"%s/no_learning_paths/%.4d.csv",folder_path,i);
    readTrajectories(buf,btraj,rtraj,ltraj);
    trajectoriesToObjectPath(btraj, rtraj, ltraj, path);
    waPaths.push_back(path);
    s = (path.size() > s ? path.size() : s);

    */
    //rrt connect
    sprintf(buf,"%s/rrt_paths/%.4d.csv",folder_path,i);
    readTrajectories(buf,btraj,rtraj,ltraj);
    trajectoriesToObjectPath(btraj, rtraj, ltraj, path);
    rrtPaths.push_back(path);
    s = (path.size() > s ? path.size() : s);
    /*

    //prm
    sprintf(buf,"%s/prm_paths/%.4d.csv",folder_path,i);
    readTrajectories(buf,btraj,rtraj,ltraj);
    trajectoriesToObjectPath(btraj, rtraj, ltraj, path);
    prmPaths.push_back(path);
    s = (path.size() > s ? path.size() : s);
    */

    //rrt*
    /*
    sprintf(buf,"%s/rrt_star_paths/%.4d.csv",folder_path,i);
    readTrajectories(buf,btraj,rtraj,ltraj);
    trajectoriesToObjectPath(btraj, rtraj, ltraj, path);
    rrtstarPaths.push_back(path);
    s = (path.size() > s ? path.size() : s);

    //rrt* first solution
    sprintf(buf,"%s/rrt_star_first_soln_paths/%.4d.csv",folder_path,i);
    readTrajectories(buf,btraj,rtraj,ltraj);
    trajectoriesToObjectPath(btraj, rtraj, ltraj, path);
    rrtstarfirstPaths.push_back(path);
    s = (path.size() > s ? path.size() : s);
    */
  }
  //s = 4304;
  printf("using %d waypoints\n",s);
  //sprintf(buf,"%s/learning",folder_path);
  //printStats(buf,learningPaths,s);
  //sprintf(buf,"%s/no_learning",folder_path);
  //printStats(buf,waPaths);
  sprintf(buf,"%s/rrt",folder_path);
  printStats(buf,rrtPaths,s);
  //sprintf(buf,"%s/prm",folder_path);
  //printStats(buf,prmPaths,s);
  //sprintf(buf,"%s/rrt_star",folder_path);
  //printStats(buf,rrtstarPaths,s);
  //sprintf(buf,"%s/rrt_star_first",folder_path);
  //printStats(buf,rrtstarfirstPaths,s);
}


/* Node --------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_parser");
  PathParser parser;
  if(!parser.init())
    ROS_ERROR("Failed to initialize arm planner node. Exiting.");
  vis = atoi(argv[3]);
  parser.parse(argv[1],atoi(argv[2]));
  return 0;
}

