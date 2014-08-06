#include<ros/ros.h>
#include<monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner_node/fbp_stat_writer.h>
#include <sbpl/planners/mha_planner.h>

int main(int argc, char** argv){
  if(argc < 2){
    printf("provide a path to a test file!\n");
    return 1;
  }
  ros::init(argc,argv,"run_tests");
  ros::NodeHandle nh;

  monolithic_pr2_planner_node::GetMobileArmPlan::Request req;
  monolithic_pr2_planner_node::GetMobileArmPlan::Response res;

  req.planner_type = mha_planner::MetaSearchType::META_A_STAR;
  req.meta_search_type = mha_planner::PlannerType::IMHA;

  //planner parameters
  req.initial_eps = 2.0;
  req.final_eps = 2.0;
  req.dec_eps = 0.2;

  req.rarm_object.pose.position.x = 0;
  req.rarm_object.pose.position.y = 0;
  req.rarm_object.pose.position.z = 0;
  req.rarm_object.pose.orientation.x = 0;
  req.rarm_object.pose.orientation.y = 0;
  req.rarm_object.pose.orientation.z = 0;
  req.rarm_object.pose.orientation.w = 1;
  req.larm_object.pose.position.x = 0;
  req.larm_object.pose.position.y = 0;
  req.larm_object.pose.position.z = 0;
  req.larm_object.pose.orientation.x = 0;
  req.larm_object.pose.orientation.y = 0;
  req.larm_object.pose.orientation.z = 0;
  req.larm_object.pose.orientation.w = 1;

  req.xyz_tolerance = .04;
  req.roll_tolerance = .1;
  req.pitch_tolerance = .1;
  req.yaw_tolerance = .1;
  req.allocated_planning_time = 30;
  req.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;

  req.body_start.resize(4);
  req.rarm_start.resize(7);
  req.larm_start.resize(7);
  req.body_goal.resize(4);
  req.rarm_goal.resize(7);
  req.larm_goal.resize(7);


  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = ros::NodeHandle().serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  FILE* fin = fopen(argv[1],"r");
  if(!fin){
    printf("file %s does not exist\n", argv[1]);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  while(1){
    int test_num = 0;
    if(fscanf(fin,"  - test: test_%d\n    start:\n", &test_num) <= 0)
      break;

    if(fscanf(fin,"      object_xyz_wxyz: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.start.pose.position.x,&req.start.pose.position.y,&req.start.pose.position.z,
              &req.start.pose.orientation.w,&req.start.pose.orientation.x,&req.start.pose.orientation.y,&req.start.pose.orientation.z) <= 0)
      break;
    if(fscanf(fin,"      base_xyzyaw: %lf %lf %lf %lf\n",
              &req.body_start[0], &req.body_start[1],
              &req.body_start[2], &req.body_start[3]) <= 0)
      break;
    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.rarm_start[0],&req.rarm_start[1],
              &req.rarm_start[2],&req.rarm_start[3],
              &req.rarm_start[4],&req.rarm_start[5],
              &req.rarm_start[6]) <= 0)
      break;
    if(fscanf(fin,"      larm: %lf %lf %lf %lf %lf %lf %lf\n    goal:\n",
              &req.larm_start[0],&req.larm_start[1],
              &req.larm_start[2],&req.larm_start[3],
              &req.larm_start[4],&req.larm_start[5],
              &req.larm_start[6]) <= 0)
      break;
    if(fscanf(fin,"      object_xyz_wxyz: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.goal.pose.position.x,&req.goal.pose.position.y,&req.goal.pose.position.z,
              &req.goal.pose.orientation.w,&req.goal.pose.orientation.x,&req.goal.pose.orientation.y,&req.goal.pose.orientation.z) <= 0)
      break;
    if(fscanf(fin,"      base_xyzyaw: %lf %lf %lf %lf\n",
              &req.body_goal[0], &req.body_goal[1],
              &req.body_goal[2], &req.body_goal[3]) <= 0)
      break;
    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.rarm_goal[0],&req.rarm_goal[1],
              &req.rarm_goal[2],&req.rarm_goal[3],
              &req.rarm_goal[4],&req.rarm_goal[5],
              &req.rarm_goal[6]) <= 0)
      break;
    if(fscanf(fin,"      larm: %lf %lf %lf %lf %lf %lf %lf\n\n",
              &req.larm_goal[0],&req.larm_goal[1],
              &req.larm_goal[2],&req.larm_goal[3],
              &req.larm_goal[4],&req.larm_goal[5],
              &req.larm_goal[6]) <= 0)
      break;

    planner.call(req,res);
    FBPStatWriter::writeStatsToFile("mha_stats.csv", first, res.stats_field_names, res.stats);
    first = false;
  }

  return 0;
}

