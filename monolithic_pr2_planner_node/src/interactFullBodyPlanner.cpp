#include <monolithic_pr2_planner_node/interactFullBodyPlanner.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner_node/fbp_stat_writer.h>
#include <sbpl/planners/mha_planner.h>

enum MenuItems{PLAN_IMHA_ROUND_ROBIN=1,
               PLAN_IMHA_META_A_STAR,
               PLAN_IMHA_DTS,
               PLAN_SMHA_ROUND_ROBIN,
               PLAN_SMHA_META_A_STAR,
               PLAN_SMHA_DTS,
               PLAN_SMHA_DTS_PLUS,
               PLAN_SMHA_DTS_FOCAL,
               PLAN_SMHA_DTS_UNCONSTRAINED,
               INTERRUPT,
               WRITE_TO_FILE};

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_planner");

  ControlPlanner cp;

  // start the ROS main loop
  ros::spin();
}

void ControlPlanner::callPlanner(){
  while(ros::ok()){
    boost::unique_lock<boost::mutex> lock(mutex);
    call_planner_cond.wait(lock);
    lock.unlock();
    planner.call(req,res);

    static bool first = true;
    FBPStatWriter::writeStatsToFile("mha_stats.csv", first, res.stats_field_names, res.stats);
    first = false;
  }
}

void ControlPlanner::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT){
    if(feedback->menu_entry_id == MenuItems::PLAN_IMHA_ROUND_ROBIN ||
       feedback->menu_entry_id == MenuItems::PLAN_IMHA_META_A_STAR ||
       feedback->menu_entry_id == MenuItems::PLAN_IMHA_DTS ||
       feedback->menu_entry_id == MenuItems::PLAN_SMHA_ROUND_ROBIN ||
       feedback->menu_entry_id == MenuItems::PLAN_SMHA_META_A_STAR ||
       feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS ||
       feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_PLUS ||
       feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_FOCAL ||
       feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_UNCONSTRAINED){

      visualization_msgs::InteractiveMarker start_base_marker;
      int_marker_server->get("start_base",start_base_marker);
      vector<double> start_base(4,0);
      start_base[0] = start_base_marker.pose.position.x;
      start_base[1] = start_base_marker.pose.position.y;
      start_base[2] = torso_z;
      start_base[3] = tf::getYaw(start_base_marker.pose.orientation);
      visualization_msgs::InteractiveMarker goal_base_marker;
      int_marker_server->get("goal_base",goal_base_marker);
      vector<double> goal_base(4,0);
      goal_base[0] = goal_base_marker.pose.position.x;
      goal_base[1] = goal_base_marker.pose.position.y;
      goal_base[2] = torso_z;
      goal_base[3] = tf::getYaw(goal_base_marker.pose.orientation);

      visualization_msgs::InteractiveMarker start_hand_marker;
      int_marker_server->get("start_hand",start_hand_marker);
      visualization_msgs::InteractiveMarker goal_hand_marker;
      int_marker_server->get("goal_hand",goal_hand_marker);

      //start and goal configurations
      req.start.pose = start_hand_marker.pose;
      req.rarm_start = start_angles0;
      req.larm_start = angles1;
      req.goal.pose = goal_hand_marker.pose;
      req.body_start = start_base;
      req.rarm_goal = goal_angles0;
      req.larm_goal = angles1;
      req.body_goal = goal_base;

    if(feedback->menu_entry_id == MenuItems::PLAN_IMHA_ROUND_ROBIN ||
       feedback->menu_entry_id == MenuItems::PLAN_IMHA_META_A_STAR ||
       feedback->menu_entry_id == MenuItems::PLAN_IMHA_DTS)
      req.planner_type = mha_planner::PlannerType::IMHA;
    else
      req.planner_type = mha_planner::PlannerType::SMHA;

    if(feedback->menu_entry_id == MenuItems::PLAN_IMHA_META_A_STAR ||
       feedback->menu_entry_id == MenuItems::PLAN_SMHA_META_A_STAR)
      req.meta_search_type = mha_planner::MetaSearchType::META_A_STAR;
    else if(feedback->menu_entry_id == MenuItems::PLAN_IMHA_DTS ||
            feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS ||
            feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_PLUS ||
            feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_FOCAL ||
            feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_UNCONSTRAINED)
      req.meta_search_type = mha_planner::MetaSearchType::DTS;
    else
      req.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;

    // Setting the type of mha plannerr
    if(feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_PLUS)
        req.mha_type = mha_planner::MHAType::PLUS;

    else if(feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_FOCAL)
        req.mha_type = mha_planner::MHAType::FOCAL;

    else if(feedback->menu_entry_id == MenuItems::PLAN_SMHA_DTS_UNCONSTRAINED)
        req.mha_type = mha_planner::MHAType::UNCONSTRAINED;

    else
        req.mha_type = mha_planner::MHAType::ORIGINAL;

      //position of the wrist in the object's frame
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
      req.allocated_planning_time = 90;
      req.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;

      //planner parameters
      req.initial_eps = 100.0;
      req.final_eps = 100.0;
      req.dec_eps = 0.2;
      printf("plan\n");

      call_planner_cond.notify_one();
    }
    else if(feedback->menu_entry_id == MenuItems::INTERRUPT){
      printf("interrupt planner\n");
      std_msgs::Empty msg;
      interrupt_pub.publish(msg);
    }
    else if(feedback->menu_entry_id == MenuItems::WRITE_TO_FILE){
      printf("write to file\n");
      visualization_msgs::InteractiveMarker start_base_marker;
      int_marker_server->get("start_base",start_base_marker);
      visualization_msgs::InteractiveMarker goal_base_marker;
      int_marker_server->get("goal_base",goal_base_marker);
      visualization_msgs::InteractiveMarker start_hand_marker;
      int_marker_server->get("start_hand",start_hand_marker);
      visualization_msgs::InteractiveMarker goal_hand_marker;
      int_marker_server->get("goal_hand",goal_hand_marker);

      fout = fopen("fbp_tests.yaml","a");
      fprintf(fout,"  - test: test_%d\n", test_num);
      fprintf(fout,"    start:\n");
      fprintf(fout,"      object_xyz_wxyz: %f %f %f %f %f %f %f\n",
              start_hand_marker.pose.position.x,
              start_hand_marker.pose.position.y,
              start_hand_marker.pose.position.z,
              start_hand_marker.pose.orientation.w,
              start_hand_marker.pose.orientation.x,
              start_hand_marker.pose.orientation.y,
              start_hand_marker.pose.orientation.z);
      fprintf(fout,"      base_xyzyaw: %f %f %f %f\n",
              start_base_marker.pose.position.x,
              start_base_marker.pose.position.y,
              torso_z,
              tf::getYaw(start_base_marker.pose.orientation));
      fprintf(fout,"      rarm: %f %f %f %f %f %f %f\n",
              start_angles0[0],
              start_angles0[1],
              start_angles0[2],
              start_angles0[3],
              start_angles0[4],
              start_angles0[5],
              start_angles0[6]);
      fprintf(fout,"      larm: %f %f %f %f %f %f %f\n",
              angles1[0],
              angles1[1],
              angles1[2],
              angles1[3],
              angles1[4],
              angles1[5],
              angles1[6]);
      fprintf(fout,"    goal:\n");
      fprintf(fout,"      object_xyz_wxyz: %f %f %f %f %f %f %f\n",
              goal_hand_marker.pose.position.x,
              goal_hand_marker.pose.position.y,
              goal_hand_marker.pose.position.z,
              goal_hand_marker.pose.orientation.w,
              goal_hand_marker.pose.orientation.x,
              goal_hand_marker.pose.orientation.y,
              goal_hand_marker.pose.orientation.z);
      fprintf(fout,"      base_xyzyaw: %f %f %f %f\n",
              goal_base_marker.pose.position.x,
              goal_base_marker.pose.position.y,
              torso_z,
              tf::getYaw(goal_base_marker.pose.orientation));
      fprintf(fout,"      rarm: %f %f %f %f %f %f %f\n",
              goal_angles0[0],
              goal_angles0[1],
              goal_angles0[2],
              goal_angles0[3],
              goal_angles0[4],
              goal_angles0[5],
              goal_angles0[6]);
      fprintf(fout,"      larm: %f %f %f %f %f %f %f\n",
              angles1[0],
              angles1[1],
              angles1[2],
              angles1[3],
              angles1[4],
              angles1[5],
              angles1[6]);
      fprintf(fout,"\n");
      fclose(fout);
      test_num++;
    }
    else{
      ROS_ERROR("Invalid menu item");
    }
  }
  else{//movement
    bool is_start = feedback->marker_name == "start_base" || feedback->marker_name == "start_hand";
    bool is_base = feedback->marker_name == "start_base" || feedback->marker_name == "goal_base";
    if(is_base){
      vector<double>* angles0 = is_start ? &start_angles0 : &goal_angles0;
      vector<double> base(3,0);
      base[0] = feedback->pose.position.x;
      base[1] = feedback->pose.position.y;
      base[2] = tf::getYaw(feedback->pose.orientation);
      string ns( is_start ? "start" : "goal" );
      int color = is_start ? 85 : 0;
      pviz.visualizeRobot(*angles0, angles1, base, torso_z, color, ns, 0, false);
      
      //snap the interative gripper marker back on the pr2 in the last valid pose
      visualization_msgs::InteractiveMarker r_gripper_marker;
      string gripper_name( is_start ? "start_hand" : "goal_hand" );
      int_marker_server->get(gripper_name, r_gripper_marker);
      r_gripper_marker.controls[0].markers[0].color.r = 0;
      r_gripper_marker.controls[0].markers[0].color.g = 1;
      r_gripper_marker.controls[0].markers[0].color.b = 0;

      std::vector<double> fk_pose;
      kdl_robot_model_.computePlanningLinkFK(*angles0, fk_pose);
      //printf("hand was %f %f %f\n",
              //r_gripper_marker.pose.position.x,
              //r_gripper_marker.pose.position.y,
              //r_gripper_marker.pose.position.z);
      r_gripper_marker.pose.position.x = fk_pose[0]*cos(base[2])-fk_pose[1]*sin(base[2])+feedback->pose.position.x;
      r_gripper_marker.pose.position.y = fk_pose[0]*sin(base[2])+fk_pose[1]*cos(base[2])+feedback->pose.position.y;
      r_gripper_marker.pose.position.z = fk_pose[2];
      tf::Quaternion q_hand;
      q_hand.setRPY(fk_pose[3], fk_pose[4], fk_pose[5]);
      tf::Quaternion q_rot;
      q_rot.setRPY(0.0, 0.0, base[2]);
      q_hand = q_rot * q_hand;
      r_gripper_marker.pose.orientation.w = q_hand.getW();
      r_gripper_marker.pose.orientation.x = q_hand.getX();
      r_gripper_marker.pose.orientation.y = q_hand.getY();
      r_gripper_marker.pose.orientation.z = q_hand.getZ();
      //printf("hand is %f %f %f\n",
              //r_gripper_marker.pose.position.x,
              //r_gripper_marker.pose.position.y,
              //r_gripper_marker.pose.position.z);
      int_marker_server->insert(r_gripper_marker);
      int_marker_server->applyChanges();
    }
    else{//is hand
      visualization_msgs::InteractiveMarker base_marker;
      string base_name( is_start ? "start_base" : "goal_base" );
      int_marker_server->get(base_name, base_marker);

      //ik_pose needs to be in the base_footprint frame
      std::vector<double> ik_pose(7, 0);
      double dx = feedback->pose.position.x - base_marker.pose.position.x;
      double dy = feedback->pose.position.y - base_marker.pose.position.y;
      double base_yaw = tf::getYaw(base_marker.pose.orientation);
      //printf("dx=%f dy=%f yaw=%f\n",dx,dy,base_yaw);
      ik_pose[0] = dx*cos(-base_yaw) - dy*sin(-base_yaw);
      ik_pose[1] = dx*sin(-base_yaw) + dy*cos(-base_yaw);
      ik_pose[2] = feedback->pose.position.z;
      tf::Quaternion q_hand(feedback->pose.orientation.x,
                            feedback->pose.orientation.y,
                            feedback->pose.orientation.z,
                            feedback->pose.orientation.w);
      tf::Quaternion q_rot;
      q_rot.setRPY(0, 0, -base_yaw);
      q_hand = q_hand * q_rot;
      ik_pose[3] = q_hand.getX();
      ik_pose[4] = q_hand.getY();
      ik_pose[5] = q_hand.getZ();
      ik_pose[6] = q_hand.getW();

      visualization_msgs::InteractiveMarker r_gripper_marker;
      string gripper_name( is_start ? "start_hand" : "goal_hand" );
      int_marker_server->get(gripper_name, r_gripper_marker);
      std::vector<double> solution(7, 0);
      vector<double>* angles0 = is_start ? &start_angles0 : &goal_angles0;
      if(kdl_robot_model_.computeIK(ik_pose, *angles0, solution)){
        *angles0 = solution;
        vector<double> base(3,0);
        base[0] = base_marker.pose.position.x;
        base[1] = base_marker.pose.position.y;
        base[2] = tf::getYaw(base_marker.pose.orientation);
        string ns( is_start ? "start" : "goal" );
        int color = is_start ? 85 : 0;
        pviz.visualizeRobot(*angles0, angles1, base, torso_z, color, ns, 0, false);
        r_gripper_marker.controls[0].markers[0].color.r = 0;
        r_gripper_marker.controls[0].markers[0].color.g = 1;
        r_gripper_marker.controls[0].markers[0].color.b = 0;
      }
      else{
        r_gripper_marker.controls[0].markers[0].color.r = 1;
        r_gripper_marker.controls[0].markers[0].color.g = 0;
        r_gripper_marker.controls[0].markers[0].color.b = 0;
      }
      int_marker_server->insert(r_gripper_marker);
      int_marker_server->applyChanges();
    }
  }
}

ControlPlanner::ControlPlanner(){
  fout = fopen("fbp_tests.yaml","w");
  fprintf(fout, "experiments:\n\n");
  fclose(fout);
  test_num = 0;

  //initialize joint angles for the start and goal markers
  start_angles0.resize(7);
  start_angles0[0] = -0.002109;
  start_angles0[1] = 0.655300;
  start_angles0[2] = 0.000000;
  start_angles0[3] = -1.517650;
  start_angles0[4] = -3.138816;
  start_angles0[5] = -0.862352;
  start_angles0[6] = 3.139786;

  goal_angles0.resize(7);
  goal_angles0[0] = -0.002109;
  goal_angles0[1] = 0.655300;
  goal_angles0[2] = 0.000000;
  goal_angles0[3] = -1.517650;
  goal_angles0[4] = -3.138816;
  goal_angles0[5] = -0.862352;
  goal_angles0[6] = 3.139786;

  torso_z = 0.3; //0.26
  angles1.resize(7);
  angles1[0] = 0.2;
  angles1[1] = 1.4;
  angles1[2] = 1.9;
  angles1[3] = -0.4;
  angles1[4] = -0.1;
  angles1[5] = -1.00;
  angles1[6] = 0.0;

  //make kdl (used for FK and IK)
  string robot_description;
  ros::NodeHandle().param<std::string>("robot_description", robot_description, "");
  std::vector<std::string> planning_joints;
  planning_joints.push_back("r_shoulder_pan_joint");
  planning_joints.push_back("r_shoulder_lift_joint");
  planning_joints.push_back("r_upper_arm_roll_joint");
  planning_joints.push_back("r_elbow_flex_joint");
  planning_joints.push_back("r_forearm_roll_joint");
  planning_joints.push_back("r_wrist_flex_joint");
  planning_joints.push_back("r_wrist_roll_joint");
  if(!kdl_robot_model_.init(robot_description, planning_joints))
    ROS_ERROR("[PR2Sim] Failed to initialize the KDLRobotModel for the PR2.");
  kdl_robot_model_.setPlanningLink("r_gripper_palm_link");

  // Get the pose of the base footprint in the torso lift link frame.
  KDL::Frame base_in_torso_lift_link;
  base_in_torso_lift_link.p.x(0.050);
  base_in_torso_lift_link.p.y(0.0);
  base_in_torso_lift_link.p.z(-0.802 - torso_z);
  base_in_torso_lift_link.M = KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  // Note that all computed poses of the end-effector are in the base footprint frame.
  kdl_robot_model_.setKinematicsToPlanningTransform(base_in_torso_lift_link.Inverse(),"base_footprint");

  int_marker_server = new interactive_markers::InteractiveMarkerServer("robot_marker");

  //make menu
  visualization_msgs::InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menu_control.name = "menu_control";

  {//Make start base marker
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "start_base";
    int_marker.description = "";

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.9;
    marker.scale.y = 0.9;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl trans_control;
    trans_control.always_visible = true;
    trans_control.markers.push_back(marker);
    trans_control.orientation.w = 1;
    trans_control.orientation.x = 0;
    trans_control.orientation.y = 1;
    trans_control.orientation.z = 0;

    trans_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(trans_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    int_marker.controls.push_back(menu_control);

    // set the z a little higher so that it doesn't z-fight with the map
    int_marker.pose.position.z = 0.1;

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    int_marker_server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }
  {//Make goal base marker
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "goal_base";
    int_marker.description = "";

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.9;
    marker.scale.y = 0.9;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl trans_control;
    trans_control.always_visible = true;
    trans_control.markers.push_back(marker);
    trans_control.orientation.w = 1;
    trans_control.orientation.x = 0;
    trans_control.orientation.y = 1;
    trans_control.orientation.z = 0;

    trans_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(trans_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    int_marker.controls.push_back(menu_control);

    // set the z a little higher so that it doesn't z-fight with the map
    int_marker.pose.position.z = 0.1;

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    int_marker_server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }
  {//Make start hand marker
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "start_hand";
    int_marker.description = "";
    int_marker.scale = 0.25;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.09;
    marker.scale.y = 0.09;
    marker.scale.z = 0.09;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl trans_control;
    trans_control.always_visible = true;
    trans_control.markers.push_back(marker);
    trans_control.orientation.w = 1;
    trans_control.orientation.x = 0;
    trans_control.orientation.y = 1;
    trans_control.orientation.z = 0;
    trans_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(trans_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    int_marker.controls.push_back(menu_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    int_marker_server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }
  {//Make goal hand marker
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "goal_hand";
    int_marker.description = "";
    int_marker.scale = 0.25;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.09;
    marker.scale.y = 0.09;
    marker.scale.z = 0.09;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl trans_control;
    trans_control.always_visible = true;
    trans_control.markers.push_back(marker);
    trans_control.orientation.w = 1;
    trans_control.orientation.x = 0;
    trans_control.orientation.y = 1;
    trans_control.orientation.z = 0;
    trans_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(trans_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    int_marker.controls.push_back(menu_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    int_marker_server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }
  
  menu_handler.insert("Plan IMHA Round Robin", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan IMHA Meta A*", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan IMHA DTS", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan SMHA Round Robin", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan SMHA Meta A*", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan SMHA DTS", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan SMHA DTS PLUS", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan SMHA DTS FOCAL", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan SMHA DTS UNCONSTRAINED", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Interrupt planner", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Write to file", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.apply(*int_marker_server, "start_base");
  menu_handler.apply(*int_marker_server, "goal_base");
  menu_handler.apply(*int_marker_server, "start_hand");
  menu_handler.apply(*int_marker_server, "goal_hand");

  // 'commit' changes and send to all clients
  int_marker_server->applyChanges();

  planner_thread = new boost::thread(boost::bind(&ControlPlanner::callPlanner, this));
  planner = ros::NodeHandle().serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path", true);
  interrupt_pub = ros::NodeHandle().advertise<std_msgs::Empty>("/sbpl_planning/interrupt", 1);

  for(int i=0; i<10; i++){
    //vector<double> rarm({0.0, 1.1072800, -1.5566882, -2.124408, 0.0, -1.57, 1.57});
    //vector<double> rarm({0.0, 1.1072800, -1.3, -2.124408, 0.0, -1.57, 1.57});
    vector<double> rarm({-0.2, 1.1072800, -1.5566882, -2.124408, 0.0, -1.57, 0.0});
    vector<double> larm({0.038946, 1.214670, 1.396356, -1.197227, -4.616317, -0.988727, 1.175568});
    vector<double> body({0.0, 0.0, 0.0});
    pviz.visualizeRobot(rarm, larm, body, 0.1, 127, "weee", 0, false);
    sleep(1);
  }
}

ControlPlanner::~ControlPlanner(){
  
}


