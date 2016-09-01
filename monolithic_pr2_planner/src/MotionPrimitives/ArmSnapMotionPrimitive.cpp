#include <monolithic_pr2_planner/MotionPrimitives/ArmSnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;

GoalState ArmSnapMotionPrimitive::m_goal;
std::vector<double> ArmSnapMotionPrimitive::m_tolerances;

bool ArmSnapMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){

    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    
    RobotState robot_pose = source_state.robot_pose();
    DiscObjectState obj = source_state.getObjectStateRelMap();
    DiscBaseState base = robot_pose.base_state();
    unsigned int r_free_angle = robot_pose.right_free_angle();

    // ContBaseState cont_goal_base_state = m_goal->getRobotState().getContBaseState();

    bool within_xyz_tol = (abs(m_goal.getObjectState().x()-base.x()) < 25*d_tol.x() &&
                           abs(m_goal.getObjectState().y()-base.y()) < 25*d_tol.y() &&
                           abs(m_goal.getObjectState().z()-base.z()) < 25*d_tol.z());

    // bool within_yaw_tol = abs(angles::shortest_angular_distance(cont_goal_base_state.theta(), robot_pose.getContBaseState().theta())) < 15*c_tol.yaw();

   // bool within_basexy_tol = (abs(m_goal.getRobotState().base_state().x()-base.x()) < 25*d_tol.x() &&
   //                           abs(m_goal.getRobotState().base_state().y()-base.y()) < 25*d_tol.y());

    bool goal_facing_arm = false;

    RobotState source_pose = source_state.robot_pose();
    ContObjectState goal_rel_map = ContObjectState(m_goal.getObjectState());
    KDL::Frame F_torsolift_goal;

    KDL::Vector goal_wrt_body;
    KDL::Rotation goal_wrt_body_rpy;


    if(within_xyz_tol) {

      KDL::Rotation goal_wrt_map_rpy = KDL::Rotation::RPY(goal_rel_map.roll(), goal_rel_map.pitch(), goal_rel_map.yaw());
      KDL::Vector goal_wrt_map(goal_rel_map.x(), goal_rel_map.y(), goal_rel_map.z());
      KDL::Frame F_map_goal(goal_wrt_map_rpy, goal_wrt_map); //Frame of goal wrt map.

      KDL::Vector vec;

      ContBaseState cont_base_state = source_pose.getContBaseState();

      //vec = KDL::Vector(source_pose.base_state().x()*0.02 , source_pose.base_state().y()*0.02, 0.803 + source_pose.base_state().z()*0.02); //Is the Z values correct?
      KDL::Rotation r = KDL::Rotation::RPY(0, 0, 0);
      vec = KDL::Vector(-0.05, 0, cont_base_state.z() + 0.803);
      KDL::Frame F_base_torsolift(r, vec); // Torse lift wrt base foorprint.

      KDL::Rotation rot = KDL::Rotation::RPY(0, 0, cont_base_state.theta());
      vec = KDL::Vector(cont_base_state.x(), cont_base_state.y(), 0);
      KDL::Frame F_map_base(rot, vec); //Frame of base wrt map.

      KDL::Frame F_map_torsolift = F_map_base * F_base_torsolift;

      KDL::Frame F_torsolift_map = F_map_torsolift.Inverse();
      KDL::Frame r_obj_to_wrist_offset = source_pose.right_arm().getObjectOffset();

      //KDL::Frame F_goal_map = F_map_goal.Inverse();

     F_torsolift_goal = F_torsolift_map * F_map_goal * source_pose.right_arm().getObjectOffset().Inverse();
      //goal_wrt_body = F_map_torsolift.Inverse() * goal_wrt_map;
      //goal_wrt_body_rpy = (F_map_torsolift).Inverse().M * goal_wrt_map_rpy;

      double goal_wrt_body_x = F_torsolift_goal.p.x(); //Direction of the arm.
      double goal_wrt_body_y = F_torsolift_goal.p.y();
      
      // ROS_INFO("X = %f, Y= %f", goal_wrt_body_x, goal_wrt_body_y);

      if(goal_wrt_body_x > 0.1 && (goal_wrt_body_y < 15*c_tol.y())) { //tol = 0.035
          goal_facing_arm = true;
      }
    }


    bool within_yaw_tol = true;
    if(within_xyz_tol && within_yaw_tol && goal_facing_arm)// && ik_success)
    { 
      //ROS_INFO("Yaw: %f", cont_base_state.theta());
      //ROS_ERROR("Robot base frame");

      //ROS_ERROR("Wrt map");
      //ROS_INFO("%f\t%f\t%f", wrt_map.p.x(), wrt_map.p.y(), wrt_map.p.z());

      double wr, wp, wy;
      F_torsolift_goal.M.GetRPY(wr, wp, wy);
      //ContObjectState goal_obj_wrt_body(F_torsolift_goal.p.x(), F_torsolift_goal.p.y(), F_torsolift_goal.p.z(), wr, wp, wy);
      ContObjectState goal_obj_wrt_body(F_torsolift_goal.p.x(), F_torsolift_goal.p.y(), F_torsolift_goal.p.z(), wr, wp, wy);
      DiscObjectState disc_goal_wrt_body(goal_obj_wrt_body);


      RobotPosePtr new_robot_pose_ptr;
      RightContArmState right_arm = source_pose.right_arm();
      LeftContArmState left_arm = source_pose.left_arm();
      RobotState temp(source_pose.getContBaseState(), right_arm, left_arm);
      bool ik_success = source_pose.computeRobotPose(disc_goal_wrt_body, temp, new_robot_pose_ptr);

      int c = 0;
      while (!ik_success && c < 10000) {
          right_arm.setUpperArmRoll(temp.randomDouble(-3.75, 0.65));
          temp.right_arm(right_arm);
          ik_success = temp.computeRobotPose(disc_goal_wrt_body, temp, new_robot_pose_ptr);
          // Also check that the disc roll didn't change.
          if (ik_success) {
            DiscObjectState obj = new_robot_pose_ptr->getObjectStateRelBody();
            if (obj.roll() != disc_goal_wrt_body.roll()) {
              ik_success = false;
            }
          }
          c++;
      }
      if(ik_success) {
        // ROS_INFO("Snapping to goal state");
        RobotState rs(new_robot_pose_ptr->getContBaseState(), new_robot_pose_ptr->right_arm(), new_robot_pose_ptr->left_arm());
        //RobotState rs(source_pose.getContBaseState(),goal_obj_wrt_body);
        successor.reset(new GraphState(rs));

        t_data.motion_type(motion_type());
        t_data.cost(cost());
        bool success_interms = computeIntermSteps(source_state, *successor, t_data);
        if (success_interms) {
          // ROS_INFO("Goal interpolation succeeded");
          rs.visualize(140);
        }
        // ROS_INFO("Goal interpolation failed");
        return success_interms;
      }
      else {
          // ROS_INFO("IK failed");
          return false;
      }
    }
    else{
        return false;
    } 
}

bool ArmSnapMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for arm snap primitive");
    std::vector<RobotState> interp_steps;
    bool interpolate = RobotState::workspaceInterpolate(source_state.robot_pose(), 
                                     successor.robot_pose(),
                                     &interp_steps);
    bool j_interpolate;

    if (!interpolate) {
        interp_steps.clear();
        j_interpolate = RobotState::jointSpaceInterpolate(source_state.robot_pose(),
                                    successor.robot_pose(), &interp_steps);
    }

    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
    }
    t_data.interm_robot_steps(interp_steps);

    if(!interpolate && !j_interpolate)
    {
        ROS_WARN("No valid arm interpolation found to snap arm to goal pose");
        return false;
    }

    ContBaseState c_base = source_state.robot_pose().getContBaseState();
    std::vector<ContBaseState> cont_base_states(interp_steps.size(), c_base);
    t_data.cont_base_interm_steps(cont_base_states);

    return true;
}

void ArmSnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "ArmSnapMotionPrimitive cost %d", cost());
}

void ArmSnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    //TODO: Calculate actual cost 
    m_cost = 1;
}
