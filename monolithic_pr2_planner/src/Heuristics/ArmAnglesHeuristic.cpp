#include <monolithic_pr2_planner/Heuristics/ArmAnglesHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <kdl/frames.hpp>

using namespace monolithic_pr2_planner;

void ArmAnglesHeuristic::setGoal(GoalState& state){
    m_goal = state;
}

int ArmAnglesHeuristic::getGoalHeuristic(GraphStatePtr state){
    
    DiscObjectState d_goal_state = m_goal.getObjectState();
    // Sanity check. Don't want to do all those steps if not within range.
    
    if((state->base_x() - d_goal_state.x())*(state->base_x() - d_goal_state.x())
        + (state->base_y() - d_goal_state.y())*(state->base_y() - d_goal_state.y())
        > m_ik_range*m_ik_range )
        return INFINITECOST;

    RobotState robot_pose = state->robot_pose();
    ContBaseState cont_seed_base_state = robot_pose.getContBaseState();
    ContObjectState goal_state = d_goal_state.getContObjectState();

    RobotPosePtr final_pose;
    KDL::Frame to_robot_frame;
    Visualizer::pviz->getMaptoRobotTransform(cont_seed_base_state.x(),
        cont_seed_base_state.y(), cont_seed_base_state.theta(), to_robot_frame);


    // seed_robot_pose.visualize();
    KDL::Frame obj_frame;
    obj_frame.p.x(goal_state.x());
    obj_frame.p.y(goal_state.y());
    obj_frame.p.z(goal_state.z());
    obj_frame.M = KDL::Rotation::RPY(goal_state.roll(), 
                                     goal_state.pitch(),
                                     goal_state.yaw());
    KDL::Frame internal_tf =
    robot_pose.right_arm().getArmModel()->computeBodyFK(cont_seed_base_state.body_pose());
    KDL::Frame transform = internal_tf.Inverse() * obj_frame;
    double rr, rp, ry;
    transform.M.GetRPY(rr, rp, ry);
    ContObjectState goal_torso_frame(transform.p.x(),
                                    transform.p.y(),
                                    transform.p.z(),
                                    rr,rp,ry);
    DiscObjectState d_goal_torso_frame(goal_torso_frame);
    bool ik_success = RobotState::computeRobotPose(d_goal_torso_frame, robot_pose,
        final_pose);
    if(ik_success){
        ROS_DEBUG_NAMED(HEUR_LOG,"IK success for arm heuristic");
        // return m_cspace_mgr->isValid(*final_pose);
        return 1;
    }
    else
        return INFINITECOST;
}