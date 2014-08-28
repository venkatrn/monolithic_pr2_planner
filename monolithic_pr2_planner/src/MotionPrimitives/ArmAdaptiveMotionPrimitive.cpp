#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <cmath>

using namespace monolithic_pr2_planner;
using namespace std;

GoalState ArmAdaptiveMotionPrimitive::m_goal;

// TODO refactor this
bool ArmAdaptiveMotionPrimitive::apply(const GraphState& source_state,
                                  GraphStatePtr& successor,
                                  TransitionData& t_data){
    DiscObjectState goal = m_goal.getObjectState();
    // TODO parameterize this distance?
    if (dist(source_state.getObjectStateRelMap(), goal) > 2){
        return false;
    }

    DiscBaseState base_state = source_state.robot_pose().base_state();

    // take the discrete orientation (theta) of the robot and convert it
    // into the discrete yaw in the object frame. object frame yaw is
    // generally discretized into 64 states, whereas the base theta is 16.
    ContBaseState c_base = source_state.robot_pose().base_state();
    ContObjectState c_obj_state = m_goal.getObjectState();
    c_obj_state.yaw(c_base.theta());

    // convert the object's rpy from map frame to body frame. Since the robot is
    // always on a flat surface, the roll and pitch of the map and body frame
    // are aligned - the only thing that needs to be transformed is the yaw.
    DiscObjectState obj_in_body_frame = source_state.getObjectStateRelBody();
    obj_in_body_frame.roll(goal.roll());
    obj_in_body_frame.pitch(goal.pitch());

    // this line is needed because we need to convert from disc
    ContObjectState c_goal(goal);
    ROS_DEBUG_NAMED(MPRIM_LOG, "c obj state %f goal yaw %f", c_obj_state.yaw(), c_goal.yaw());
    double short_ang = shortest_angular_distance(c_obj_state.yaw(),c_goal.yaw());
    ContObjectState blah(obj_in_body_frame);
    blah.yaw(short_ang);
    DiscObjectState blah2 = blah;
    ROS_DEBUG_NAMED(MPRIM_LOG, "obj_in_body_yaw %f", blah.yaw());
    obj_in_body_frame.yaw(blah2.yaw());

    RobotState seed_pose(base_state, 
                         source_state.robot_pose().right_arm(), 
                         source_state.robot_pose().left_arm());
    RobotPosePtr successor_robot_pose;
    bool isIKSuccess = RobotState::computeRobotPose(obj_in_body_frame, 
                                                    seed_pose, 
                                                    successor_robot_pose,
                                                    true);
    if (isIKSuccess){
        ROS_DEBUG_NAMED(MPRIM_LOG, "successful arm adaptive motion!");
        ContObjectState new_obj_state = successor_robot_pose->getObjectStateRelMap();
        new_obj_state.printToDebug(MPRIM_LOG);
        //assert(fabs(new_obj_state.x() == c_obj_state.x()) < .0001);
        //assert(fabs(new_obj_state.y() == c_obj_state.y()) < .0001);
        //assert(fabs(new_obj_state.z()-c_obj_state.z()) < .0001);
        //assert(fabs(new_obj_state.roll() == c_obj_state.roll()) < .0001);
        //assert(fabs(new_obj_state.pitch()-c_obj_state.pitch()) < .0001);
        //assert(fabs(new_obj_state.yaw()-c_obj_state.yaw()) < .0001);
        successor = boost::make_shared<GraphState>(*successor_robot_pose);
    } else {
        ROS_DEBUG_NAMED(MPRIM_LOG, "IK failed on arm AMP");
        return false;
    }

    t_data.motion_type(motion_type());
    // TODO compute real cost
    t_data.cost(cost());
    computeIntermSteps(source_state, *successor, t_data);

    return isIKSuccess;
}


void ArmAdaptiveMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){
    std::vector<RobotState> interp_steps;
    RobotState::workspaceInterpolate(source_state.robot_pose(), 
                                     successor.robot_pose(),
                                     &interp_steps);

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for arm AMP");
    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
    }
    t_data.interm_robot_steps(interp_steps);
    // fill in the cont base steps to be the same throughout; this is an arm
    // only motion
    ContBaseState c_base = source_state.robot_pose().base_state();
    std::vector<ContBaseState> cont_base_states(interp_steps.size(), c_base);
    t_data.cont_base_interm_steps(cont_base_states);

}

void ArmAdaptiveMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "ArmAdaptiveMotionPrimitive cost %d", cost());
}

void ArmAdaptiveMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}
