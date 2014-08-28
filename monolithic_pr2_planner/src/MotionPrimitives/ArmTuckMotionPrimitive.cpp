#include <monolithic_pr2_planner/MotionPrimitives/ArmTuckMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;

bool ArmTuckMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){

    ContBaseState b = source_state.robot_pose().getContBaseState();
    //RightContArmState r({0.0, 1.1072800, -1.5566882, -2.124408, 0.0, 0.0, 0.0});
    RightContArmState r({-0.2, 1.1072800, -1.5566882, -2.124408, 0.0, -1.57, 0.0});

    LeftContArmState l = source_state.robot_pose().left_arm();
    RobotState rs(b,r,l);
    successor.reset(new GraphState(rs));

    t_data.motion_type(motion_type());
    t_data.cost(cost());
    computeIntermSteps(source_state, *successor, t_data);

    return true;
}


void ArmTuckMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){
    std::vector<RobotState> interp_steps;
    bool interpolate = RobotState::workspaceInterpolate(source_state.robot_pose(), 
                                     successor.robot_pose(),
                                     &interp_steps);
    if (!interpolate) {
        interp_steps.clear();
        RobotState::jointSpaceInterpolate(source_state.robot_pose(),
                                    successor.robot_pose(), &interp_steps);
    }

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for arm tuck AMP");
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


void ArmTuckMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "ArmTuckMotionPrimitive cost %d", cost());
}

void ArmTuckMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}
