#include <monolithic_pr2_planner/MotionPrimitives/ArmUntuckMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;

bool ArmUntuckMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){

    ContBaseState b = source_state.robot_pose().getContBaseState();
    LeftContArmState l = source_state.robot_pose().left_arm();
    //RightContArmState r({-0.002109, 0.655300, 0.000000, -1.517650, -3.138816, -0.862352, 3.139786});

    vector<double> r_angles;

    if(full_untuck_)
      r_angles = {-0.002109, 0.655300, 0.000000, -1.517650, -3.138816, -0.862352, 3.139786};
    else
      r_angles = {-0.2, 1.1072800, -1.5566882, -2.124408, 1.57, -1.57, 0.0};
    RightContArmState r(r_angles);

    RobotState rs(b,r,l);
    successor.reset(new GraphState(rs));

    t_data.motion_type(motion_type());
    t_data.cost(cost());
    computeIntermSteps(source_state, *successor, t_data);

    return true;
}

void ArmUntuckMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "ArmUntuckMotionPrimitive cost %d", cost());
}

void ArmUntuckMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}

void ArmUntuckMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){
    std::vector<RobotState> interp_steps;
    bool interpolate = RobotState::workspaceInterpolate(source_state.robot_pose(), 
                                     successor.robot_pose(),
                                     &interp_steps);
    if (!interpolate) {
        interp_steps.clear();
        RobotState::jointSpaceInterpolate(source_state.robot_pose(),
                                            successor.robot_pose(),
                                            &interp_steps);
    }

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for arm untuck AMP");
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
