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

    return true;
}

void ArmTuckMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "ArmTuckMotionPrimitive cost %d", cost());
}

void ArmTuckMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}
