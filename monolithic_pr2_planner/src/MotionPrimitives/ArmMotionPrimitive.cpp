#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

void ArmMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, "Arm Primitive");
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tgroup: %d", getGroup());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tcost: %d", cost());
    printEndCoord();
    printIntermSteps();
}

bool ArmMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){
    successor.reset(new GraphState(source_state));

    bool isSuccessorCreated = successor->applyMPrim(m_end_coord);
    if (isSuccessorCreated){
        t_data.motion_type(motion_type());
        t_data.cost(cost());
    }
    // the arm movements are so small that i'm ignoring the intermediate points
    //computeIntermSteps(source_state, *successor, t_data);

    return isSuccessorCreated;
}

void ArmMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
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

}

void ArmMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 40;
}
