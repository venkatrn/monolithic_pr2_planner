#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Visualizer.h>

using namespace monolithic_pr2_planner;

//GoalState::GoalState(SearchRequestPtr search_request):
//    m_goal_state(search_request->m_params->obj_goal), m_tolerances(4,0){
//
//    m_tolerances[Tolerances::XYZ] = search_request->m_params->xyz_tolerance;
//    m_tolerances[Tolerances::ROLL] = search_request->m_params->roll_tolerance;
//    m_tolerances[Tolerances::PITCH] = search_request->m_params->pitch_tolerance;
//    m_tolerances[Tolerances::YAW] = search_request->m_params->yaw_tolerance;
//}

GoalState::GoalState(DiscObjectState obj_goal, double xyz_tol, 
                     double roll_tol, double pitch_tol, double yaw_tol):
    m_goal_state(obj_goal), m_tolerances(4,0){

    m_tolerances[Tolerances::XYZ] = xyz_tol;
    m_tolerances[Tolerances::ROLL] = roll_tol;
    m_tolerances[Tolerances::PITCH] = pitch_tol;
    m_tolerances[Tolerances::YAW] = yaw_tol;
}

double GoalState::endeffectorXYDistanceSqr(const GraphStatePtr& graph_state){
    ContObjectState obj = ContObjectState(graph_state->getObjectStateRelMap());
    ContObjectState cont_goal = ContObjectState(m_goal_state);

    double dX = abs(obj.x() - cont_goal.x());
    double dY = abs(obj.y() - cont_goal.y());
    double dZ = abs(obj.z() - cont_goal.z());
    return dX * dX + dY * dY + dZ * dZ;
}

bool GoalState::withinXYZTol(const GraphStatePtr& graph_state){
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    DiscObjectState obj = graph_state->getObjectStateRelMap();


    bool within_xyz_tol = (abs(m_goal_state.x()-obj.x()) < d_tol.x() &&
                           abs(m_goal_state.y()-obj.y()) < d_tol.y() &&
                           abs(m_goal_state.z()-obj.z()) < d_tol.z());
    return within_xyz_tol;
}

// OLD
// bool GoalState::isSatisfiedBy(const GraphStatePtr& graph_state){
//     // not sure why there's a .005 here. ask ben
//     ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
//                           m_tolerances[Tolerances::XYZ]-.005, 
//                           m_tolerances[Tolerances::XYZ]-.005,
//                           m_tolerances[Tolerances::ROLL],
//                           m_tolerances[Tolerances::PITCH],
//                           m_tolerances[Tolerances::YAW]);
//     DiscObjectState d_tol = c_tol.getDiscObjectState();
//     DiscObjectState obj = graph_state->getObjectStateRelMap();
//
//
//     bool within_xyz_tol = (abs(m_goal_state.x()-obj.x()) < d_tol.x() &&
//                            abs(m_goal_state.y()-obj.y()) < d_tol.y() &&
//                            abs(m_goal_state.z()-obj.z()) < d_tol.z());
//     bool within_rpy_tol = (abs(m_goal_state.roll()-obj.roll()) < d_tol.roll() &&
//                            abs(m_goal_state.pitch()-obj.pitch()) < d_tol.pitch() &&
//                            abs(m_goal_state.yaw()-obj.yaw()) < d_tol.yaw());
//
//     if (within_xyz_tol && within_rpy_tol){
//         return true;
//     } else {
//         return false;
//     }
// }

bool GoalState::isSatisfiedBy(const GraphStatePtr& graph_state){
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    // TODO: this should be getObjectStateRelMapFromState if we do lazy
    // (no IK) successor generation.
    DiscObjectState obj = graph_state->getObjectStateRelMap();


    bool within_xyz_tol = (abs(m_goal_state.x()-obj.x()) < d_tol.x() &&
                           abs(m_goal_state.y()-obj.y()) < d_tol.y() &&
                           abs(m_goal_state.z()-obj.z()) < d_tol.z());
    bool within_rpy_tol = (abs(m_goal_state.roll()-obj.roll()) < d_tol.roll() &&
                           abs(m_goal_state.pitch()-obj.pitch()) < d_tol.pitch() &&
                           abs(m_goal_state.yaw()-obj.yaw()) < d_tol.yaw());

    bool within_quat_tol;
    ContObjectState cont_goal_obj(m_goal_state);
    ContObjectState cont_current_obj(obj);
     tf::Quaternion quat_state(cont_goal_obj.yaw(),cont_goal_obj.pitch(),cont_goal_obj.roll());
     tf::Quaternion quat_goal(cont_current_obj.yaw(),cont_current_obj.pitch(),cont_current_obj.roll());

    double diff = quat_state.angleShortestPath(quat_goal);

    within_quat_tol = fabs(diff) < m_tolerances[Tolerances::YAW];      //should be another parameter d_tol.quat()

    if (within_xyz_tol && within_quat_tol){
    // Revert to RPY tolerance checking; quaternion tolerance needs another
    // look.
    // if (within_xyz_tol && within_rpy_tol){
        return true;
    } else {
        return false;
    }
}

bool GoalState::isSolnStateID(int state_id){
    for (auto& goal : m_possible_goals){
        if (goal == state_id){
            return true;
        }
    }
    return false;
}
void GoalState::addPotentialSolnState(const GraphStatePtr& graph_state) { 
    m_possible_goals.push_back(graph_state->id());
}

void GoalState::visualize(){
    ContObjectState cont_goal = ContObjectState(m_goal_state);
    std::vector<double> pose;
    pose.push_back(cont_goal.x());
    pose.push_back(cont_goal.y());
    pose.push_back(cont_goal.z());
    pose.push_back(cont_goal.roll());
    pose.push_back(cont_goal.pitch());
    pose.push_back(cont_goal.yaw());
    Visualizer::pviz->visualizePose(pose, "goal");
}
