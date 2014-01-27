#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Visualizer.h>


using namespace monolithic_pr2_planner;
using namespace std;

PathPostProcessor::PathPostProcessor(HashManagerPtr hash_mgr, CSpaceMgrPtr cspace_mgr):
m_cspace_mgr(cspace_mgr), m_hash_mgr(hash_mgr)
{
}


/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> PathPostProcessor::reconstructPath(vector<int> soln_path,
                                                         GoalState& goal_state,
                                                         vector<MotionPrimitivePtr> mprims){
    double temptime = clock();
    vector<TransitionData> transition_states;
    // the last state in the soln path return by the SBPL planner will always be
    // the goal state ID. Since this doesn't actually correspond to a real state
    // in the heap, we have to look it up.
    soln_path[soln_path.size()-1] = goal_state.getSolnState()->id();
    ROS_DEBUG_NAMED(SEARCH_LOG, "setting goal state id to %d", 
                                 goal_state.getSolnState()->id());
    for (size_t i=0; i < soln_path.size()-1; i++){
        TransitionData best_transition;
        bool success = findBestTransition(soln_path[i], 
                                          soln_path[i+1], 
                                          best_transition,
                                          mprims);
        if (success){
            transition_states.push_back(best_transition);
        } else {
            ROS_ERROR("Successor not found during path reconstruction!");
        }
    }
    
    vector<FullBodyState> final_path = getFinalPath(soln_path, 
                                                    transition_states,
                                                    goal_state);
    visualizeFinalPath(final_path);
    ROS_INFO("my reconstruct took %.3f", (clock()-temptime)/(double)CLOCKS_PER_SEC);
    return final_path;
}

void PathPostProcessor::visualizeFinalPath(vector<FullBodyState> path){
    for (auto& state : path){
        vector<double> l_arm, r_arm, base;
        l_arm = state.left_arm;
        r_arm = state.right_arm;
        base = state.base;
        BodyPose bp;
        bp.x = base[0];
        bp.y = base[1];
        bp.z = base[2];
        bp.theta = base[3];
        Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
        usleep(5000);
    }
}

/*! \brief Given a start and end state id, find the motion primitive with the
 * least cost that gets us from start to end. Return that information with a
 * TransitionData object.
 */
bool PathPostProcessor::findBestTransition(int start_id, int end_id, 
                                     TransitionData& best_transition,
                                     vector<MotionPrimitivePtr> mprims){
    ROS_DEBUG_NAMED(SEARCH_LOG, "searching from %d for successor id %d", 
                                start_id, end_id);
    GraphStatePtr source_state = m_hash_mgr->getGraphState(start_id);
    GraphStatePtr successor;
    int best_cost = 1000000;
    GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(end_id);
    for (auto mprim : mprims){
        TransitionData t_data;
        if (!mprim->apply(*source_state, successor, t_data)){
            continue;
        }

        successor->id(m_hash_mgr->getStateID(successor));
        bool matchesEndID = successor->id() == end_id;
        bool isCheaperAction = t_data.cost() < best_cost;
        if (matchesEndID && isCheaperAction){
            if (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
                    m_cspace_mgr->isValidTransitionStates(t_data)){
                best_cost = t_data.cost();
                best_transition = t_data;
                best_transition.successor_id(successor->id());
            }
        }

    }
    return (best_cost != 1000000);
}

FullBodyState PathPostProcessor::createFBState(const RobotState& robot){
    vector<double> l_arm, r_arm, base;
    robot.right_arm().getAngles(&r_arm);
    robot.left_arm().getAngles(&l_arm);
    ContBaseState c_base = robot.base_state();
    c_base.getValues(&base);
    FullBodyState state;
    state.left_arm = l_arm;
    state.right_arm = r_arm;
    state.base = base;
    return state;
}

/*! \brief Retrieve the final path, with intermediate states and all. There's
 * some slight funny business with TransitionData for intermediate base states
 * because they're not captured in the RobotState. Instead, they're held
 * separately in TransitionData ONLY if the motion primitive has the base
 * moving. There's one more state_id than transition state.
 * */
std::vector<FullBodyState> PathPostProcessor::getFinalPath(const vector<int>& state_ids,
                                 const vector<TransitionData>& transition_states,
                                 GoalState& goal_state){
    vector<FullBodyState> fb_states;
    { // throw in the first point
        GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[0]);
        fb_states.push_back(createFBState(source_state->robot_pose()));
    }
    for (size_t i=0; i < transition_states.size(); i++){
        int motion_type = transition_states[i].motion_type();
        bool isInterpBaseMotion = (motion_type == MPrim_Types::BASE || 
                                   motion_type == MPrim_Types::BASE_ADAPTIVE);
        for (size_t j=0; j < transition_states[i].interm_robot_steps().size(); j++){
            RobotState robot = transition_states[i].interm_robot_steps()[j];
            FullBodyState state = createFBState(robot);
            if (isInterpBaseMotion){
                assert(transition_states[i].interm_robot_steps().size() == 
                       transition_states[i].cont_base_interm_steps().size());
                ContBaseState cont_base = transition_states[i].cont_base_interm_steps()[j];
                std::vector<double> base;
                cont_base.getValues(&base);
                state.base = base;
            } 
            fb_states.push_back(state);
        }
    }
    { // throw in the last point
        GraphStatePtr soln_state = goal_state.getSolnState();
        fb_states.push_back(createFBState(soln_state->robot_pose()));
    }
    return fb_states;
}
