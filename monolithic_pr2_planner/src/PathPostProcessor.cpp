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
    
    ROS_INFO("Finding best transition took %.3f", (clock()-temptime)/(double)CLOCKS_PER_SEC);
    // vector<FullBodyState> final_path = getFinalPath(soln_path, 
    //                                                 transition_states,
    //                                                 goal_state);

    // temptime = clock();
    std::vector<FullBodyState> final_path = shortcutPath(soln_path,
        transition_states, goal_state);
    // ROS_INFO("Shortcutting took %.3f", (clock()-temptime)/(double)CLOCKS_PER_SEC);
    return final_path;
}


std::vector<FullBodyState> PathPostProcessor::shortcutPath(const vector<int>&
    state_ids, const vector<TransitionData>& transition_states, GoalState& goal_state){
    ROS_DEBUG_NAMED(HEUR_LOG, "Original request : States : %ld, transition data : %ld",
        state_ids.size(), transition_states.size());
    std::vector<FullBodyState> final_path;
    // index into the solution path.
    size_t i = 0;
    size_t j = 1;
    
    { // throw in the first point : NOTE: Probably not needed.
        GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[0]);
        final_path.push_back(PathPostProcessor::createFBState(source_state->robot_pose()));
    }

    std::vector<FullBodyState> interp_states;
    while(j < state_ids.size()){
        assert(i<j);
        ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "Shortcutting : %lu - %lu", i, j);
        // Get the two states between which we are going to try interpolating.
        GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[i]);
        GraphStatePtr end_state = m_hash_mgr->getGraphState(state_ids[j]);
        RobotState first_pose = source_state->robot_pose();
        RobotState second_pose = end_state->robot_pose();
        // first_pose.visualize(120);
        // second_pose.visualize(180);
        // Store the previous interpolation results. In case the current
        // attempt fails.
        std::vector<FullBodyState> interp_states_prev(interp_states.begin(),
            interp_states.end());
        // Attempt to interpolate.
        interp_states.clear();
        bool interpolate = PathPostProcessor::stateInterpolate(first_pose, second_pose, &interp_states);
        ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "Interp_states size : %lu ",
            interp_states.size());
        // Check if the interpolation was successful and has no collisions.
        bool no_collision = true;
        if(interpolate) {
            for (size_t k = 0; k < interp_states.size() && no_collision; ++k)
            {
                if(!m_cspace_mgr->isValidContState(interp_states[k].left_arm,interp_states[k].right_arm,
                    interp_states[k].base)) {
                    ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "invalid state found");
                    RobotState failed_state = PathPostProcessor::createRobotState(interp_states[k]);
                    no_collision = false;
                    // failed_state.visualize(0);
                    // std::cin.get();
                }
            }
        }
        if(interpolate && no_collision){
            ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "Successful shortcutting %lu - %lu",
                i, j);
            // If it was successful and there were no collisions, try until the
            // next state from the current source state.
            j++;
        } else {
            // interpolation and/or collision checking failed.
            // Check if it is a consecutive state; if it is, then we need to
            // use the transition data. If not, we would have interpolated earlier. We use
            // the results from the last interpolation then.
            if(i == (j - 1)){
                if (!interpolate)
                    ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "failed interpolate");
                if (!no_collision)
                    ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "collision!");
                // This part is the same as from getFinalPath
                // int motion_type = transition_states[i].motion_type();
                bool has_t_data = !transition_states[i].interm_robot_steps().empty();
                // bool isInterpBaseMotion = (motion_type == MPrim_Types::BASE || 
                                   // motion_type == MPrim_Types::BASE_ADAPTIVE);
                if(!has_t_data){
                    // This is for the arm motions. They don't have transition
                    // data.
                    GraphStatePtr arm_state = m_hash_mgr->getGraphState(state_ids[i]);
                    final_path.push_back(PathPostProcessor::createFBState(arm_state->robot_pose()));
                }
                for (size_t t=0; t < transition_states[i].interm_robot_steps().size(); t++){
                    RobotState robot = transition_states[i].interm_robot_steps()[t];
                    FullBodyState state = PathPostProcessor::createFBState(robot);
                    // if(isInterpBaseMotion){
                    assert(transition_states[i].interm_robot_steps().size() == 
                           transition_states[i].cont_base_interm_steps().size());
                    ContBaseState cont_base = transition_states[i].cont_base_interm_steps()[t];
                    std::vector<double> base;
                    cont_base.getValues(&base);
                    state.base = base;
                    // }
                    final_path.push_back(state);
                }
                ++i;
                ++j;
            } else {
                ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "failed interpolation. shoving in [%lu, %lu)", i, j);
                // Shove things in.
                // [) format. We'll shove the last point in the end or in the
                // next cycle
                final_path.insert(final_path.end(), interp_states_prev.begin(),
                    interp_states_prev.end() - 1);
                i = j - 1;
            }
        }
        ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "final path size : %lu", final_path.size());
        // std::cin.get();
    }
    if(i != state_ids.size() - 1){
        final_path.insert(final_path.end(), interp_states.begin(),
            interp_states.end());
    }
    ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "Size of shortcutPath: %d",
        static_cast<int>(final_path.size()));
    { // throw in the last point
        GraphStatePtr soln_state = goal_state.getSolnState();
        final_path.push_back(PathPostProcessor::createFBState(soln_state->robot_pose()));
    }
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
        RobotState robot_state = PathPostProcessor::createRobotState(state);
        m_cspace_mgr->visualizeAttachedObject(robot_state);
        // m_cspace_mgr->visualizeCollisionModel(robot_state);
        robot_state.printToDebug(POSTPROCESSOR_LOG);
        robot_state.getObjectStateRelBody().getContObjectState().printToDebug(POSTPROCESSOR_LOG);
        // std::cin.get();
        usleep(30000);
    }
}

/**
 * @brief compares two paths for the base path length
 * @details If the distance covered by the base is lower in the new path,
 * the value that is returned is True.
 * @param new_path The path to compare
 * @param original_path The path to compare against
 */
bool PathPostProcessor::isBasePathBetter(std::vector<FullBodyState> &new_path,
    std::vector<FullBodyState> &original_path){
    double new_dist = 0;
    for (size_t i = 0; i < new_path.size()-1; ++i)
    {
        new_dist += (new_path[i].base[BodyDOF::X] -
        new_path[i+1].base[BodyDOF::X])*(new_path[i].base[BodyDOF::X] -
        new_path[i+1].base[BodyDOF::X]) + 
                    (new_path[i].base[BodyDOF::Y] -
        new_path[i+1].base[BodyDOF::Y])*(new_path[i].base[BodyDOF::Y] -
        new_path[i+1].base[BodyDOF::Y]);
    }
    double original_dist = 0;
    for (size_t i = 0; i < original_path.size()-1; ++i)
    {
        original_dist += (original_path[i].base[BodyDOF::X] -
        original_path[i+1].base[BodyDOF::X])*(original_path[i].base[BodyDOF::X] -
        original_path[i+1].base[BodyDOF::X]) + 
                    (original_path[i].base[BodyDOF::Y] -
        original_path[i+1].base[BodyDOF::Y])*(original_path[i].base[BodyDOF::Y] -
        original_path[i+1].base[BodyDOF::Y]);
    }
    return (new_dist < original_dist);
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
        int successor_id;
        if(!m_hash_mgr->exists(successor, successor_id))
            continue;
        successor->id(successor_id);
        bool matchesEndID = successor->id() == end_id;
        if(!matchesEndID)
            continue;
        
        if (!(m_cspace_mgr->isValidSuccessor(*successor, t_data))){
            continue;
        }
        if(!(m_cspace_mgr->isValidTransitionStates(t_data))){
            continue;
        }
        
        bool isCheaperAction = t_data.cost() < best_cost;

        if (matchesEndID && isCheaperAction){
            best_cost = t_data.cost();
            best_transition = t_data;
            best_transition.successor_id(successor->id());
        }

    }
    return (best_cost != 1000000);
}

/**
 * @brief Creates a fullBodyState given a RobotState
 * @details fullBodyState is the type used by all functions that deal with the
 * final path. This function converts a RobotState to a FBState
 * 
 * @param robot a RobotState object
 * @return a FullBodyState
 */

FullBodyState PathPostProcessor::createFBState(const RobotState& robot){
    vector<double> l_arm, r_arm, base;
    vector<double> obj(6,0);
    robot.right_arm().getAngles(&r_arm);
    robot.left_arm().getAngles(&l_arm);
    ContBaseState c_base = robot.base_state();
    c_base.getValues(&base);
    FullBodyState state;
    state.left_arm = l_arm;
    state.right_arm = r_arm;
    state.base = base;
    ContObjectState obj_state = robot.getObjectStateRelMap();
    obj[0] = obj_state.x();
    obj[1] = obj_state.y();
    obj[2] = obj_state.z();
    obj[3] = obj_state.roll();
    obj[4] = obj_state.pitch();
    obj[5] = obj_state.yaw();
    state.obj = obj;
    return state;
}

RobotState PathPostProcessor::createRobotState(const FullBodyState& fb_state){
    ContBaseState cbase_state = createContBaseState(fb_state);
    LeftContArmState l_arm(fb_state.left_arm);
    RightContArmState r_arm(fb_state.right_arm);
    RobotState state(cbase_state, r_arm, l_arm);
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
    // { // throw in the first point
    //     GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[0]);
    //     fb_states.push_back(PathPostProcessor::createFBState(source_state->robot_pose()));
    // }
    for (size_t i=0; i < transition_states.size(); i++){
        int motion_type = transition_states[i].motion_type();
        bool isInterpBaseMotion = (motion_type == MPrim_Types::BASE || 
                                   motion_type == MPrim_Types::BASE_ADAPTIVE);
        if (!isInterpBaseMotion) {
            GraphStatePtr current_state = m_hash_mgr->getGraphState(state_ids[i]);
            fb_states.push_back(PathPostProcessor::createFBState(current_state->robot_pose()));
        }
        // ROS_DEBUG_NAMED(SEARCH_LOG, "Transition number : %d; Type: %d; intermediate states: %d", i, motion_type, transition_states[i].interm_robot_steps().size());
        // The interm_robot_steps() has the start and the end state included.
        // We want to consistently exclude the end state, because it will be added in
        // the next transition data. That's why j runs from 0 to
        // interm_robot_steps().size() - 1
        for (int j=0; j < static_cast<int>(transition_states[i].interm_robot_steps().size()-1); j++){
            RobotState robot = transition_states[i].interm_robot_steps()[j];
            FullBodyState state = PathPostProcessor::createFBState(robot);
            if (isInterpBaseMotion){
                assert(transition_states[i].interm_robot_steps().size() == 
                       transition_states[i].cont_base_interm_steps().size());
                ContBaseState cont_base = transition_states[i].cont_base_interm_steps()[j];
                std::vector<double> base;
                cont_base.getValues(&base);
                state.base = base;
            }
            // RobotState robot_state = PathPostProcessor::createRobotState(state);
            // robot_state.visualize();
            // m_cspace_mgr->visualizeAttachedObject(robot_state);
            // std::cin.get();
            fb_states.push_back(state);
        }
        // std::cin.get();
    }
    { // throw in the last point
        GraphStatePtr soln_state = goal_state.getSolnState();
        fb_states.push_back(PathPostProcessor::createFBState(soln_state->robot_pose()));
    }
    return fb_states;
}


/*! \brief Given two robot states, we interpolate all steps in between them. The
 * delta step size is based on the RPY resolution of the object pose and the XYZ
 * resolution of the base. This returns a vector of RobotStates, which are
 * discretized to the grid (meaning if the arms move a lot, but the base barely
 * moves, the base discrete values will likely stay the same). This determines
 * the number of interpolation steps based on which part of the robot moves more
 * (arms or base).
 * TODO need to test this a lot more;
 * Bug alert! Returns only the end states even for a base motion. NEED TO DEBUG
 */
bool PathPostProcessor::stateInterpolate(const RobotState& start, const RobotState& end, vector<FullBodyState>* interp_steps){
    std::vector<RobotState> robot_states;
    // try workspace interpolate. This is what we prefer.
    bool interpolate = RobotState::workspaceInterpolate(start, end, &robot_states);
    if (!interpolate) {
        robot_states.clear();
        RobotState::jointSpaceInterpolate(start, end, &robot_states);
    }

    // now we should have all the robot states. Convert them to FBStates
    interp_steps->clear();
    for (auto& state : robot_states) {
        interp_steps->push_back(PathPostProcessor::createFBState(state));
    }
    return true;
}

ContBaseState PathPostProcessor::createContBaseState(const FullBodyState& state){
    ContBaseState c_base_state;
    c_base_state.x(state.base[BodyDOF::X]);
    c_base_state.y(state.base[BodyDOF::Y]);
    c_base_state.z(state.base[BodyDOF::Z]);
    c_base_state.theta(state.base[BodyDOF::THETA]);
    return c_base_state;
}
