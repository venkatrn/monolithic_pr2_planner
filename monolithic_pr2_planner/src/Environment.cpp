#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <assert.h>

#define GOAL_STATE 1
using namespace monolithic_pr2_planner;
using namespace boost;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
Environment::Environment(ros::NodeHandle nh) : 
    m_hash_mgr(new HashManager(&StateID2IndexMapping)), 
    m_nodehandle(nh), m_mprims(m_goal), 
    m_heur_mgr(new HeuristicMgr()) {
    m_param_catalog.fetch(nh);
    configurePlanningDomain();
}

bool Environment::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id){
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(search_request_params));
    configureQuerySpecificParams(search_request);
    if (!setStartGoal(search_request, start_id, goal_id)){
        return false;
    }
    return true;
}

int Environment::GetGoalHeuristic(int stateID){
    // For now, return the max of all the heuristics
    return GetGoalHeuristic(stateID, 0);
}

int Environment::GetGoalHeuristic(int stateID, int goal_id){
    // For now, return the max of all the heuristics
    std::vector<int> values = m_heur_mgr->getGoalHeuristic(m_hash_mgr->getGraphState(stateID));
    // ROS_DEBUG_NAMED(HEUR_LOG, "Heuristic values: Arm : %d\t Base 1: %d\t Base 2: %d", values[0], values[1], values[2]);
    return std::max(values[0], values[goal_id+1]);
    // switch(goal_id){
    //     case 0://anchor search - same for ARA* and MHA* : max(end eff, )
    //         // return values[0];
    //         return std::max(values[0], values[1]);
    //         // return std::max(values[0], values[2]);
    //     default:    // This is for ARA* : Must be the same as case 0
    //         return std::max(values[0], values[goal_id+1]);
    // }
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs){
    assert(sourceStateID != GOAL_STATE);
    ROS_DEBUG_NAMED(SEARCH_LOG, 
            "==================Expanding state %d==================", 
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(m_mprims.getMotionPrims().size());
    costs->clear();
    costs->reserve(m_mprims.getMotionPrims().size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    source_state->robot_pose().visualize();
    usleep(10000);

    for (auto mprim : m_mprims.getMotionPrims()){
        ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;
        if (!mprim->apply(*source_state, successor, t_data)){
            ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
            continue;
        }
        if (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
            m_cspace_mgr->isValidTransitionStates(t_data)){
            ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
            source_state->printToDebug(SEARCH_LOG);
            m_hash_mgr->save(successor);
            ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:", 
                            successor->id());
            successor->printToDebug(MPRIM_LOG);

            if (m_goal->isSatisfiedBy(successor)){
                m_goal->storeAsSolnState(successor);
                ROS_INFO_NAMED(SEARCH_LOG, "Found potential goal at state %d", successor->id());
                succIDs->push_back(GOAL_STATE);
            } else {
                succIDs->push_back(successor->id());
            }
            costs->push_back(mprim->cost());
            ROS_DEBUG_NAMED(SEARCH_LOG, "motion succeeded with cost %d", mprim->cost());
        } else {
            //successor->robot_pose().visualize();
            ROS_DEBUG_NAMED(SEARCH_LOG, "successor failed collision checking");
        }
    }
}

bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getObjectStateRelMap();
    obj_state.printToInfo(SEARCH_LOG);

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        return false;
    }

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(m_hash_mgr->getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    start_pose.visualize();

    // Compute the arm's reach
    // double basex,basey;
    // basex = start_pose.getContBaseState().x();
    // basey = start_pose.getContBaseState().y();
    // double armx, army;
    // armx = obj_state.x();
    // army = obj_state.y();
    // ROS_DEBUG_NAMED(HEUR_LOG, "(arm's reach: %f, %f)\n", sqrt((basex-armx)*(basex-armx)),
    //     sqrt((basey-army)*(basey-army)));


    m_goal = search_request->createGoalState();

    goal_id = saveFakeGoalState(start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    ContObjectState c_goal = m_goal->getObjectState();
    c_goal.printToInfo(SEARCH_LOG);
    m_goal->visualize();

    // This informs the adaptive motions about the goal.
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    BaseAdaptiveMotionPrimitive::goal(*m_goal);

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);

    // Create additional heuristics for MHA planner
    m_heur_mgr->initializeMHAHeuristics(m_base_heur_id, 1);

    return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int Environment::saveFakeGoalState(const GraphStatePtr& start_graph_state){
    GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
    RobotState fake_robot_state = fake_goal->robot_pose();
    DiscBaseState fake_base = fake_robot_state.base_state();
    fake_base.x(0); fake_base.y(0); fake_base.z(0);
    fake_robot_state.base_state(fake_base);
    fake_goal->robot_pose(fake_robot_state);
    m_hash_mgr->save(fake_goal);
    int goal_id = fake_goal->id();
    return goal_id;
}

// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                        m_param_catalog.m_robot_resolution_params);
    

    // used for discretization of robot movements
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);

#ifdef USE_IKFAST_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using IKFast");
#endif
#ifdef USE_KDL_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using KDL");
#endif

    // Initialize the heuristics. The (optional) parameter defines the cost multiplier.
    // TODO: It's 40 for now, until the actual cost for arm costs are computed.

    // 3DHeur is unit costs - multiply by whatever you want.
    // To get them in terms of mm distance
    m_heur_mgr->add3DHeur(20);  //0 - multiply by 20 : grid resolution in mm :
    // underestimated
    
    // Already in mm.
    m_base_heur_id = m_heur_mgr->add2DHeur(1, 0.7);//1
    // m_heur_mgr->add2DHeur(1, 0.0);//2
    // m_heur_mgr->numberOfMHAHeuristics(3);

    // used for arm kinematics
    LeftContArmState::initArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::initArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    m_cspace_mgr = make_shared<CollisionSpaceMgr>(r_arm.getArmModel(),
                                                  l_arm.getArmModel());

    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

    // load up static pviz instance for visualizations. 
    Visualizer::createPVizInstance();
    Visualizer::setReferenceFrame(std::string("/map"));
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){
    // sets the location of the object in the frame of the wrist
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    l_arm.setObjectOffset(search_request->m_params->left_arm_object);
    r_arm.setObjectOffset(search_request->m_params->right_arm_object);
    RobotState::setPlanningMode(search_request->m_params->planning_mode);
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> Environment::reconstructPath(vector<int> soln_path){
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    return postprocessor.reconstructPath(soln_path, *m_goal, m_mprims.getMotionPrims());
}
