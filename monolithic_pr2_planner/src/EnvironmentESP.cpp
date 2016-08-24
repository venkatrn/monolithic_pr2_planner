#include <monolithic_pr2_planner/EnvironmentESP.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/Heuristics/DistanceTransform2D.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <cassert>

#define GOAL_STATE 1
using namespace monolithic_pr2_planner;
using namespace boost;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
Environment::Environment(ros::NodeHandle nh)
  :   m_hash_mgr(new HashManager(&StateID2IndexMapping)),
      m_nodehandle(nh), m_mprims(m_goal),
      m_heur_mgr(new HeuristicMgr()),
      m_using_lazy(true),
      m_planner_type(T_SMHA) {
  m_param_catalog.fetch(nh);
  configurePlanningDomain();
}

/**
 * @brief Resets the environment.
 * @details Intended to be used between calls to subsequent planning
 * requests.
 */
void Environment::reset() {
  m_heur_mgr->reset();
  // m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
  m_hash_mgr.reset(new HashManager(&StateID2IndexMapping));
  m_edges.clear();

  // Fetch params again, in case they're being modified between calls.
  // m_param_catalog.fetch(m_nodehandle);
}

/**
 * @brief sets the planner type - mainly for experiments for the MHA paper
 * @details change the internal planner type to any of the different planners
 */
void Environment::setPlannerType(int planner_type) {
  m_planner_type = planner_type;
  m_heur_mgr->setPlannerType(planner_type);
  ROS_INFO_NAMED(SEARCH_LOG, "Setting planner type: %d", m_planner_type);
}

bool Environment::configureRequest(SearchRequestParamsPtr
                                   search_request_params,
                                   int &start_id, int &goal_id) {
  SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(
                                                       search_request_params));
  configureQuerySpecificParams(search_request);

  if (search_request->m_params->underspecified_start) {
    ROS_DEBUG_NAMED(CONFIG_LOG,
                    "underspecified_start. Will generate start state.");
    generateStartState(search_request);
  }

  if (!setStartGoal(search_request, start_id, goal_id)) {
    return false;
  }

  return true;
}

int Environment::GetGoalHeuristic(int stateID) {
  // For now, return the max of all the heuristics
  return GetGoalHeuristic(0, stateID);
}

int Environment::GetGoalHeuristic(int heuristic_id, int stateID) {
  GraphStatePtr successor = m_hash_mgr->getGraphState(stateID);

  if (m_goal->isSatisfiedBy(successor) || stateID == GOAL_STATE) {
    return 0;
  }

  std::unique_ptr<stringintmap> values;
  m_heur_mgr->getGoalHeuristic(successor, values);

  for (auto &heur : (*values)) {
    ROS_DEBUG_NAMED(HEUR_LOG, "%s : %d", heur.first.c_str(), heur.second);
  }

  const int base_heur = (*values).at("admissible_base");
  const int endeff_heur = (*values).at("admissible_endeff");

  // if (base_heur < 100) {
  //   ROS_INFO("Base_heur: %d", base_heur);
  //   return endeff_heur;
  // }
  // return base_heur;
  return std::max(base_heur, endeff_heur);
}

void Environment::GetSuccs(int sourceStateID, vector<int> *succIDs,
                           vector<int> *costs) {
  GetSuccs(0, sourceStateID, succIDs, costs);
}

void Environment::GetSuccs(int q_id, int sourceStateID, vector<int> *succIDs,
                           vector<int> *costs) {
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

  if (m_param_catalog.m_visualization_params.expansions) {
    RobotState expansion_pose = source_state->robot_pose();
    expansion_pose.visualize(250 / NUM_SMHA_HEUR * q_id);
    // source_state->robot_pose().visualize(250/NUM_SMHA_HEUR*q_id);
    m_cspace_mgr->visualizeAttachedObject(expansion_pose,
                                          250 / NUM_SMHA_HEUR * q_id);
    // m_cspace_mgr->visualizeCollisionModel(expansion_pose);
    usleep(5000);
  }

  for (auto mprim : m_mprims.getMotionPrims()) {
    ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
    // mprim->printEndCoord();
    GraphStatePtr successor;
    TransitionData t_data;

    if (!mprim->apply(*source_state, successor, t_data)) {
      ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
      continue;
    }

    if (m_cspace_mgr->isValidSuccessor(*successor, t_data) &&
        m_cspace_mgr->isValidTransitionStates(t_data)) {
      ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
      source_state->printToDebug(SEARCH_LOG);
      m_hash_mgr->save(successor);
      ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:",
                      successor->id());
      successor->printToDebug(MPRIM_LOG);

      if (m_goal->isSatisfiedBy(successor)) {
        m_goal->storeAsSolnState(successor);
        ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d",
                        successor->id(),
                        mprim->cost());
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

void Environment::GetLazySuccs(int sourceStateID, vector<int> *succIDs,
                               vector<int> *costs, std::vector<bool> *isTrueCost) {
  if (!m_using_lazy) {
    GetSuccs(0, sourceStateID, succIDs, costs);
    isTrueCost->clear();
    isTrueCost->resize(succIDs->size(), 1);
    return;
  }

  GetLazySuccs(0, sourceStateID, succIDs, costs, isTrueCost);
}

void Environment::GetLazySuccs(int q_id, int sourceStateID,
                               vector<int> *succIDs,
                               vector<int> *costs, std::vector<bool> *isTrueCost) {

  if (!m_using_lazy) {
    GetSuccs(q_id, sourceStateID, succIDs, costs);
    isTrueCost->clear();
    isTrueCost->resize(succIDs->size(), 1);
    return;
  }

  double expansion_color = 250 / NUM_SMHA_HEUR * q_id;
  vector<MotionPrimitivePtr> all_mprims = m_mprims.getMotionPrims();
  ROS_DEBUG_NAMED(SEARCH_LOG,
                  "==================Expanding state %d==================",
                  sourceStateID);
  succIDs->clear();
  succIDs->reserve(all_mprims.size());
  costs->clear();
  costs->reserve(all_mprims.size());

  GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
  ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
  source_state->robot_pose().printToDebug(SEARCH_LOG);

  if (m_param_catalog.m_visualization_params.expansions) {
    source_state->robot_pose().visualize(expansion_color);
    usleep(10000);
  }

  for (auto mprim : all_mprims) {
    //ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
    //mprim->printEndCoord();
    GraphStatePtr successor;
    TransitionData t_data;

    // if (mprim->motion_type() == MPrim_Types::ARM){
    if (false) {
      successor.reset(new GraphState(*source_state));
      successor->lazyApplyMPrim(mprim->getEndCoord());
      ROS_DEBUG_NAMED(SEARCH_LOG, "arm mprim/source/successor");
      mprim->printEndCoord();
      source_state->printToDebug(MPRIM_LOG);
      successor->printToDebug(MPRIM_LOG);
      ROS_DEBUG_NAMED(SEARCH_LOG, "done");
    } else {

      if (!mprim->apply(*source_state, successor, t_data)) {
        //ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
        continue;
      }

      ROS_DEBUG_NAMED(SEARCH_LOG, "non-arm mprim/source/successor");
      mprim->printEndCoord();
      source_state->printToDebug(MPRIM_LOG);
      successor->printToDebug(MPRIM_LOG);
      ROS_DEBUG_NAMED(SEARCH_LOG, "done");
    }
    bool valid_successor = (m_cspace_mgr->isValidSuccessorOnlySelf(*successor, t_data) &&
                          m_cspace_mgr->isValidTransitionStatesOnlySelf(t_data));
    if (!valid_successor) {
      continue;
    }
    auto true_cost_it = m_true_cost_cache.find(Edge(sourceStateID, successor->id()));
    if (true_cost_it != m_true_cost_cache.end()) {
      if (true_cost_it->second == -1) {
        continue;
      }
    }

    m_hash_mgr->save(successor);
    Edge key;

    if (m_goal->isSatisfiedBy(successor)) {
      m_goal->storeAsSolnState(successor);
      //ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
      //  mprim->cost());
      ROS_INFO("Found potential goal at: source->id %d, successor->id %d,"
               "cost: %d, mprim type: %d ", source_state->id(), successor->id(),
               mprim->cost(), mprim->motion_type());
      succIDs->push_back(GOAL_STATE);
      key = Edge(sourceStateID, GOAL_STATE);
    } else {
      succIDs->push_back(successor->id());
      key = Edge(sourceStateID, successor->id());
    }

    //        succIDs->push_back(successor->id());
    //        key = Edge(sourceStateID, successor->id());
    m_edges.insert(map<Edge, MotionPrimitivePtr>::value_type(key, mprim));

    // If this transition has already been evaluated, then return the true
    // value.
    if (true_cost_it != m_true_cost_cache.end()) {
      costs->push_back(true_cost_it->second);
      isTrueCost->push_back(true);
    } else {
      costs->push_back(mprim->cost());
      isTrueCost->push_back(false);
    }
  }
}

/*
 * Evaluates the edge. Assumes that the edge has already been generated and we
 * know the motion primitive used
 */
int Environment::GetTrueCost(int parentID, int childID) {

  // Return the true cost if it has already been computed.
  auto it = m_true_cost_cache.find(Edge(parentID, childID));
  if (it != m_true_cost_cache.end()) {
    return it->second;
  }

  TransitionData t_data;

  vector<MotionPrimitivePtr> small_mprims;

  if (m_edges.find(Edge(parentID, childID)) == m_edges.end()) {
    ROS_ERROR("transition hasn't been found between %d and %d??", parentID,
              childID);
    assert(false);
  }

  small_mprims.push_back(m_edges[Edge(parentID, childID)]);
  PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);

  ROS_DEBUG_NAMED(SEARCH_LOG, "evaluating edge (%d %d)", parentID, childID);
  GraphStatePtr source_state = m_hash_mgr->getGraphState(parentID);
  GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(childID);
  GraphStatePtr successor;
  MotionPrimitivePtr mprim = m_edges.at(Edge(parentID, childID));

  if (!mprim->apply(*source_state, successor, t_data)) {
    m_true_cost_cache[Edge(parentID, childID)] = -1;
    return -1;
  }

  mprim->printEndCoord();
  mprim->print();
  //source_state->printToInfo(SEARCH_LOG);
  //successor->printToInfo(SEARCH_LOG);
  successor->id(m_hash_mgr->getStateID(successor));

  // right before this point, the successor's graph state does not match the
  // stored robot state (because we modified the graph state without calling
  // ik and all that). this call updates the stored robot pose.
  real_next_successor->robot_pose(successor->robot_pose());

  bool matchesEndID = (successor->id() == childID) || (childID == GOAL_STATE);
  assert(matchesEndID);

  bool valid_successor = (m_cspace_mgr->isValidSuccessor(*successor, t_data) &&
                          m_cspace_mgr->isValidTransitionStates(t_data));

  if (!valid_successor) {
    m_true_cost_cache[Edge(parentID, childID)] = -1;
    return -1;
  }

  m_true_cost_cache[Edge(parentID, childID)] = t_data.cost();
  return t_data.cost();
}

void Environment::GetSuccs(int parent_id, std::vector<int> *succ_ids,
                           std::vector<int> *costs, std::vector<double> *edge_probabilities,
                           std::vector<double> *edge_eval_times,
                           std::vector<int> *edge_groups) {
  std::vector<bool> is_true_cost;
  GetLazySuccs(parent_id, succ_ids, costs, &is_true_cost);
  // GetSuccs(parent_id, succ_ids, costs);
  edge_eval_times->clear();
  edge_eval_times->resize(succ_ids->size(), 0.1);
  // TODO: implement a probability model.
  edge_probabilities->clear();
  edge_probabilities->resize(succ_ids->size(), 1.0);


  // ROS_INFO("Succs for %d", parent_id);
  const DistanceTransform2D &probabilities = dynamic_cast<DistanceTransform2D &>
                                             (*(m_heur_mgr->getHeuristic("distance_transform")));

  for (size_t ii = 0; ii < succ_ids->size(); ++ii) {
    GraphStatePtr state = m_hash_mgr->getGraphState(succ_ids->at(ii));
    // for (auto& heur : (*values)) {
    //     ROS_DEBUG_NAMED(HEUR_LOG, "%s : %d", heur.first.c_str(), heur.second);
    // }

    // Hack for goal state.
    if (state->id() == GOAL_STATE) {
      state = m_goal->getSolnState();
    }

    const double prob = probabilities.getIncomingEdgeProbability(state);
    // ROS_INFO("Prob for %d:   %f", succ_ids->at(ii), prob);
    edge_probabilities->at(ii) = prob;
  }
}

bool Environment::EvaluateEdge(int parent_id, int child_id) {
  const int true_cost = GetTrueCost(parent_id, child_id);
  return (true_cost >= 0);
}

bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int &start_id, int &goal_id) {
  RobotState start_pose(search_request->m_params->base_start,
                        search_request->m_params->right_arm_start,
                        search_request->m_params->left_arm_start);
  ContObjectState obj_state = start_pose.getObjectStateRelMap();
  obj_state.printToInfo(SEARCH_LOG);

  m_edges.clear();

  if (!search_request->isValid(m_cspace_mgr)) {
    obj_state.printToInfo(SEARCH_LOG);
    start_pose.visualize();
    return false;
  }

  start_pose.visualize();
  m_cspace_mgr->visualizeAttachedObject(start_pose);
  //m_cspace_mgr->visualizeCollisionModel(start_pose);
  //std::cin.get();

  GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
  m_hash_mgr->save(start_graph_state);
  start_id = start_graph_state->id();
  assert(m_hash_mgr->getGraphState(start_graph_state->id()) ==
         start_graph_state);

  ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
  start_pose.printToInfo(SEARCH_LOG);
  obj_state.printToInfo(SEARCH_LOG);
  // start_pose.visualize();


  m_goal = search_request->createGoalState();

  if (m_hash_mgr->size() < 2) {
    goal_id = saveFakeGoalState(start_graph_state);
  } else {
    goal_id = 1;
  }

  ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
  ContObjectState c_goal = m_goal->getObjectState();
  c_goal.printToInfo(SEARCH_LOG);
  m_goal->visualize();

  // This informs the adaptive motions about the goal.
  ArmAdaptiveMotionPrimitive::goal(*m_goal);
  BaseAdaptiveMotionPrimitive::goal(*m_goal);

  // informs the heuristic about the goal
  m_heur_mgr->setGoal(*m_goal);

  return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int Environment::saveFakeGoalState(const GraphStatePtr &start_graph_state) {
  GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
  RobotState fake_robot_state = fake_goal->robot_pose();
  DiscBaseState fake_base = fake_robot_state.base_state();
  fake_base.x(0);
  fake_base.y(0);
  fake_base.z(0);
  fake_robot_state.base_state(fake_base);
  fake_goal->robot_pose(fake_robot_state);
  m_hash_mgr->save(fake_goal);
  int goal_id = fake_goal->id();
  assert(goal_id == GOAL_STATE);
  return goal_id;
}

// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain() {
  // used for collision space and discretizing plain xyz into grid world
  OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                          m_param_catalog.m_robot_resolution_params);


  // used for discretization of robot movements
  ContArmState::setRobotResolutionParams(
    m_param_catalog.m_robot_resolution_params);

#ifdef USE_IKFAST_SOLVER
  ROS_DEBUG_NAMED(CONFIG_LOG, "Using IKFast");
#endif
#ifdef USE_KDL_SOLVER
  ROS_DEBUG_NAMED(CONFIG_LOG, "Using KDL");
#endif

  // Initialize the heuristics. The (optional) parameter defines the cost multiplier.

  m_heur_mgr->initializeHeuristics();

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
  m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
  // load up motion primitives
  m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

  // load up static pviz instance for visualizations.
  Visualizer::createPVizInstance();
  Visualizer::setReferenceFrame(std::string("/map"));
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr
                                               search_request) {
  // sets the location of the object in the frame of the wrist
  // have to do this funny thing  of initializing an object because of static
  // variable + inheritance (see ContArmState for details)
  LeftContArmState l_arm;
  RightContArmState r_arm;
  l_arm.setObjectOffset(search_request->m_params->left_arm_object);
  r_arm.setObjectOffset(search_request->m_params->right_arm_object);
  ROS_DEBUG_NAMED(SEARCH_LOG, "Setting planning mode to : %d",
                  search_request->m_params->planning_mode);
  RobotState::setPlanningMode(search_request->m_params->planning_mode);
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> Environment::reconstructPath(vector<int> soln_path) {
  PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
  std::vector<FullBodyState> final_path = postprocessor.reconstructPath(
                                            soln_path, *m_goal, m_mprims.getMotionPrims());

  if (m_param_catalog.m_visualization_params.final_path) {
    postprocessor.visualizeFinalPath(final_path);
  }

  return final_path;
}

void Environment::generateStartState(SearchRequestPtr search_request) {
  ContObjectState start_obj_state(search_request->m_params->obj_start);
  ContBaseState base_start(search_request->m_params->base_start);
  RobotState start_robot_state(base_start, start_obj_state);
  start_robot_state.visualize();
  m_cspace_mgr->visualizeAttachedObject(start_robot_state);
  ROS_DEBUG_NAMED(CONFIG_LOG, "Generate start state : Keyboard");
  // std::cin.get();
  search_request->m_params->base_start = start_robot_state.getContBaseState();
  search_request->m_params->right_arm_start = start_robot_state.right_arm();
  search_request->m_params->left_arm_start = start_robot_state.left_arm();
}
