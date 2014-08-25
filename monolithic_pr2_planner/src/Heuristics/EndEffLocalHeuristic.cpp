#include <monolithic_pr2_planner/Heuristics/EndEffLocalHeuristic.h>

using namespace monolithic_pr2_planner;

void EndEffLocalHeuristic::setGoal(GoalState& state){
  m_goal = state;
}

int EndEffLocalHeuristic::getGoalHeuristic(GraphStatePtr state){

  /*
  DiscObjectState eef_goal = m_goal.getObjectState();
  eef_goal.z(eef_goal.z() - state->base_z()); //adjust the eef goal for the current torso height

  DiscObjectState eef = state->getObjectStateRelBody();

  int dx = eef.x() - eef_goal.x();
  int dy = eef.y() - eef_goal.y();
  int dz = eef.z() - eef_goal.z();
  int dist = sqrt(dx*dx + dy*dy + dz*dz);
  */

  DiscObjectState eef_goal = m_goal.getObjectState();
  //eef_goal.z(eef_goal.z() - state->base_z()); //adjust the eef goal for the current torso height
  int dx = state->obj_x() - eef_goal.x();
  int dy = state->obj_y() - eef_goal.y();
  int dz = state->obj_z() - eef_goal.z();
  double dist = sqrt(dx*dx + dy*dy + dz*dz);


  static bool print = true;
  if(print){
    print = false;
    ROS_ERROR("eef local dist=%f cost=%d",dist,dist*getCostMultiplier());
    std::cin.get();
  }

  return dist * getCostMultiplier();
}
