#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <vector>
#include <memory>
#include <boost/shared_ptr.hpp>


namespace monolithic_pr2_planner {
  class EndEffLocalHeuristic : public AbstractHeuristic, public OccupancyGridUser {
    public:
      EndEffLocalHeuristic(){};

      int getGoalHeuristic(GraphStatePtr state);
      void setGoal(GoalState& state);

    private:
      GoalState m_goal;
  };
  typedef boost::shared_ptr<EndEffLocalHeuristic> EndEffLocalHeuristicPtr;
}
