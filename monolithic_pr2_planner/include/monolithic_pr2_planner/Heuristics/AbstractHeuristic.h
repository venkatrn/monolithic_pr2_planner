#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    /*! \brief The Abstract Heuristic class that has to be inherited
     * for use in the environment.
     */
    class AbstractHeuristic{
        public:
            AbstractHeuristic();
            virtual int getGoalHeuristic(GraphStatePtr state) = 0;
            virtual void setGoal(GoalState& state) = 0;

            // For 3D heuristics that need the obstacles
            virtual void update3DHeuristicMap() {};

            // For 2D heuristics at the base that need only the map
            virtual void update2DHeuristicMap(const std::vector<signed char>& data) {};
    };
    typedef boost::shared_ptr<AbstractHeuristic> AbstractHeuristicPtr;
}
