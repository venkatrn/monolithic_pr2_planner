#pragma once
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    /*! \brief Manages heuristic computation used by the SBPL planner. Currently
     * implements a 3D breadth first search for the end effector.
     */
    class EndEffOnlyRotationHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser {
        public:
            EndEffOnlyRotationHeuristic();
            int getGoalHeuristic(GraphStatePtr state);
            void setGoal(GoalState& state);
            void loadObstaclesFromOccupGrid();
            void update3DHeuristicMap();
            void setGripperRadius(double radius) {}
            void setDesiredOrientation(KDL::Rotation desired_orientation);
        private:
            GoalState m_goal;
            KDL::Rotation m_desired_orientation;
    };
    typedef boost::shared_ptr<EndEffOnlyRotationHeuristic> EndEffOnlyRotationHeuristicPtr;
}