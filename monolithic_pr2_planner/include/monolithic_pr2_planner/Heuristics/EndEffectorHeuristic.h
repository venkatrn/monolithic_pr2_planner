#pragma once
#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BFS3DHeuristic.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <vector>
#include <bfs3d/BFS_3D.h>
#include <memory>
#include <boost/shared_ptr.hpp>




namespace monolithic_pr2_planner {
    /*! \brief Manages heuristic computation used by the SBPL planner. Currently
     * implements a 3D breadth first search for the end effector.
     */
    class EndEffectorHeuristic : public BFS2DHeuristic, public BFS3DHeuristic {
        public:
            EndEffectorHeuristic();
            int getGoalHeuristic(GraphStatePtr state);
            void setGoal(GoalState& state);
        //     void loadObstaclesFromOccupGrid();
        //     void visualize();
        //     void update3DHeuristicMap();
            // Set the cost_multiplier
            // inline void setCostMultiplier(const int cost_multiplier) { m_cost_multiplier
            //     = cost_multiplier; };

            // // Get the cost multiplier
            // inline int getCostMultiplier(){ return m_cost_multiplier; };
        private:
            unsigned int m_size_col;
            unsigned int m_size_row;
            unsigned char** m_grid;
            GoalState m_goal;
            std::unique_ptr<sbpl_arm_planner::BFS_3D> m_bfs;
    };
    typedef boost::shared_ptr<EndEffectorHeuristic> EndEffectorHeuristicPtr;
}
