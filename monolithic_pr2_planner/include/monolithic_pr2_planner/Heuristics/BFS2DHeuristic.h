#pragma once
#include <sbpl/headers.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace monolithic_pr2_planner {
    class BFS2DHeuristic : public AbstractHeuristic, public OccupancyGridUser{
        public:
            BFS2DHeuristic();
            // BFS2DHeuristic(boost::shared_ptr<costmap_2d::Costmap2DROS>
                // costmap_ros);
            ~BFS2DHeuristic();
            void setGoal(GoalState& state);
            void loadMap(const std::vector<signed char>& data);
            int getGoalHeuristic(GraphStatePtr state);
            void update2DHeuristicMap(const std::vector<signed char>& data);
            void visualize();
        private:
            std::unique_ptr<sbpl_bfs_2d> m_bfs;
            unsigned int m_size_col;
            unsigned int m_size_row;
            int** m_grid;
    };
    typedef boost::shared_ptr<BFS2DHeuristic> BaseHeuristicPtr;
}