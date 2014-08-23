#pragma once
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/2Dgridsearch.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <dynamicvoronoi/dynamicvoronoi.h>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace monolithic_pr2_planner {
    class VoronoiOrientationHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser{
        public:
            VoronoiOrientationHeuristic();
            ~VoronoiOrientationHeuristic();

            void setGoal(GoalState& state);

            void update2DHeuristicMap(const std::vector<unsigned char>& data);
            void loadMap(const std::vector<unsigned char>& data);

            int getGoalHeuristic(GraphStatePtr state);

        private:
            // m_grid stores the NavMap - the values \in [0,100], where
            // anything > threshold (=80) is an obstacle, 0 is free space.

            unsigned int m_size_col;
            unsigned int m_size_row;
            unsigned char** m_grid;
            GoalState m_goal;

            bool** m_obstacle_binary_grid;

            // In this case, the m_gridsearch is initialized with all 
            // the points from the voronoi diagram
            std::unique_ptr<SBPL2DGridSearch> m_gridsearch;

            // The voronoi object.
            DynamicVoronoi m_dynamic_voronoi;
    };
    typedef boost::shared_ptr<VoronoiOrientationHeuristic>
    VoronoiOrientationHeuristicPtr;
}
