#pragma once
// #include <sbpl/headers.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/2Dgridsearch.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace monolithic_pr2_planner {
    class DistanceTransform2D : public virtual AbstractHeuristic, public OccupancyGridUser{
        public:
            DistanceTransform2D();
            ~DistanceTransform2D();
            
            void setGoal(GoalState& state);
            void loadMap(const std::vector<unsigned char>& data);
            int getGoalHeuristic(GraphStatePtr state);
            double getIncomingEdgeProbability(GraphStatePtr state) const;
            void update2DHeuristicMap(const std::vector<unsigned char>& data);

            // Set radius for the circle around the goal for which the
            // heuristic will be zero. Default is the PR2's arm reach
            // TODO: Compute this instead of hardcoding
            void setRadiusAroundGoal(double radius_m = 0.7);
            double getRadiusAroundGoal(){ return m_radius; };

            static void visualizeCenter(int x, int y);
            static void getBresenhamCirclePoints(int x0, int y0, int radius, std::vector<int>& x,
                std::vector<int>& y);
            static void getBresenhamCirclePoints(int x0, int y0, int radius,
                std::vector<std::pair<int,int> >& points);
            static void getBresenhamLinePoints(int x1, int y1, int x2, int y2, std::vector<std::pair<int, int> >& points);
            static void getBresenhamLinePoints(int x1, int y1, int x2, int y2, std::vector<int>& pts_x, std::vector<int>& pts_y);
            void setUniformCostSearch(bool ucs = false);
        protected:
            std::unique_ptr<SBPL2DGridSearch> m_gridsearch;
            unsigned int m_size_col;
            unsigned int m_size_row;
            unsigned char** m_grid;
            double m_radius;
            void visualizeRadiusAroundGoal(int x0, int y0);
            GoalState m_goal;
            std::vector<std::pair<int, int>> m_obstacle_coordinates;
            // ros::NodeHandle m_nh;
            // ros::Publisher m_circlepub;

    };
    typedef boost::shared_ptr<DistanceTransform2D> DistanceTransform2DPtr;
}
