#pragma once
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/2Dgridsearch.h>
#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace monolithic_pr2_planner {
    class BaseWithRotationHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser{
        public:
            BaseWithRotationHeuristic();
            ~BaseWithRotationHeuristic();

            void setGoal(GoalState& state);
            inline void setOriginalGoal(GoalState& original_state) { m_original_goal
                = original_state; };
            void setDesiredOrientation(KDL::Rotation desired_orientation);

            void update2DHeuristicMap(const std::vector<signed char>& data);
            void loadMap(const std::vector<signed char>& data);

            int getGoalHeuristic(GraphStatePtr state);

        private:
            void visualizeLineToOriginalGoal(int x0, int y0, int x1, int y1);
            std::unique_ptr<SBPL2DGridSearch> m_gridsearch;
            unsigned int m_size_col;
            unsigned int m_size_row;
            unsigned char** m_grid;
            GoalState m_goal;
            GoalState m_original_goal;
            double m_desired_orientation;

            std::vector<double> m_soln_arm_angles;

    };
    typedef boost::shared_ptr<BaseWithRotationHeuristic> BaseWithRotationHeuristicPtr;
}