#pragma once
#include <bfs3d/BFS_3D.h>
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
    class BFS3DHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser {
        public:
            BFS3DHeuristic();
            int getGoalHeuristic(GraphStatePtr state);
            void setGoal(GoalState& state);
            void loadObstaclesFromOccupGrid();
            void visualize();
            void update3DHeuristicMap();
            void setGripperRadius(double radius) { m_gripper_sphere_radius =
                radius; }
        private:
            GoalState m_goal;
            double m_gripper_sphere_radius;
            std::unique_ptr<sbpl_arm_planner::BFS_3D> m_bfs;
    };
    typedef boost::shared_ptr<BFS3DHeuristic> BFS3DHeuristicPtr;
}
