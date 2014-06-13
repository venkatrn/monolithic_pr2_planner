#pragma once
#include <monolithic_pr2_planner/Heuristics/BFS3DHeuristic.h>

namespace monolithic_pr2_planner {
    /*! \brief Manages heuristic computation used by the SBPL planner. Currently
     * implements a 3D breadth first search for the end effector.
     */
    class BFS3DWithRotationHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser {
        public:
            BFS3DWithRotationHeuristic();
            int getGoalHeuristic(GraphStatePtr state);
            void setGoal(GoalState& state);
            void loadObstaclesFromOccupGrid();
            void update3DHeuristicMap();
            void setGripperRadius(double radius) { m_gripper_sphere_radius =
                radius; }
            void setDesiredOrientation(KDL::Rotation desired_orientation);
        private:
            GoalState m_goal;
            double m_gripper_sphere_radius;
            std::unique_ptr<sbpl_arm_planner::BFS_3D> m_bfs;
            KDL::Rotation m_desired_orientation;
    };
    typedef boost::shared_ptr<BFS3DWithRotationHeuristic> BFS3DWithRotationHeuristicPtr;
}