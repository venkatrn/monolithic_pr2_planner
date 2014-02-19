#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <vector>
#include <memory>
#include <boost/shared_ptr.hpp>


namespace monolithic_pr2_planner {
    /*! \brief Manages heuristic computation used by the SBPL planner. Currently
     * implements a 3D breadth first search for the end effector.
     */
    class ArmAnglesHeuristic : public AbstractHeuristic, public OccupancyGridUser {
        public:
            ArmAnglesHeuristic(CSpaceMgrPtr cspace_mgr, double ik_range = 0.7) :
                m_ik_range(ik_range),
                m_cspace_mgr(cspace_mgr)
                {};

            int getGoalHeuristic(GraphStatePtr state);
            void setGoal(GoalState& state);

            inline void setGoalArmState(RightContArmState& soln_r_arm_state) {
                m_soln_r_arm_state = soln_r_arm_state; };
        
        private:
            GoalState m_goal;
            double m_ik_range;
            RightContArmState m_soln_r_arm_state;
            CSpaceMgrPtr m_cspace_mgr;
    };
    typedef boost::shared_ptr<ArmAnglesHeuristic> ArmAnglesHeuristicPtr;
}