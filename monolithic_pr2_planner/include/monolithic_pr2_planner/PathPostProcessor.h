#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <vector>

namespace monolithic_pr2_planner {
    struct FullBodyState {
        std::vector<double> base;
        std::vector<double> left_arm;
        std::vector<double> right_arm;
        std::vector<double> obj;
    };
    class PathPostProcessor {
        public:
            PathPostProcessor(HashManagerPtr hash_mgr, CSpaceMgrPtr cspace_mgr);
            std::vector<FullBodyState> reconstructPath(std::vector<int> state_ids,
                                                       GoalState& goal_state,
                                                       std::vector<MotionPrimitivePtr> mprims);
            static void visualizeFinalPath(std::vector<FullBodyState> path);
        private:
            std::vector<FullBodyState> getFinalPath(const vector<int>& state_ids,
                                            const vector<TransitionData>& transition_states,
                                            GoalState& goal_state);
            bool findBestTransition(int start_id, int end_id, TransitionData& t_data,
                                    std::vector<MotionPrimitivePtr> mprims);
            FullBodyState createFBState(const RobotState& robot);

            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
    };
}
