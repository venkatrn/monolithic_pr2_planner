#pragma once
#include <monolithic_pr2_planner/ExperimentFramework/randomStartGoalGenerator.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>

class ExpInterface {
    public:
        ExpInterface(monolithic_pr2_planner::CSpaceMgrPtr cspace);
        void generatePairs();
    private:
        StartGoalGenerator m_generator;
};
