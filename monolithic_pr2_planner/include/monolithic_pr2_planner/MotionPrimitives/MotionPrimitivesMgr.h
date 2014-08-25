#pragma once
#include <vector>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmTuckMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmUntuckMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/TorsoMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>

namespace monolithic_pr2_planner {
    typedef std::vector<MotionPrimitivePtr> MPrimList;
    class MotionPrimitivesMgr {
        public:
            MotionPrimitivesMgr(){};
            MotionPrimitivesMgr(GoalStatePtr& goal);
            bool loadMPrims(const MotionPrimitiveParams& files);
            void loadMPrimSet(int planning_mode);
            std::vector<MotionPrimitivePtr> getMotionPrims() { return m_active_mprims; };
        private:
            void loadBaseOnlyMPrims();
            void loadArmOnlyMPrims();
            void loadAllMPrims();
            void loadTorsoMPrims();
            // these are all the possible mprims we have
            std::vector<std::vector<MotionPrimitivePtr> > m_all_mprims;
            void computeAllMPrimCosts(std::vector<MPrimList> mprims);
            double dist(DiscObjectState s1, DiscObjectState s2);
            MotionPrimitiveFileParser m_parser;

            // we're taking v1 and adding it into v2
            void combineVectors(const MPrimList& v1, MPrimList& v2);
            // these are the mprims that have been selected for use for this
            // planning request
            std::vector<MotionPrimitivePtr> m_active_mprims;
            MotionPrimitiveParams m_params;
            GoalStatePtr m_goal;
    };
}
