#pragma once
#include <ros/ros.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/goals/GoalState.h>
#include <monolithic_pr2_planner_node/ompl_collision_checker.h>
#include <memory>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/SearchRequest.h>

typedef ompl::base::RealVectorStateSpace::StateType VectorState;
typedef ompl::base::SE2StateSpace::StateType SE2State;
typedef ompl::base::ScopedState<ompl::base::CompoundStateSpace> FullState;
typedef monolithic_pr2_planner_node::GetMobileArmPlan::Request NodeRequest;
class OMPLPR2Planner{
    public:
        OMPLPR2Planner(const monolithic_pr2_planner::CSpaceMgrPtr& cspace);
        bool planPathCallback(monolithic_pr2_planner::SearchRequestParams& search_request);
        bool createStartGoal(FullState& start, FullState& goal, monolithic_pr2_planner::SearchRequestParams& req);
    private:
        bool convertFullState(ompl::base::State* state,
                              monolithic_pr2_planner::RobotState& robot_state);
        ompl::base::StateSpacePtr fullBodySpace;
        ompl::base::ProblemDefinition* pdef;
        ompl::base::Planner* planner;
        ompl::geometric::PathSimplifier* pathSimplifier;
};
