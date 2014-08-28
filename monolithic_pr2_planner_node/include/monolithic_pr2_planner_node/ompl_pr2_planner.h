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
#include <monolithic_pr2_planner/StatsWriter.h>

typedef ompl::base::RealVectorStateSpace::StateType VectorState;
typedef ompl::base::SE2StateSpace::StateType SE2State;
typedef ompl::base::ScopedState<ompl::base::CompoundStateSpace> FullState;
typedef monolithic_pr2_planner_node::GetMobileArmPlan::Request NodeRequest;

#define RRT 1
#define PRM_P 2
#define RRTSTAR 3
#define RRTSTARFIRSTSOL 4
class OMPLPR2Planner{
    public:
        OMPLPR2Planner(const monolithic_pr2_planner::CSpaceMgrPtr& cspace, int planner_id);
        bool planPathCallback(monolithic_pr2_planner::SearchRequestParams& search_request, int trial_id,
            StatsWriter& m_stats_writer);
        bool checkRequest(monolithic_pr2_planner::SearchRequestParams& search_request);
        bool createStartGoal(FullState& start, FullState& goal, monolithic_pr2_planner::SearchRequestParams& req);
        void setPlanningTime(double t){m_allocated_planning_time = t;};
    private:
        bool convertFullState(ompl::base::State* state,
                              monolithic_pr2_planner::RobotState& robot_state,
                              monolithic_pr2_planner::ContBaseState& base);
        ompl::base::StateSpacePtr fullBodySpace;
        ompl::base::ProblemDefinition* pdef;
        ompl::base::Planner* planner;
        ompl::geometric::PathSimplifier* pathSimplifier;
        omplFullBodyCollisionChecker* m_collision_checker;
        // StatsWriter m_stats_writer;
        int m_planner_id;
        double m_allocated_planning_time;
};
