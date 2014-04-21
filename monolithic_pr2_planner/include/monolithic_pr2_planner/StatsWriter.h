#pragma once
#include <ros/ros.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <monolithic_pr2_planner/ExperimentFramework/randomStartGoalGenerator.h>
#include <vector>
#include <sstream>

#define RRT 1
#define PRM_P 2
#define RRTSTAR 3
#define RRTSTARFIRSTSOL 4

struct RRTData {
    bool planned;
    double plan_time;
    double shortcut_time;
    size_t path_length;
    std::vector<monolithic_pr2_planner::RobotState> robot_state; 
    std::vector<monolithic_pr2_planner::ContBaseState> base; 
};

class StatsWriter {
    public:
        StatsWriter(int planner_id=-1);
        void writeSBPL(std::vector<double> &stats, 
                      std::vector<monolithic_pr2_planner::FullBodyState> &states, 
                      int trial_id, std::string
                      planner_prefix);
        void writeARA(std::vector<double> &stats, 
                      std::vector<monolithic_pr2_planner::FullBodyState> &states, 
                      int trial_id);
        void write(int trial_id, RRTData data);
        void writeStartGoal(int trial_id, std::pair<monolithic_pr2_planner::RobotState, monolithic_pr2_planner::RobotState>
          start_goal, int seed);
        inline void setPlannerId(int planner_id){ m_planner_id = planner_id; };
    private:
        FILE* ara;
        FILE* mha;
        FILE* rrt;
        FILE* prm;
        int m_planner_id;
        stringstream m_current_path;
};
