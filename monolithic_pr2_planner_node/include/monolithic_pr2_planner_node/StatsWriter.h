#pragma once
#include <ros/ros.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <vector>

struct RRTData {
    bool planned;
    double plan_time;
    double shortcut_time;
    std::vector<monolithic_pr2_planner::RobotState> robot_state; 
    std::vector<monolithic_pr2_planner::ContBaseState> base; 
};

class StatsWriter {
    public:
        StatsWriter();
        void writeARA(std::vector<double> stats, 
                      std::vector<monolithic_pr2_planner::FullBodyState> states, 
                      int trial_id);
        void writeMHA();
        void writeRRT(int trial_id, RRTData data);
    private:
        FILE* ara;
        FILE* mha;
        FILE* rrt;
        FILE* prm;
};
