#pragma once
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>


#define X_MIN 0
#define X_MAX 9
#define Y_MIN 0
#define Y_MAX 6
#define Z_MIN 0
#define Z_MAX 2

struct Region {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
};

class StartGoalGenerator {
    public:
        StartGoalGenerator(monolithic_pr2_planner::CSpaceMgrPtr cspace);
        monolithic_pr2_planner::RobotState generateRandomState(Region* region);
        bool generateRandomValidState(monolithic_pr2_planner::RobotState& robot_state,
                                      int region_id=-1, bool is_start_state =
                                      false);
        bool generateUniformPairs(int num_pairs, std::vector<std::pair<monolithic_pr2_planner::RobotState, monolithic_pr2_planner::RobotState> >& pair);

        inline double randomDouble(double min, double max){
            return min + (max-min) * ( double(rand()) / RAND_MAX );
        }

        void initializeRegions();
        inline std::vector<Region> getGoalRegions(){ return m_goal_regions;};
    private:
        vector<Region> m_goal_regions;
        Region m_start_region;
        monolithic_pr2_planner::CSpaceMgrPtr m_cspace;
};