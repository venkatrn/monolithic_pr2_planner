#include <monolithic_pr2_planner_node/StatsWriter.h>
#include <stdio.h>
#include <sstream>

using namespace std;
using namespace monolithic_pr2_planner;

StatsWriter::StatsWriter(){ }

void StatsWriter::writeRRT(int trial_id, RRTData data){
    ROS_INFO("writing rrt stats");
    stringstream ss;
    ss << "/tmp/rrt_" << trial_id << ".stats";
    FILE* stats = fopen(ss.str().c_str(), "w");
    if (data.planned){
        fprintf(stats, "%f %f\n", data.plan_time, data.shortcut_time);
        stringstream ss2;
        ss2 << "/tmp/rrt_" << trial_id << ".path";
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < data.robot_state.size(); i++){
            vector<double> l_arm;
            vector<double> r_arm;
            vector<double> base;

            data.robot_state[i].right_arm().getAngles(&r_arm);
            data.robot_state[i].left_arm().getAngles(&l_arm);

            fprintf(path, "%f %f %f %f %f %f %f\n",
                          r_arm[0], 
                          r_arm[1], 
                          r_arm[2], 
                          r_arm[3], 
                          r_arm[4], 
                          r_arm[5], 
                          r_arm[6]);
            fprintf(path, "%f %f %f %f %f %f %f\n",
                          l_arm[0], 
                          l_arm[1], 
                          l_arm[2], 
                          l_arm[3], 
                          l_arm[4], 
                          l_arm[5], 
                          l_arm[6]);
            fprintf(path, "%f %f %f %f\n",
                          data.base[i].x(),
                          data.base[i].y(),
                          data.base[i].z(),
                          data.base[i].theta());
        }
        fclose(path);
    } else {
        fprintf(stats, "failed to plan\n");
    }
    fclose(stats);
}

void StatsWriter::writeARA(std::vector<double> stats_v, std::vector<FullBodyState> states, 
                           int trial_id){
    ROS_INFO("writing ara stats");
    stringstream ss;
    ss << "/tmp/ara_" << trial_id << ".stats";
    FILE* stats = fopen(ss.str().c_str(), "w");
    fprintf(stats, "%f %f %f %f %f %f %f %f %f %f\n", 
            stats_v[0],
            stats_v[1],
            stats_v[2],
            stats_v[3],
            stats_v[4],
            stats_v[5],
            stats_v[6],
            stats_v[7],
            stats_v[8],
            stats_v[9]);
    stringstream ss2;

    if (states.size()){
        ss2 << "/tmp/ara_" << trial_id << ".path";
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < states.size(); i++){
            vector<double> l_arm = states[i].left_arm;
            vector<double> r_arm = states[i].right_arm;
            vector<double> base = states[i].base;

            fprintf(path, "%f %f %f %f %f %f %f\n",
                    r_arm[0], 
                    r_arm[1], 
                    r_arm[2], 
                    r_arm[3], 
                    r_arm[4], 
                    r_arm[5], 
                    r_arm[6]);
            fprintf(path, "%f %f %f %f %f %f %f\n",
                    l_arm[0], 
                    l_arm[1], 
                    l_arm[2], 
                    l_arm[3], 
                    l_arm[4], 
                    l_arm[5], 
                    l_arm[6]);
            fprintf(path, "%f %f %f %f\n",
                    base[0],
                    base[1],
                    base[2],
                    base[3]);
        }
        fclose(path);
    }
    fclose(stats);
}
