#include <monolithic_pr2_planner/StatsWriter.h>
#include <stdio.h>
#include <ctime>
#include <sys/stat.h>

using namespace std;
using namespace monolithic_pr2_planner;

StatsWriter::StatsWriter(int planner_id):m_planner_id(planner_id){
    struct stat stats_path;
    if (!(stat("/tmp/planning_stats", &stats_path) == 0 && S_ISDIR(stats_path.
          st_mode))) {
      // the directory does not exist.
      ROS_INFO("Creating directory /tmp/planning_stats for dumping stats.");
      mkdir("/tmp/planning_stats/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    time_t now = time(0);
    m_current_path << "/tmp/planning_stats/" << time(&now) <<"/";
    int status;
    status = mkdir(m_current_path.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(status==0){
      ROS_INFO("Created path : %s", m_current_path.str().c_str());
    }
    else{
      ROS_ERROR("Cannot create path for dumping stats!");
    }
}

void StatsWriter::write(int trial_id, RRTData data){
    //if (m_planner_id == PRM_P)
    //    ROS_INFO("writing PRM stats");
    //if (m_planner_id == RRT)
    //    ROS_INFO("writing RRT stats");
    //if (m_planner_id == RRTSTAR)
    //    ROS_INFO("writing RRTStar stats");
    stringstream ss;
    if (m_planner_id == PRM_P)
        ss << m_current_path.str().c_str() << "prm_" << std::setfill('0') << std::setw(2) << trial_id << ".stats";
    if (m_planner_id == RRT)
        ss << m_current_path.str().c_str() << "rrt_" << std::setfill('0') << std::setw(2) << trial_id << ".stats";
    if (m_planner_id == RRTSTAR)
        ss << m_current_path.str().c_str() << "rrtstar_" << std::setfill('0') << std::setw(2) << trial_id << ".stats";
    if (m_planner_id == RRTSTARFIRSTSOL)
      ss << m_current_path.str().c_str() << "rrtstarfirstsol_" << std::setfill('0') << std::setw(2) << trial_id << ".stats";
    ROS_DEBUG_NAMED(HEUR_LOG, "Opening file : %s", ss.str().c_str());
    FILE* stats = fopen(ss.str().c_str(), "w");
    if (data.planned){
        fprintf(stats, "%f %f %lu\n", data.plan_time, data.shortcut_time, data.path_length);
        stringstream ss2;
        if (m_planner_id == PRM_P)
            ss2 << m_current_path.str().c_str() << "prm_" << std::setfill('0') << std::setw(2) << trial_id << ".path";
        if (m_planner_id == RRT)
            ss2 << m_current_path.str().c_str() << "rrt_" << std::setfill('0') << std::setw(2) << trial_id << ".path";
        if (m_planner_id == RRTSTAR)
            ss2 << m_current_path.str().c_str() << "rrtstar_" << std::setfill('0') << std::setw(2) << trial_id << ".path";
        if (m_planner_id == RRTSTARFIRSTSOL)
            ss2 << m_current_path.str().c_str() << "rrtstarfirstsol_" << std::setfill('0') << std::setw(2) << trial_id << ".path";
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < data.robot_state.size(); i++){
            vector<double> l_arm;
            vector<double> r_arm;
            vector<double> base;

            data.robot_state[i].right_arm().getAngles(&r_arm);
            data.robot_state[i].left_arm().getAngles(&l_arm);

            fprintf(path, "%f %f %f %f ",
                          data.base[i].x(),
                          data.base[i].y(),
                          data.base[i].theta(),
                          data.base[i].z());
            fprintf(path, "%f %f %f %f %f %f %f ",
                          r_arm[0], 
                          r_arm[1], 
                          r_arm[2], 
                          r_arm[3], 
                          r_arm[4], 
                          r_arm[5], 
                          r_arm[6]);
            ContObjectState obj = data.robot_state[i].getObjectStateRelMap(data.base[i]);
            fprintf(path, "%f %f %f %f %f %f\n",
                          obj.x(),
                          obj.y(),
                          obj.z(),
                          obj.roll(),
                          obj.pitch(),
                          obj.yaw());
        }
        fclose(path);
    } else {
        fprintf(stats, "failed to plan");
        ROS_ERROR("failed to plan\n");
    }
    fclose(stats);
}


void StatsWriter::writeARA(std::vector<double> &stats_v, std::vector<FullBodyState> &states, 
                           int trial_id){
    ROS_INFO("writing ara stats");
    stringstream ss;

    ss << m_current_path.str().c_str() << "ara_" << std::setfill('0') << std::setw(2) << trial_id << ".stats";
    ROS_DEBUG_NAMED(HEUR_LOG, "Opening file : %s", ss.str().c_str());
    FILE* stats = fopen(ss.str().c_str(), "w");
    fprintf(stats, "%f %f %f %f %f %f %f %f %f %f\n", stats_v[0],
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
        ss2 << m_current_path.str().c_str() << "ara_" << std::setfill('0') << std::setw(2) << trial_id << ".path";
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < states.size(); i++){
            vector<double> l_arm = states[i].left_arm;
            vector<double> r_arm = states[i].right_arm;
            vector<double> base = states[i].base;
            vector<double> obj = states[i].obj;

            // theta is in [2]
            fprintf(path, "%f %f %f %f ",
                    base[0],
                    base[1],
                    base[3],
                    base[2]);
            fprintf(path, "%f %f %f %f %f %f %f ",
                          r_arm[0], 
                          r_arm[1], 
                          r_arm[2], 
                          r_arm[3], 
                          r_arm[4], 
                          r_arm[5], 
                          r_arm[6]);
            fprintf(path, "%f %f %f %f %f %f\n",
                        obj[0],
                        obj[1],
                        obj[2],
                        obj[3],
                        obj[4],
                        obj[5]);
        }
        fclose(path);
    }
    fclose(stats);
}

void StatsWriter::writeSBPL(std::vector<double> &stats_v, std::vector<FullBodyState> &states,
                           int trial_id, std::string file_prefix){
    ROS_INFO("writing %s stats", file_prefix.c_str());
    stringstream ss;
    // if(imha == true){
      ss << m_current_path.str().c_str() << file_prefix.c_str() << std::setfill('0') << std::setw(2) << trial_id << ".stats";
    // } else {
      // ss << m_current_path.str().c_str() << "smha_" << std::setfill('0') << std::setw(2) << trial_id << ".stats";
    // }
    ROS_DEBUG_NAMED(HEUR_LOG, "Opening file : %s", ss.str().c_str());
    FILE* stats = fopen(ss.str().c_str(), "w");
    fprintf(stats, "%f %f %f %f %f %f %f %f %f %f\n", stats_v[0],
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
        // if(imha == true){
          ss2 << m_current_path.str().c_str() << file_prefix.c_str() << std::setfill('0') << std::setw(2) << trial_id << ".path";
        // } else {
          // ss2 << m_current_path.str().c_str() << "smha_" << std::setfill('0') << std::setw(2) << trial_id << ".path";
        // }
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < states.size(); i++){
            vector<double> l_arm = states[i].left_arm;
            vector<double> r_arm = states[i].right_arm;
            vector<double> base = states[i].base;
            vector<double> obj = states[i].obj;

            // theta is in [2]
            fprintf(path, "%f %f %f %f ",
                    base[0],
                    base[1],
                    base[3],
                    base[2]);
            fprintf(path, "%f %f %f %f %f %f %f ",
                          r_arm[0], 
                          r_arm[1], 
                          r_arm[2], 
                          r_arm[3], 
                          r_arm[4], 
                          r_arm[5], 
                          r_arm[6]);
            fprintf(path, "%f %f %f %f %f %f\n",
                        obj[0],
                        obj[1],
                        obj[2],
                        obj[3],
                        obj[4],
                        obj[5]);
        }
        fclose(path);
    }
    fclose(stats);
}

void StatsWriter::writeStartGoal(int trial_id, std::pair<RobotState, RobotState>
          start_goal, int seed) {
    ROS_INFO("writing current start goal");
    stringstream ss;
    // if(imha == true){
    ss << m_current_path.str().c_str() << "start_goal_" << std::setfill('0') 
        << std::setw(2) << trial_id << ".env";

    ROS_DEBUG_NAMED(HEUR_LOG, "Opening file : %s", ss.str().c_str());
    FILE* stats = fopen(ss.str().c_str(), "w");
    start_goal.first.printToFile(stats);
    start_goal.second.printToFile(stats);
    fprintf(stats, "%d\n", seed);
    fclose(stats);

    /* The file should look like:
    start_base
    start_larm
    start_rarm
    goal_base
    goal_larm
    goal_rarm
    */
}