#include <monolithic_pr2_planner/Heuristics/EndEffOnlyRotationHeuristic.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

using namespace monolithic_pr2_planner;
using namespace sbpl_arm_planner;
using namespace std;

EndEffOnlyRotationHeuristic::EndEffOnlyRotationHeuristic(){ 
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
}

int EndEffOnlyRotationHeuristic::getGoalHeuristic(GraphStatePtr state){
    if (m_goal.withinXYZTol(state)){
        return 0;
    }
    double roll, pitch, yaw;
    m_desired_orientation.GetRPY(roll, pitch, yaw);
    ContObjectState robot_obj = state->robot_pose().getObjectStateRelMap();
    double angular_dist = std::fabs(shortest_angular_distance(robot_obj.roll(),
                                                            roll)) + 
                          std::fabs(shortest_angular_distance(robot_obj.pitch(),
                                                            pitch)) +
                          std::fabs(shortest_angular_distance(robot_obj.yaw(),
                                                            yaw));
    int rot_heur = static_cast<int>(1000*angular_dist);

    return rot_heur;
}


void EndEffOnlyRotationHeuristic::setGoal(GoalState& goal_state){
    DiscObjectState state = goal_state.getObjectState(); 
    m_goal = goal_state;
    ROS_DEBUG_NAMED(HEUR_LOG, "EndEffOnlyRotationHeuristic initialized on new goal %d %d %d",
                    state.x(), state.y(), state.z());
}

void EndEffOnlyRotationHeuristic::update3DHeuristicMap(){
    loadObstaclesFromOccupGrid();
}

void EndEffOnlyRotationHeuristic::loadObstaclesFromOccupGrid(){
    // int dimX, dimY, dimZ;
    // m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    // int walls = 0;
    // for (int z = 0; z < dimZ - 2; z++){
    //     for (int y = 0; y < dimY - 2; y++){
    //         for (int x = 0; x < dimX - 2; x++){
    //             if(m_occupancy_grid->getDistance(x,y,z) <= m_gripper_sphere_radius){
    //                 m_bfs->setWall(x + 1, y + 1, z + 1); //, true);
    //                 walls++;
    //             }
    //         }
    //     }
    // }
    // ROS_DEBUG_NAMED(HEUR_LOG, "Initialized EndEffOnlyRotationHeuristic with %d walls", walls);
    // ROS_DEBUG_NAMED(HEUR_LOG, "using gripper sphere radius %f", 
    //                 m_gripper_sphere_radius);
}

void EndEffOnlyRotationHeuristic::setDesiredOrientation(KDL::Rotation
    desired_orientation) {
    m_desired_orientation = desired_orientation;
}