#include <monolithic_pr2_planner/Heuristics/BFS3DHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>

using namespace monolithic_pr2_planner;
using namespace sbpl_arm_planner;
using namespace std;

BFS3DHeuristic::BFS3DHeuristic(){ 
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_bfs.reset(new BFS_3D(dimX, dimY, dimZ));
    m_gripper_sphere_radius = m_resolution_params.gripper_sphere_radius;
}

int BFS3DHeuristic::getGoalHeuristic(GraphStatePtr state){
    if (m_goal.withinXYZTol(state)){
        return 0;
    }
    DiscObjectState obj_state = state->getObjectStateRelMap();
    int cost = m_bfs->getDistance(obj_state.x(), obj_state.y(), obj_state.z());
    // ROS_DEBUG_NAMED(HEUR_LOG, "3D dijkstra's cost to %d %d %d is %d", 
    //                 obj_state.x(), obj_state.y(), obj_state.z(), cost);
    return getCostMultiplier()*cost;
}

void BFS3DHeuristic::setGoal(GoalState& goal_state){
    DiscObjectState state = goal_state.getObjectState(); 
    m_goal = goal_state;
    m_bfs->run(state.x(),
               state.y(),
               state.z());
    ROS_DEBUG_NAMED(HEUR_LOG, "running BFS3Dheuristic on new goal %d %d %d",
                    state.x(), state.y(), state.z());
}

void BFS3DHeuristic::update3DHeuristicMap(){
    loadObstaclesFromOccupGrid();
}

void BFS3DHeuristic::loadObstaclesFromOccupGrid(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    int walls = 0;
    for (int z = 0; z < dimZ - 2; z++){
        for (int y = 0; y < dimY - 2; y++){
            for (int x = 0; x < dimX - 2; x++){
                if(m_occupancy_grid->getDistance(x,y,z) <= m_gripper_sphere_radius){
                    m_bfs->setWall(x + 1, y + 1, z + 1); //, true);
                    walls++;
                }
            }
        }
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "Initialized BFS3Dheuristic with %d walls", walls);
    ROS_DEBUG_NAMED(HEUR_LOG, "using gripper sphere radius %f", 
                    m_gripper_sphere_radius);
}

void BFS3DHeuristic::visualize(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    vector<vector<double> > obstacles; 
    for (int z = 0; z < dimZ - 2; z++){
        for (int y = 0; y < dimY - 2; y++){
            for (int x = 0; x < dimX - 2; x++){
                double gripper_radius = m_resolution_params.gripper_sphere_radius;
                if(m_occupancy_grid->getDistance(x,y,z) <= gripper_radius){
                    vector<double> point;
                    point.push_back(x+1);
                    point.push_back(y+1);
                    point.push_back(z+1);
                    point.push_back(.1);
                    point.push_back(.1);
                    point.push_back(.1);

                    obstacles.push_back(point);
                }
            }
        }
    }
    Visualizer::pviz->visualizeObstacles(obstacles);
}
