#include <monolithic_pr2_planner/Heuristics/EndEffectorHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/2Dgridsearch.h>

using namespace monolithic_pr2_planner;

EndEffectorHeuristic::EndEffectorHeuristic() {
	
	int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX+1);
    m_size_row = static_cast<unsigned int>(dimY+1);
    // ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] initialized BFS2D of size %d %d", 
    //                           m_size_col, m_size_row);

    // m_gridsearch.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
    //     m_occupancy_grid->getResolution()));

    
    // Initialize the grid here itself so that you don't wait for the map
    // callback to be called
    m_grid = new unsigned char*[m_size_col];
    for (unsigned int i=0; i < m_size_col; i++){
        m_grid[i] = new unsigned char[m_size_row];
        for (unsigned int j=0; j < m_size_row; j++){
            m_grid[i][j] = 0;
        }
    }
    
    m_bfs.reset(new sbpl_arm_planner::BFS_3D(dimX, dimY, dimZ));
}

void EndEffectorHeuristic::setGoal(GoalState& goal_state){
    DiscObjectState state = goal_state.getObjectState(); 
    m_goal = goal_state;
    m_bfs->run(state.x(),
               state.y(),
               state.z());
    ROS_DEBUG_NAMED(HEUR_LOG, "running Dijkstra for EndEffectorHeuristic on new goal %d %d %d", state.x(), state.y(), state.z());
}

int EndEffectorHeuristic::getGoalHeuristic(GraphStatePtr state){
    if (m_goal.withinXYZTol(state)){
        return 0;
    }
    DiscObjectState obj_state = state->getObjectStateRelMap();
    int cost = m_bfs->getDistance(obj_state.x(), obj_state.y(), obj_state.z());
    int base_x = state->base_x();
    int base_y = state->base_y();
    int threshold = 80;
    if(m_grid[base_x][base_y] > threshold)
    	return INFINITECOST;
    // ROS_DEBUG_NAMED(HEUR_LOG, "3D dijkstra's cost to %d %d %d is %d", 
    //                 obj_state.x(), obj_state.y(), obj_state.z(), cost);
    return getCostMultiplier()*cost;
}
