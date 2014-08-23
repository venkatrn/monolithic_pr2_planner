#include <monolithic_pr2_planner/Heuristics/VoronoiOrientationHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>

using namespace monolithic_pr2_planner;

VoronoiOrientationHeuristic::VoronoiOrientationHeuristic(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX+1);
    m_size_row = static_cast<unsigned int>(dimY+1);
    ROS_DEBUG_NAMED(HEUR_LOG, "[VoronoiOrientationHeuristic] initialized heuristic of size %d %d", 
                              m_size_col, m_size_row);

    m_gridsearch.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
        m_occupancy_grid->getResolution()));
    
    // Initialize the grid here itself so that you don't wait for the map
    // callback to be called
    m_grid = new unsigned char*[m_size_col];
    for (unsigned int i=0; i < m_size_col; i++){
        m_grid[i] = new unsigned char[m_size_row];
        for (unsigned int j=0; j < m_size_row; j++){
            m_grid[i][j] = 0;
        }
    }

    m_obstacle_binary_grid = new bool*[m_size_col];
    for (unsigned int i=0; i < m_size_col; i++){
        m_obstacle_binary_grid[i] = new bool[m_size_row];
        for (unsigned int j=0; j < m_size_row; j++){
            m_obstacle_binary_grid[i][j] = false;
        }
    }
}

VoronoiOrientationHeuristic::~VoronoiOrientationHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete m_grid[i];
        delete m_obstacle_binary_grid[i];
    }
    delete m_grid;
    delete m_obstacle_binary_grid;
}

void VoronoiOrientationHeuristic::update2DHeuristicMap(const std::vector<unsigned char>& data){
    loadMap(data);
}

void VoronoiOrientationHeuristic::loadMap(const std::vector<unsigned char>& data){
    for (unsigned int j=0; j < m_size_row; j++){
        for (unsigned int i=0; i < m_size_col; i++){
            m_grid[i][j] = (data[j*m_size_col+i]);
            m_obstacle_binary_grid[i][j] = (m_grid[i][j] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) ? true :
            false;
        }
    }

    ROS_DEBUG_NAMED(HEUR_LOG, "[VoronoiOrientationHeuristic] updated grid of size %d %d from the map", m_size_col, m_size_row);
    // compute the voronoi diagram
    m_dynamic_voronoi.initializeMap(m_size_col, m_size_row, m_obstacle_binary_grid);
    m_dynamic_voronoi.update();
    m_dynamic_voronoi.prune();
    m_dynamic_voronoi.visualize("/tmp/envt_voronoi.ppm");
    ROS_DEBUG_NAMED(HEUR_LOG, "Output envt_voronoi.ppm");
}

void VoronoiOrientationHeuristic::setGoal(GoalState& goal_state) {
    // Save the goal for future use.
    m_goal = goal_state;
    // it doesn't matter where the goal is for this guy. We only deal with the
    // map as such.
}

int VoronoiOrientationHeuristic::getGoalHeuristic(GraphStatePtr state){
    DiscObjectState obj_state = state->getObjectStateRelMap();
    
    // Check if within bounds. We need to do this here because the bfs2d
    // implementation doesn't take care of this.
    if (state->base_x() < 0 || state->base_x() >= int(m_size_col) || 
        state->base_y() < 0 || state->base_y() >= int(m_size_row)) {
        ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Out of bounds: %d %d", 
                                  state->base_x(), state->base_y());
        return INFINITECOST;
    }
    // Cost of the 2D search.
    int cost = m_gridsearch->getlowerboundoncostfromstart_inmm(state->base_x(), state->base_y());

    // Get the angle to the goal from the sampled point.

    return getCostMultiplier()*cost;
}
