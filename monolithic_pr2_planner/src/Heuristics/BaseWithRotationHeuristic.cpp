#include <monolithic_pr2_planner/Heuristics/BaseWithRotationHeuristic.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <costmap_2d/cost_values.h>

using namespace monolithic_pr2_planner;

BaseWithRotationHeuristic::BaseWithRotationHeuristic(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX+1);
    m_size_row = static_cast<unsigned int>(dimY+1);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BaseWithRotationHeur] initialized BaseWithRotationHeuristic of size %d %d", 
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
}

BaseWithRotationHeuristic::~BaseWithRotationHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete[] m_grid[i];
    }
    delete[] m_grid;
    m_grid = NULL;
}

void BaseWithRotationHeuristic::update2DHeuristicMap(const std::vector<unsigned char>& data){
    loadMap(data);
}

void BaseWithRotationHeuristic::loadMap(const std::vector<unsigned char>& data){
    
    for (unsigned int j=0; j < m_size_row; j++){
        for (unsigned int i=0; i < m_size_col; i++){
            m_grid[i][j] = (data[j*m_size_col+i]);
        }
    }

    ROS_DEBUG_NAMED(HEUR_LOG, "[BaseWithRotationHeur] updated grid of size %d %d from the map", m_size_col, m_size_row);
}

// This must be called after setting original goal.
void BaseWithRotationHeuristic::setGoal(GoalState& goal_state) {
    // Save the goal for future use.
    m_goal = goal_state;

    DiscObjectState state = goal_state.getObjectState(); 
    BFS2DHeuristic::visualizeCenter(state.x(), state.y());

    // Get the initial points for the dijkstra search
    // std::vector<std::pair<int,int> > init_points;
    // BFS2DHeuristic::getBresenhamLinePoints(state.x(), state.y(), orig_state.x(), orig_state.y(), init_points);

    // Set the goal state to 0,0 - just make sure it's not the start state.
    m_gridsearch->search(m_grid, costmap_2d::INSCRIBED_INFLATED_OBSTACLE, state.x(), state.y(),
        0,0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BaseWithRotationHeur] Setting goal %d %d", state.x(), state.y());
}

int BaseWithRotationHeuristic::getGoalHeuristic(GraphStatePtr state){
    DiscObjectState obj_state = state->getObjectStateRelMapFromState();

    
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

    DiscBaseState disc_base_state(state->base_x(), state->base_y(), state->base_z(), state->base_theta());
    ContBaseState cont_base_state(disc_base_state);
    // Get the angle to the goal from the sampled point.
    double rot_dist = std::fabs(shortest_angular_distance(cont_base_state.theta(),
                                                          m_desired_orientation));
    if(rot_dist < 2*M_PI/m_resolution_params.num_base_angles)
      rot_dist = 0;
    int rot_heur = 1000*rot_dist;

    return getCostMultiplier()*cost + rot_heur;
}

void BaseWithRotationHeuristic::visualizeLineToOriginalGoal(int x0, int y0, int x1, int y1,
    double res){
    
    std::vector<std::pair<int,int> > points;
    
    BFS2DHeuristic::getBresenhamLinePoints(x0, y0, x1, y1, points);

    // geometry_msgs::PolygonStamped circle;
    
    // circle.header.frame_id = "/map";
    // circle.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Point> line_points;
    
    for (size_t i = 0; i < points.size(); ++i)
    {
        geometry_msgs::Point out_pt;
        out_pt.x = points[i].first*res;
        out_pt.y = points[i].second*res;
        out_pt.z = 0.0;
        line_points.push_back(out_pt);
    }
    std::stringstream ss;
    ss<<"line_goal"<<x0<<y0;
    Visualizer::pviz->visualizeLine(line_points, ss.str(), x0 + y0, 240, 0.01);
}

void BaseWithRotationHeuristic::setDesiredOrientation(KDL::Rotation desired_orientation){
    double roll, pitch, yaw;
    desired_orientation.GetRPY(roll, pitch, yaw);
    m_desired_orientation = normalize_angle_positive(yaw);
}
