#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace monolithic_pr2_planner;

BFS2DHeuristic::BFS2DHeuristic(){
    // int threshold = 80;
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX+1);
    m_size_row = static_cast<unsigned int>(dimY+1);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] initialized BFS2D of size %d %d", 
                              m_size_col, m_size_row);

    // 0.7710 is the arm's length in meters when extended fully.
    // Set it to something slightly smaller so that we underestimate it just a
    // little.
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

    // m_circlepub = m_nh.advertise<geometry_msgs::PolygonStamped>("/goal_circle", 1,
        // true);
}

BFS2DHeuristic::~BFS2DHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete m_grid[i];
    }
}

void BFS2DHeuristic::update2DHeuristicMap(const std::vector<signed char>& data){
    loadMap(data);
}

void BFS2DHeuristic::loadMap(const std::vector<signed char>& data){
    
    for (unsigned int j=0; j < m_size_row; j++){
        for (unsigned int i=0; i < m_size_col; i++){
            m_grid[i][j] = (data[j*m_size_col+i]);
        }
    }

    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] updated grid of size %d %d from the map", m_size_col, m_size_row);
}

void BFS2DHeuristic::setGoal(GoalState& goal_state){
    unsigned char threshold = 80;
    DiscObjectState state = goal_state.getObjectState(); 
    // Set the goal state to 0,0 - just make sure it's not the start state.
    visualizeRadiusAroundGoal(state.x(), state.y());
    visualizeCenter(state.x(), state.y());
    m_gridsearch->search(m_grid, threshold, state.x(), state.y(),
        0,0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Setting goal %d %d", state.x(), state.y());
    m_goal = goal_state;
}

int BFS2DHeuristic::getGoalHeuristic(GraphStatePtr state){
    DiscObjectState obj_state = state->getObjectStateRelMap();
    
    // Check if within bounds. We need to do this here because the bfs2d
    // implementation doesn't take care of this.
    if (state->base_x() < 0 || state->base_x() >= int(m_size_col) || 
        state->base_y() < 0 || state->base_y() >= int(m_size_row)) {
        ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Out of bounds: %d %d", 
                                  state->base_x(), state->base_y());
        return INFINITECOST;
    }
    int cost = m_gridsearch->getlowerboundoncostfromstart_inmm(state->base_x(), state->base_y());
    // ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] 2Ddijkstra's cost to %d %d is %d", 
    //                           state->base_x(), state->base_y(),
    //                           (cost)<0?INFINITECOST:cost );
    if (cost < 0){
        return INFINITECOST;
    }

    // Add rotation cost
    // For inadmissible heuristic alone
    if(!m_radius){
        DiscObjectState goal_state = m_goal.getObjectState();
        double current_angle = normalize_angle_positive(std::atan2(goal_state.y() - state->base_y(),
            goal_state.x() - state->base_x()));
        cost += 10*shortest_angular_distance(current_angle,state->base_theta());
    }
    // if (cost < m_costmap_ros->getInscribedRadius()/0.02)
        // return 0;
    return getCostMultiplier()*cost;
}

void BFS2DHeuristic::visualizeCenter(int x, int y){
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2DHeuristic] Visualizing %d %d,", x, y);
    vector<vector<double> > startpoint; 
    std::vector<double> color(4,1);
    color[1] = color[2] = 0;
    vector<double> point;
    point.push_back(x*m_occupancy_grid->getResolution());
    point.push_back(y*m_occupancy_grid->getResolution());
    point.push_back(0.0);
    startpoint.push_back(point);
    std::stringstream ss;
    ss<<"start_point"<<x<<y;
    Visualizer::pviz->visualizeBasicStates(startpoint, color,ss.str(), 0.02);
}

void BFS2DHeuristic::setRadiusAroundGoal(double radius_m){
    m_radius = radius_m;
    m_gridsearch.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
        m_occupancy_grid->getResolution(), radius_m));
}

void BFS2DHeuristic::visualizeRadiusAroundGoal(int x0, int y0){
    if(!m_radius)
        return;
    std::vector<int> circle_x;
    std::vector<int> circle_y;
    double res = m_occupancy_grid->getResolution();
    int discrete_radius = m_radius/res;
    getBresenhamCirclePoints(x0, y0, discrete_radius, circle_x, circle_y);
    
    // geometry_msgs::PolygonStamped circle;
    
    // circle.header.frame_id = "/map";
    // circle.header.stamp = ros::Time::now();
    unsigned char threshold = 80;
    std::vector<geometry_msgs::Point> circle_points;
    
    for (size_t i = 0; i < circle_x.size(); ++i)
    {
        // ROS_DEBUG_NAMED(HEUR_LOG, "Circle points: %f %f", circle_x[i]*res,
        //     circle_y[i]*res);
        if(m_grid[circle_x[i]][circle_y[i]] <= threshold){
            geometry_msgs::Point out_pt;
            out_pt.x = circle_x[i]*res;
            out_pt.y = circle_y[i]*res;
            out_pt.z = 0.0;
            circle_points.push_back(out_pt);
        }
    }
    std::stringstream ss;
    ss<<"radius_around_goal"<<x0<<y0;
    Visualizer::pviz->visualizeLine(circle_points, ss.str(), x0 + y0, 114, 0.01);
}
