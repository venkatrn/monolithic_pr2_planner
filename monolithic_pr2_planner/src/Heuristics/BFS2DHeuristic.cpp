#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/cost_values.h>

using namespace monolithic_pr2_planner;

BFS2DHeuristic::BFS2DHeuristic(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX+1);
    m_size_row = static_cast<unsigned int>(dimY+1);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] initialized BFS2D of size %d %d", 
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

    // m_circlepub = m_nh.advertise<geometry_msgs::PolygonStamped>("/goal_circle", 1,
        // true);
}

BFS2DHeuristic::~BFS2DHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete[] m_grid[i];
    }
    delete[] m_grid;
    m_grid = NULL;
}

void BFS2DHeuristic::update2DHeuristicMap(const std::vector<unsigned char>& data){
    loadMap(data);
}

void BFS2DHeuristic::loadMap(const std::vector<unsigned char>& data){
    
    for (unsigned int j=0; j < m_size_row; j++){
        for (unsigned int i=0; i < m_size_col; i++){
            m_grid[i][j] = (data[j*m_size_col+i]);
        }
    }

    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] updated grid of size %d %d from the map", m_size_col, m_size_row);
}

void BFS2DHeuristic::setGoal(GoalState& goal_state){
    // Save the goal for future use.
    m_goal = goal_state;


    DiscObjectState state = goal_state.getObjectState(); 
    visualizeRadiusAroundGoal(state.x(), state.y());
    visualizeCenter(state.x(), state.y());

    // Get the initial points for the dijkstra search
    std::vector<std::pair<int,int> > init_points;
    double res = m_occupancy_grid->getResolution();
    int discrete_radius = m_radius/res;
    getBresenhamCirclePoints(state.x(), state.y(), discrete_radius, init_points);

    // Set the goal state to 0,0 - just make sure it's not the start state.
    m_gridsearch->search(m_grid, costmap_2d::INSCRIBED_INFLATED_OBSTACLE, state.x(), state.y(),
        0,0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS, init_points);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Setting goal %d %d", state.x(), state.y());
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

    
    // if (cost < m_costmap_ros->getInscribedRadius()/0.02)
        // return 0;
    return getCostMultiplier()*cost;
}

void BFS2DHeuristic::visualizeCenter(int x, int y) {
    return; //MIKE: disabled this visualization because it makes my rviz cry
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2DHeuristic] Visualizing %d %d,", x, y);
    vector <vector <double> > startpoint;
    std::vector <double> color(4, 1);
    color[1] = color[2] = 0;
    vector<double> point;
    point.push_back(x*m_occupancy_grid->getResolution());
    point.push_back(y*m_occupancy_grid->getResolution());
    point.push_back(0.0);
    startpoint.push_back(point);
    std::stringstream ss;
    ss << "start_point" << x << "_" << y;
    Visualizer::pviz->visualizeBasicStates(startpoint, color, ss.str(), 0.02);
}

void BFS2DHeuristic::setRadiusAroundGoal(double radius_m) {
    m_radius = radius_m;
    m_gridsearch.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
        m_occupancy_grid->getResolution(), radius_m));
}

void BFS2DHeuristic::visualizeRadiusAroundGoal(int x0, int y0) {
    if (!m_radius)
        return;
    std::vector<int> circle_x;
    std::vector<int> circle_y;
    double res = m_occupancy_grid->getResolution();
    int discrete_radius = m_radius/res;
    getBresenhamCirclePoints(x0, y0, discrete_radius, circle_x, circle_y);

    // geometry_msgs::PolygonStamped circle;

    // circle.header.frame_id = "/map";
    // circle.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Point> circle_points;

    for (size_t i = 0; i < circle_x.size(); ++i) {
        // Prune the points to display only the ones that are within the
        // threshold
        if (m_grid[circle_x[i]][circle_y[i]] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            geometry_msgs::Point out_pt;
            out_pt.x = circle_x[i]*res;
            out_pt.y = circle_y[i]*res;
            out_pt.z = 0.0;
            circle_points.push_back(out_pt);
        }
    }
    std::stringstream ss;
    ss<< "radius_around_goal";
    Visualizer::pviz->visualizeLine(
        circle_points, ss.str(), x0 + y0, 114, 0.01);
}

// ------------- Bresenham circle points -------------------//
void BFS2DHeuristic::getBresenhamCirclePoints(int x0, int y0, int radius, std::vector<int>&
    ret_x,
    std::vector<int>& ret_y) {
    int x = 0;
    int y = radius;
    int delta = 2 - 2 * radius;
    int err = 0;
    ret_x.clear();
    ret_y.clear();
    while(y >= 0){
        ret_x.push_back(x0 + x);
        ret_x.push_back(x0 - x);
        ret_x.push_back(x0 + x);
        ret_x.push_back(x0 - x);
        ret_y.push_back(y0 - y);
        ret_y.push_back(y0 - y);
        ret_y.push_back(y0 + y);
        ret_y.push_back(y0 + y);
        err = 2 * (delta + y) - 1;
        if(delta < 0 && err <= 0){
                x = x + 1;
                delta = delta + 2 * x + 1;
                continue;
        }
        err = 2 * (delta - x) - 1;
        if(delta > 0 && err > 0){
                y = y - 1;
                delta = delta + 1 - 2 * y;
                continue;
        }
        x = x + 1;
        delta = delta + 2 * (x - y);
        y = y - 1;
    }
}

void BFS2DHeuristic::getBresenhamCirclePoints(int x0, int y0, int radius, std::vector<std::pair<int,int> >&
    points){
    points.clear();
    std::vector<int> ret_x;
    std::vector<int> ret_y;
    getBresenhamCirclePoints(x0, y0, radius, ret_x, ret_y);
    for (size_t i = 0; i < ret_x.size(); ++i)
    {
        points.push_back(std::make_pair(ret_x[i], ret_y[i]));
    }
}

// ---------------- Bresenham Line points --------------------

void BFS2DHeuristic::getBresenhamLinePoints(int x1, int y1, int x2, int y2, std::vector<std::pair<int, int> >&
    points){
    std::vector<int> pts_x;
    std::vector<int> pts_y;
    getBresenhamLinePoints(x1, y1, x2, y2, pts_x, pts_y);
    for (size_t i = 0; i < pts_x.size(); ++i)
    {
        points.push_back(std::make_pair(pts_x[i], pts_y[i]));
    }
}


void BFS2DHeuristic::getBresenhamLinePoints(int x1, int y1, int x2, int y2, std::vector<int>& pts_x, std::vector<int>&
    pts_y){
    pts_x.clear();
    pts_y.clear();
    bool steep = (std::abs(y2 - y1) > std::abs(x2 - x1));
    if(steep){
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2){
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    int dx = x2 - x1;
    int dy = std::abs(y2 - y1);

    double err = dx / 2.0f;
    
    int ystep = (y1 < y2)? 1 : -1;
    int y = (y1);

    for (int x = x1; x <= x2; ++x) {
        if(steep){
            // y,x
            pts_x.push_back(y);
            pts_y.push_back(x);
        } else {
            // x, y
            pts_x.push_back(x);
            pts_y.push_back(y);
        }
        err = err - dy;
        if(err < 0){
            y = y + ystep;
            err = err +  dx;
        }
    }
    assert(pts_y.size() == pts_x.size());
}
