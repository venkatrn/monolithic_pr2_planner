#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
using namespace monolithic_pr2_planner;

BFS2DHeuristic::BFS2DHeuristic(){
    int threshold = 80;
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX+1);
    m_size_row = static_cast<unsigned int>(dimY+1);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] initialized BFS2D of size %d %d", 
                              m_size_col, m_size_row);
    m_bfs.reset(new sbpl_bfs_2d(m_size_col, m_size_row, threshold));
    
    // Initialize the grid here itself so that you don't wait for the map
    // callback to be called
    m_grid = new int*[m_size_col];
    for (unsigned int i=0; i < m_size_col; i++){
        m_grid[i] = new int[m_size_row];
        for (unsigned int j=0; j < m_size_row; j++){
            m_grid[i][j] = 0;
        }
    }
    m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub", 1);
    m_costmap_publisher.reset(new
        costmap_2d::Costmap2DPublisher(m_nodehandle,1,"/map"));
}

// BFS2DHeuristic::BFS2DHeuristic(boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros):
//     m_costmap_ros(costmap_ros) {
//     BFS2DHeuristic();
// }

BFS2DHeuristic::~BFS2DHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete m_grid[i];
    }
}

void BFS2DHeuristic::update2DHeuristicMap(const std::vector<signed char>& data){
    loadMap(data);
}

void BFS2DHeuristic::loadMap(const std::vector<signed char>& data){
    // sleep(5);
    m_costmap_ros.reset(new costmap_2d::Costmap2DROS("costmap_2d", m_tf));
    costmap_2d::Costmap2D cost_map;
    m_costmap_ros->getCostmapCopy(cost_map);
    m_costmap_publisher->updateCostmapData(cost_map,
        m_costmap_ros->getRobotFootprint());
    m_costmap_publisher->publishCostmap();
    m_costmap_publisher->publishFootprint();



    int numel =0;
    int numObs = 0;
    int numUnknown = 0;
    int numFree = 0;
    std::vector<signed char> uncropped_map;
    for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
    {
        for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
        {
            numel++;
            int c = cost_map.getCost(j,i);
            c = (c==costmap_2d::NO_INFORMATION)?costmap_2d::FREE_SPACE:c;
            uncropped_map.push_back(static_cast<double>(c)/255.0*100);
            if(c == -1)
                numUnknown++;
            else if (c > 50)
                numObs++;
            else
                numFree++;
        }
    }

    // uncropped_map is the costmap of full size. Need to crop it.
    int width = (m_size_col-1)*0.02;
    int height = (m_size_row-1)*0.02;
    std::vector<signed char> final_map;
    crop2DMap(uncropped_map, 0, 0, width, height, final_map);

    for (unsigned int j=0; j < m_size_row; j++){
        for (unsigned int i=0; i < m_size_col; i++){
            m_grid[i][j] = (final_map[j*m_size_col+i]);
        }
    }
    nav_msgs::OccupancyGrid costmap_pub;
    costmap_pub.header.frame_id = "/map";
    costmap_pub.header.stamp = ros::Time::now();
    costmap_pub.info.map_load_time = ros::Time::now();
    costmap_pub.info.resolution = 0.02;
    // done in the crop function too.
    costmap_pub.info.width = (width/0.02+1);
    costmap_pub.info.height = (height/0.02+1);
    costmap_pub.info.origin.position.x = 0;
    costmap_pub.info.origin.position.y = 0;
    costmap_pub.data = final_map;
    m_costmap_pub.publish(costmap_pub);

    ROS_DEBUG_NAMED(HEUR_LOG, "Numel: %d, numUnknown: %d, numFree: %d, numObs: %d", numel, numUnknown, numFree, numObs);


    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] updated grid of size %d %d from the map", m_size_col, m_size_row);
    // visualize();
}

void BFS2DHeuristic::setGoal(GoalState& goal_state){
    DiscObjectState state = goal_state.getObjectState(); 
    m_bfs->compute_distance_from_point(m_grid, state.x(), state.y());
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
    int cost = m_bfs->get_distance(state->base_x(), state->base_y());
    // ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] 2Ddijkstra's cost to %d %d is %d", 
    //                           state->base_x(), state->base_y(),
    //                           (getCostMultiplier()*cost)<0?INFINITECOST:getCostMultiplier()*cost );
    if (cost < 0){
        return INFINITECOST;
    }
    if (cost < m_costmap_ros->getInscribedRadius()/0.02)
        return 0;
    return getCostMultiplier()*cost;
}

void BFS2DHeuristic::visualize(){
    int dimX, dimY, dimZ;
    int threshold = 250;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    vector<vector<double> > obstacles; 
    for (int y = 0; y < dimY; y++){
        for (int x = 0; x < dimX; x++){
            if(m_grid[x][y] >= threshold){
                vector<double> point;
                point.push_back(x);
                point.push_back(y);
                point.push_back(0.1);
                point.push_back(.1);
                point.push_back(.1);
                point.push_back(.1);

                obstacles.push_back(point);
            }
        }
    }
    Visualizer::pviz->visualizeObstacles(obstacles);
}

void BFS2DHeuristic::crop2DMap(const std::vector<signed char>& v,
                              double new_origin_x, double new_origin_y,
                              double width, double height, 
                              vector<signed char>& final_map){
    vector<vector<signed char> > tmp_map(1251);
    for (unsigned int i=0; i < 1251; i++){
        for (unsigned int j=0; j < 1251; j++){
            tmp_map[i].push_back(/*map->data.at(i*map->info.height+j)*/v[i*1251+j]);
        }
    }

    double res = 0.02;
    int new_origin_x_idx = (new_origin_x+12.5)/res;
    int new_origin_y_idx = (new_origin_y+12.5)/res;
    int new_width = width/res + 1;
    int new_height = height/res + 1;
    ROS_DEBUG_NAMED(HEUR_LOG, "new origin: %d %d, width and height: %d %d",
                              new_origin_x_idx, new_origin_y_idx, new_width, 
                              new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of map %lu %lu", tmp_map.size(), 
                                                     tmp_map[0].size());

    vector<vector<signed char> > new_map(new_height);
    int row_count = 0;
    for (int i=new_origin_y_idx; i < new_origin_y_idx + new_height; i++){
        for (int j=new_origin_x_idx; j < new_origin_x_idx + new_width; j++){
            new_map[row_count].push_back(tmp_map[i][j]);
        }
        row_count++;
    }
    final_map.resize(new_width * new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu; %d %d",
        final_map.size(), new_map.size(), new_map[0].size());
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++){
            final_map[i*new_map[i].size() + j] = new_map[i][j];
        }
    }
}
