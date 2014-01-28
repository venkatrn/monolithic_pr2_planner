#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/Constants.h>
#include <kdl/frames.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace KDL;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.
EnvInterfaces::EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::Environment> env) : 
    m_env(env), m_collision_space_interface(env->getCollisionSpace(), env->getHeuristicMgr()),
    m_generator(env->getCollisionSpace()), m_ompl_planner(env->getCollisionSpace()){
        getParams();
    bool forward_search = true;
    // m_planner.reset(new ARAPlanner(m_env.get(), forward_search));
    m_planner.reset(new MPlanner(m_env.get(), 3, forward_search));
    m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub", 1);
    m_costmap_publisher.reset(new
        costmap_2d::Costmap2DPublisher(m_nodehandle,1,"/map"));
}

void EnvInterfaces::getParams(){
    m_nodehandle.param<string>("reference_frame", m_params.ref_frame, 
                                    string("map"));
}

void EnvInterfaces::bindPlanPathToEnv(string service_name){
    m_plan_service = m_nodehandle.advertiseService(service_name, 
                                                   &EnvInterfaces::planPathCallback,
                                                   this);
}

void EnvInterfaces::bindExperimentToEnv(string service_name){
    m_experiment_service = m_nodehandle.advertiseService(service_name, 
                                                         &EnvInterfaces::experimentCallback,
                                                         this);
}

/*! \brief this is callback is purely for simulation purposes
 */
bool EnvInterfaces::experimentCallback(GetMobileArmPlan::Request &req,
                                       GetMobileArmPlan::Response &res){
    ROS_INFO("running simulations!");
    vector<pair<RobotState, RobotState> > start_goal_pairs;
    RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
    m_generator.initializeRegions();
    m_generator.generateUniformPairs(1, start_goal_pairs);

    for (auto& start_goal : start_goal_pairs){
        ROS_INFO("using start:");
        start_goal.first.printToInfo(SEARCH_LOG);
        ROS_INFO("using goal:");
        start_goal.second.printToInfo(SEARCH_LOG);
        SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
        search_request->initial_epsilon = req.initial_eps;
        search_request->final_epsilon = req.final_eps;
        search_request->decrement_epsilon = req.dec_eps;
        search_request->obj_goal= start_goal.second.getObjectStateRelMap();
        search_request->base_start = start_goal.first.base_state();
        search_request->base_goal = start_goal.second.base_state();
        search_request->left_arm_start = start_goal.first.left_arm();
        search_request->right_arm_start = start_goal.first.right_arm();
        search_request->left_arm_goal = start_goal.second.left_arm();
        search_request->right_arm_goal = start_goal.second.right_arm();

        KDL::Frame rarm_offset, larm_offset;
        rarm_offset.p.x(req.rarm_object.pose.position.x);
        rarm_offset.p.y(req.rarm_object.pose.position.y);
        rarm_offset.p.z(req.rarm_object.pose.position.z);
        larm_offset.p.x(req.larm_object.pose.position.x);
        larm_offset.p.y(req.larm_object.pose.position.y);
        larm_offset.p.z(req.larm_object.pose.position.z);

        rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                             req.rarm_object.pose.orientation.y, 
                                             req.rarm_object.pose.orientation.z, 
                                             req.rarm_object.pose.orientation.w);
        larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                             req.larm_object.pose.orientation.y, 
                                             req.larm_object.pose.orientation.z, 
                                             req.larm_object.pose.orientation.w);
        search_request->left_arm_object = larm_offset;
        search_request->right_arm_object = rarm_offset;
        search_request->xyz_tolerance = req.xyz_tolerance;
        search_request->roll_tolerance = req.roll_tolerance;
        search_request->pitch_tolerance = req.pitch_tolerance;
        search_request->yaw_tolerance = req.yaw_tolerance;
        search_request->planning_mode = req.planning_mode;

        res.stats_field_names.resize(18);
        res.stats.resize(18);
        int start_id, goal_id;
        clock_t total_planning_time = clock();
        if(!m_env->configureRequest(search_request, start_id, goal_id))
            return false;

        //exp happening here. note - the configureRequest call must happen before
        //the ompl planner is called (because we have to configure RobotState's IK
        //solver from the planning request.
        // m_ompl_planner.planPathCallback(*search_request);

        m_planner->set_initialsolution_eps(search_request->initial_epsilon);
        m_planner->set_initialsolution_eps1(50);
        m_planner->set_initialsolution_eps2(3);
        bool return_first_soln = true;
        m_planner->set_search_mode(return_first_soln);
        m_planner->set_start(start_id);
        ROS_INFO("setting goal id to %d", goal_id);
        m_planner->set_goal(goal_id);
        m_planner->force_planning_from_scratch();
        vector<int> soln;
        int soln_cost;
        bool isPlanFound = m_planner->replan(req.allocated_planning_time, 
                                             &soln, &soln_cost);

        if (isPlanFound){
            ROS_INFO("Plan found. Moving on to reconstruction.");
            vector<FullBodyState> states =  m_env->reconstructPath(soln);
            total_planning_time = clock() - total_planning_time;
            vector<string> stat_names;
            vector<double> stats;
            packageStats(stat_names, stats, soln_cost, states.size(),
                total_planning_time);
            res.stats_field_names = stat_names;
            res.stats = stats;
        } else {
            ROS_INFO("No plan found!");
        }
        return isPlanFound;
    }

    return true;

}

bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res){
    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    search_request->obj_goal= req.goal;
    search_request->base_start = req.body_start;
    search_request->left_arm_start = LeftContArmState(req.larm_start);
    search_request->right_arm_start = RightContArmState(req.rarm_start);

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                         req.rarm_object.pose.orientation.y, 
                                         req.rarm_object.pose.orientation.z, 
                                         req.rarm_object.pose.orientation.w);
    larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                         req.larm_object.pose.orientation.y, 
                                         req.larm_object.pose.orientation.z, 
                                         req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = req.xyz_tolerance;
    search_request->roll_tolerance = req.roll_tolerance;
    search_request->pitch_tolerance = req.pitch_tolerance;
    search_request->yaw_tolerance = req.yaw_tolerance;
    search_request->planning_mode = req.planning_mode;

    res.stats_field_names.resize(18);
    res.stats.resize(18);
    int start_id, goal_id;
    double total_planning_time = clock();
    bool retVal = m_env->configureRequest(search_request, start_id, goal_id);
    if(!retVal){
        return false;
    }

    m_planner->set_initialsolution_eps(search_request->initial_epsilon);
    m_planner->set_initialsolution_eps1(20);
    m_planner->set_initialsolution_eps2(5);
    bool return_first_soln = true;
    m_planner->set_search_mode(return_first_soln);
    m_planner->set_start(start_id);
    ROS_INFO("setting goal id to %d", goal_id);
    m_planner->set_goal(goal_id);
    m_planner->force_planning_from_scratch();
    vector<int> soln;
    int soln_cost;
    bool isPlanFound = m_planner->replan(req.allocated_planning_time, 
                                         &soln, &soln_cost);

    if (isPlanFound){
        ROS_INFO("Plan found. Moving on to reconstruction.");
        vector<FullBodyState> states =  m_env->reconstructPath(soln);
        total_planning_time = clock() - total_planning_time;
        vector<string> stat_names;
        vector<double> stats;
        packageStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        res.stats_field_names = stat_names;
        res.stats = stats;
    } else {
        ROS_INFO("No plan found!");
    }
    return isPlanFound;
}

void EnvInterfaces::packageStats(vector<string>& stat_names, 
                                 vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time){
    
    stat_names.resize(10);
    stats.resize(10);
    stat_names[0] = "total plan time";
    stat_names[1] = "initial solution planning time";
    stat_names[2] = "initial epsilon";
    stat_names[3] = "initial solution expansions";
    stat_names[4] = "final epsilon planning time";
    stat_names[5] = "final epsilon";
    stat_names[6] = "solution epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution cost";
    stat_names[9] = "path length";

    // TODO fix the total planning time
    //stats[0] = totalPlanTime;
    stats[0] = total_planning_time/static_cast<double>(CLOCKS_PER_SEC);
    stats[1] = m_planner->get_initial_eps_planning_time();
    stats[2] = m_planner->get_initial_eps();
    stats[3] = m_planner->get_n_expands_init_solution();
    stats[4] = m_planner->get_final_eps_planning_time();
    stats[5] = m_planner->get_final_epsilon();
    stats[6] = m_planner->get_solution_eps();
    stats[7] = m_planner->get_n_expands();
    stats[8] = static_cast<double>(solution_cost);
    stats[9] = static_cast<double>(solution_size);
}

bool EnvInterfaces::bindCollisionSpaceToTopic(string topic_name){
    m_collision_space_interface.bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}

void EnvInterfaces::bindNavMapToTopic(string topic){
    m_nav_map = m_nodehandle.subscribe(topic, 1, &EnvInterfaces::loadNavMap, this);
}

void EnvInterfaces::crop2DMap(const nav_msgs::MapMetaData& map_info, const std::vector<signed char>&
    v,
                              double new_origin_x, double new_origin_y,
                              double width, double height, 
                              vector<signed char>& final_map){
    vector<vector<signed char> > tmp_map(map_info.height);
    for (unsigned int i=0; i < map_info.height; i++){
        for (unsigned int j=0; j < map_info.width; j++){
            tmp_map[i].push_back(v[i*map_info.height+j]);
        }
    }

    double res = map_info.resolution;
    int new_origin_x_idx = (new_origin_x-map_info.origin.position.x)/res;
    int new_origin_y_idx = (new_origin_y-map_info.origin.position.y)/res;
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
    ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu", final_map.size());
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++){
            final_map[i*new_map[i].size() + j] = new_map[i][j];
        }
    }
}

void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr& map){
    ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                    map->info.width, map->info.height, map->info.resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                                                      map->info.origin.position.y);

    // look up the values from the occup grid parameters
    // This stuff is in cells.
    int dimX, dimY, dimZ;
    m_collision_space_interface.getOccupancyGridSize(dimX, dimY,
        dimZ);
    // This costmap_ros object listens to the map topic as defined
    // in the costmap_2d.yaml file.
    m_costmap_ros.reset(new costmap_2d::Costmap2DROS("costmap_2d", m_tf));

    // Get the underlying costmap in the cost_map object.
    // Publish for visualization. Publishing is done for the entire (uncropped) costmap.
    costmap_2d::Costmap2D cost_map;
    m_costmap_ros->getCostmapCopy(cost_map);

    // Normalize and convert to array.
    std::vector<signed char> uncropped_map;
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Row major. X is row wise, Y is column wise.
            int c = cost_map.getCost(i,j);

            // Set unknowns to free space (we're dealing with static maps for
            // now)
            c = (c==costmap_2d::NO_INFORMATION)?costmap_2d::FREE_SPACE:c;

            // Re-set the cost.
            cost_map.setCost(i,j,c);
        }
    }

    // Re-inflate because we modified the unknown cells to be free space.
    cost_map.reinflateWindow(dimX*map->info.resolution/2, dimY*map->info.resolution/2, dimX*map->info.resolution, dimY*map->info.resolution);

    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Normalize the values from 0 to 100. Not absolutely needed, but
            // makes life easier when dealing with the heuristic later.
            uncropped_map.push_back(static_cast<double>(cost_map.getCost(i,j))/costmap_2d::NO_INFORMATION*100);
        }
    }
    m_costmap_publisher->updateCostmapData(cost_map,
    m_costmap_ros->getRobotFootprint());

    // Publish the full costmap
    m_costmap_publisher->publishCostmap();
    m_costmap_publisher->publishFootprint();
    
    std::vector<signed char> final_map;

    // TODO: Check if this is the right thing to do : Take the resolution from
    // the map for the occupancy grid's values.
    double width = dimX*map->info.resolution;
    double height = dimY*map->info.resolution;
    
    crop2DMap(map->info, uncropped_map, 0, 0, width, height, final_map);
    
    // Don't want to publish this.
    nav_msgs::OccupancyGrid costmap_pub;
    costmap_pub.header.frame_id = "/map";
    costmap_pub.header.stamp = ros::Time::now();
    costmap_pub.info.map_load_time = ros::Time::now();
    costmap_pub.info.resolution = map->info.resolution;
    // done in the crop function too.
    costmap_pub.info.width = (width/map->info.resolution+1);
    costmap_pub.info.height = (height/map->info.resolution+1);
    costmap_pub.info.origin.position.x = 0;
    costmap_pub.info.origin.position.y = 0;
    costmap_pub.data = final_map;

    // Publish the cropped version of the costmap
    m_costmap_pub.publish(costmap_pub);

    m_collision_space_interface.update2DHeuristicMaps(final_map);
}
