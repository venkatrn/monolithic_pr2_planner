#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/Heuristics/BFS3DHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/EndEffOnlyRotationHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/EndEffLocalHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BFS3DWithRotationHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BaseWithRotationHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BFS2DRotFootprintHeuristic.h>
#include <costmap_2d/cost_values.h>
#include <kdl/frames.hpp>
#include <memory>
#include <vector>
#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include <cmath>

using namespace monolithic_pr2_planner;
using namespace boost;

typedef pair<int, int> Point;

bool pairCompare(const std::pair<double, Point>& firstElem,
                 const std::pair<double, Point>& secondElem) {
    return firstElem.first < secondElem.first;
}
 
void deletePoint(Point end_pt, vector<int>& circle_x, vector<int>& circle_y) 
{
    for (size_t i=0; i < circle_x.size(); i++){
        if (circle_x[i] == end_pt.first && circle_y[i] == end_pt.second){
            circle_x.erase(circle_x.begin()+i);
            circle_y.erase(circle_y.begin()+i);
            return;
        }
    }
}
 
std::vector<Point> sample_points(int radius, int center_x, int center_y,
                  vector<int> circle_x, vector<int> circle_y, int num_p = 2)
{
    double itr = 0;
    vector<Point> full_circle;
    while (itr < 2*M_PI){
        Point point(radius * cos(itr) + center_x, radius * sin(itr) + center_y);
        itr += .1;
        full_circle.push_back(point);
    }
    vector<pair<double, Point> > distances;
    for (unsigned int i=0; i < full_circle.size(); i++){
        double min_dist = 100000000;
        Point closest_point;
        for (unsigned int j=0; j < circle_x.size(); j++){
            double dist = pow(pow((circle_x[j]-full_circle[i].first),2) +
                           pow((circle_y[j]-full_circle[i].second),2),.5);
            if (dist < min_dist){
                min_dist = dist;
                Point this_pt(circle_x[j], circle_y[j]);
                closest_point = this_pt;
            }
        }
        distances.push_back(pair<double, Point>(min_dist, closest_point));
    }
    std::sort(distances.begin(), distances.end(), pairCompare);
    int first_pt_idx=0;
    while (distances[first_pt_idx].first < 3){
        first_pt_idx++;
    }
    Point pt = distances[first_pt_idx].second;
    deletePoint(pt, circle_x, circle_y);
    vector<Point> sorted_pts;
    sorted_pts.push_back(pt);
    while (circle_x.size()){
        double min_dist = 100000;
        int delete_id = -1;
        for (size_t i=0; i < circle_x.size(); i++){
            double dist = pow(pow((circle_x[i]-pt.first),2) +
                           pow((circle_y[i]-pt.second),2),.5);
            if (dist < min_dist){
                min_dist = dist;
                delete_id = i;
            }
        }
        sorted_pts.push_back(Point(circle_x[delete_id], circle_y[delete_id]));
        pt = Point(circle_x[delete_id], circle_y[delete_id]);
        deletePoint(pt, circle_x, circle_y);
    }
    int i = sorted_pts.size()/(num_p + 1);
    std::vector<Point> final_points;
    for (size_t k = 1; static_cast<int>(final_points.size()) < num_p; ++k)
    {
        final_points.push_back(sorted_pts[k*i]);
    }
    return final_points;
}

Point get_approach_point(int center_x, int center_y, std::vector<int> circle_x,
    std::vector<int> circle_y, double goal_yaw) {
    // The idea is to loop through the valid points on the circle and find the
    // one that has the minimum angular distance to the center from the goal.
    // In other words, we want the robot to be facing the same way as the goal, and
    // not just facing the goal.
    int min_idx = -1;
    double min_angular_distance = 2*M_PI;
    assert(circle_x.size());
    assert(circle_y.size());

    for (size_t idx = 0; idx < circle_x.size(); ++idx) {
        // angle that the current point makes with the center of the goal.
        double angle_with_goal = normalize_angle_positive(std::atan2(
            static_cast<double>(center_y - circle_y[idx]),
            static_cast<double>(center_x - circle_x[idx])));
        // We want the distance of this with the yaw of the original goal.
        double angular_distance = std::fabs(
            shortest_angular_distance(
                goal_yaw,
                angle_with_goal
                )
            );
        // ROS_DEBUG_NAMED(HEUR_LOG, "Point : %d %d, angular_distance: %f",
        //     circle_x[idx], circle_y[idx], angular_distance);
        if (angular_distance < min_angular_distance) {
            min_angular_distance = angular_distance;
            min_idx = idx;
        }
    }
    return Point(circle_x[min_idx], circle_y[min_idx]);
}

HeuristicMgr::HeuristicMgr() : 
    m_num_mha_heuristics(NUM_MHA_BASE_HEUR) {
}

HeuristicMgr::~HeuristicMgr()
{
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);

    for (int i=0; i < dimX + 1; i++){
        delete[] m_grid[i];
    }
    delete[] m_grid;
}

/**
 * @brief Resets the heuristic manager.
 */
void HeuristicMgr::reset(){
    ROS_DEBUG_NAMED(HEUR_LOG, "Resetting the heuristic manager.");
    m_heuristics.clear();
    m_heuristic_map.clear();
    initializeHeuristics();
    // update3DHeuristicMaps();
    update2DHeuristicMaps(m_grid_data);
}

/**
 * @brief sets the planner type - mainly for experiments for the MHA paper
 * @details change the internal planner type to any of the different planners
 */
void HeuristicMgr::setPlannerType(int planner_type) {
    m_planner_type = planner_type;
    switch (planner_type) {
        case T_SMHA:
        case T_IMHA:
        case T_MHG_REEX:
        case T_MHG_NO_REEX:
            m_num_mha_heuristics = 1;
            break;
        case T_ARA:
        case T_MPWA:
            m_num_mha_heuristics = 0;
            break;
        case T_EES:
            m_num_mha_heuristics = 1;
            addUniformCost3DHeur("uniform_3d");
            addUniformCost2DHeur("uniform_2d", 0.7);
            m_heuristics[m_heuristic_map["uniform_2d"]]->update2DHeuristicMap(m_grid_data);
            break;
    }
}

void HeuristicMgr::initializeHeuristics() {
    // NOTE: It's 40 for now, until the actual cost for arm costs are computed.
    // 3DHeur is unit costs - multiply by whatever you want.
    // To get them in terms of mm distance
    // add3DHeur(20);  //0 - multiply by 20 : grid resolution in mm :
    // underestimated
    {
        int cost_multiplier = 20;
        add3DHeur("admissible_endeff", cost_multiplier);  // 0 - multiply by 20 : grid resolution in mm :
    }

    // Already in mm.
    {
        int cost_multiplier = 1;
        double radius_around_goal = 0.60; //0.75;
        add2DHeur("admissible_base", cost_multiplier, radius_around_goal);
    }

    {
        int cost_multiplier = 1;
        KDL::Rotation rot = KDL::Rotation::RPY(M_PI/2, 0, 0);
        addEndEffOnlyRotationHeur("endeff_rot_vert", rot, cost_multiplier);
    }

    // {
    //     int cost_multiplier = 1;
    //     addVoronoiOrientationHeur("voronoi_heur", cost_multiplier);
    // }
}

void HeuristicMgr::add3DHeur(std::string name, const int cost_multiplier, double* gripper_radius) {
    // Initialize the new heuristic.
    BFS3DHeuristicPtr new_3d_heur = make_shared<BFS3DHeuristic>();
    // MUST set the cost multiplier here. If not, it is taken as 1.
    new_3d_heur->setCostMultiplier(cost_multiplier);
    // if gripper radius is provided, set it.
    if (gripper_radius){
        new_3d_heur->setGripperRadius(*gripper_radius);
    }
    new_3d_heur->update3DHeuristicMap();
    // Add it to the list of heuristics
    m_heuristics.push_back(new_3d_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

void HeuristicMgr::addEndEffWithRotHeur(std::string name, KDL::Rotation desired_orientation, const int cost_multiplier) {
    // Initialize the new heuristic.
    BFS3DWithRotationHeuristicPtr new_endeff_with_rot_heur =
    make_shared<BFS3DWithRotationHeuristic>();
    // MUST set the cost multiplier here. If not, it is taken as 1.
    new_endeff_with_rot_heur->setCostMultiplier(cost_multiplier);

    new_endeff_with_rot_heur->update3DHeuristicMap();
    new_endeff_with_rot_heur->setDesiredOrientation(desired_orientation);
    // Add it to the list of heuristics
    m_heuristics.push_back(new_endeff_with_rot_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

void HeuristicMgr::addEndEffOnlyRotationHeur(std::string name, KDL::Rotation desired_orientation, const int cost_multiplier) {
    // Initialize the new heuristic.
    EndEffOnlyRotationHeuristicPtr new_endeff_only_rot_heur =
    make_shared<EndEffOnlyRotationHeuristic>();

    // MUST set the cost multiplier here. If not, it is taken as 1.
    new_endeff_only_rot_heur->setCostMultiplier(cost_multiplier);

    new_endeff_only_rot_heur->update3DHeuristicMap();
    new_endeff_only_rot_heur->setDesiredOrientation(desired_orientation);

    // Add it to the list of heuristics
    m_heuristics.push_back(new_endeff_only_rot_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

void HeuristicMgr::addEndEffLocalHeur(std::string name, const int cost_multiplier, GoalState eefGoalRelBody){
    EndEffLocalHeuristicPtr heur = make_shared<EndEffLocalHeuristic>();
    heur->setCostMultiplier(cost_multiplier);
    heur->setGoal(eefGoalRelBody);
  
    // Add it to the list of heuristics
    m_heuristics.push_back(heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

void HeuristicMgr::addUniformCost3DHeur(std::string name){

    // Initialize the new heuristic.
    BFS3DHeuristicPtr new_3d_heur = make_shared<BFS3DHeuristic>();
    // MUST set the cost multiplier here. If not, it is taken as 1.
    new_3d_heur->setCostMultiplier(1);
    // Add it to the list of heuristics
    m_heuristics.push_back(new_3d_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

// int HeuristicMgr::addEndEffHeur(const int cost_multiplier){

//     // Initialize the new heuristic.
//     AbstractHeuristicPtr new_end_eff_heur = make_shared<EndEffectorHeuristic>();
//     // MUST set the cost multiplier here. If not, it is taken as 1.
//     new_end_eff_heur->setCostMultiplier(cost_multiplier);
//     // Add it to the list of heuristics
//     m_heuristics.push_back(new_end_eff_heur);
//     return m_heuristics.size() - 1;
// }

void HeuristicMgr::add2DHeur(std::string name, const int cost_multiplier, const double radius_m){
    // Initialize the new heuristic
    AbstractHeuristicPtr new_2d_heur = make_shared<BFS2DHeuristic>();
    // Set cost multiplier here.
    new_2d_heur->setCostMultiplier(cost_multiplier);
    new_2d_heur->setRadiusAroundGoal(radius_m);
    // Add to the list of heuristics
    m_heuristics.push_back(new_2d_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

void HeuristicMgr::addUniformCost2DHeur(std::string name, const double radius_m){
    // Initialize the new heuristic
    BFS2DHeuristicPtr new_ucs_heur = make_shared<BFS2DHeuristic>();
    // Set cost multiplier here.
    new_ucs_heur->setCostMultiplier(1);
    new_ucs_heur->setRadiusAroundGoal(radius_m);
    new_ucs_heur->setUniformCostSearch(true);
    // Add to the list of heuristics
    m_heuristics.push_back(new_ucs_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

void HeuristicMgr::addBaseWithRotationHeur(std::string name, const int cost_multiplier){
    // Initialize the new heuristic
    AbstractHeuristicPtr new_base_with_rot_heur = make_shared<BaseWithRotationHeuristic>();
    // Set cost multiplier here.
    new_base_with_rot_heur->setCostMultiplier(cost_multiplier);
    // Add to the list of heuristics
    m_heuristics.push_back(new_base_with_rot_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

void HeuristicMgr::addBFS2DRotFootprint(std::string name, const int cost_multiplier, 
                                        const double theta, const vector<sbpl_2Dpt_t>& footprintPolygon,
                                        const double radius_m){
    // Initialize the new heuristic
    BFS2DRotFootprintHeuristicPtr heur = make_shared<BFS2DRotFootprintHeuristic>();
    // Set cost multiplier here.
    heur->setCostMultiplier(cost_multiplier);
    // set the footprint
    heur->setFootprint(footprintPolygon, theta);
    heur->setRadiusAroundGoal(radius_m);
    heur->update2DHeuristicMap(m_grid_data);
    heur->setGoal(m_goal);

    static bool draw = true;
    if(draw){
      draw = false;
      heur->draw();
    }

    // Add to the list of heuristics
    m_heuristics.push_back(heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

// void HeuristicMgr::addVoronoiOrientationHeur(std::string name, const int cost_multiplier){
//     // Initialize the new heuristic
//     VoronoiOrientationHeuristicPtr new_voronoi_heur = make_shared<VoronoiOrientationHeuristic>();
//     // Set cost multiplier here.
//     new_voronoi_heur->setCostMultiplier(cost_multiplier);
//     // Add to the list of heuristics
//     m_heuristics.push_back(new_voronoi_heur);
//     m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
// }

//int HeuristicMgr::addArmAnglesHeur(const int cost_multiplier){
//  // Initialize the new heuristic
//  ArmAnglesHeuristicPtr new_arm_angles_heur = make_shared<ArmAnglesHeuristic>(m_cspace_mgr);
//  // Set cost multiplier here.
//  new_arm_angles_heur->setCostMultiplier(cost_multiplier);
//  // Add to the list of heuristics
//  m_heuristics.push_back(new_arm_angles_heur);
//  return m_heuristics.size() - 1;
//}

// most heuristics won't need both 2d and 3d maps. however, the abstract
// heuristic type has function stubs for both of them so we don't need to pick
// and choose who to update. it is up to the implementor to implement a derived
// function for the following, otherwise they won't do anything.
void HeuristicMgr::update2DHeuristicMaps(const std::vector<unsigned char>& data){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);

    m_grid = new unsigned char*[dimX + 1];
    for (int i=0; i < dimX + 1; i++){
        m_grid[i] = new unsigned char[dimY + 1];
        for (int j=0; j < dimY + 1; j++){
            m_grid[i][j] = (data[j*(dimX + 1)+i]);
        }
    }
    m_grid_data.assign(data.begin(), data.end());

    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->update2DHeuristicMap(data);
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "Size of m_heuristics: %ld", m_heuristics.size());
}

/**
 * @brief Updates the 3D Heuristic map for heuristics
 */
void HeuristicMgr::update3DHeuristicMaps(){
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->update3DHeuristicMap();
    }
}

void HeuristicMgr::setGoal(GoalState& goal_state){

    // Save goal state for future use
    m_goal = goal_state;

    // At this point, there are no dynamic heuristics.
    // NOTE: Change this if we initialize the grids before the planning request
    for (size_t i = 0; i < m_heuristics.size(); ++i) {
        ROS_DEBUG_NAMED(HEUR_LOG, "[HeurMgr] Setting goal for heuristic %d", 
            int(i));
        m_heuristics[i]->setGoal(goal_state);
    }
    {
        // Create additional heuristics for MHA planner
        int cost_multiplier = 1;
        initializeMHAHeuristics(cost_multiplier);
    }
}

void HeuristicMgr::getGoalHeuristic(const GraphStatePtr& state, std::unique_ptr<stringintmap>& values)
{
    if (!m_heuristics.size()){
        ROS_ERROR_NAMED(HEUR_LOG, "No heuristics initialized!");
    }
    values.reset(new stringintmap(m_heuristic_map));
    for (auto& heur: m_heuristic_map){
        (*values)[heur.first] = m_heuristics[heur.second]->getGoalHeuristic(state);
        // values[i] = m_heuristics[i]->getGoalHeuristic(state);
    }
}

bool HeuristicMgr::checkIKAtPose(int g_x, int g_y, RobotPosePtr& final_pose){
    DiscObjectState state = m_goal.getObjectState(); 
    int center_x = state.x();
    int center_y = state.y();

    double angle_with_center = normalize_angle_positive(
        std::atan2(static_cast<double>(g_y -
            center_y),static_cast<double>(g_x
                    - center_x)) - M_PI);
        RobotState seed_robot_pose;
        int theta = DiscBaseState::convertContTheta(angle_with_center);
        int torso = DiscBaseState::convertContDistance(randomDouble(0.0,0.3));
        DiscBaseState seed_base_state(g_x,g_y,torso, theta);
        seed_robot_pose.base_state(seed_base_state);

        // Randomize free angle
        RightContArmState r_arm;
        r_arm.setUpperArmRoll(randomDouble(-3.75, 0.65));

        seed_robot_pose.right_arm(r_arm);

        vector <double> init_l_arm(7, 0);
        init_l_arm[0] = (0.038946287971107774);
        init_l_arm[1] = (1.2146697069025374);
        init_l_arm[2] = (1.3963556492780154);
        init_l_arm[3] = -1.1972269899800325;
        init_l_arm[4] = (-4.616317135720829);
        init_l_arm[5] = -0.9887266887318599;
        init_l_arm[6] = 1.1755681069775656;
        LeftContArmState init_l_arm_v(init_l_arm);
        seed_robot_pose.left_arm(init_l_arm_v);
        
        // ROS_DEBUG_NAMED(HEUR_LOG, "Visualizing the robot for IK: Theta %d Torso: %d", theta,
        //     torso);

        ContBaseState cont_seed_base_state = seed_base_state.getContBaseState();
        
        // KDL::Frame to_robot_frame;
        // Visualizer::pviz->getMaptoRobotTransform(cont_seed_base_state.x(),
        //     cont_seed_base_state.y(), cont_seed_base_state.theta(), 
        //     to_robot_frame);

        ContObjectState goal_c = m_goal.getObjectState().getContObjectState();

        // seed_robot_pose.visualize();
        KDL::Frame obj_frame;
        obj_frame.p.x(goal_c.x());
        obj_frame.p.y(goal_c.y());
        obj_frame.p.z(goal_c.z());
        obj_frame.M = KDL::Rotation::RPY(goal_c.roll(), 
                                         goal_c.pitch(),
                                         goal_c.yaw());
        KDL::Frame internal_tf =
        seed_robot_pose.right_arm().getArmModel()->computeBodyFK(cont_seed_base_state.body_pose());
        KDL::Frame transform = internal_tf.Inverse() * obj_frame;
        double rr, rp, ry;
        transform.M.GetRPY(rr, rp, ry);
        ContObjectState goal_torso_frame(transform.p.x(),
                                        transform.p.y(),
                                        transform.p.z(),
                                        rr,rp,ry);
        DiscObjectState d_goal_torso_frame(goal_torso_frame);
        bool ik_success = RobotState::computeRobotPose(d_goal_torso_frame, seed_robot_pose,
            final_pose);
        return ik_success;
}

bool HeuristicMgr::isValidIKForGoalState(int g_x, int g_y){
    RobotPosePtr final_pose;
    bool ik_success = checkIKAtPose(g_x, g_y, final_pose);
    if(ik_success)
        return m_cspace_mgr->isValid(*final_pose);
    else
        return false;
}

void HeuristicMgr::initNewMHABaseHeur(std::string name, int g_x, int g_y, const int cost_multiplier,
    double desired_orientation){
    DiscObjectState state = m_goal.getObjectState(); 
    state.x(g_x);
    state.y(g_y);
    GoalState new_goal_state(m_goal);
    new_goal_state.setGoal(state);

    // Create the new heuristic
    addBaseWithRotationHeur(name, cost_multiplier);

    // Update its costmap
    m_heuristics[m_heuristic_map[name]]->update2DHeuristicMap(m_grid_data);

    m_heuristics[m_heuristic_map[name]]->setGoal(new_goal_state);

    // desired_orientation is a KDL frame. We set the yaw for the base.
    KDL::Rotation rot = KDL::Rotation::RPY(0, 0, desired_orientation);
    m_heuristics[m_heuristic_map[name]]->setDesiredOrientation(rot);

    ROS_DEBUG_NAMED(HEUR_LOG, "Initialized new MHA Base Heuristic with desired_orientation: %f", desired_orientation);
}

void HeuristicMgr::initializeMHAHeuristics(const int cost_multiplier){
    
    if(!m_num_mha_heuristics)
        return;
    // Get the radius around the goal from the base heuristic.
    double radius_around_goal = m_heuristics[m_heuristic_map["admissible_base"]]->getRadiusAroundGoal();

    // Get points on the circle around the base heuristic.
    DiscObjectState state = m_goal.getObjectState(); 
    std::vector<int> circle_x;
    std::vector<int> circle_y;
    double res = m_occupancy_grid->getResolution();
    int discrete_radius = radius_around_goal/res;
    BFS2DHeuristic::getBresenhamCirclePoints(state.x(), state.y(), discrete_radius, circle_x, circle_y);

    // No, we cannot have more number of heuristics than there are points on
    // the cirlce. That's just redundant.
    assert(circle_x.size() > m_num_mha_heuristics);
    
    // Sample along the circle.
    /* Get the list of points that are not on an obstacle.
     * Get the size of this list. Sample from a uniform distribution. 
     * Make sure you don't repeat points. */
    for (size_t i = 0; i < circle_x.size();) {
        // Reject points on obstacles.
        if(m_grid[circle_x[i]][circle_y[i]] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){    //Obstacle!
            circle_x.erase(circle_x.begin() + i);
            circle_y.erase(circle_y.begin() + i);
        }
        else {
            i++;
        }
    }

    int center_x = state.x();
    int center_y = state.y();

    std::vector<int> ik_circle_x;
    std::vector<int> ik_circle_y;

    for (size_t i = 0; i < circle_x.size(); ++i) {
        if(isValidIKForGoalState(circle_x[i], circle_y[i])){
            ik_circle_x.push_back(circle_x[i]);
            ik_circle_y.push_back(circle_y[i]);
        }
    }

    // If there are enough points that are valid from the IK test, then select
    // two points out of that. If not, just discard the whole IK thing and select
    // from the original circle itself.

    // std::vector<Point> selected_points;
    // if (static_cast<int>(ik_circle_x.size()) < m_num_mha_heuristics) {
    //     selected_points = sample_points(discrete_radius,
    //             center_x, center_y, circle_x, circle_y, m_num_mha_heuristics);
    // } else {
    //     selected_points = sample_points(discrete_radius,
    //             center_x, center_y, ik_circle_x, ik_circle_y, m_num_mha_heuristics);
    // }

    // Select only the point that is directly behind the goal. This is given by
    // the get_approach_point function
    std::vector<Point> selected_points;
    Point selected_point = get_approach_point(center_x, center_y, circle_x,
        circle_y, m_goal.getObjectState().getContObjectState().yaw());
    selected_points.push_back(selected_point);

    for (size_t num_base_heur = 0; num_base_heur < selected_points.size(); ++num_base_heur) {
        stringstream ss;
        ss << "base_with_rot_" << num_base_heur;

        // Compute the desired orientation.
        double orientation = normalize_angle_positive(std::atan2(
            static_cast<double>(m_goal.getObjectState().y() -
                selected_points[num_base_heur].second),
            static_cast<double>(m_goal.getObjectState().x() -
                selected_points[num_base_heur].first)));
        
        // Initialize with the desired orientation.
        initNewMHABaseHeur(ss.str(), selected_points[num_base_heur].first,
            selected_points[num_base_heur].second,
            cost_multiplier, orientation);

        // Visualize the line from the sampled point to the original goal point.
        // BaseWithRotationHeuristic::visualizeLineToOriginalGoal(m_goal.getObjectState().x(),
        //     m_goal.getObjectState().y(), selected_points[num_base_heur].first,
        //     selected_points[num_base_heur].second,
        //     m_occupancy_grid->getResolution());
    }
    initNewMHABaseHeur("base_with_rot_door", selected_points[0].first,
        selected_points[0].second, cost_multiplier, 0.0);
    {
        int cost_multiplier = 20;
        ContObjectState goal_state = m_goal.getObjectState().getContObjectState();
        KDL::Rotation rot = KDL::Rotation::RPY(goal_state.roll(), goal_state.pitch(),
            goal_state.yaw());
        addEndEffWithRotHeur("endeff_rot_goal", rot, cost_multiplier);
        m_heuristics[m_heuristic_map["endeff_rot_goal"]]->setGoal(m_goal);
    }

    clock_t new_heur_t0 = clock();
    vector<sbpl_2Dpt_t> footprint;
    double halfwidth = 0.39;
    double halflength = 0.39;
    sbpl_2Dpt_t pt_m;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    footprint.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    footprint.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    footprint.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    footprint.push_back(pt_m);
    for(int i=0; i<m_resolution_params.num_base_angles; i++){
      ROS_ERROR("init bfsRotFoot %d",i);
      double theta = DiscTheta2Cont(i, m_resolution_params.num_base_angles);
      addBFS2DRotFootprint("bfsRotFoot" + std::to_string(i), 1, theta, footprint, radius_around_goal+0.15);
    }

    ROS_ERROR("init arm_angles_folded");
    ContBaseState dummy_base;
    LeftContArmState dummy_larm;
    //RightContArmState folded_rarm({0.0, 1.1072800, -1.5566882, -2.124408, 0.0, 0.0, 0.0});
    RightContArmState folded_rarm({-0.2, 1.1072800, -1.5566882, -2.124408, 0.0, -1.57, 0.0});

    RobotState rs(dummy_base, folded_rarm, dummy_larm);
    DiscObjectState localFoldedArmObject = rs.getObjectStateRelBody();
    GoalState localFoldedArmGoal;
    localFoldedArmGoal.setGoal(localFoldedArmObject);
    addEndEffLocalHeur("arm_angles_folded", 500, localFoldedArmGoal);

    clock_t new_heur_t1 = clock();
    ROS_ERROR("new heuristics took %f time to compute",double(new_heur_t1-new_heur_t0)/CLOCKS_PER_SEC);
    //std::cin.get();

    printSummaryToInfo(HEUR_LOG);
}

void HeuristicMgr::printSummaryToInfo(char* logger){
    ROS_INFO_NAMED(logger, "--------------------------");
    ROS_INFO_NAMED(logger, "Summary of heuristics");
    ROS_INFO_NAMED(logger, "--------------------------");

    ROS_INFO_NAMED(logger, "Size of m_heuristics: %d", static_cast<int>(
        m_heuristic_map.size()));
    ROS_INFO_NAMED(logger, "What they are : ");
    for (auto& heuristic: m_heuristic_map){
        ROS_INFO_NAMED(logger, "%s -- id %d",
            heuristic.first.c_str(), heuristic.second);
    }
}

void HeuristicMgr::printSummaryToDebug(char* logger){
    ROS_DEBUG_NAMED(logger, "--------------------------");
    ROS_DEBUG_NAMED(logger, "Summary of heuristics");
    ROS_DEBUG_NAMED(logger, "--------------------------");

    ROS_DEBUG_NAMED(logger, "Size of m_heuristics: %d", static_cast<int>(
        m_heuristic_map.size()));
    ROS_DEBUG_NAMED(logger, "What they are : ");
    for (auto& heuristic: m_heuristic_map){
        ROS_DEBUG_NAMED(logger, "%s -- id %d",
            heuristic.first.c_str(), heuristic.second);
    }
}
