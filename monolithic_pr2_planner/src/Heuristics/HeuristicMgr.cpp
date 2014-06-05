#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/Heuristics/BFS3DHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/EndEffectorHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/MHABaseHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/ArmAnglesHeuristic.h>
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

HeuristicMgr::HeuristicMgr() : 
    m_num_mha_heuristics(NUM_MHA_BASE_HEUR) {
}

/**
 * @brief Resets the heuristic manager.
 */
void HeuristicMgr::reset(){
    ROS_DEBUG_NAMED(HEUR_LOG, "Resetting the heuristic manager.");
    m_heuristics.clear();
    m_mha_heur_ids.clear();
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
            m_num_mha_heuristics = 2;
            break;
        case T_ARA:
        case T_MPWA:
            m_num_mha_heuristics = 0;
            break;
        case T_EES:
            m_num_mha_heuristics = 1;
            addUniformCost3DHeur();
            int unif_2d = addUniformCost2DHeur(1, 0.7);
            m_heuristics[unif_2d]->update2DHeuristicMap(m_grid_data);
            break;
    }
}

void HeuristicMgr::initializeHeuristics() {
    // NOTE: It's 40 for now, until the actual cost for arm costs are computed.
    // 3DHeur is unit costs - multiply by whatever you want.
    // To get them in terms of mm distance
    // add3DHeur(20);  //0 - multiply by 20 : grid resolution in mm :
    // underestimated
    int endeff_heur = add3DHeur(20);  // 0 - multiply by 20 : grid resolution in mm :
    // underestimated

    // Already in mm.
    m_base_heur_id = add2DHeur(1, 0.7);  // 1
    // The argument is TOTAL_HEUR_NUM - 1 (TOTAL_HEUR_NUM is what you
    // entered in the MPlanner parameters)

    // m_arm_angles_heur_id = addArmAnglesHeur(1);
}

int HeuristicMgr::add3DHeur(const int cost_multiplier, double* gripper_radius) {
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
    return m_heuristics.size() - 1;
}

int HeuristicMgr::addUniformCost3DHeur(){

    // Initialize the new heuristic.
    BFS3DHeuristicPtr new_3d_heur = make_shared<BFS3DHeuristic>();
    // MUST set the cost multiplier here. If not, it is taken as 1.
    new_3d_heur->setCostMultiplier(1);
    // Add it to the list of heuristics
    m_heuristics.push_back(new_3d_heur);
    return m_heuristics.size() - 1;
}

int HeuristicMgr::addEndEffHeur(const int cost_multiplier){

    // Initialize the new heuristic.
    AbstractHeuristicPtr new_end_eff_heur = make_shared<EndEffectorHeuristic>();
    // MUST set the cost multiplier here. If not, it is taken as 1.
    new_end_eff_heur->setCostMultiplier(cost_multiplier);
    // Add it to the list of heuristics
    m_heuristics.push_back(new_end_eff_heur);
    return m_heuristics.size() - 1;
}

int HeuristicMgr::add2DHeur(const int cost_multiplier, const double radius_m){
    // Initialize the new heuristic
    AbstractHeuristicPtr new_2d_heur = make_shared<BFS2DHeuristic>();
    // Set cost multiplier here.
    new_2d_heur->setCostMultiplier(cost_multiplier);
    new_2d_heur->setRadiusAroundGoal(radius_m);
    // Add to the list of heuristics
    m_heuristics.push_back(new_2d_heur);
    return m_heuristics.size() - 1;
}

int HeuristicMgr::addUniformCost2DHeur(const int cost_multiplier, const double radius_m){
    // Initialize the new heuristic
    BFS2DHeuristicPtr new_ucs_heur = make_shared<BFS2DHeuristic>();
    // Set cost multiplier here.
    new_ucs_heur->setCostMultiplier(cost_multiplier);
    new_ucs_heur->setRadiusAroundGoal(radius_m);
    new_ucs_heur->setUniformCostSearch(true);
    // Add to the list of heuristics
    m_heuristics.push_back(new_ucs_heur);
    return m_heuristics.size() - 1;
}

int HeuristicMgr::addMHABaseHeur(const int cost_multiplier){
    // Initialize the new heuristic
    AbstractHeuristicPtr new_mha_base_heur = make_shared<MHABaseHeuristic>();
    // Set cost multiplier here.
    new_mha_base_heur->setCostMultiplier(cost_multiplier);
    // Add to the list of heuristics
    m_heuristics.push_back(new_mha_base_heur);
    return m_heuristics.size() - 1;
}

int HeuristicMgr::addArmAnglesHeur(const int cost_multiplier){
    // Initialize the new heuristic
    AbstractHeuristicPtr new_arm_angles_heur =
    make_shared<ArmAnglesHeuristic>(m_cspace_mgr);
    // Set cost multiplier here.
    new_arm_angles_heur->setCostMultiplier(cost_multiplier);
    // Add to the list of heuristics
    m_heuristics.push_back(new_arm_angles_heur);
    return m_heuristics.size() - 1;
}

// most heuristics won't need both 2d and 3d maps. however, the abstract
// heuristic type has function stubs for both of them so we don't need to pick
// and choose who to update. it is up to the implementor to implement a derived
// function for the following, otherwise they won't do anything.
void HeuristicMgr::update2DHeuristicMaps(const std::vector<signed char>& data){
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
    // Create additional heuristics for MHA planner
    initializeMHAHeuristics(m_base_heur_id, 1);
}

std::vector<int> HeuristicMgr::getGoalHeuristic(const GraphStatePtr& state)
{
    if (!m_heuristics.size()){
        ROS_ERROR_NAMED(HEUR_LOG, "No heuristics initialized!");
    }
    std::vector<int> values(m_heuristics.size(),0);
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        values[i] = m_heuristics[i]->getGoalHeuristic(state);
    }

    return values;
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
        
        KDL::Frame to_robot_frame;
        Visualizer::pviz->getMaptoRobotTransform(cont_seed_base_state.x(),
            cont_seed_base_state.y(), cont_seed_base_state.theta(), 
            to_robot_frame);

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

RightContArmState HeuristicMgr::getRightArmIKSol(int g_x, int g_y){
    RobotPosePtr final_pose;
    bool ik_success = checkIKAtPose(g_x, g_y, final_pose);
    bool collision_free = false;
    if(ik_success)
        if(m_cspace_mgr->isValid(*final_pose))
            collision_free = true;
    if(collision_free)
        return final_pose->right_arm();
    else {
        return RightContArmState();
    }
}


void HeuristicMgr::initNewMHABaseHeur(int g_x, int g_y, RightContArmState& r_arm_state, const int cost_multiplier){
        // ROS_DEBUG_NAMED(HEUR_LOG, "IK was a success! Wooohoooo!");
        // Valid - push the state
        ROS_DEBUG_NAMED(HEUR_LOG, "New MHA Base Heuristic initialized : %d %d", 
            g_x, g_y);
        DiscObjectState state = m_goal.getObjectState(); 
        state.x(g_x);
        state.y(g_y);
        GoalState new_goal_state(m_goal);
        new_goal_state.setGoal(state);

        // Create the new heuristic
        int heur_num = addMHABaseHeur(cost_multiplier);
        // Update its costmap
        m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);

        // NOTE: Uncomment this if you want to use the base heuristic with
        // rotation.
        m_heuristics[heur_num]->setOriginalGoal(m_goal);
        m_heuristics[heur_num]->setGoal(new_goal_state);
        

        // m_heuristics[heur_num]->setGoalArmState(r_arm_state);
        m_mha_heur_ids.push_back(heur_num);
}

void HeuristicMgr::initializeMHAHeuristics(const int
    base_heur_id, const int cost_multiplier){
    
    if(!m_num_mha_heuristics)
        return;
    // Get the radius around the goal from the base heuristic.
    double radius_around_goal = m_heuristics[base_heur_id]->getRadiusAroundGoal();

    // Get points on the circle around the base heuristic.
    DiscObjectState state = m_goal.getObjectState(); 
    std::vector<int> circle_x;
    std::vector<int> circle_y;
    double res = m_occupancy_grid->getResolution();
    int discrete_radius = radius_around_goal/res;
    BFS2DHeuristic::getBresenhamCirclePoints(state.x(), state.y(), discrete_radius, circle_x, circle_y);

    // ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Radius around goal: %f, resolution: %f, discrete radius: %d", radius_around_goal, res, discrete_radius);
    // No, we cannot have more number of heuristics than there are points on
    // the cirlce. That's just redundant.
    assert(circle_x.size() > m_num_mha_heuristics);
    
    // ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Circle points %d", static_cast<int>(circle_x.size()));
    // Sample along the circle.
    /* Get the list of points that are not on an obstacle.
     * Get the size of this list. Sample from a uniform distribution. 
     * Make sure you don't repeat points. */
    unsigned char threshold = 80;
    for (size_t i = 0; i < circle_x.size();) {
        // Reject points on obstacles.
        if(m_grid[circle_x[i]][circle_y[i]] > threshold){    //Obstacle!
            circle_x.erase(circle_x.begin() + i);
            circle_y.erase(circle_y.begin() + i);
        }
        else {
            i++;
        }
    }

    // ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Circle points cropped %d",static_cast<int>(circle_x.size()));

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

    std::vector<Point> selected_points;
    if (static_cast<int>(ik_circle_x.size()) < m_num_mha_heuristics) {
        selected_points = sample_points(discrete_radius,
                center_x, center_y, circle_x, circle_y, m_num_mha_heuristics);
    } else {
        selected_points = sample_points(discrete_radius,
                center_x, center_y, ik_circle_x, ik_circle_y, m_num_mha_heuristics);
    }

    // assert(static_cast<int>(circle_x.size()) >= m_num_mha_heuristics);
    for (size_t i = 0; i < selected_points.size(); ++i) {
        RightContArmState r_arm_state =
        getRightArmIKSol(selected_points[i].first, selected_points[i].second);
        initNewMHABaseHeur(selected_points[i].first, selected_points[i].second,
            r_arm_state,
            cost_multiplier);
    }
    
    ROS_DEBUG_NAMED(HEUR_LOG, "--------------------");
    ROS_DEBUG_NAMED(HEUR_LOG, "Size of m_heuristics : %d", static_cast<int>
        (m_heuristics.size()));
    ROS_DEBUG_NAMED(HEUR_LOG, "--------------------");
}
