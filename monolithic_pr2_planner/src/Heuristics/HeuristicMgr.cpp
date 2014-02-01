#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/Heuristics/BFS3DHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
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
 
pair<Point, Point> sample_points(int radius, int center_x, int center_y,
                  vector<int> circle_x, vector<int> circle_y)
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
    int i = sorted_pts.size()/3;
    return pair<Point, Point>(sorted_pts[i], sorted_pts[2*i]);
}

HeuristicMgr::HeuristicMgr(){
    m_num_mha_heuristics = 0;
}

int HeuristicMgr::add3DHeur(const int cost_multiplier){

    // Initialize the new heuristic.
    AbstractHeuristicPtr new_3d_heur = make_shared<BFS3DHeuristic>();
    // MUST set the cost multiplier here. If not, it is taken as 1.
    new_3d_heur->setCostMultiplier(cost_multiplier);
    // Add it to the list of heuristics
    m_heuristics.push_back(new_3d_heur);
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
}

void HeuristicMgr::update3DHeuristicMaps(){
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->update3DHeuristicMap();
    }
}

void HeuristicMgr::setGoal(GoalState& goal_state){

    // Save goal state for future use
    m_goal = goal_state;

    // At this point, there are no dynamic heuristics.
    // TODO: Change this if we initialize the grids before the planning request
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        ROS_DEBUG_NAMED(HEUR_LOG, "[HeurMgr] Setting goal for heuristic %d", int(i));
        m_heuristics[i]->setGoal(goal_state);
    }
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
    
    DiscObjectState goal_state = m_goal.getObjectState();
     
    // Euclidean Heuristic : only for the MHA heuristics
    for (int i = 0; i < m_mha_heur_ids.size(); ++i)
    {
        int center_x = m_sampled_x[i];
        int center_y = m_sampled_y[i];
        double goal_angle = normalize_angle_positive(std::atan2(static_cast<double>(m_goal.getObjectState().y() -
            center_y),static_cast<double>(m_goal.getObjectState().x()
                    - center_x)));
        int heur =
            static_cast<int>(1000*std::fabs(shortest_angular_distance(state->robot_pose().getContBaseState().theta(),
            goal_angle)));
        values[m_mha_heur_ids[i]] += heur;
        // ROS_DEBUG_NAMED(HEUR_LOG, "Heur idd : %d Goal point : %d %d", m_mha_heur_ids[i], m_sampled_x[i],
        //     m_sampled_y[i]);
    }

    return values;
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
    getBresenhamCirclePoints(state.x(), state.y(), discrete_radius, circle_x, circle_y);
    
    ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Radius around goal: %f, resolution: %f, discrete radius: %d", radius_around_goal, res, discrete_radius);
    // No, we cannot have more number of heuristics than there are points on
    // the cirlce. That's just redundant.
    assert(circle_x.size() > m_num_mha_heuristics);
    
    ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Circle points %d", static_cast<int>(circle_x.size()));
    // Sample along the circle.
    /* Get the list of points that are not on an obstacle.
     * Get the size of this list. Sample from a uniform distribution. 
     * Make sure you don't repeat points. */
    unsigned char threshold = 80;
    for (size_t i = 0; i < circle_x.size();)
    {
        // Reject points on obstacles.
        if(m_grid[circle_x[i]][circle_y[i]] > threshold){    //Obstacle!
            circle_x.erase(circle_x.begin() + i);
            circle_y.erase(circle_y.begin() + i);
        }
        else
            i++;
    }

    ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Circle points cropped %d",static_cast<int>(circle_x.size()));
    // We have a list of points in circle_x and circle_y that are not on
    // obstacles. Let's sample uniformly.

    // Pick symmetrically

    // Get the angles that the free points describe
    // std::vector<double> angles_with_center(circle_x.size(),0);

    // for (size_t i = 0; i < angles_with_center.size(); ++i)
    // {
    //     angles_with_center[i] = normalize_angle_positive(std::atan2(static_cast<double>(circle_y[i] -
    //         center_y),static_cast<double>(circle_x[i]
    //                 - center_x)) - M_PI);
    //     RobotState seed_robot_pose;
    //     int theta = DiscBaseState::convertContTheta(angles_with_center[i]);
    //     DiscBaseState seed_base_state(circle_x[i],circle_y[i],0.1,
    //         theta);
    //     seed_robot_pose.base_state(seed_base_state);
    //     ROS_DEBUG_NAMED(HEUR_LOG, "Visualizing the robot for IK: Theta %d",
    //         theta);
    //     RobotPosePtr final_pose;
        
    //     KDL::Vector v;
    //     ContBaseState seed_base_c(seed_base_state);
    //     v.x(seed_base_c.x() - 0.05);
    //     v.y(seed_base_c.y());
    //     v.z(0.739675 + 0.051 + 0.3);
    //     KDL::Rotation rot = KDL::Rotation::RPY(0,0,angles_with_center[i]);
    //     KDL::Frame robot_tf(rot, v);
    //     KDL::Vector v_g;
    //     ContObjectState goal_c = m_goal.getObjectState();
    //     v_g.x(goal_c.x());
    //     v_g.y(goal_c.y());
    //     v_g.z(goal_c.z());
    //     KDL::Rotation rot_g = KDL::Rotation::RPY(goal_c.roll(),goal_c.pitch(),
    //         goal_c.yaw());
    //     double qx,qy,qz,qw;
    //     rot_g.GetQuaternion(qx,qy,qz,qw);
    //     KDL::Frame goal_frame(rot_g, v_g);

    //     KDL::Frame displacement = robot_tf.Inverse()*goal_frame;
    //     double r,p,y;
    //     displacement.M.GetRPY(r,p,y);

    //     seed_base_c.printToDebug(HEUR_LOG);
    //     ContObjectState goal_base_frame(displacement.p.x(),
    //                                     displacement.p.y(),
    //                                     displacement.p.z(),
    //                                     r,p,y);
    //     ROS_INFO("displacement Z: %f", displacement.p.z());
    //     ROS_INFO("goal base");
    //     goal_base_frame.printToDebug(HEUR_LOG);
    //     DiscObjectState d_goal_base(goal_base_frame);
    //     ROS_INFO("seed robot pose");
    //     std::vector<double> right_arm_start(7,0);
    //     right_arm_start[0] = -0.034127;
    //     right_arm_start[1] = 0.309261;
    //     right_arm_start[2] = 0.000000;
    //     right_arm_start[3] = -1.614009;
    //     right_arm_start[4] = 2.987015;
    //     right_arm_start[5] = -1.413143;
    //     right_arm_start[6] = 2.889659;
    //     RightContArmState r_arm_seed(right_arm_start);

    //     LeftContArmState l_arm;
    //     l_arm.setShoulderPan(0.038946287971107774);
    //     l_arm.setShoulderLift(1.2146697069025374);
    //     l_arm.setUpperArmRoll(1.3963556492780154);
    //     l_arm.setElbowFlex(-1.1972269899800325);
    //     l_arm.setForearmRoll(-4.616317135720829);
    //     l_arm.setWristFlex(-0.9887266887318599);
    //     l_arm.setWristRoll(1.1755681069775656);
    //     r_arm_seed.setUpperArmRoll(randomDouble(-3.75, .65));
    //     seed_robot_pose.right_arm(r_arm_seed);
    //     seed_robot_pose.left_arm(l_arm);
    //     seed_robot_pose.printToInfo(HEUR_LOG);
    //     seed_robot_pose.visualize();
    //     std::vector<double> obj_new_ik(7,0);
    //     obj_new_ik[0] = (displacement.p.x());
    //     obj_new_ik[1] = (displacement.p.y());
    //     obj_new_ik[2] = (displacement.p.z());
    //     obj_new_ik[3] = qx;
    //     obj_new_ik[4] = qy;
    //     obj_new_ik[5] = qz;
    //     obj_new_ik[6] = qz;

    //     std::vector<double> seed_new_ik(7,0);
    //     std::vector<double> joint_angles_new_ik(7,0);
    //     bool ik_success = getRightIK(obj_new_ik, seed_new_ik, joint_angles_new_ik);
    //     // bool ik_success = RobotState::computeRobotPose(d_goal_base, seed_robot_pose,
    //         // final_pose);
    //     ROS_INFO("ik success is %d", ik_success);
    //     if(ik_success){
    //         state.x(circle_x[i]);
    //         state.y(circle_y[i]);
    //         ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    //         circle_x[i], circle_y[i]);
    //         GoalState another_goal_state(m_goal);
    //         another_goal_state.setGoal(state);
    //         int heur_num = add2DHeur(cost_multiplier, 0);
    //         m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
    //         m_heuristics[heur_num]->setGoal(another_goal_state);
    //         m_mha_heur_ids.push_back(heur_num);
    //         break;
    //     }
    //     ROS_ERROR("at cin");
    //     std::cin.get();

    // }

    // RobotState computeRobotPose(const DiscObjectState& disc_obj_state,
    //                              const RobotState& seed_robot_pose,
    //                              RobotPosePtr& new_robot_pose,
    //                              bool free_angle_search){

    int center_x = state.x();
    int center_y = state.y();
    pair <Point, Point> selected_points = sample_points(discrete_radius,
        center_x, center_y, circle_x, circle_y);

    // Add the first one
    state.x(selected_points.first.first);
    state.y(selected_points.first.second);
    ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    selected_points.first.first, selected_points.first.second);
    {
        GoalState new_goal_state(m_goal);
        new_goal_state.setGoal(state);
        int heur_num = add2DHeur(cost_multiplier, 0);
        m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
        m_heuristics[heur_num]->setGoal(new_goal_state);
        m_mha_heur_ids.push_back(heur_num);
        m_sampled_x.push_back(selected_points.first.first);
        m_sampled_y.push_back(selected_points.first.second);
    }
    // Add the second one
    state.x(selected_points.second.first);
    state.y(selected_points.second.second);
    ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    selected_points.second.first, selected_points.second.second);
    {
        GoalState new_goal_state(m_goal);
        new_goal_state.setGoal(state);
        int heur_num = add2DHeur(cost_multiplier, 0);
        m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
        m_heuristics[heur_num]->setGoal(new_goal_state);
        m_mha_heur_ids.push_back(heur_num);
        m_sampled_x.push_back(selected_points.second.first);
        m_sampled_y.push_back(selected_points.second.second);
    }
    
    // Random selection
    // int number_of_points = circle_x.size();
    // int num_selected = 0;
    // while(num_selected < m_num_mha_heuristics) {
    //     int p = rand()%number_of_points;
    //     ++num_selected;
    //     state.x(circle_x[p]);
    //     state.y(circle_y[p]);
    //     m_sampled_x.push_back(circle_x[p]);
    //     m_sampled_y.push_back(circle_y[p]);
    //     ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    //         circle_x[p], circle_y[p]);
    //     GoalState new_goal_state(m_goal);
    //     new_goal_state.setGoal(state);
    //     int heur_num = add2DHeur(cost_multiplier, 0);
    //     m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
    //     m_heuristics[heur_num]->setGoal(new_goal_state);
    //     m_mha_heur_ids.push_back(heur_num);

    //     // Temporary, to make sure points are selected on opposing sides.
    //     int side = (circle_x[p] - m_goal.getObjectState().x() < 0)?-1:1;
    //     for (size_t i = 0; i < circle_x.size();)
    //     {
    //         if((circle_x[i] - m_goal.getObjectState().x())*side > 0){
    //             // erase
    //             ROS_DEBUG_NAMED(HEUR_LOG, "Removing %d %d", circle_x[i],
    //                 circle_y[i]);
    //             circle_x.erase(circle_x.begin() + i);
    //             circle_y.erase(circle_y.begin() + i);
    //         }
    //         else
    //             ++i;
    //     }
    //     number_of_points = circle_x.size();
    // }
    // GoalState new_goal_state = boost::make_shared<GoalState>(goal_state);
}
