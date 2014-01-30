#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/Heuristics/BFS3DHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
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
 
void deletePoint(Point end_pt, vector<int>& circle_x, vector<int>& circle_y){
    for (size_t i=0; i < circle_x.size(); i++){
        if (circle_x[i] == end_pt.first && circle_y[i] == end_pt.second){
            circle_x.erase(circle_x.begin()+i);
            circle_y.erase(circle_y.begin()+i);
            return;
        }
    }
}
 
pair<Point, Point> sample_points(int radius, int center_x, int center_y,
                  vector<int> circle_x, vector<int> circle_y){
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

std::vector<int> HeuristicMgr::getGoalHeuristic(const GraphStatePtr& state){
    if (!m_heuristics.size()){
        ROS_ERROR_NAMED(HEUR_LOG, "No heuristics initialized!");
    }
    std::vector<int> values(m_heuristics.size(),0);
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        values[i] = m_heuristics[i]->getGoalHeuristic(state);
    }
    
    for (size_t i = 0; i < m_mha_heur_ids.size(); ++i)
    {
        DiscObjectState goal_state = m_goal.getObjectState();
        double current_angle = normalize_angle_positive(std::atan2(goal_state.y() - state->base_y(),
        goal_state.x() - state->base_x()));
        // ROS_DEBUG_NAMED(HEUR_LOG, "Goal: %d %d; Base: %d %d", goal_state.x(), goal_state.y(), state->base_x(), state->base_y());
        ContBaseState cbase = state->robot_pose().base_state();
        // ROS_DEBUG_NAMED(HEUR_LOG, "Within Zone: current_angle: %f ; Base_angle: %f, cost added: %d", current_angle,
        normalize_angle_positive(cbase.theta()), 10*std::abs(static_cast<int>(shortest_angular_distance(normalize_angle_positive(cbase.theta()), current_angle))));
        values[m_mha_heur_ids[i]] += 10*std::abs(static_cast<int>(shortest_angular_distance(normalize_angle_positive(cbase.theta()), current_angle)));
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
    int center_x = state.x();
    int center_y = state.y();

    pair <Point, Point> selected_points = sample_points(discrete_radius,
        center_x, center_y, circle_x, circle_y);

    // Add the first one
    state.x(selected_points.first.first);
    state.y(selected_points.first.second);
    ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    selected_points.first.first, selected_points.first.second);
    GoalState new_goal_state(m_goal);
    new_goal_state.setGoal(state);
    int heur_num = add2DHeur(cost_multiplier, 0);
    m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
    m_heuristics[heur_num]->setGoal(new_goal_state);
    m_mha_heur_ids.push_back(heur_num);
    // Add the second one
    state.x(selected_points.second.first);
    state.y(selected_points.second.second);
    ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    selected_points.second.first, selected_points.second.second);
    GoalState another_goal_state(m_goal);
    another_goal_state.setGoal(state);
    heur_num = add2DHeur(cost_multiplier, 0);
    m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
    m_heuristics[heur_num]->setGoal(another_goal_state);
    m_mha_heur_ids.push_back(heur_num);


    // -----------------Uniform picking----------------
    // std::vector<double> angles_with_center(circle_x.size(), 0);
    // for (size_t i = 0; i < angles_with_center.size(); ++i)
    // {
    //     // angles_with_center[i] = (circle_x[i] == center_x)?((circle_y[i] < center_y)?normalize_angle_positive(-M_PI/2):normalize_angle_positive(M_PI/2)):normalize_angle_positive(static_cast<double>(circle_y[i] -
    //     //     center_y)/(circle_x[i] - center_x));
    //     angles_with_center[i] = normalize_angle(std::atan2(static_cast<double>(circle_y[i] -
    //         center_y),static_cast<double>(circle_x[i]
    //                 - center_x)));
    //     ROS_DEBUG_NAMED(HEUR_LOG, "Angle with center %d:: %d %d : %f",
    //         static_cast<int>(i), circle_x[i], circle_y[i],
    //         angles_with_center[i]);
    // }
    // double max_angle = *std::max_element(angles_with_center.begin(),
    //     angles_with_center.end());
    // double min_angle = *std::min_element(angles_with_center.begin(),
    //     angles_with_center.end());
    // double increment = (max_angle - min_angle)/(m_num_mha_heuristics+1);
    // ROS_DEBUG_NAMED(HEUR_LOG,"Min angle: %f, Max angle: %f, increment : %f",
    //     min_angle, max_angle, increment);
    // double current_angle = min_angle;
    // int num_selected = 0;
    // int direction = 1;
    // while(num_selected < m_num_mha_heuristics) {
    //     current_angle += direction*increment;
    //     ROS_DEBUG_NAMED(HEUR_LOG, "current_angle : %f", current_angle);
    //     int idx = 0;
    //     double min_difference = std::fabs(min_angle - max_angle);
    //     for (size_t i = 0; i < circle_x.size(); ++i)
    //     {
    //         if(std::fabs(current_angle - angles_with_center[i]) <= min_difference){
    //             min_difference = std::fabs(current_angle - angles_with_center[i]);
    //             idx = i;
    //         }
    //     }
    //     if(idx == 0 || idx == circle_x.size() -1){
    //         // Go the other way
    //         direction = -1;
    //         current_angle += direction*increment;
    //         continue;
    //     }

    //     ROS_DEBUG_NAMED(HEUR_LOG,"Selected point: %d :: %d %d",
    //         static_cast<int>(idx),
    //         circle_x[idx], circle_y[idx]);
    //     state.x(circle_x[idx]);
    //     state.y(circle_y[idx]);
    //     ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    //     circle_x[idx], circle_y[idx]);
    //     GoalState new_goal_state(m_goal);
    //     new_goal_state.setGoal(state);
    //     int heur_num = add2DHeur(cost_multiplier, 0);
    //     m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
    //     m_heuristics[heur_num]->setGoal(new_goal_state);
    //     num_selected++;
    // }
    
    // Random selection
    // int num_selected = 0;
    // while(num_selected < m_num_mha_heuristics) {
    //     int p = rand()%number_of_points;
    //     if(selected[p])
    //         continue;
    //     else{
    //         selected[p] = 1;
    //         ++num_selected;
    //         state.x(circle_x[p]);
    //         state.y(circle_y[p]);
    //         ROS_DEBUG_NAMED(HEUR_LOG, "[MHAHeur] Selected point: %d %d",
    //             circle_x[p], circle_y[p]);
    //         GoalState new_goal_state(m_goal);
    //         new_goal_state.setGoal(state);
    //         int heur_num = add2DHeur(cost_multiplier, 0);
    //         m_heuristics[heur_num]->update2DHeuristicMap(m_grid_data);
    //         m_heuristics[heur_num]->setGoal(new_goal_state);

    //         // Temporary, to make sure points are selected on opposing sides.
    //         // int side = (circle_x[p] - m_goal.getObjectState().x() < 0)?-1:1;
    //         // for (size_t i = 0; i < circle_x.size();)
    //         // {
    //         //     if((circle_x[i] - m_goal.getObjectState().x())*side > 0){
    //         //         // erase
    //         //         ROS_DEBUG_NAMED(HEUR_LOG, "Removing %d %d", circle_x[i],
    //         //             circle_y[i]);
    //         //         circle_x.erase(circle_x.begin() + i);
    //         //         circle_y.erase(circle_y.begin() + i);
    //         //         selected.erase(selected.begin() + i);
    //         //     }
    //         //     else
    //         //         ++i;
    //         // }
    //         // number_of_points = circle_x.size();
    //     }
    // }
    // GoalState new_goal_state = boost::make_shared<GoalState>(goal_state);
}
