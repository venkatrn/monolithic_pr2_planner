#include <monolithic_pr2_planner/Heuristics/DistanceTransform2D.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/cost_values.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <boost/algorithm/clamp.hpp>

using namespace monolithic_pr2_planner;
using boost::algorithm::clamp;

const double kCircumscribedRadius = 0.8;

DistanceTransform2D::DistanceTransform2D() {
  int dimX, dimY, dimZ;
  m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
  m_size_col = static_cast<unsigned int>(dimX + 1);
  m_size_row = static_cast<unsigned int>(dimY + 1);
  ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] initialized BFS2D of size %d %d",
                  m_size_col, m_size_row);

  m_gridsearch.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
                                          m_occupancy_grid->getResolution()));
  m_gridsearch_prob.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
                                          m_occupancy_grid->getResolution()));

  // Initialize the grid here itself so that you don't wait for the map
  // callback to be called
  m_grid = new unsigned char *[m_size_col];

  for (unsigned int i = 0; i < m_size_col; i++) {
    m_grid[i] = new unsigned char[m_size_row];

    for (unsigned int j = 0; j < m_size_row; j++) {
      m_grid[i][j] = 0;
    }
  }

  // m_circlepub = m_nh.advertise<geometry_msgs::PolygonStamped>("/goal_circle", 1,
  // true);
}

DistanceTransform2D::~DistanceTransform2D() {
  for (unsigned int i = 0; i < m_size_col; i++) {
    delete[] m_grid[i];
  }

  delete[] m_grid;
  m_grid = NULL;
}

void DistanceTransform2D::update2DHeuristicMap(const
                                               std::vector<unsigned char> &data) {
  loadMap(data);
}

void DistanceTransform2D::loadMap(const std::vector<unsigned char> &data) {

  m_obstacle_coordinates.clear();
  for (unsigned int j = 0; j < m_size_row; j++) {
    for (unsigned int i = 0; i < m_size_col; i++) {
      m_grid[i][j] = (data[j * m_size_col + i]);

      if (m_grid[i][j] == costmap_2d::LETHAL_OBSTACLE
          || (i == 0)
          || (j == 0)
          || (i == (m_size_col - 1))
          || (j == (m_size_row - 1))) {
        // Skip the "goal" state for the grid search.
        if (i == 0 && j == 0) {
          continue;
        }
      // if (m_grid[i][j] == costmap_2d::LETHAL_OBSTACLE) {
        // World (x,y) corresponds to Grid (col, row), and we want the
        // obstacle coordinates to be world coordinates.
        m_obstacle_coordinates.push_back(std::make_pair(i, j));
      }

      // Set the actual cell costs to 0, since we don't care about
      // traversal costs.
      m_grid[i][j] = 0;
    }
  }

  ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] updated grid of size %d %d from the map",
                  m_size_col, m_size_row);

  // m_gridsearch->search(m_grid, costmap_2d::LETHAL_OBSTACLE, m_size_col - 1, m_size_row - 1,
  //     0,0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS, m_obstacle_coordinates);
  m_gridsearch->search(m_grid, costmap_2d::LETHAL_OBSTACLE,
                       m_obstacle_coordinates[0].first, m_obstacle_coordinates[0].second,
                       0, 0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS, m_obstacle_coordinates);
  ROS_INFO("Finished computing distance transform for %d cells", static_cast<int>(m_obstacle_coordinates.size()));
  cv::Mat dist_transform;
  dist_transform.create(m_size_col, m_size_row, CV_64FC1);

  for (size_t ii = 0; ii < m_size_col; ++ii) {
    for (size_t jj = 0; jj < m_size_row; ++jj) {
      int value = m_gridsearch->getlowerboundoncostfromstart_inmm(ii, jj);

      if (value < INFINITECOST) {
        dist_transform.at<double>(ii,
                                  jj) = std::min(static_cast<double>(value) * 0.001, kCircumscribedRadius);
      } else {
        dist_transform.at<double>(ii, jj) = kCircumscribedRadius;
      }
    }
  }

  // dist_transform.setTo(1.0, dist_transform > kCircumscribedRadius);
  cv::Mat colored_mat;
  cv::normalize(1.0 - dist_transform, colored_mat, 0, 255, cv::NORM_MINMAX,
                CV_8UC1);
  cv::applyColorMap(colored_mat, colored_mat, cv::COLORMAP_JET);
  cv::imwrite("/tmp/dist_transform.png", colored_mat);
  ROS_INFO("Printed distance transform");
}

void DistanceTransform2D::ComputeProbabilityHeuristic(double prob_multiplier) {

  DiscObjectState state = m_goal.getObjectState();

  // Get the initial points for the dijkstra search
  std::vector<std::pair<int,int> > init_points;
  double res = m_occupancy_grid->getResolution();
  int discrete_radius = m_radius/res;
  getBresenhamCirclePoints(state.x(), state.y(), discrete_radius, init_points);

  const unsigned char kLethalObstacle = std::numeric_limits<unsigned char>::max();

  for (unsigned int j = 0; j < m_size_row; j++) {
    for (unsigned int i = 0; i < m_size_col; i++) {

      int dist_transform_value = m_gridsearch->getlowerboundoncostfromstart_inmm(i, j);

      if (dist_transform_value < static_cast<int>(0.34 * 1000.0)) {
        m_grid[i][j] = kLethalObstacle;
      } else if (!(state.x() == i && state.y() == j)) {

        const double probability = std::min(1.0,
                               (static_cast<double>(dist_transform_value) * 0.001) / kCircumscribedRadius);
        m_grid[i][j] = static_cast<unsigned char>(clamp(-prob_multiplier * log(probability), 0.0, kLethalObstacle - 1));
        // cout << "Prob: " << static_cast<int>(m_grid[i][j]) << endl;
        // m_grid[i][j] = static_cast<unsigned char>(prob_multiplier * (1.0 - probability));
      }
    }
  }

  ROS_INFO("Number of obstacle cells: %d", m_obstacle_coordinates.size());

  ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] updated grid of size %d %d from the map",
                  m_size_col, m_size_row);

  // m_gridsearch->search(m_grid, costmap_2d::LETHAL_OBSTACLE, m_size_col - 1, m_size_row - 1,
  //     0,0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS, m_obstacle_coordinates);
  m_gridsearch_prob->search(m_grid, kLethalObstacle,
                       state.x(), state.y(),
                       0, 0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS, init_points);
  ROS_INFO("Finished computing distance transform");
  cv::Mat prob_heuristic;
  prob_heuristic.create(m_size_col, m_size_row, CV_64FC1);

  for (size_t ii = 0; ii < m_size_col; ++ii) {
    for (size_t jj = 0; jj < m_size_row; ++jj) {
      int value = m_gridsearch_prob->getlowerboundoncostfromstart_inmm(ii, jj);

      if (value < INFINITECOST) {
        prob_heuristic.at<double>(ii,
                                  jj) = value;
      } else {
        prob_heuristic.at<double>(ii, jj) = 0;
      }
    }
  }

  // prob_heuristic.setTo(1.0, prob_heuristic > kCircumscribedRadius);
  cv::Mat colored_mat;
  cv::normalize(prob_heuristic, colored_mat, 0, 255, cv::NORM_MINMAX,
                CV_8UC1);
  cv::applyColorMap(colored_mat, colored_mat, cv::COLORMAP_JET);
  std::string name = "/tmp/prob_heuristic" + std::to_string(prob_multiplier) + ".png";
  cv::imwrite(name.c_str(), colored_mat);
  ROS_INFO("Printed prob heuristic");

}

void DistanceTransform2D::setUniformCostSearch(bool ucs) {
  m_gridsearch->setUniformCostSearch(ucs);
}

void DistanceTransform2D::setGoal(GoalState &goal_state) {
  // Save the goal for future use.
  m_goal = goal_state;


  ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Setting goal %d %d", m_goal.getObjectState().x(), m_goal.getObjectState().y());
  // ComputeProbabilityHeuristic(1000.0);
  // ComputeProbabilityHeuristic(255.0);
  // ComputeProbabilityHeuristic(100.0);
  ComputeProbabilityHeuristic(20.0);
  // ComputeProbabilityHeuristic(10.0);
  // ComputeProbabilityHeuristic(5.0);
  // ComputeProbabilityHeuristic(2.0);
  // ComputeProbabilityHeuristic(1.0);
  // ComputeProbabilityHeuristic(0.0);
}

int DistanceTransform2D::getGoalHeuristic(GraphStatePtr state) {
  DiscObjectState obj_state = state->getObjectStateRelMapFromState();

  // Check if within bounds. We need to do this here because the bfs2d
  // implementation doesn't take care of this.
  if (state->base_x() < 0 || state->base_x() >= int(m_size_col) ||
      state->base_y() < 0 || state->base_y() >= int(m_size_row)) {
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Out of bounds: %d %d",
                    state->base_x(), state->base_y());
    return INFINITECOST;
  }

  int cost_base = m_gridsearch_prob->getlowerboundoncostfromstart_inmm(state->base_x(),
                                                             state->base_y());
  int cost_endeff = m_gridsearch_prob->getlowerboundoncostfromstart_inmm(obj_state.x(),
                                                             obj_state.y());
  if (cost_base < 0 || cost_endeff < 0) {
    return INFINITECOST;
  }

  return std::max(cost_base, cost_endeff);
  // return static_cast<int>(0.45 * cost_base + 0.05 * cost_endeff);
}

double DistanceTransform2D::getIncomingEdgeProbability(GraphStatePtr state)
const {
  DiscObjectState obj_state = state->getObjectStateRelMapFromState();

  // Check if within bounds. We need to do this here because the bfs2d
  // implementation doesn't take care of this.
  if (state->base_x() < 0 || state->base_x() >= int(m_size_col) ||
      state->base_y() < 0 || state->base_y() >= int(m_size_row)) {
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Out of bounds: %d %d",
                    state->base_x(), state->base_y());
    return 0.0;
  }

  if (obj_state.x() < 0 || obj_state.x() >= int(m_size_col) ||
      obj_state.y() < 0 || obj_state.y() >= int(m_size_row)) {
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Out of bounds: %d %d",
                    obj_state.x(), obj_state.y());
    return 0.0;
  }

  int cost_base = m_gridsearch->getlowerboundoncostfromstart_inmm(state->base_x(),
                                                             state->base_y());
  int cost_endeff = m_gridsearch->getlowerboundoncostfromstart_inmm(obj_state.x(),
                                                             obj_state.y());

  // ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] 2Ddijkstra's cost to %d %d is %d",
  //                           state->base_x(), state->base_y(),
  //                           (cost)<0?INFINITECOST:cost );
  if (cost_base < 0) {
    return 0.0;
  }


  // if (cost < m_costmap_ros->getInscribedRadius()/0.02)
  // return 0;
  // In meters.
  // const double kCircumscribedRaidus = m_costmap_ros->getInscribedRadius() + 1.0;
  // ROS_INFO("Cost : %d", cost);
  const double prob_base = std::min(1.0,
                               (static_cast<double>(cost_base) * 0.001) / kCircumscribedRadius);
  const double prob_endeff = std::min(1.0,
                               (static_cast<double>(cost_endeff) * 0.001) / kCircumscribedRadius);
  const double prob = std::min(prob_base, prob_endeff);
  return prob;
}

void DistanceTransform2D::visualizeCenter(int x, int y) {
  return; //MIKE: disabled this visualization because it makes my rviz cry
  ROS_DEBUG_NAMED(HEUR_LOG, "[DistanceTransform2D] Visualizing %d %d,", x, y);
  vector <vector <double>> startpoint;
  std::vector <double> color(4, 1);
  color[1] = color[2] = 0;
  vector<double> point;
  point.push_back(x * m_occupancy_grid->getResolution());
  point.push_back(y * m_occupancy_grid->getResolution());
  point.push_back(0.0);
  startpoint.push_back(point);
  std::stringstream ss;
  ss << "start_point" << x << "_" << y;
  Visualizer::pviz->visualizeBasicStates(startpoint, color, ss.str(), 0.02);
}

void DistanceTransform2D::setRadiusAroundGoal(double radius_m) {
  m_radius = radius_m;
  m_gridsearch_prob.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
                                          m_occupancy_grid->getResolution(), radius_m));
}

void DistanceTransform2D::visualizeRadiusAroundGoal(int x0, int y0) {
  if (!m_radius) {
    return;
  }

  std::vector<int> circle_x;
  std::vector<int> circle_y;
  double res = m_occupancy_grid->getResolution();
  int discrete_radius = m_radius / res;
  getBresenhamCirclePoints(x0, y0, discrete_radius, circle_x, circle_y);

  // geometry_msgs::PolygonStamped circle;

  // circle.header.frame_id = "/map";
  // circle.header.stamp = ros::Time::now();
  std::vector<geometry_msgs::Point> circle_points;

  for (size_t i = 0; i < circle_x.size(); ++i) {
    // Prune the points to display only the ones that are within the
    // threshold
    if (m_grid[circle_x[i]][circle_y[i]] <
        costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      geometry_msgs::Point out_pt;
      out_pt.x = circle_x[i] * res;
      out_pt.y = circle_y[i] * res;
      out_pt.z = 0.0;
      circle_points.push_back(out_pt);
    }
  }

  std::stringstream ss;
  ss << "radius_around_goal";
  Visualizer::pviz->visualizeLine(
    circle_points, ss.str(), x0 + y0, 114, 0.01);
}

// ------------- Bresenham circle points -------------------//
void DistanceTransform2D::getBresenhamCirclePoints(int x0, int y0, int radius,
                                                   std::vector<int> &
                                                   ret_x,
                                                   std::vector<int> &ret_y) {
  int x = 0;
  int y = radius;
  int delta = 2 - 2 * radius;
  int err = 0;
  ret_x.clear();
  ret_y.clear();

  while (y >= 0) {
    ret_x.push_back(x0 + x);
    ret_x.push_back(x0 - x);
    ret_x.push_back(x0 + x);
    ret_x.push_back(x0 - x);
    ret_y.push_back(y0 - y);
    ret_y.push_back(y0 - y);
    ret_y.push_back(y0 + y);
    ret_y.push_back(y0 + y);
    err = 2 * (delta + y) - 1;

    if (delta < 0 && err <= 0) {
      x = x + 1;
      delta = delta + 2 * x + 1;
      continue;
    }

    err = 2 * (delta - x) - 1;

    if (delta > 0 && err > 0) {
      y = y - 1;
      delta = delta + 1 - 2 * y;
      continue;
    }

    x = x + 1;
    delta = delta + 2 * (x - y);
    y = y - 1;
  }
}

void DistanceTransform2D::getBresenhamCirclePoints(int x0, int y0, int radius,
                                                   std::vector<std::pair<int, int>> &
                                                   points) {
  points.clear();
  std::vector<int> ret_x;
  std::vector<int> ret_y;
  getBresenhamCirclePoints(x0, y0, radius, ret_x, ret_y);

  for (size_t i = 0; i < ret_x.size(); ++i) {
    points.push_back(std::make_pair(ret_x[i], ret_y[i]));
  }
}

// ---------------- Bresenham Line points --------------------

void DistanceTransform2D::getBresenhamLinePoints(int x1, int y1, int x2,
                                                 int y2, std::vector<std::pair<int, int>> &
                                                 points) {
  std::vector<int> pts_x;
  std::vector<int> pts_y;
  getBresenhamLinePoints(x1, y1, x2, y2, pts_x, pts_y);

  for (size_t i = 0; i < pts_x.size(); ++i) {
    points.push_back(std::make_pair(pts_x[i], pts_y[i]));
  }
}


void DistanceTransform2D::getBresenhamLinePoints(int x1, int y1, int x2,
                                                 int y2, std::vector<int> &pts_x, std::vector<int> &
                                                 pts_y) {
  pts_x.clear();
  pts_y.clear();
  bool steep = (std::abs(y2 - y1) > std::abs(x2 - x1));

  if (steep) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  int dx = x2 - x1;
  int dy = std::abs(y2 - y1);

  double err = dx / 2.0f;

  int ystep = (y1 < y2) ? 1 : -1;
  int y = (y1);

  for (int x = x1; x <= x2; ++x) {
    if (steep) {
      // y,x
      pts_x.push_back(y);
      pts_y.push_back(x);
    } else {
      // x, y
      pts_x.push_back(x);
      pts_y.push_back(y);
    }

    err = err - dy;

    if (err < 0) {
      y = y + ystep;
      err = err +  dx;
    }
  }

  assert(pts_y.size() == pts_x.size());
}