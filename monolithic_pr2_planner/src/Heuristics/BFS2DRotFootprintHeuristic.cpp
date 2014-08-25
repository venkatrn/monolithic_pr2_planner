#include <monolithic_pr2_planner/Heuristics/BFS2DRotFootprintHeuristic.h>
#include <costmap_2d/cost_values.h>

using namespace monolithic_pr2_planner;

BFS2DRotFootprintHeuristic::BFS2DRotFootprintHeuristic(){
  cache_ = new unsigned char*[m_size_col];
  for (unsigned int i=0; i < m_size_col; i++){
    cache_[i] = new unsigned char[m_size_row];
    for (unsigned int j=0; j < m_size_row; j++){
      cache_[i][j] = 0;
    }
  }

}

BFS2DRotFootprintHeuristic::~BFS2DRotFootprintHeuristic(){
  for (unsigned int i=0; i < m_size_col; i++){
    delete cache_[i];
  }
  delete cache_;
}

// This must be called after setting original goal.
void BFS2DRotFootprintHeuristic::setGoal(GoalState& goal_state) {
  BFS2DHeuristic::setGoal(goal_state);
  for (unsigned int i=0; i < m_size_col; i++)
    for (unsigned int j=0; j < m_size_row; j++)
      cache_[i][j] = 0;
}

void BFS2DRotFootprintHeuristic::setFootprint(const vector<sbpl_2Dpt_t>& footprintPolygon, double rotateFootprint){
  sbpl_xy_theta_pt_t pt;
  pt.x = 0;
  pt.y = 0;
  pt.theta = rotateFootprint;
  get_2d_footprint_cells(footprintPolygon, &footprint_, pt, m_occupancy_grid->getResolution());
  theta_ = rotateFootprint;
}

int BFS2DRotFootprintHeuristic::getGoalHeuristic(GraphStatePtr state){
  int base_x = state->base_x();
  int base_y = state->base_y();

  //to avoid checking the footprint, use our cache if we already did the check for this cell
  
  //cache==0 means we haven't evaluated this cell yet, so check the footprint
  if(cache_[base_x][base_y] == 0){
    //check footprint for collision
    for(unsigned int i=0; i<footprint_.size(); i++){
      int x = base_x + footprint_[i].x;
      int y = base_y + footprint_[i].y;
      if(x < 0 || x >= int(m_size_col) ||
         y < 0 || y >= int(m_size_row) ||
         m_grid[x][y] >= costmap_2d::LETHAL_OBSTACLE){
        cache_[base_x][base_y] = 1; //cache a collision for this cell
        return INFINITECOST;
      }
    }
    cache_[base_x][base_y] = 2; //cache collision free for this cell
  }

  //cache==1 means we evaluated this cell and found a collision
  if(cache_[base_x][base_y] == 1)
    return INFINITECOST;

  //cache==2 means we evaluated this cell and found no collision
  if(cache_[base_x][base_y] == 2){
    int transCost = BFS2DHeuristic::getGoalHeuristic(state);
    int rotCost = 500*std::fabs(shortest_angular_distance(state->robot_pose().getContBaseState().theta(), theta_));
    if(transCost == 0)//if we are already near the goal, this heuristic is done
      return 0;

    //translation has a max gradient of about 28
    //rotation has a max gradient of about 196

    static bool print = true;
    if(print){
      print = false;
      vector<double> v = state->getContCoords();
      v[GraphStateElement::BASE_X] += 0.02;
      GraphStatePtr tmp = boost::make_shared<GraphState>(v);
      ROS_ERROR("transCost=%d transCost+1dx=%d rotCost=%d robot_theta=%f theta_=%f cost=%d",
          transCost, BFS2DHeuristic::getGoalHeuristic(tmp), rotCost, state->robot_pose().getContBaseState().theta(), theta_,
          getCostMultiplier() * (transCost + rotCost));
      std::cin.get();
    }

    return getCostMultiplier() * (transCost + rotCost);
  }
  
  ROS_ERROR("cache is in an invalid state!");
  return -1;
}

void BFS2DRotFootprintHeuristic::draw(){
  int num_valid = 0;
  int num_invalid = 0;

  FILE* fout = fopen("grid.csv","w");
  for (unsigned int i=0; i < m_size_col; i++){
    for (unsigned int j=0; j < m_size_row; j++){
      fprintf(fout,"%d ",m_grid[i][j]);
    }
    fprintf(fout,"\n");
  }
  fclose(fout);

  fout = fopen("ft_mask.csv","w");
  for (unsigned int i=0; i < m_size_col; i++){
    for (unsigned int j=0; j < m_size_row; j++){
      int base_x = i;
      int base_y = j;
      bool valid = true;
      for(unsigned int i=0; i<footprint_.size(); i++){
        int x = base_x + footprint_[i].x;
        int y = base_y + footprint_[i].y;
        if(x < 0 || x >= int(m_size_col) ||
            y < 0 || y >= int(m_size_row) ||
            m_grid[x][y] >= costmap_2d::LETHAL_OBSTACLE){
          valid = false;
          break;
        }
      }
      if(valid){
        num_valid++;
        fprintf(fout,"1 ");
      }
      else{
        num_invalid++;
        fprintf(fout,"0 ");
      }
    }
    fprintf(fout,"\n");
  }
  fclose(fout);

  fout = fopen("bfs.csv","w");
  for (unsigned int i=0; i < m_size_col; i++){
    for (unsigned int j=0; j < m_size_row; j++){
      int cost = m_gridsearch->getlowerboundoncostfromstart_inmm(i, j);
      fprintf(fout,"%d ",cost);
    }
    fprintf(fout,"\n");
  }
  fclose(fout);

  ROS_ERROR("%d cells in footprint",footprint_.size());
  ROS_ERROR("%d cells in map",m_size_col*m_size_row);
  ROS_ERROR("%d cells found valid",num_valid);
  ROS_ERROR("%d cells found invalid",num_invalid);
}

