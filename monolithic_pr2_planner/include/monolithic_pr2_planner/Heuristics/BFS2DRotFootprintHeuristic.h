#pragma once
#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>

namespace monolithic_pr2_planner {

class BFS2DRotFootprintHeuristic : public BFS2DHeuristic {
  public:
    BFS2DRotFootprintHeuristic();
    ~BFS2DRotFootprintHeuristic();
    void setGoal(GoalState& goal_state);
    int getGoalHeuristic(GraphStatePtr state);
    void setFootprint(const vector<sbpl_2Dpt_t>& footprintPolygon, double rotateFootprint);

    void draw();

  private:
    unsigned char** cache_;
    std::vector<sbpl_2Dcell_t> footprint_;
    double theta_;
};
typedef boost::shared_ptr<BFS2DRotFootprintHeuristic> BFS2DRotFootprintHeuristicPtr;

}

