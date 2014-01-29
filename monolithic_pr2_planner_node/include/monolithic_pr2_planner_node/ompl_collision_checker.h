#pragma once
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>

class omplFullBodyCollisionChecker : public ompl::base::StateValidityChecker {
  public:
    omplFullBodyCollisionChecker(const ompl::base::SpaceInformationPtr &si) : 
        ompl::base::StateValidityChecker(si){}
    monolithic_pr2_planner::CSpaceMgrPtr m_cspace;

    void initialize(monolithic_pr2_planner::CSpaceMgrPtr cspace, std::vector<double> l_arm);
    //void readFile(char filename[], std::vector<std::pair<State, State> >& pairs);
    void initializeRegions(std::string file);
    //bool generateRandomValidState(State& s, vector<double>& arm_right, vector<double>& arm_left, int idx, int region_id=0);
    //void generateRandomState(State& s, int region_id);
    inline double randomDouble(double min, double max);
    virtual bool isValid(const ompl::base::State *state) const;
    std::vector<double> l_arm_init;

};
