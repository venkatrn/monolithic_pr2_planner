#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <memory>
#include <kdl/frames.hpp>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    /*! \brief The Abstract Heuristic class that has to be inherited
     * for use in the environment.
     */
    class AbstractHeuristic{
        public:
            AbstractHeuristic() : m_cost_multiplier(1) {};

            // The function that has to return the heuristic value at the queried graph state
            virtual int getGoalHeuristic(GraphStatePtr state) = 0;
            virtual void setGoal(GoalState& state) = 0;

            // For 3D heuristics that need the obstacles - this function has to be implemented for the heuristic to receive the call for obstacle grid update.
            virtual void update3DHeuristicMap() {};

            // For 2D heuristics at the base that need only the map - this function has to be implemented for the heuristic to receive the call for map update.
            virtual void update2DHeuristicMap(const std::vector<unsigned char>& data) {};

            // Set the cost_multiplier
            virtual inline void setCostMultiplier(const int cost_multiplier) { m_cost_multiplier
                = cost_multiplier; };

            // Get the cost multiplier
            virtual inline int getCostMultiplier(){ return m_cost_multiplier; };

            // Add all the virtual stuff that may or may not be implemented
            // For the 2DHeuristic
            virtual void setRadiusAroundGoal(double radius_m) {};
            virtual double getRadiusAroundGoal() {return 0;};
            // For MHABaseHeur
            virtual void setDesiredOrientation(KDL::Rotation rot) {};
            // For the ArmAnglesHeur
            virtual void setGoalArmState(RightContArmState& soln_r_arm_state) {};
        private:
            int m_cost_multiplier;
    };
    typedef boost::shared_ptr<AbstractHeuristic> AbstractHeuristicPtr;
}
