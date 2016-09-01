#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
  class ArmSnapMotionPrimitive : public MotionPrimitive {
    public:
      ArmSnapMotionPrimitive()
      {
       //Quickly hardcoded. Should be read from SearchReqParam

     }
      virtual bool apply(const GraphState& graph_state,
          GraphStatePtr& successor,
          TransitionData& t_data);
      virtual void print() const;
      virtual int motion_type() const { return MPrim_Types::ARM_ADAPTIVE; };
      virtual void computeCost(const MotionPrimitiveParams& params);
      bool computeIntermSteps(const GraphState& source_state,
                        const GraphState& successor,
                        TransitionData& t_data);
      // void getUpdatedGoalandTolerances(GoalStatePtr& goal,double xyz_tol, double roll_tol, double pitch_tol, double yaw_tol)
      // {
      //    m_goal = goal;
      //    m_tolerances[Tolerances::XYZ] =  xyz_tol;
      //    m_tolerances[Tolerances::ROLL] =  roll_tol;
      //    m_tolerances[Tolerances::PITCH] =  pitch_tol;
      //    m_tolerances[Tolerances::YAW] =  yaw_tol;
      // }
      static void goal(GoalState& goal) { m_goal = goal; };
      static void tolerances(double xyz_tol, double roll_tol, double pitch_tol, double yaw_tol) {
         m_tolerances.resize(4, 0.0);
         m_tolerances[Tolerances::XYZ] =  xyz_tol;
         m_tolerances[Tolerances::ROLL] =  roll_tol;
         m_tolerances[Tolerances::PITCH] =  pitch_tol;
         m_tolerances[Tolerances::YAW] =  yaw_tol;
      }

      static GoalState goal() { return m_goal; };

      static GoalState m_goal;
      static std::vector<double> m_tolerances;
      RobotState m_activationCenter;
      RobotState m_activationRadius;
  };
  typedef boost::shared_ptr<ArmSnapMotionPrimitive> ArmSnapMotionPrimitivePtr;

}
