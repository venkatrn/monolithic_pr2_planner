#pragma once
#include <boost/shared_ptr.hpp>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/TransitionData.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <Eigen/Core>
#include <memory>

namespace monolithic_pr2_planner {
    typedef boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> SBPLArmModelPtr;
    /*! \brief Does collision checking on MotionPrimitive, GraphState,
     * RobotState, and TransitionData types.
     */
    class CollisionSpaceMgr : public OccupancyGridUser {
        public:
            CollisionSpaceMgr(SBPLArmModelPtr right_arm,
                              SBPLArmModelPtr left_arm);
            bool isValid(DiscObjectState& obj_state);
            bool isValid(ContBaseState& base, RightContArmState& r_arm, LeftContArmState& l_arm);
            bool isValid(RobotState& robot_pose);
            bool isValidSuccessor(const GraphState& successor,
                                  const TransitionData& t_data);
            bool isValidTransitionStates(const TransitionData& t_data);
            bool isValidContState(std::vector<double>& l_arm,std::vector<double>&
                r_arm, const std::vector<double> body);

            void updateMap(const arm_navigation_msgs::CollisionMap& map);
            bool loadMap(const std::vector<Eigen::Vector3d>& points);

            // Attached object interface.
            void visualizeAttachedObject(RobotState& robot_state, int hue = 100);
            void attachCube(std::string name, std::string reference_frame,
                geometry_msgs::Pose relative_pose, double dim_x, double dim_y,
                double dim_z);


        private:
            boost::shared_ptr<pr2_collision_checker::PR2CollisionSpace> m_cspace;
    };
    typedef boost::shared_ptr<CollisionSpaceMgr> CSpaceMgrPtr;
}
