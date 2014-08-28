#pragma once
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <ikfast_pr2/ik_interface.h>
#include <pviz/pviz.h>
#include <geometry_msgs/Pose.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class RobotState {
        public:
            bool operator==(const RobotState& other);
            bool operator!=(const RobotState& other);

            RobotState(){};
            RobotState(ContBaseState base_state, RightContArmState r_arm, LeftContArmState l_arm);
            RobotState(ContBaseState base_state, ContObjectState object_state);
            DiscBaseState base_state() const;
            ContBaseState getContBaseState() const { return m_base_state; };
            RightContArmState right_arm() const { return m_right_arm; };
            LeftContArmState left_arm() const { return m_left_arm; };

            unsigned int left_free_angle() const { return m_left_arm.getDiscFreeAngle(); };
            unsigned int right_free_angle() const { return m_right_arm.getDiscFreeAngle(); };
            void left_free_angle(int value) { m_left_arm.setDiscFreeAngle(value); };
            void right_free_angle(int value) { m_right_arm.setDiscFreeAngle(value); };
            void base_state(const DiscBaseState& base_state);
            void right_arm(const RightContArmState& arm) { m_right_arm = arm; };
            void left_arm(const LeftContArmState& arm){ m_left_arm = arm; };

            void printToDebug(char* log_level) const;
            void printToInfo(char* log_level) const;
            void printToFile(FILE *& path) const;

            // this sets which arm to use for computing the object pose
            static void setPlanningMode(int planning_mode){ m_planning_mode = planning_mode; };
            void visualize(int hue = 240);

            ContObjectState getObjectStateRelMap() const;
            ContObjectState getObjectStateRelMap(ContBaseState base) const;
            DiscObjectState getObjectStateRelBody() const;

            static bool computeRobotPose(const DiscObjectState& disc_obj_state,
                                         const RobotState& robot_pose,
                                         boost::shared_ptr<RobotState>& new_robot_pose,
                                         bool free_angle_search=false);
            static bool workspaceInterpolate(const RobotState& start, const RobotState& end,
                                             std::vector<RobotState>* interp_steps);
            static bool jointSpaceInterpolate(const RobotState& start,
                    const RobotState& end, std::vector<RobotState>* interp_steps);
            static int numInterpSteps(const RobotState& start, const RobotState& end);

            inline double randomDouble(double min, double max){
                return min + (max-min) * ( double(rand()) / RAND_MAX );
            }

        private:
            static IKFastPR2 m_ikfast_solver;
            static int ik_calls;
            static int ik_time;
            static int m_planning_mode;
            ContBaseState m_base_state;
            RightContArmState m_right_arm;
            LeftContArmState m_left_arm;
            DiscObjectState m_obj_state; // this is in BODY frame!
    };
    typedef boost::shared_ptr<RobotState> RobotPosePtr;
}
