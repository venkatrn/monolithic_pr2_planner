#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <ikfast_pr2/ik_interface.h>
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    typedef boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> SBPLArmModelPtr;

    /*! \brief Represents a state of the left/right arm PR2.
     *
     * The PR2 arm has associated with it seven joints, an object offset, and
     * inverse and forward kinematic functions. This class provides the
     * necessary functions to retrieve and set continuous joint information about
     * the arms.
     */
    class ContArmState {
        public:
            ContArmState();
            ContArmState(std::vector<double> arm_state);

            bool operator==(const ContArmState& other);
            bool operator!=(const ContArmState& other);

            static void setRobotResolutionParams(const RobotResolutionParams& params);

            void enableJointLimitChecking();

            inline double getShoulderPanAngle() const{return m_angles[Joints::SHOULDER_PAN]; };
            inline double getShoulderLiftAngle() const{return m_angles[Joints::SHOULDER_LIFT];};
            inline double getUpperArmRollAngle() const {return m_angles[Joints::UPPER_ARM_ROLL];};
            inline double getElbowFlexAngle() const { return m_angles[Joints::ELBOW_FLEX]; };
            inline double getForearmRollAngle() const { return m_angles[Joints::FOREARM_ROLL]; };
            inline double getWristFlexAngle() const { return m_angles[Joints::WRIST_FLEX]; };
            inline double getWristRollAngle() const { return m_angles[Joints::WRIST_ROLL]; };
            int getDiscFreeAngle() const;
            inline void setShoulderPan(double cont_value) { m_angles[Joints::SHOULDER_PAN] = normalize_angle_positive(cont_value); };
            inline void setShoulderLift(double cont_value){ m_angles[Joints::SHOULDER_LIFT] = normalize_angle_positive(cont_value); };
            inline void setUpperArmRoll(double cont_value){ m_angles[Joints::UPPER_ARM_ROLL] = normalize_angle_positive(cont_value); };
            inline void setElbowFlex(double cont_value){ m_angles[Joints::ELBOW_FLEX] = normalize_angle_positive(cont_value); };
            inline void setForearmRoll(double cont_value){ m_angles[Joints::FOREARM_ROLL] = normalize_angle_positive(cont_value); };
            inline void setWristFlex(double cont_value){ m_angles[Joints::WRIST_FLEX] = normalize_angle_positive(cont_value); };
            inline void setWristRoll(double cont_value){ m_angles[Joints::WRIST_ROLL] = normalize_angle_positive(cont_value); };
            void setArm(ArmSide arm);
            void setDiscFreeAngle(int value) { setUpperArmRoll(convertDiscFreeAngleToCont(value)); };

            inline double convertDiscFreeAngleToCont(int disc_angle) const {
                double free_angle_res = m_params.arm_free_angle_resolution;
                return normalize_angle_positive(double(disc_angle)*free_angle_res);
            };

            void getAngles(std::vector<double>* angles) const; 
            std::vector<double>::const_iterator getAnglesBegin() const{return m_angles.begin();};
            std::vector<double>::const_iterator getAnglesEnd() const { return m_angles.end(); };

            virtual int getArm() const = 0;

            virtual KDL::Frame getObjectOffset() const = 0;
            virtual void setObjectOffset(KDL::Frame& object_offset) = 0;

            virtual SBPLArmModelPtr getArmModel() const = 0;


            DiscObjectState getObjectStateRelBody();

        private:
            //static SBPLArmModelPtr m_arm_model;
            //static KDL::Frame m_object_offset;
            virtual void setArmModel(SBPLArmModelPtr arm_model) = 0;
            static RobotResolutionParams m_params;
            static IKFastPR2 m_ikfast_solver;

            bool m_is_enforcing_joint_limits;
            std::vector<double> m_angles;
    };
    
    class LeftContArmState : public ContArmState {
        public:
            LeftContArmState(){};
            LeftContArmState(std::vector<double> angles) : ContArmState(angles) { }
            virtual SBPLArmModelPtr getArmModel() const { return m_arm_model; }
            virtual KDL::Frame getObjectOffset() const { return m_object_offset; }
            virtual int getArm() const { return m_arm_side; };
            virtual void setObjectOffset(KDL::Frame& object_offset){ m_object_offset = object_offset; };
            virtual void setArmModel(SBPLArmModelPtr arm_model) { m_arm_model = arm_model; };

            static void initArmModel(ArmDescriptionParams& params);

            static std::vector<LeftContArmState> joint_space_interpolate(
                const LeftContArmState&& start, const LeftContArmState&& end,
                int num_interp_steps);

        private:
            static int m_arm_side;
            static SBPLArmModelPtr m_arm_model;
            static KDL::Frame m_object_offset;
    };

    class RightContArmState : public ContArmState {
        public:
            RightContArmState(){};
            RightContArmState(std::vector<double> angles) : ContArmState(angles) { }
            virtual SBPLArmModelPtr getArmModel() const { return m_arm_model; }
            virtual int getArm() const { return m_arm_side; };
            virtual KDL::Frame getObjectOffset() const { return m_object_offset; }
            virtual void setObjectOffset(KDL::Frame& object_offset){ m_object_offset = object_offset; };
            virtual void setArmModel(SBPLArmModelPtr arm_model) { m_arm_model = arm_model; };

            static void initArmModel(ArmDescriptionParams& params);

            static std::vector<RightContArmState> joint_space_interpolate(
                const RightContArmState&& start, const RightContArmState&& end,
                int num_interp_steps);

        private:
            static int m_arm_side;
            static SBPLArmModelPtr m_arm_model;
            static KDL::Frame m_object_offset;
    };
}
