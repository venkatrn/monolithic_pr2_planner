#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <angles/angles.h>
#include <string>
#include <ros/console.h>
#include <stdexcept>

using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

IKFastPR2 ContArmState::m_ikfast_solver;
RobotResolutionParams ContArmState::m_params;
KDL::Frame LeftContArmState::m_object_offset;
KDL::Frame RightContArmState::m_object_offset;
int LeftContArmState::m_arm_side;
int RightContArmState::m_arm_side;
SBPLArmModelPtr LeftContArmState::m_arm_model;
SBPLArmModelPtr RightContArmState::m_arm_model;

bool ContArmState::operator==(const ContArmState& other){
    return (m_angles == other.m_angles);
}

bool ContArmState::operator!=(const ContArmState& other){
    return !(*this == other);
}

ContArmState::ContArmState() : 
    m_is_enforcing_joint_limits(true), m_angles(7,0){
}

ContArmState::ContArmState(vector<double> arm_state) : 
    m_angles(arm_state) {
}

void ContArmState::setRobotResolutionParams(const RobotResolutionParams& params){
    m_params = params;
}

int ContArmState::getDiscFreeAngle() const {
    double free_angle_res = m_params.arm_free_angle_resolution;
    double free_angle = m_angles[Joints::UPPER_ARM_ROLL];
    int disc_angle = static_cast<int>((normalize_angle_positive(free_angle + 
                                              free_angle_res*0.5))/free_angle_res);
    assert(free_angle-convertDiscFreeAngleToCont(disc_angle)<free_angle_res);
    return disc_angle;
}

void ContArmState::getAngles(std::vector<double>* angles) const {
    *angles = m_angles;
}


void LeftContArmState::initArmModel(ArmDescriptionParams& params){
    FILE* fp_arm= fopen(params.arm_file.c_str(), "r");
    if (!fp_arm){
        ROS_ERROR("Couldn't open left arm model file (%s)!",
                   params.arm_file.c_str());
    }
    SBPLArmModelPtr arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
    arm_model->setResolution(params.env_resolution);
    if (!params.robot_description_string.compare("ROS_PARAM")){
        ROS_DEBUG_NAMED(CONFIG_LOG, "getting kdl chain from paramserver");
        arm_model->initKDLChainFromParamServer();
    } else {
        ROS_DEBUG_NAMED(CONFIG_LOG, "getting kdl chain from string");
        arm_model->initKDLChain(params.robot_description_string);
    }
    m_arm_model = arm_model;
    m_arm_side = ArmSide::LEFT;
}

void RightContArmState::initArmModel(ArmDescriptionParams& params){
    FILE* fp_arm= fopen(params.arm_file.c_str(), "r");
    if (!fp_arm){
        ROS_ERROR("Couldn't open right arm model file (%s)!",
                   params.arm_file.c_str());
    }
    SBPLArmModelPtr arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
    arm_model->setResolution(params.env_resolution);
    if (!params.robot_description_string.compare("ROS_PARAM")){
        ROS_DEBUG_NAMED(CONFIG_LOG, "getting kdl chain from paramserver");
        arm_model->initKDLChainFromParamServer();
    } else {
        ROS_DEBUG_NAMED(CONFIG_LOG, "getting kdl chain from string");
        arm_model->initKDLChain(params.robot_description_string);
    }
    m_arm_model = arm_model;
    m_arm_side = ArmSide::RIGHT;
}

DiscObjectState ContArmState::getObjectStateRelBody(){
    KDL::Frame to_wrist;
#ifdef USE_KDL_SOLVER
    // 10 is the link number for the r_wrist_roll_link
    getArmModel()->computeArmFK(m_angles, 10, &to_wrist);
#endif
#ifdef USE_IKFAST_SOLVER
    if (getArm() == ArmSide::RIGHT){
        to_wrist = m_ikfast_solver.fkRightArm(m_angles);
    } else if (getArm() == ArmSide::LEFT){
        to_wrist = m_ikfast_solver.fkLeftArm(m_angles);
    } else {
        throw std::invalid_argument("Arm specified isn't the left or right??");
    }
#endif
    KDL::Frame f = to_wrist * getObjectOffset().Inverse();

    double wr,wp,wy;
    f.M.GetRPY(wr,wp,wy);

    ContObjectState cont_obj_state(f.p.x(), f.p.y(), f.p.z(), wr, wp, wy);
    return DiscObjectState(cont_obj_state);
}

std::vector<LeftContArmState> LeftContArmState::joint_space_interpolate(
                                                const LeftContArmState&& start,
                                                const LeftContArmState&& end,
                                                int num_interp_steps) {
    // create the vector of arm states we're going to return
    std::vector<LeftContArmState> interp_states;

    // will return at least the start and the end
    if (num_interp_steps < 2) {
        interp_states.push_back(start);        
        interp_states.push_back(end);
        return interp_states;        
    }
    
    // subtract 1 from the num_interp_steps because we need to include the start
    // and the end
    num_interp_steps--;


    // get the start and end angles
    std::vector<double> start_angles;
    std::vector<double> end_angles;
    start.getAngles(&start_angles);
    end.getAngles(&end_angles);

    // compute the difference to be achieved for each joint
    std::vector<double> dTheta(start_angles.size(), 0);
    for (size_t i = 0; i < start_angles.size(); i++) {
        dTheta[i] = shortest_angular_distance(start_angles[i], end_angles[i]);
    }

    // compute the step size
    double step_size = 1/static_cast<double>(num_interp_steps);

    for (int i = 0; i <= num_interp_steps; ++i) {
        std::vector<double> this_step_angles(dTheta.size(), 0);
        // generate the vector of doubles for this state
        for (int j = 0; j < static_cast<int>(dTheta.size()); ++j) {
            this_step_angles[j] = normalize_angle_positive(
                                    start_angles[j] + i*step_size* dTheta[j]
                                );
        }
        LeftContArmState left_arm_state(this_step_angles);
        interp_states.push_back(left_arm_state);
    }
    return interp_states;
}

std::vector<RightContArmState> RightContArmState::joint_space_interpolate(
                                                const RightContArmState&& start,
                                                const RightContArmState&& end,
                                                int num_interp_steps) {
    // create the vector of arm states we're going to return
    std::vector<RightContArmState> interp_states;

    // will return at least the start and the end
    if (num_interp_steps < 2) {
        interp_states.push_back(start);        
        interp_states.push_back(end);
        return interp_states;        
    }
    // subtract 1 from the num_interp_steps because we need to include the start
    // and the end
    num_interp_steps--;


    // get the start and end angles
    std::vector<double> start_angles;
    std::vector<double> end_angles;
    start.getAngles(&start_angles);
    end.getAngles(&end_angles);

    // compute the difference to be achieved for each joint
    std::vector<double> dTheta(start_angles.size(), 0);
    for (size_t i = 0; i < start_angles.size(); i++) {
        dTheta[i] = shortest_angular_distance(start_angles[i], end_angles[i]);
    }

    // compute the step size
    double step_size = 1/static_cast<double>(num_interp_steps);

    for (int i = 0; i <= num_interp_steps; ++i) {
        std::vector<double> this_step_angles(dTheta.size(), 0);
        // generate the vector of doubles for this state
        for (int j = 0; j < static_cast<int>(dTheta.size()); ++j) {
            this_step_angles[j] = normalize_angle_positive(
                                    start_angles[j] + i*step_size* dTheta[j]
                                );
        }
        RightContArmState left_arm_state(this_step_angles);
        interp_states.push_back(left_arm_state);
    }
    return interp_states;
}