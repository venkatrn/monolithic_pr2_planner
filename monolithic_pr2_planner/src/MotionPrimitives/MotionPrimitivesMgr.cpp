#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/foreach.hpp>

using namespace monolithic_pr2_planner;
using namespace std;
using namespace boost;

MotionPrimitivesMgr::MotionPrimitivesMgr(boost::shared_ptr<GoalState>& goal) : m_all_mprims(5){ }

/*! \brief loads all mprims from configuration. also sets up amps. note that
 * these are not necessarily the exact mprims used during search, because
 * there's a user parameter that selects which mprims to actually use. 
 */
bool MotionPrimitivesMgr::loadMPrims(const MotionPrimitiveParams& params){
    m_params = params;

    MPrimList arm_mprims;
    m_parser.parseArmMotionPrimitives(params.arm_motion_primitive_file, arm_mprims);

    MPrimList base_mprims;
    m_parser.parseBaseMotionPrimitives(params.base_motion_primitive_file, base_mprims);
    ArmAdaptiveMotionPrimitivePtr armAMP = make_shared<ArmAdaptiveMotionPrimitive>();
    ArmTuckMotionPrimitivePtr tuckAMP = make_shared<ArmTuckMotionPrimitive>();
    ArmUntuckMotionPrimitivePtr untuckAMP = make_shared<ArmUntuckMotionPrimitive>(true);
    ArmUntuckMotionPrimitivePtr untuckPartialAMP = make_shared<ArmUntuckMotionPrimitive>(false);

    MPrimList arm_amps;
    arm_amps.push_back(armAMP);
    arm_amps.push_back(tuckAMP);
    arm_amps.push_back(untuckAMP);

    MPrimList base_amps;
    int NEG_TURN = -1;
    int POS_TURN = 1;
    BaseAdaptiveMotionPrimitivePtr bamp1 = make_shared<BaseAdaptiveMotionPrimitive>(NEG_TURN);
    BaseAdaptiveMotionPrimitivePtr bamp2 = make_shared<BaseAdaptiveMotionPrimitive>(POS_TURN);
    base_amps.push_back(bamp1);
    base_amps.push_back(bamp2);

    MPrimList torso_mprims;
    int VERTICAL_UP = 1;
    int VERTICAL_DOWN = -1;
    TorsoMotionPrimitivePtr t_mprim1 = make_shared<TorsoMotionPrimitive>(VERTICAL_UP);
    TorsoMotionPrimitivePtr t_mprim2 = make_shared<TorsoMotionPrimitive>(VERTICAL_DOWN);
    torso_mprims.push_back(t_mprim1);
    torso_mprims.push_back(t_mprim2);

    m_all_mprims[MPrim_Types::ARM] = arm_mprims;
    m_all_mprims[MPrim_Types::BASE] = base_mprims;
    m_all_mprims[MPrim_Types::TORSO] = torso_mprims;
    m_all_mprims[MPrim_Types::ARM_ADAPTIVE] = arm_amps;
    m_all_mprims[MPrim_Types::BASE_ADAPTIVE] = base_amps;

    computeAllMPrimCosts(m_all_mprims);

    loadAllMPrims();

    for (auto& mprim: m_active_mprims){    
        mprim->print();
    }

    return true;
}

void MotionPrimitivesMgr::loadMPrimSet(int planning_mode){
    m_active_mprims.clear();
    bool is_arm_only = (planning_mode == PlanningModes::RIGHT_ARM || 
                        planning_mode == PlanningModes::LEFT_ARM || 
                        planning_mode == PlanningModes::DUAL_ARM);
    bool is_mobile = (planning_mode == PlanningModes::RIGHT_ARM_MOBILE || 
                      planning_mode == PlanningModes::LEFT_ARM_MOBILE || 
                      planning_mode == PlanningModes::DUAL_ARM_MOBILE);
    if (planning_mode == PlanningModes::BASE_ONLY){
        loadBaseOnlyMPrims();
    } else if (is_arm_only){
        loadArmOnlyMPrims();
    } else if (is_mobile){
        loadAllMPrims();
    } else {
        ROS_ERROR("Invalid planning mode!");
        assert(false);
    }
}

void MotionPrimitivesMgr::combineVectors(const MPrimList& v1, MPrimList& v2){
    for (auto& mprim : v1){
        v2.push_back(mprim);
    }
}

void MotionPrimitivesMgr::loadBaseOnlyMPrims(){
    combineVectors(m_all_mprims[MPrim_Types::BASE], m_active_mprims);
    combineVectors(m_all_mprims[MPrim_Types::BASE_ADAPTIVE], m_active_mprims);
}

void MotionPrimitivesMgr::loadTorsoMPrims(){
    combineVectors(m_all_mprims[MPrim_Types::TORSO], m_active_mprims);
}

// note that we don't separate left and right arm mprims here, since the mprims
// are in cartesian space
void MotionPrimitivesMgr::loadArmOnlyMPrims(){
    combineVectors(m_all_mprims[MPrim_Types::ARM], m_active_mprims);
    combineVectors(m_all_mprims[MPrim_Types::ARM_ADAPTIVE], m_active_mprims);
}

void MotionPrimitivesMgr::loadAllMPrims(){
    loadBaseOnlyMPrims();
    loadArmOnlyMPrims();
    loadTorsoMPrims();
}

void MotionPrimitivesMgr::computeAllMPrimCosts(vector<MPrimList> mprims){
    for (auto& mprim_list : mprims){
        for (auto& mprim : mprim_list){
            mprim->computeCost(m_params);
        }
    }
}
