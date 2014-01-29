#include <monolithic_pr2_planner_node/ompl_collision_checker.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>

using namespace monolithic_pr2_planner;
using namespace std;
void omplFullBodyCollisionChecker::initialize(CSpaceMgrPtr cspace, vector<double> l_arm){
    m_cspace = cspace;
    l_arm_init = l_arm;
}


bool omplFullBodyCollisionChecker::isValid(const ompl::base::State *state) const {
    const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);

    ContObjectState obj_state((*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0],
                              (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1],
                              (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2],
                              (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3],
                              (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4],
                              (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5]);
    vector<double> wpose(12,0);
    wpose[0] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1]; //arm x
    wpose[1] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2]; //arm y
    wpose[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3]; //arm z
    wpose[3] = 0; //roll
    wpose[4] = 0; //pitch
    wpose[5] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4]; //yaw
    wpose[6] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6]; //right arm free angle
    wpose[7] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5]; //left arm free angle
    wpose[8] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getX(); //base x
    wpose[9] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getY(); //base y
    wpose[10] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[8]; //spine z
    wpose[11] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw(); //base yaw

    vector<double> arm0(7,0); //right arm angles
    vector<double> arm1(7,0); //left arm angles

    arm1 = l_arm_init;
    //arm1[0] = 0.137274;
    //arm1[1] = 0.314918;
    //arm1[2] = 0.185035;
    //arm1[3] = -1.662954;
    //arm1[4] = 2.923877;
    //arm1[5] = -1.305254;
    //arm1[6] = -0.370584;

    arm0[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6];
    //arm1[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5];
    RightContArmState r_arm(arm0);
    LeftContArmState l_arm(arm1);
    ContBaseState base(wpose[8], wpose[9], wpose[10], wpose[11]);
    base.printToDebug(SEARCH_LOG);
    RobotState seed_state(base, r_arm, l_arm);

    RobotPosePtr new_robot_state;
    if (!RobotState::computeRobotPose(DiscObjectState(obj_state), seed_state, new_robot_state)){
        return false;
    }
    RightContArmState new_r_arm = new_robot_state->right_arm();
    if ( m_cspace->isValid(base, new_r_arm, l_arm)){
        return true;
    } else {
        return false;
    }
}
