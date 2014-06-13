#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <cmath>
#include <vector>
#include <angles/angles.h>
#include <assert.h>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

// TODO this entire function could be dramatically improved by just doing a
// matrix rotation about the end effector pose. i should do this.
GoalState BaseAdaptiveMotionPrimitive::m_goal;

BaseAdaptiveMotionPrimitive::BaseAdaptiveMotionPrimitive(int direction):
    m_direction(direction){ }

void BaseAdaptiveMotionPrimitive::rotateObjToGoalYawUsingBase(const GraphState& source_state,
                                                              RobotState& rotated_state){
    ROS_DEBUG_NAMED(MPRIM_LOG, "\trotating object to goal yaw only by turning the base");
    ContObjectState cont_goal = m_goal.getObjectState();
    ContBaseState orig_base_state = source_state.robot_pose().base_state();
    ContObjectState cur_obj_state = source_state.getObjectStateRelMap();

    ROS_DEBUG_NAMED(MPRIM_LOG, "\toriginal continuous base: %f %f %f %f", 
                               orig_base_state.x(),
                               orig_base_state.y(),
                               orig_base_state.z(),
                               orig_base_state.theta());
    double delta_cont_theta = shortest_angular_distance(cur_obj_state.yaw(),cont_goal.yaw());
    int sign = static_cast<int>(delta_cont_theta/fabs(delta_cont_theta));
    assert(abs(sign) == 1);
    int del_disc_theta = DiscBaseState::convertContTheta(fabs(delta_cont_theta));
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tTurned the base by %f (or %d theta steps).", 
                    delta_cont_theta, del_disc_theta);

    DiscBaseState d_base_state = source_state.robot_pose().base_state();
    d_base_state.theta(d_base_state.theta() + sign*del_disc_theta);
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tThe new base theta is now %d", d_base_state.theta());
    RobotState base_turned_pose(d_base_state, 
                                source_state.robot_pose().right_arm(), 
                                source_state.robot_pose().left_arm());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tObject theta should be closer to the goal's theta now.");
    ContObjectState c_unadjusted_obj = base_turned_pose.getObjectStateRelMap();
    c_unadjusted_obj.printToDebug(MPRIM_LOG);
    ContBaseState new_base = base_turned_pose.base_state();
    rotated_state = base_turned_pose;
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tNew base theta: %f", new_base.theta());
}

bool BaseAdaptiveMotionPrimitive::moveObjToGoalPositionUsingBase(const GraphState& source_state,
                                                                 const RobotState& rotated_state,
                                                                 RobotPosePtr& final_state){
    ContObjectState c_goal = m_goal.getObjectState();
    ContObjectState c_unadjusted_obj = rotated_state.getObjectStateRelMap();
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tNow need to readjust the object state back "
                               "to the original xyz pose by translating the base.");
    ROS_DEBUG_NAMED(MPRIM_LOG, "Goal XY is (%f %f), current hand XY is (%f %f)",
                    c_goal.x(), c_goal.y(), c_unadjusted_obj.x(), c_unadjusted_obj.y());
    double c_dx = c_goal.x() - c_unadjusted_obj.x();
    double c_dy = c_goal.y() - c_unadjusted_obj.y();
    int sign_x = static_cast<int>(c_dx/fabs(c_dx));
    int sign_y = static_cast<int>(c_dy/fabs(c_dy));
    int dx = DiscBaseState::convertContDistance(fabs(c_dx));
    int dy = DiscBaseState::convertContDistance(fabs(c_dy));
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tNeed to move base by %f (%d) %f (%d)",
                    c_dx, dx, c_dy, dy);
    DiscBaseState new_base_state = rotated_state.base_state();
    new_base_state.x(new_base_state.x() + sign_x*dx);
    new_base_state.y(new_base_state.y() + sign_y*dy);
    ContBaseState c_new_base_state = new_base_state;
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tnew base state is now %f %f %f %f (%d %d %d %d)", 
                                c_new_base_state.x(), 
                                c_new_base_state.y(),
                                c_new_base_state.z(),
                                c_new_base_state.theta(),
                                new_base_state.x(),
                                new_base_state.y(),
                                new_base_state.z(),
                                new_base_state.theta());

    final_state = make_shared<RobotState>(new_base_state, 
                                          rotated_state.right_arm(),
                                          rotated_state.left_arm());
    ROS_DEBUG_NAMED(MPRIM_LOG, "Final state is");
    final_state->printToDebug(MPRIM_LOG);
    final_state->getObjectStateRelMap().printToDebug(MPRIM_LOG);
    return true;
                                                
    // we "convert" the obj from map frame to body frame. really, all we have to
    // do is adjust the object yaw based on where the base theta is, since pitch
    // and roll in the body frame is always aligned with the map frame (unless
    // the PR2 does some off-roading).
    // ContObjectState obj_in_body_frame = source_state.getObjectStateRelBody();
    // obj_in_body_frame.roll(c_goal.roll());
    // obj_in_body_frame.pitch(c_goal.pitch());
    // obj_in_body_frame.yaw(shortest_angular_distance(rotated_state.base_state().theta(),c_unadjusted_obj.yaw()));


    // ROS_DEBUG_NAMED(MPRIM_LOG, "\tRunning IK to object state:");
    // obj_in_body_frame.printToDebug(MPRIM_LOG);
    // RobotState seed_state(new_base_state, source_state.robot_pose().right_arm(),
    //                                       source_state.robot_pose().left_arm());

    // bool isIKSuccess = RobotState::computeRobotPose(DiscObjectState(obj_in_body_frame), 
    //                                                 seed_state, 
    //                                                 final_state);
    // return isIKSuccess;
}


bool BaseAdaptiveMotionPrimitive::apply(const GraphState& source_state, 
                                         GraphStatePtr& successor,
                                         TransitionData& t_data){
    DiscObjectState d_goal = m_goal.getObjectState();
    ContObjectState c_goal = m_goal.getObjectState();
    ContObjectState cur_obj_state = source_state.getObjectStateRelMap();

    bool isWithinAMPDistance = (dist(cur_obj_state, d_goal) > 2);
    if (isWithinAMPDistance){
        return false;
    }

    if (fabs(cur_obj_state.yaw()-c_goal.yaw()) < ContBaseState::getThetaResolution()){
        ROS_DEBUG_NAMED(MPRIM_LOG, "Object is already aligned in yaw to goal. "
                                   "Skipping base AMP.");
        return false;
    }

    ROS_DEBUG_NAMED(MPRIM_LOG, "====== applying base AMP ======");
    ROS_DEBUG_NAMED(MPRIM_LOG, "Source state (with visualization):");
    source_state.printToDebug(MPRIM_LOG);
    ROS_DEBUG_NAMED(MPRIM_LOG, "Original object staterelative to map:");
    cur_obj_state.printToDebug(MPRIM_LOG);
    ROS_DEBUG_NAMED(MPRIM_LOG, "Goal state:");
    d_goal.printToDebug(MPRIM_LOG);
    c_goal.printToDebug(MPRIM_LOG);

    RobotState rotated_state;
    rotateObjToGoalYawUsingBase(source_state, rotated_state);

    RobotPosePtr final_state;
    bool mprim_success = moveObjToGoalPositionUsingBase(source_state, rotated_state,
                                                        final_state);
    if (!mprim_success){
        return false;
    }

    ContObjectState final_obj_state = final_state->getObjectStateRelMap();
    ROS_DEBUG_NAMED(MPRIM_LOG, "Final obj state is now");
    final_obj_state.printToDebug(MPRIM_LOG);
    successor = make_shared<GraphState>(*final_state);
    successor->printContToDebug(MPRIM_LOG);
    computeIntermSteps(source_state, *successor, t_data);

    t_data.motion_type(motion_type());
    // TODO compute proper cost
    t_data.cost(cost());
    return true;
}


void BaseAdaptiveMotionPrimitive::computeIntermSteps(const GraphState& source_state,
                                                     const GraphState& successor,
                                                     TransitionData& t_data){
    // TODO parameterize the number of degrees to check
    // compute the intermediate base poses
    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolating between:");
    ContBaseState start_base_state = source_state.robot_pose().base_state();
    ContBaseState end_base_state = successor.robot_pose().base_state();
    start_base_state.printToDebug(MPRIM_LOG);
    end_base_state.printToDebug(MPRIM_LOG);
    double del_theta = shortest_angular_distance(start_base_state.theta(),
                                                 end_base_state.theta());
    int num_interp_steps = static_cast<int>(ceil(fabs(del_theta / (2.0*M_PI/180))));
    ROS_DEBUG_NAMED(MPRIM_LOG, "number of interpolation steps: %d", num_interp_steps);
    ROS_DEBUG_NAMED(MPRIM_LOG, "del theta is %f", del_theta);

    ContObjectState final_obj = successor.getObjectStateRelMap();
    double zero_x = start_base_state.x() - final_obj.x();
    double zero_y = start_base_state.y() - final_obj.y();
    
    vector<ContBaseState> interp_base_states;
    for (int i=0; i <= num_interp_steps; i++){
        ContBaseState interm_step = start_base_state;
        double rotate_by_angle = i*del_theta/num_interp_steps;
        double new_x = cos(rotate_by_angle) * zero_x - sin(rotate_by_angle) * zero_y;
        double new_y = sin(rotate_by_angle) * zero_x + cos(rotate_by_angle) * zero_y;
        interm_step.x(new_x + final_obj.x());
        interm_step.y(new_y + final_obj.y());
        interm_step.theta(start_base_state.theta() + rotate_by_angle);
        interp_base_states.push_back(interm_step);


        // for visualization
        //vector<double> l_arm;
        //vector<double> r_arm;
        //source_state.robot_pose().right_arm().getAngles(&r_arm);
        //source_state.robot_pose().left_arm().getAngles(&l_arm);
        //BodyPose body_pose = interm_step.body_pose();
        //Visualizer::pviz->visualizeRobot(r_arm, l_arm, body_pose, 150, 
        //                                std::string("planner"), 0);
    }

    vector<RobotState> interm_robot_steps;
    vector<ContBaseState> cont_base_state_steps;
    ROS_DEBUG_NAMED(MPRIM_LOG, "generated %lu intermediate base AMP motion primitive vectors:",
                                interp_base_states.size());
    LeftContArmState l_arm = source_state.robot_pose().left_arm();
    RightContArmState r_arm = source_state.robot_pose().right_arm();
    for (auto base_step : interp_base_states){
        RobotState robot_state(base_step, r_arm, l_arm);
        interm_robot_steps.push_back(robot_state);
        robot_state.printToDebug(MPRIM_LOG);
        cont_base_state_steps.push_back(base_step);
    }
    t_data.interm_robot_steps(interm_robot_steps);
    t_data.cont_base_interm_steps(cont_base_state_steps);
    assert(interm_robot_steps.size() == cont_base_state_steps.size());
}


// TODO: add to this comment
// Since these intermediate steps are computed at every step, the values
// generated here are absolute coordinates, NOT delta coordinates.
//void BaseAdaptiveMotionPrimitive::computeIntermSteps(const GraphState& source_state,
//                                                     const GraphState& successor,
//                                                     TransitionData& t_data){
//    vector<ContBaseState> delta_base_steps = computeDeltaBaseSteps(source_state, successor);
//    // TODO API fail
//    double r_free_angle = source_state.robot_pose().right_arm().getUpperArmRollAngle();
//    double l_free_angle = source_state.robot_pose().left_arm().getUpperArmRollAngle();
//
//    ContObjectState obj_state = source_state.getObjectStateRelMap();
//    vector<double> obj_state_coord;
//    obj_state.getStateValues(&obj_state_coord);
//
//    // create a motion primitive element by slapping the interpolated base
//    // states together with the object state xyzrpy, free angle left and right
//    ROS_DEBUG_NAMED(MPRIM_LOG, "generated %lu intermediate base AMP motion primitive vectors:",
//                                delta_base_steps.size());
//    for(auto& base_step : delta_base_steps){
//        vector<double> step;
//        step.insert(step.end(), obj_state_coord.begin(), obj_state_coord.end());
//        step.push_back(r_free_angle);
//        step.push_back(l_free_angle);
//        vector<double> base_vector;
//        base_step.getValues(&base_vector);
//        step.insert(step.end(), base_vector.begin(), base_vector.end());
//        m_interm_steps.push_back(step);
//        assert(step.size() == 12);
//        ROS_DEBUG_NAMED(MPRIM_LOG, "%f %f %f %f %f %f %f %f %f %f %f %f",
//                                   step[0], step[1], step[2], step[3],
//                                   step[4], step[5], step[6], step[7],
//                                   step[8], step[9], step[10], step[11]);
//    }
//
//    ContBaseState source_base = source_state.robot_pose().base_state();
//    double final_x = source_base.x()+m_interm_steps[m_interm_steps.size()-1][GraphStateElement::BASE_X];
//    double final_y = source_base.y()+m_interm_steps[m_interm_steps.size()-1][GraphStateElement::BASE_Y];
//    double final_z = source_base.z()+m_interm_steps[m_interm_steps.size()-1][GraphStateElement::BASE_Z];
//    double final_theta = source_base.theta()+m_interm_steps[m_interm_steps.size()-1][GraphStateElement::BASE_THETA];
//    final_theta = normalize_angle_positive(final_theta);
//
//    ContBaseState final_base = successor.robot_pose().base_state();
//    ROS_DEBUG_NAMED(MPRIM_LOG, "source state");
//    source_state.printContToDebug(MPRIM_LOG);
//    ROS_DEBUG_NAMED(MPRIM_LOG, "successor state");
//    successor.printContToDebug(MPRIM_LOG);
//    ROS_DEBUG_NAMED(MPRIM_LOG, "source state + final motion primitive (%f %f %f %f)",
//                               final_x, final_y, final_z, final_theta);
//    assert(fabs(final_x-final_base.x()) < .001);
//    assert(fabs(final_y-final_base.y()) < .001);
//    assert(fabs(final_z-final_base.z()) < .001);
//    assert(fabs(final_theta-final_base.theta()) < .001);
//
//}
//
//vector<ContBaseState> BaseAdaptiveMotionPrimitive::computeDeltaBaseSteps(
//                                                        const GraphState& source_state,
//                                                        const GraphState& successor){
//    // TODO parameterize the number of degrees to check
//    // compute the intermediate base poses
//    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolating between:");
//    ContBaseState start_base_state = source_state.robot_pose().base_state();
//    ContBaseState end_base_state = successor.robot_pose().base_state();
//    start_base_state.printToDebug(MPRIM_LOG);
//    end_base_state.printToDebug(MPRIM_LOG);
//    double del_theta = fabs(end_base_state.theta()-start_base_state.theta());
//    int num_interp_steps = static_cast<int>(del_theta / (2.0*M_PI/180));
//    ROS_DEBUG_NAMED(MPRIM_LOG, "number of interpolation steps: %d", num_interp_steps);
//    ROS_DEBUG_NAMED(MPRIM_LOG, "del theta is %f", del_theta);
//    vector<ContBaseState> interp_base_states =  ContBaseState::interpolate(start_base_state, 
//                                                                           end_base_state, 
//                                                                           num_interp_steps);
//    // need to grab the delta motions for the motion primitive vector
//    for (auto& base_state_step : interp_base_states){
//        base_state_step.x(base_state_step.x()-start_base_state.x());
//        base_state_step.y(base_state_step.y()-start_base_state.y());
//        base_state_step.z(base_state_step.z()-start_base_state.z());
//        base_state_step.theta(base_state_step.theta()-start_base_state.theta());
//    }
//    return interp_base_states;
//}

void BaseAdaptiveMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, "Base Adaptive Primitive");
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tcost: %d", cost());
    printEndCoord();
    printIntermSteps();
}

void BaseAdaptiveMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    // TODO find the right value;
    m_cost = 2;
}
