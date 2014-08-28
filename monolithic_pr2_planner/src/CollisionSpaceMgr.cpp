#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <boost/foreach.hpp>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

using namespace monolithic_pr2_planner;
using namespace pr2_collision_checker;
using namespace boost;
using namespace std;

CollisionSpaceMgr::CollisionSpaceMgr(SBPLArmModelPtr right_arm,
                                     SBPLArmModelPtr left_arm){
    m_cspace = make_shared<PR2CollisionSpace>(right_arm.get(),
                                              left_arm.get(),
                                              m_occupancy_grid.get());
    if (!m_cspace->init()){
        ROS_ERROR("cspace failed to initialize! Exiting.");
        exit(1);
    }
    ROS_DEBUG_NAMED(INIT_LOG, "Launched collision space manager");
    // geometry_msgs::Pose pose;
    // pose.position.x = 0.20;
    // pose.position.y = 0;
    // pose.position.z = 0;

    // pose.orientation.x = 0;
    // pose.orientation.y = 0;
    // pose.orientation.z = 0;
    // pose.orientation.w = 1;
}

/*! \brief Updates the internal collision map of the collision checker.
 */
void CollisionSpaceMgr::updateMap(const arm_navigation_msgs::CollisionMap& map){
    std::vector<Eigen::Vector3d> points;
    for (int i=0; i < (int)map.boxes.size(); i++){
        Eigen::Vector3d vect;
        vect << map.boxes[i].center.x,
        map.boxes[i].center.y,
        map.boxes[i].center.z;
        points.push_back(vect);
    }
    m_occupancy_grid->addPointsToField(points);
}


bool CollisionSpaceMgr::loadMap(const vector<Eigen::Vector3d>& points){
    m_occupancy_grid->addPointsToField(points);
    return true;
}


bool CollisionSpaceMgr::isValid(RobotState& robot_pose){
    vector<double> l_arm;
    vector<double> r_arm;
    robot_pose.left_arm().getAngles(&l_arm);
    robot_pose.right_arm().getAngles(&r_arm);
    DiscBaseState discbody_pose = robot_pose.base_state();
    BodyPose body_pose = robot_pose.base_state().getBodyPose();

    double dist_temp;
    int debug_code;
    ROS_DEBUG_NAMED(CSPACE_LOG, "collision checking pose");
    ROS_DEBUG_NAMED(CSPACE_LOG, "body pose is %f %f %f", body_pose.x, 
                                body_pose.y, body_pose.z);
    robot_pose.printToDebug(CSPACE_LOG);
    // Visualizer::pviz->visualizeRobot(r_arm, l_arm, body_pose, 150, 
                                    // std::string("planner"), 0);
    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, false, dist_temp, 
                                    debug_code);
}

bool CollisionSpaceMgr::isValid(ContBaseState& base, RightContArmState& r_arm, 
                                LeftContArmState& l_arm){
    vector<double> l_arm_v;
    vector<double> r_arm_v;
    l_arm.getAngles(&l_arm_v);
    r_arm.getAngles(&r_arm_v);
    double dist_temp;
    int debug_code;
    BodyPose bp = base.body_pose();
    return m_cspace->checkAllMotion(l_arm_v, r_arm_v, bp, false, dist_temp, 
                                    debug_code);
}

/*! \brief Given the transition data from a state expansion, this does a smart
 * collision check on the successor.
 *
 * If the motion primitive that generated this successor only moves the base,
 * then we don't need to collision check the arms against each other. If only
 * the arm moves, then we don't need to collision check the base for anything.
 *
 * TODO bounds check spine, bounds check base
 */
bool CollisionSpaceMgr::isValidSuccessor(const GraphState& successor,
                                         const TransitionData& t_data){
    RobotState pose = successor.robot_pose();
    vector<double> r_arm(7), l_arm(7);
    pose.right_arm().getAngles(&r_arm);
    pose.left_arm().getAngles(&l_arm);
    BodyPose body_pose = pose.base_state().getBodyPose();
    bool verbose = false;
    double dist;
    int debug;

    bool onlyBaseMotion = (t_data.motion_type() == MPrim_Types::BASE ||
                           t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE);
    bool onlyArmMotion = (t_data.motion_type() == MPrim_Types::ARM ||
                          t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);
    if (onlyBaseMotion){
        return m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist,
                                         debug);
    } else if (onlyArmMotion){
        bool isvalid = m_cspace->checkArmsMotion(l_arm, r_arm, body_pose, 
                                                 verbose, dist, debug);
        return isvalid;
    } else if (t_data.motion_type() == MPrim_Types::TORSO){
        return m_cspace->checkSpineMotion(l_arm, r_arm, body_pose, verbose, 
                                          dist, debug);
    } else {
        throw std::invalid_argument("not a valid motion primitive type");
    }

    return true;
}

/*! \brief Given the transition data from a state expansion, this collision
 * checks all continuous, intermediate states.
 *
 * We cannot just collision check the base state stored in RobotState IF the
 * type of motion is a base motion because RobotState only stores the discrete
 * base state. Thus, we need to look at the continuous base state that is also
 * stored in the transition data.
 */
bool CollisionSpaceMgr::isValidTransitionStates(const TransitionData& t_data){
    bool onlyBaseMotion = (t_data.motion_type() == MPrim_Types::BASE ||
                           t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE);
    // bool onlyArmMotion = (t_data.motion_type() == MPrim_Types::ARM ||
    //                       t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);
    vector<ContBaseState> interp_base_motions;
    if (onlyBaseMotion){
        interp_base_motions = t_data.cont_base_interm_steps();
        assert(interp_base_motions.size() == t_data.interm_robot_steps().size());
    }
    int idx = 0;
    for (auto& robot_state : t_data.interm_robot_steps()){
        vector<double> r_arm(7), l_arm(7);
        robot_state.right_arm().getAngles(&r_arm);
        robot_state.left_arm().getAngles(&l_arm);
        bool verbose = false;
        double dist;
        int debug;
    
        // let's check the validity of all intermediate poses
        if (onlyBaseMotion){
            BodyPose body_pose = interp_base_motions[idx].body_pose();
            if (!m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist,
                debug)){
                return false;
            }
        } else if (t_data.motion_type() == MPrim_Types::ARM) {
            ROS_DEBUG_NAMED(CSPACE_LOG, "skipping the intermediate points for arms because there are none.");
        } else if (t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE) {
            BodyPose body_pose = robot_state.getContBaseState().body_pose();
            if(!m_cspace->checkArmsMotion(l_arm, r_arm, body_pose, 
                                                 verbose, dist, debug)) {
                return false;
            }
        } else if (t_data.motion_type() == MPrim_Types::TORSO) {
            BodyPose body_pose = robot_state.getContBaseState().body_pose();
            if (!m_cspace->checkSpineMotion(l_arm, r_arm, body_pose, verbose, 
                                                      dist, debug)) {
                return false;
            }
        } else {
            throw std::invalid_argument("not a valid motion primitive type");
        }
        idx++;
    }
    return true;
}

bool CollisionSpaceMgr::isValidContState(std::vector<double>& l_arm, std::vector<double>&
    r_arm, const std::vector<double> body){
    BodyPose body_pose;
    body_pose.x = body[BodyDOF::X];
    body_pose.y = body[BodyDOF::Y];
    body_pose.z = body[BodyDOF::Z];
    body_pose.theta = body[BodyDOF::THETA];
    bool verbose = false;
    int debug;
    double dist;
    if(!m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist, debug))
        return false;
    return true;
}

/**
 * \brief visualizes the object attached to the robot.
 * 
 */
 void CollisionSpaceMgr::visualizeAttachedObject(RobotState& robot_state,
    int hue){
    vector<double> t1, t2;
    robot_state.right_arm().getAngles(&t2);
    robot_state.left_arm().getAngles(&t1);
    BodyPose bp = robot_state.base_state().getBodyPose();
    std::vector<std::vector<double> > spheres;
    m_cspace->getAttachedObjectSpheres(t1, t2, bp, spheres);
    stringstream ss;
    ss << "attached_object_" << hue;
    Visualizer::pviz->visualizeSpheres(spheres, hue, ss.str(), .04);
}

void CollisionSpaceMgr::attachCube(std::string name, std::string reference_frame,
                geometry_msgs::Pose relative_pose, double dim_x, double dim_y,
                double dim_z) {
    m_cspace->attachCube(name, reference_frame, relative_pose,
        dim_x,dim_y,dim_z);
}

void CollisionSpaceMgr::visualizeCollisionModel(RobotState& robot_state) {
    std::vector<double> l_arm, r_arm;
    robot_state.right_arm().getAngles(&r_arm);
    robot_state.left_arm().getAngles(&l_arm);
    BodyPose body_pose = robot_state.base_state().getBodyPose();
    m_cspace->visualizeRobotCollisionModel(r_arm, l_arm, body_pose,
        "collision_model", 42);
}
