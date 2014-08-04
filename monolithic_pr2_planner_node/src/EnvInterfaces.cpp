#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <kdl/frames.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>
#include <climits>
#include <sbpl/planners/mha_planner.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace KDL;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.

EnvInterfaces::EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::Environment> env, ros::NodeHandle nh) :
    m_nodehandle(nh),
    m_env(env), m_collision_space_interface(new CollisionSpaceInterface(env->getCollisionSpace(), env->getHeuristicMgr())),
    m_generator(new StartGoalGenerator(env->getCollisionSpace())), 
    m_rrt(new OMPLPR2Planner(env->getCollisionSpace(), RRT)),
    m_prm(new OMPLPR2Planner(env->getCollisionSpace(), PRM_P)),
    m_rrtstar(new OMPLPR2Planner(env->getCollisionSpace(), RRTSTAR)),
    m_rrtstar_first_sol(new OMPLPR2Planner(env->getCollisionSpace(),
        RRTSTARFIRSTSOL))
{
        getParams();
    bool forward_search = true;
    m_ara_planner.reset(new ARAPlanner(m_env.get(), forward_search));
    m_mha_planner.reset(new MHAPlanner(m_env.get(), NUM_SMHA_HEUR, forward_search));
    m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub", 1);
    m_costmap_publisher.reset(new
        costmap_2d::Costmap2DPublisher(m_nodehandle,1,"/map"));
}


void EnvInterfaces::getParams(){
    m_nodehandle.param<string>("reference_frame", m_params.ref_frame, 
                                    string("map"));
    m_nodehandle.param<bool>("run_trajectory", m_params.run_trajectory, false);
    m_nodehandle.param<string>("controller_service",
        m_params.controller_service, "/monolithic_controller/execute_path");
}

void EnvInterfaces::bindPlanPathToEnv(string service_name){
    m_plan_service = m_nodehandle.advertiseService(service_name, 
                                                   &EnvInterfaces::planPathCallback,
                                                   this);
}

void EnvInterfaces::bindExperimentToEnv(string service_name){
    m_experiment_service = m_nodehandle.advertiseService(service_name, 
                                                         &EnvInterfaces::experimentCallback,
                                                         this);
}

void EnvInterfaces::bindDemoToEnv(string service_name){
    m_demo_service = m_nodehandle.advertiseService(service_name, 
                                                &EnvInterfaces::demoCallback,
                                                this);
}

/*! \brief this is callback is purely for simulation purposes
 */
bool EnvInterfaces::experimentCallback(GetMobileArmPlan::Request &req,
                                       GetMobileArmPlan::Response &res){
    ROS_INFO("running simulations!");
    vector<pair<RobotState, RobotState> > start_goal_pairs;
    RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
    int number_of_trials = 10;
    int counter = 0;
    while (counter < number_of_trials){
        m_generator->initializeRegions();
        m_generator->generateUniformPairs(number_of_trials, start_goal_pairs);

        for (auto& start_goal : start_goal_pairs){
            ROS_ERROR("running trial %d", counter);
            // start_goal.first.visualize();
            SearchRequestParamsPtr search_request = 
                make_shared<SearchRequestParams>();
            search_request->initial_epsilon = req.initial_eps;
            search_request->final_epsilon = req.final_eps;
            search_request->decrement_epsilon = req.dec_eps;
            search_request->obj_goal= start_goal.second.getObjectStateRelMap();
            search_request->base_start = start_goal.first.base_state();
            search_request->base_goal = start_goal.second.base_state();
            search_request->left_arm_start = start_goal.first.left_arm();
            search_request->right_arm_start = start_goal.first.right_arm();
            search_request->left_arm_goal = start_goal.second.left_arm();
            search_request->right_arm_goal = start_goal.second.right_arm();
            search_request->obj_goal= start_goal.second.getObjectStateRelMap();

            KDL::Frame rarm_offset, larm_offset;
            rarm_offset.p.x(req.rarm_object.pose.position.x);
            rarm_offset.p.y(req.rarm_object.pose.position.y);
            rarm_offset.p.z(req.rarm_object.pose.position.z);
            larm_offset.p.x(req.larm_object.pose.position.x);
            larm_offset.p.y(req.larm_object.pose.position.y);
            larm_offset.p.z(req.larm_object.pose.position.z);

            rarm_offset.M = Rotation::Quaternion(
                req.rarm_object.pose.orientation.x, 
                req.rarm_object.pose.orientation.y, 
                req.rarm_object.pose.orientation.z, 
                req.rarm_object.pose.orientation.w);
            larm_offset.M = Rotation::Quaternion(
                req.larm_object.pose.orientation.x, 
                req.larm_object.pose.orientation.y, 
                req.larm_object.pose.orientation.z, 
                req.larm_object.pose.orientation.w);
            search_request->left_arm_object = larm_offset;
            search_request->right_arm_object = rarm_offset;
            search_request->xyz_tolerance = req.xyz_tolerance;
            search_request->roll_tolerance = req.roll_tolerance;
            search_request->pitch_tolerance = req.pitch_tolerance;
            search_request->yaw_tolerance = req.yaw_tolerance;
            search_request->planning_mode = req.planning_mode;

            res.stats_field_names.resize(18);
            res.stats.resize(18);
            int start_id, goal_id;
            bool return_first_soln = true;
            bool forward_search = true;
            clock_t total_planning_time;
            bool isPlanFound;
            vector<double> stats;
            vector<string> stat_names;
            vector<FullBodyState> states;
            vector<int> soln;
            int soln_cost;

            int environment_seed;

            m_nodehandle.getParam("/monolithic_pr2_planner_node/experiments/seed",
                environment_seed);

            if (!m_rrt->checkRequest(*search_request)){
                    ROS_WARN("bad start goal for ompl");
            } else {
                // Here starts the actual planning requests
                start_goal.first.visualize();
                runMHAPlanner(monolithic_pr2_planner::T_SMHA, "smha_", req, res, search_request, counter);
                runMHAPlanner(monolithic_pr2_planner::T_IMHA, "imha_", req, res, search_request, counter);
                // runMHAPlanner(monolithic_pr2_planner::T_MPWA, "mpwa_", req, res, search_request, counter);
                // runMHAPlanner(monolithic_pr2_planner::T_MHG_REEX, "mhg_reex_", req, res, search_request, counter);
                // runMHAPlanner(monolithic_pr2_planner::T_MHG_NO_REEX, "mhg_no_reex_", req, res, search_request, counter);
                // runMHAPlanner(monolithic_pr2_planner::T_EES, "ees_", req, res, search_request, counter);

                // ARA Planner
                /*** BEGIN ARA PLANNER ****
                m_env->reset();
                m_env->setPlannerType(monolithic_pr2_planner::T_ARA);
                m_ara_planner.reset(new ARAPlanner(m_env.get(), forward_search));
                total_planning_time = clock();
                if(!m_env->configureRequest(search_request, start_id, goal_id))
                    ROS_ERROR("Unable to configure request for ARA!"
                        " Trial ID: %d", counter);

                m_ara_planner->set_initialsolution_eps(EPS1*EPS2);
                m_ara_planner->set_search_mode(return_first_soln);
                m_ara_planner->set_start(start_id);
                ROS_INFO("setting ARA goal id to %d", goal_id);
                m_ara_planner->set_goal(goal_id);
                m_ara_planner->force_planning_from_scratch();
                soln.clear();
                soln_cost = 0;
                isPlanFound = m_ara_planner->replan(req.allocated_planning_time, 
                                                     &soln, &soln_cost);

                if (isPlanFound){
                    ROS_INFO("Plan found in ARA Planner." 
                        " Moving on to reconstruction.");
                    states =  m_env->reconstructPath(soln);
                    total_planning_time = clock() - total_planning_time;
                    packageStats(stat_names, stats, soln_cost, states.size(),
                        total_planning_time);
                    m_stats_writer.writeARA(stats, states, counter);
                    res.stats_field_names = stat_names;
                    res.stats = stats;
                } else {
                    packageStats(stat_names, stats, soln_cost, states.size(),
                        total_planning_time);
                    ROS_INFO("No plan found!");
                }
                /*** END ARA PLANNER ****/

                // OMPL
                // m_env->reset();
                // if(!m_env->configureRequest(search_request, start_id, goal_id)){
                //     ROS_ERROR("Unable to configure request for OMPL! Trial ID: %d", counter);
                // }

                // Run OMPL
                /*** OMPL PLANNERS ****
                m_rrt->planPathCallback(*search_request, counter, m_stats_writer);
                m_prm->planPathCallback(*search_request, counter, m_stats_writer);
                m_rrtstar->planPathCallback(*search_request, counter, m_stats_writer);
                m_rrtstar_first_sol->planPathCallback(*search_request, counter, m_stats_writer);
                /**** END OMPL PLANNERS ***/
                
                // Write env if the whole thing didn't crash.
                m_stats_writer.writeStartGoal(counter, start_goal, environment_seed);
                counter++;
            }
        }
    }
    return true;
}

bool EnvInterfaces::runMHAPlanner(int planner_type,
    std::string planner_prefix,
    GetMobileArmPlan::Request &req,
    GetMobileArmPlan::Response &res,
    SearchRequestParamsPtr search_request,
    int counter) {
    // std::cin.get();
    int start_id, goal_id;
    bool return_first_soln = true;
    bool forward_search = true;
    clock_t total_planning_time;
    bool isPlanFound;
    vector<double> stats;
    vector<string> stat_names;
    vector<FullBodyState> states;

    int planner_queues = NUM_SMHA_HEUR;
    if (planner_type == monolithic_pr2_planner::T_EES)
        planner_queues = 3;
    else if (planner_type == monolithic_pr2_planner::T_IMHA)
        planner_queues = NUM_IMHA_HEUR;

    m_env->reset();
    m_env->setPlannerType(planner_type);
    m_mha_planner.reset(new MHAPlanner(m_env.get(), planner_queues, forward_search,
        planner_type));
    total_planning_time = clock();
    if (!m_env->configureRequest(search_request, start_id, goal_id))
        ROS_ERROR("Unable to configure request for %s! Trial ID: %d",
         planner_prefix.c_str(), counter);
    //m_mha_planner->set_initialsolution_eps1(EPS1); TODO:MIKE
    //m_mha_planner->set_initialsolution_eps2(EPS2); TODO:MIKE
    m_mha_planner->set_search_mode(return_first_soln);
    m_mha_planner->set_start(start_id);
    ROS_INFO("setting %s goal id to %d", planner_prefix.c_str(), goal_id);
    m_mha_planner->set_goal(goal_id);
    m_mha_planner->force_planning_from_scratch();
    vector<int> soln;
    int soln_cost;
    isPlanFound = m_mha_planner->replan(req.allocated_planning_time, 
                                         &soln, &soln_cost);

    if (isPlanFound) {
        ROS_INFO("Plan found in %s Planner. Moving on to reconstruction.",
            planner_prefix.c_str());
        states =  m_env->reconstructPath(soln);
        total_planning_time = clock() - total_planning_time;
        packageMHAStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        m_stats_writer.writeSBPL(stats, states, counter, planner_prefix);
        res.stats_field_names = stat_names;
        res.stats = stats;
    } else {
        packageMHAStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        ROS_INFO("No plan found in %s!", planner_prefix.c_str());
    }
    if(m_params.run_trajectory) {
        ROS_INFO("Running trajectory!");
        runTrajectory(states);
    }
    return isPlanFound;
}

bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res)
{
    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    search_request->base_start = req.body_start;
    search_request->left_arm_start = LeftContArmState(req.larm_start);
    search_request->right_arm_start = RightContArmState(req.rarm_start);
    search_request->underspecified_start = req.underspecified_start;

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                         req.rarm_object.pose.orientation.y, 
                                         req.rarm_object.pose.orientation.z, 
                                         req.rarm_object.pose.orientation.w);
    larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                         req.larm_object.pose.orientation.y, 
                                         req.larm_object.pose.orientation.z, 
                                         req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = req.xyz_tolerance;
    search_request->roll_tolerance = req.roll_tolerance;
    search_request->pitch_tolerance = req.pitch_tolerance;
    search_request->yaw_tolerance = req.yaw_tolerance;
    search_request->planning_mode = req.planning_mode;
    search_request->obj_goal= req.goal;
    search_request->obj_start = req.start;

    // Uncomment this stuff if you want to send the robot pose as the goal
    // instead of the object pose
    // RobotState::setPlanningMode(req.planning_mode);
    // RightContArmState rarm_goal(req.rarm_goal);
    // LeftContArmState larm_goal(req.larm_goal);
    // ContBaseState base_goal(req.body_goal);
    // RobotPosePtr goal_robot = boost::make_shared<RobotState>(base_goal,
    //     rarm_goal, larm_goal);

    // ContObjectState obj_goal = goal_robot->getObjectStateRelMap();
    // search_request->obj_goal= obj_goal;
    // goal_robot->visualize();

    // RobotState::setPlanningMode(req.planning_mode);
    // Create the continuous object state.
    // ContObjectState start_obj_state(req.start);
    // Create the continous base state.
    // ContBaseState base_start(req.body_start);
    // Shove it into a robot state. This should give us the state with IK for
    // arms and everything.


    // std::cin.get();
    double object_dim_x = 0.5;
    double object_dim_y = 1.0;
    double object_dim_z = 0.1;
    KDL::Vector req_obj_vector = KDL::Vector(
        req.rarm_object.pose.position.x - object_dim_x/2 - 0.1,
        req.rarm_object.pose.position.y,
        req.rarm_object.pose.position.z);
    KDL::Rotation req_obj_rot = KDL::Rotation::Quaternion(
        req.rarm_object.pose.orientation.x,
        req.rarm_object.pose.orientation.y,
        req.rarm_object.pose.orientation.z,
        req.rarm_object.pose.orientation.w);
    KDL::Frame gripper_wrt_obj(req_obj_rot, req_obj_vector);
    KDL::Frame obj_wrt_gripper = gripper_wrt_obj.Inverse();

    geometry_msgs::Pose attached_object_pose;
    attached_object_pose.position.x = obj_wrt_gripper.p.x();
    attached_object_pose.position.y = obj_wrt_gripper.p.y();
    attached_object_pose.position.z = obj_wrt_gripper.p.z();
    
    KDL::Rotation rot = obj_wrt_gripper.M;
    
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);
    attached_object_pose.orientation.x = qx;
    attached_object_pose.orientation.y = qy;
    attached_object_pose.orientation.z = qz;
    attached_object_pose.orientation.w = qw;

    m_env->getCollisionSpace()->attachCube("picture", "r_wrist_roll_link",
        attached_object_pose, object_dim_x, object_dim_y, object_dim_z);

    res.stats_field_names.resize(18);
    res.stats.resize(18);
    int start_id, goal_id;
    int counter = 42;
    bool isPlanFound;
    
    double total_planning_time = clock();
    // bool retVal = m_env->configureRequest(search_request, start_id, goal_id);
    // if(!retVal){
    //     return false;
    // }
    bool forward_search = true;
    // isPlanFound = runMHAPlanner(monolithic_pr2_planner::T_SMHA, "smha_", req, res, search_request, counter);
    isPlanFound = runMHAPlanner(monolithic_pr2_planner::T_IMHA, "imha_", req, res, search_request, counter);
    // runMHAPlanner(monolithic_pr2_planner::T_MPWA, "mpwa_", req, res, search_request, counter);
    // runMHAPlanner(monolithic_pr2_planner::T_MHG_REEX, "mhg_reex_",
    //     req, res, search_request, counter);
    // runMHAPlanner(monolithic_pr2_planner::T_MHG_NO_REEX,
        // "mhg_no_reex_", req, res, search_request, counter);
    // runMHAPlanner(monolithic_pr2_planner::T_EES, "ees_", req, res, search_request, counter);
    // m_ara_planner.reset(new MHAPlanner(m_env.get(), NUM_SMHA_HEUR, forward_search,
    //     false));
    // ARA Planner
    /*** BEGIN ARA PLANNER ****
    m_env->reset();
    m_env->setPlannerType(monolithic_pr2_planner::T_ARA);
    m_ara_planner.reset(new ARAPlanner(m_env.get(), forward_search));
    total_planning_time = clock();
    if(!m_env->configureRequest(search_request, start_id, goal_id))
        ROS_ERROR("Unable to configure request for ARA!"
            " Trial ID: %d", counter);

    m_ara_planner->set_initialsolution_eps(EPS1*EPS2);
    m_ara_planner->set_search_mode(return_first_soln);
    m_ara_planner->set_start(start_id);
    ROS_INFO("setting ARA goal id to %d", goal_id);
    m_ara_planner->set_goal(goal_id);
    m_ara_planner->force_planning_from_scratch();
    soln.clear();
    soln_cost = 0;
    isPlanFound = m_ara_planner->replan(req.allocated_planning_time, 
                                         &soln, &soln_cost);

    if (isPlanFound){
        ROS_INFO("Plan found in ARA Planner." 
            " Moving on to reconstruction.");
        states =  m_env->reconstructPath(soln);
        total_planning_time = clock() - total_planning_time;
        packageStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        m_stats_writer.writeARA(stats, states, counter);
        res.stats_field_names = stat_names;
        res.stats = stats;
    } else {
        packageStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        ROS_INFO("No plan found!");
    }
    /*** END ARA PLANNER ****/
    return isPlanFound;
}

bool EnvInterfaces::demoCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res)
{
    
    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    
    // Get the current state of the robot from the real robot.
    BodyPose start_body_pos;
    std::vector<double> start_rangles;
    std::vector<double> start_langles;
    getRobotState(m_tf, start_body_pos, start_rangles, start_langles);
    search_request->left_arm_start = LeftContArmState(start_langles);
    search_request->right_arm_start = RightContArmState(start_rangles);
    ContBaseState start_base_pose(start_body_pos);
    search_request->base_start = start_base_pose;
    // Underspecified goal
    search_request->underspecified_start = req.underspecified_start;

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                         req.rarm_object.pose.orientation.y, 
                                         req.rarm_object.pose.orientation.z, 
                                         req.rarm_object.pose.orientation.w);
    larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                         req.larm_object.pose.orientation.y, 
                                         req.larm_object.pose.orientation.z, 
                                         req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = req.xyz_tolerance;
    search_request->roll_tolerance = req.roll_tolerance;
    search_request->pitch_tolerance = req.pitch_tolerance;
    search_request->yaw_tolerance = req.yaw_tolerance;
    search_request->planning_mode = req.planning_mode;

    // relative goal for now.
    geometry_msgs::PoseStamped goal_pose = req.goal;
    goal_pose.pose.position.x += start_body_pos.x;
    goal_pose.pose.position.y += start_body_pos.y;

    search_request->obj_goal = goal_pose;
    search_request->obj_start = req.start;

    // std::cin.get();
    double object_dim_x = 0.5;
    double object_dim_y = 1.0;
    double object_dim_z = 0.1;
    KDL::Vector req_obj_vector = KDL::Vector(
        req.rarm_object.pose.position.x - object_dim_x/2 - 0.1,
        req.rarm_object.pose.position.y,
        req.rarm_object.pose.position.z);
    KDL::Rotation req_obj_rot = KDL::Rotation::Quaternion(
        req.rarm_object.pose.orientation.x,
        req.rarm_object.pose.orientation.y,
        req.rarm_object.pose.orientation.z,
        req.rarm_object.pose.orientation.w);
    KDL::Frame gripper_wrt_obj(req_obj_rot, req_obj_vector);
    KDL::Frame obj_wrt_gripper = gripper_wrt_obj.Inverse();

    geometry_msgs::Pose attached_object_pose;
    attached_object_pose.position.x = obj_wrt_gripper.p.x();
    attached_object_pose.position.y = obj_wrt_gripper.p.y();
    attached_object_pose.position.z = obj_wrt_gripper.p.z();
    
    KDL::Rotation rot = obj_wrt_gripper.M;
    
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);
    attached_object_pose.orientation.x = qx;
    attached_object_pose.orientation.y = qy;
    attached_object_pose.orientation.z = qz;
    attached_object_pose.orientation.w = qw;

    m_env->getCollisionSpace()->attachCube("picture", "r_wrist_roll_link",
        attached_object_pose, object_dim_x, object_dim_y, object_dim_z);

    res.stats_field_names.resize(18);
    res.stats.resize(18);
    int start_id, goal_id;
    int counter = 42;
    bool isPlanFound;

    bool forward_search = true;
    isPlanFound = runMHAPlanner(monolithic_pr2_planner::T_SMHA, "smha_", req, res, search_request, counter);

    return isPlanFound;
}

void EnvInterfaces::packageStats(vector<string>& stat_names, 
                                 vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time)
{
    
    stat_names.resize(10);
    stats.resize(10);
    stat_names[0] = "total plan time";
    stat_names[1] = "initial solution planning time";
    stat_names[2] = "initial epsilon";
    stat_names[3] = "initial solution expansions";
    stat_names[4] = "final epsilon planning time";
    stat_names[5] = "final epsilon";
    stat_names[6] = "solution epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution cost";
    stat_names[9] = "path length";

    // TODO fix the total planning time
    //stats[0] = totalPlanTime;
    stats[0] = total_planning_time/static_cast<double>(CLOCKS_PER_SEC);
    stats[1] = m_ara_planner->get_initial_eps_planning_time();
    stats[2] = m_ara_planner->get_initial_eps();
    stats[3] = m_ara_planner->get_n_expands_init_solution();
    stats[4] = m_ara_planner->get_final_eps_planning_time();
    stats[5] = m_ara_planner->get_final_epsilon();
    stats[6] = m_ara_planner->get_solution_eps();
    stats[7] = m_ara_planner->get_n_expands();
    stats[8] = static_cast<double>(solution_cost);
    stats[9] = static_cast<double>(solution_size);
}

void EnvInterfaces::packageMHAStats(vector<string>& stat_names,
                                 vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time){
    stat_names.resize(10);
    stats.resize(10);
    stat_names[0] = "total plan time";
    stat_names[1] = "initial solution planning time";
    stat_names[2] = "epsilon 1";
    stat_names[3] = "initial solution expansions";
    stat_names[4] = "final epsilon planning time";
    stat_names[5] = "epsilon 2";
    stat_names[6] = "solution epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution cost";
    stat_names[9] = "path length";

    stats[0] = total_planning_time/static_cast<double>(CLOCKS_PER_SEC);
    stats[1] = m_mha_planner->get_initial_eps_planning_time();
    stats[2] = m_mha_planner->get_initial_eps();
    stats[3] = m_mha_planner->get_n_expands_init_solution();
    stats[4] = m_mha_planner->get_final_eps_planning_time();
    stats[5] = m_mha_planner->get_final_epsilon();
    stats[6] = m_mha_planner->get_solution_eps();
    stats[7] = m_mha_planner->get_n_expands();
    stats[8] = static_cast<double>(solution_cost);
    stats[9] = static_cast<double>(solution_size);
}

bool EnvInterfaces::bindCollisionSpaceToTopic(string topic_name){
    m_collision_space_interface->bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}

void EnvInterfaces::bindNavMapToTopic(string topic){
    m_nav_map = m_nodehandle.subscribe(topic, 1, &EnvInterfaces::loadNavMap, this);
}

void EnvInterfaces::crop2DMap(const nav_msgs::MapMetaData& map_info, const
    std::vector<unsigned char>& v, double new_origin_x, double new_origin_y,
                              double width, double height){
    ROS_DEBUG_NAMED(CONFIG_LOG, "to be cropped to : %f (width), %f (height)", width, height);
    vector<vector<unsigned char> > tmp_map(map_info.height);
    for (unsigned int i=0; i < map_info.height; i++){
        for (unsigned int j=0; j < map_info.width; j++){
            tmp_map[i].push_back(v[i*map_info.width+j]);
        }
    }

    double res = map_info.resolution;
    ROS_DEBUG_NAMED(CONFIG_LOG, "resolution : %f", res);
    int new_origin_x_idx = (new_origin_x-map_info.origin.position.x)/res;
    int new_origin_y_idx = (new_origin_y-map_info.origin.position.y)/res;
    int new_width = static_cast<int>((width/res) + 1 + 0.5);
    int new_height = static_cast<int>((height/res) + 1 + 0.5);
    ROS_DEBUG_NAMED(HEUR_LOG, "new origin: %d %d, new_width and new_height: %d %d",
                              new_origin_x_idx, new_origin_y_idx, new_width, 
                              new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of map %lu %lu", tmp_map.size(), 
                                                     tmp_map[0].size());

    vector<vector<unsigned char> > new_map(new_height);
    int row_count = 0;
    for (int i=new_origin_y_idx; i < new_origin_y_idx + new_height; i++){
        for (int j=new_origin_x_idx; j < new_origin_x_idx + new_width; j++){
            new_map[row_count].push_back(tmp_map[i][j]);
        }
        row_count++;
    }
    m_final_map.clear();
    // m_final_map.resize(new_width * new_height);
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++){
            m_final_map.push_back(static_cast<signed char>(new_map[i][j]));
        }
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu", m_final_map.size());
}

void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr& map){
    ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                    map->info.width, map->info.height, map->info.resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                                                      map->info.origin.position.y);

    // look up the values from the occup grid parameters
    // This stuff is in cells.
    int dimX, dimY, dimZ;
    m_collision_space_interface->getOccupancyGridSize(dimX, dimY,
        dimZ);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Size of OccupancyGrid : %d %d %d", dimX, dimY,
        dimZ);
    // This costmap_ros object listens to the map topic as defined
    // in the costmap_2d.yaml file.
    m_costmap_ros.reset(new costmap_2d::Costmap2DROS("costmap_2d", m_tf));

    // Get the underlying costmap in the cost_map object.
    // Publish for visualization. Publishing is done for the entire (uncropped) costmap.
    costmap_2d::Costmap2D cost_map;
    m_costmap_ros->getCostmapCopy(cost_map);

    // Normalize and convert to array.
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Row major. X is row wise, Y is column wise.
            int c = cost_map.getCost(i,j);

            // Set unknowns to free space (we're dealing with static maps for
            // now)
            if (c == costmap_2d::NO_INFORMATION) {
                c = costmap_2d::FREE_SPACE;
            }
            // c = (c == (costmap_2d::NO_INFORMATION)) ? (costmap_2d::FREE_SPACE) : (c);

            // Re-set the cost.
            cost_map.setCost(i,j,c);
        }
    }

    // Re-inflate because we modified the unknown cells to be free space.
    // API : center point of window x, center point of window y, size_x ,
    // size_y
    cost_map.reinflateWindow(dimX*map->info.resolution/2, dimY*map->info.resolution/2, dimX*map->info.resolution, dimY*map->info.resolution);

    std::vector<unsigned char> uncropped_map;
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Normalize the values from 0 to 100.
            // makes life easier when dealing with the heuristic later.
            uncropped_map.push_back(
                static_cast<unsigned char>(
                    static_cast<double>(cost_map.getCost(i,j))/UCHAR_MAX*100.0f)
                );
        }
    }

    m_costmap_publisher->updateCostmapData(cost_map, m_costmap_ros->getRobotFootprint());

    // Publish the full costmap
    // topic : /monolithic_pr2_planner_node/inflated_obstacles (RViz: Grid
    // Cells)
    m_costmap_publisher->publishCostmap();
    // topic : /monolithic_pr2_planner_node/robot_footprint (RViz: polygon)
    m_costmap_publisher->publishFootprint();

    // TODO: Check if this is the right thing to do : Take the resolution from
    // the map for the occupancy grid's values.
    double width = dimX*map->info.resolution;
    double height = dimY*map->info.resolution;
    
    crop2DMap(map->info, uncropped_map, 0, 0, width, height);
    
    // Don't want to publish this.
    nav_msgs::OccupancyGrid costmap_pub;
    costmap_pub.header.frame_id = "/map";
    costmap_pub.header.stamp = ros::Time::now();
    costmap_pub.info.map_load_time = ros::Time::now();
    costmap_pub.info.resolution = map->info.resolution;
    // done in the crop function too.
    costmap_pub.info.width = (width/map->info.resolution+1 + 0.5);
    costmap_pub.info.height = (height/map->info.resolution+1 + 0.5);
    costmap_pub.info.origin.position.x = 0;
    costmap_pub.info.origin.position.y = 0;
    costmap_pub.data = m_final_map;

    // Publish the cropped version of the costmap; publishes
    // /monolithic_pr2_planner/costmap_pub
    ROS_INFO_NAMED(CONFIG_LOG, "Publishing the final map that's supposed to fit"
        " within the occupancy grid.");
    m_costmap_pub.publish(costmap_pub);

    m_collision_space_interface->update2DHeuristicMaps(m_final_map);

}

void EnvInterfaces::getRobotState(tf::TransformListener &tf_, BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles){
  tf::StampedTransform base_map_transform;
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage < sensor_msgs::JointState > ("joint_states");
  rangles.resize(7);
  langles.resize(7);

  rangles[0] = getJointAngle("r_shoulder_pan_joint",state);
  rangles[1] = getJointAngle("r_shoulder_lift_joint",state);
  rangles[2] = getJointAngle("r_upper_arm_roll_joint",state);
  rangles[3] = getJointAngle("r_elbow_flex_joint",state);
  rangles[4] = getJointAngle("r_forearm_roll_joint",state);
  rangles[5] = getJointAngle("r_wrist_flex_joint",state);
  rangles[6] = getJointAngle("r_wrist_roll_joint",state);

  langles[0] = getJointAngle("l_shoulder_pan_joint",state);
  langles[1] = getJointAngle("l_shoulder_lift_joint",state);
  langles[2] = getJointAngle("l_upper_arm_roll_joint",state);
  langles[3] = getJointAngle("l_elbow_flex_joint",state);
  langles[4] = getJointAngle("l_forearm_roll_joint",state);
  langles[5] = getJointAngle("l_wrist_flex_joint",state);
  langles[6] = getJointAngle("l_wrist_roll_joint",state);

  body_pos.z = getJointAngle("torso_lift_joint",state);

  bool done = false;
  while(!done){
    try {
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform);
      body_pos.x = base_map_transform.getOrigin().x();
      body_pos.y = base_map_transform.getOrigin().y();
      body_pos.theta = 2 * atan2(base_map_transform.getRotation().getZ(), base_map_transform.getRotation().getW());
      done = true;
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("[EnvInterfaces] Is there a map? The map-robot transform failed. (%s)", ex.what());
      sleep(1);
    }
  }
}

double EnvInterfaces::getJointAngle(std::string name, sensor_msgs::JointStateConstPtr msg){
  for(unsigned int i=0; i<msg->name.size(); i++){
    if(msg->name[i] == name)
      return msg->position[i];
  }
  ROS_ERROR("joint doesn't exist! (exit)\n");
  exit(1);
}

/**
 * @brief calls the monolithic_trajectory controller
 * @details Converts the FullBodyState objects to a full_body_controller
 * message and subsequently calls the service given by the parameter.
 * 
 * @param controller_service The name of the service to be called
 * @param states The final path
 * @return status of the call
 */
void EnvInterfaces::runTrajectory(std::vector<FullBodyState>& states) {

    // Create the messages from the full body states
    trajectory_msgs::JointTrajectory arms_trajectory;
    trajectory_msgs::JointTrajectory body_trajectory;
    trajectory_msgs::JointTrajectory gripper_trajectory;

    // prepare arms trajectory
    arms_trajectory.joint_names.push_back("r_shoulder_pan_joint");
    arms_trajectory.joint_names.push_back("r_shoulder_lift_joint");
    arms_trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    arms_trajectory.joint_names.push_back("r_elbow_flex_joint");
    arms_trajectory.joint_names.push_back("r_forearm_roll_joint");
    arms_trajectory.joint_names.push_back("r_wrist_flex_joint");
    arms_trajectory.joint_names.push_back("r_wrist_roll_joint");
    arms_trajectory.joint_names.push_back("l_shoulder_pan_joint");
    arms_trajectory.joint_names.push_back("l_shoulder_lift_joint");
    arms_trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    arms_trajectory.joint_names.push_back("l_elbow_flex_joint");
    arms_trajectory.joint_names.push_back("l_forearm_roll_joint");
    arms_trajectory.joint_names.push_back("l_wrist_flex_joint");
    arms_trajectory.joint_names.push_back("l_wrist_roll_joint");

    arms_trajectory.points.resize(states.size());
    body_trajectory.points.resize(states.size());
    gripper_trajectory.points.resize(states.size());
    for (size_t i = 0; i < states.size(); ++i) {
        // // Insert the right arm joint angles
        // auto it = arms_trajectory.points[i].positions.begin();
        // arms_trajectory.points[i].positions.insert(it,
        //     states[i].right_arm.begin(), states[i].right_arm.end());
        // // insert the left arm joint angles
        // it = arms_trajectory.points[i].positions.end();
        // arms_trajectory.points[i].positions.insert(it,
        //     states[i].left_arm.begin(), states[i].left_arm.end());

        // Insert the right arm joint angles
        for (int r_arm = 0; r_arm < 7; ++r_arm) {
            arms_trajectory.points[i].positions.push_back(states[i].right_arm[r_arm]);
        }
        // insert the left arm joint angles
        for (int l_arm = 0; l_arm < 7; ++l_arm) {
            arms_trajectory.points[i].positions.push_back(states[i].left_arm[l_arm]);
        }
        // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
        //         arms_trajectory.points[i].positions[0],
        //         arms_trajectory.points[i].positions[1],
        //         arms_trajectory.points[i].positions[2],
        //         arms_trajectory.points[i].positions[3],
        //         arms_trajectory.points[i].positions[4],
        //         arms_trajectory.points[i].positions[5],
        //         arms_trajectory.points[i].positions[6],
        //         arms_trajectory.points[i].positions[7],
        //         arms_trajectory.points[i].positions[8],
        //         arms_trajectory.points[i].positions[9],
        //         arms_trajectory.points[i].positions[10],
        //         arms_trajectory.points[i].positions[11],
        //         arms_trajectory.points[i].positions[12],
        //         arms_trajectory.points[i].positions[13]
        //         );

        // insert the base X, Y, Z, Theta
        body_trajectory.points[i].positions.assign(states[i].base.begin(),
            states[i].base.end());

        // Set the gripper poses
        gripper_trajectory.points[i].positions.resize(2,0);
    }

    // Package into full body message
    full_body_controller::ExecutePath::Request req;
    full_body_controller::ExecutePath::Response res;

    req.trajectory = arms_trajectory;
    req.body_trajectory = body_trajectory;
    req.gripper_trajectory= gripper_trajectory;

    // Check if service has been advertised
    ros::service::waitForService(m_params.controller_service);
    ros::ServiceClient client =
    m_nodehandle.serviceClient<full_body_controller::ExecutePath>(m_params.controller_service,
        true);
    if (client.call(req, res)) {
        ROS_INFO("It ran! Oh yeah!!");
    } else {
        ROS_INFO("Trajectory failed.");
    }
}
