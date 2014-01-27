#include <monolithic_pr2_planner_node/ompl_pr2_planner.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>
#include <monolithic_pr2_planner/LoggerNames.h>

using namespace monolithic_pr2_planner;
using namespace monolithic_pr2_planner_node;
OMPLPR2Planner::OMPLPR2Planner(const CSpaceMgrPtr& cspace){
    //create the StateSpace (defines the dimensions and their bounds)
    ROS_INFO("initializing OMPL");
    ompl::base::SE2StateSpace* se2 = new ompl::base::SE2StateSpace();
    ompl::base::RealVectorBounds base_bounds(2);
    base_bounds.setLow(0,0);
    base_bounds.setHigh(0,7.2);//3
    base_bounds.setLow(1,0);
    base_bounds.setHigh(1,6.2);//3
    se2->setBounds(base_bounds);
    ompl::base::RealVectorStateSpace* r7 = new ompl::base::RealVectorStateSpace(9);
    r7->setDimensionName(0,"arms_x");
    r7->setDimensionName(1,"arms_y");
    r7->setDimensionName(2,"arms_z");
    r7->setDimensionName(3,"arms_roll");
    r7->setDimensionName(4,"arms_pitch");
    r7->setDimensionName(5,"arms_yaw");
    r7->setDimensionName(6,"free_angle_right");
    r7->setDimensionName(7,"free_angle_left");
    r7->setDimensionName(8,"torso");
    ompl::base::RealVectorBounds bounds(9);
    bounds.setLow(0,0.35);//arms_x
    bounds.setHigh(0,1.2);//arms_x
    bounds.setLow(1,-0.6);//arms_y
    bounds.setHigh(1,0.6);//arms_y
    bounds.setLow(2,-0.6);//arms_z
    bounds.setHigh(2,0.6);//arms_z

    // TODO may need to fix this!
    bounds.setLow(3,0);//arms_roll
    bounds.setHigh(3,2*M_PI);//arms_roll
    bounds.setLow(4,0);//arms_pitch
    bounds.setHigh(4,2*M_PI);//arms_pitch
    bounds.setLow(5,0);//arms_yaw
    bounds.setHigh(5,2*M_PI);//arms_yaw
    bounds.setLow(6,-3.75);//fa_right
    bounds.setHigh(6,0.65);//fa_right
    bounds.setLow(7,-0.65);//fa_left
    bounds.setHigh(7,3.75);//fa_left
    bounds.setLow(8,0); //torso
    bounds.setHigh(8,0.30); //torso
    r7->setBounds(bounds);
    ompl::base::StateSpacePtr se2_p(se2);
    ompl::base::StateSpacePtr r7_p(r7);
    fullBodySpace = r7_p + se2_p;
    
    //Define our SpaceInformation (combines the state space and collision checker)
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(fullBodySpace));
    omplFullBodyCollisionChecker* m_collision_checker = new omplFullBodyCollisionChecker(si);
    m_collision_checker->initialize(cspace);

    ompl::base::StateValidityChecker* temp2 = m_collision_checker;
    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(temp2));
    si->setStateValidityCheckingResolution(0.0009); // 0.1%
    si->setup();

    //Define a ProblemDefinition (a start/goal pair)
    pdef = new ompl::base::ProblemDefinition(si);

    planner = new ompl::geometric::RRTConnect(si);

    planner->setup();
    planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr(pdef));
    pathSimplifier = new ompl::geometric::PathSimplifier(si);
    ROS_INFO("finished initializing RRTConnect");
}

// given the start and goal from the request, create a start and goal that
// conform to the ompl types
bool OMPLPR2Planner::createStartGoal(FullState& ompl_start, FullState& ompl_goal, 
                                     NodeRequest& req){
    ROS_INFO("createStartGoal received a start of ");
    LeftContArmState left_arm_start(req.larm_start);
    RightContArmState right_arm_start(req.rarm_start);
    ContBaseState base_start = req.body_start;
    ContObjectState obj_state = right_arm_start.getObjectStateRelBody();

    (*(ompl_start->as<VectorState>(0)))[0] = obj_state.x();
    (*(ompl_start->as<VectorState>(0)))[1] = obj_state.y();
    (*(ompl_start->as<VectorState>(0)))[2] = obj_state.z();
    (*(ompl_start->as<VectorState>(0)))[3] = obj_state.roll();
    (*(ompl_start->as<VectorState>(0)))[4] = obj_state.pitch();
    (*(ompl_start->as<VectorState>(0)))[5] = obj_state.yaw();
    (*(ompl_start->as<VectorState>(0)))[6] = right_arm_start.getUpperArmRollAngle();
    (*(ompl_start->as<VectorState>(0)))[7] = left_arm_start.getUpperArmRollAngle();
    (*(ompl_start->as<VectorState>(0)))[8] = base_start.z();
    ompl_start->as<SE2State>(1)->setXY(base_start.x(),
                                       base_start.y());
    // may need to normalize the theta?
    ompl_start->as<SE2State>(1)->setYaw(base_start.theta());
    ROS_INFO("obj xyz (%f %f %f) base xytheta (%f %f %f)",
             obj_state.x(), obj_state.y(), obj_state.z(),
             base_start.x(), base_start.y(), base_start.theta());

    KDL::Frame goal_kdl;
    tf::PoseMsgToKDL(req.goal.pose, goal_kdl);
    ContBaseState base_goal = req.body_goal;
    double r,p,y;
    goal_kdl.M.GetRPY(r,p,y);

    (*(ompl_goal->as<VectorState>(0)))[0] = 0.560000;
    (*(ompl_goal->as<VectorState>(0)))[1] = -.2;
    (*(ompl_goal->as<VectorState>(0)))[2] = .18;
    (*(ompl_goal->as<VectorState>(0)))[3] = r;
    (*(ompl_goal->as<VectorState>(0)))[4] = p;
    (*(ompl_goal->as<VectorState>(0)))[5] = y;
    (*(ompl_goal->as<VectorState>(0)))[6] = 0;//req.rarm_goal[2];
    (*(ompl_goal->as<VectorState>(0)))[7] = 0;//req.larm_goal[2];
    (*(ompl_goal->as<VectorState>(0)))[8] = .1;
    ompl_goal->as<SE2State>(1)->setXY(6,5);
    // may need to normalize the theta?
    ompl_goal->as<SE2State>(1)->setYaw(0);
    
    //(*(ompl_goal->as<VectorState>(0)))[8] = base_goal.z();
    //ompl_goal->as<SE2State>(1)->setXY(base_goal.x(),
    //                                   base_goal.y());
    //// may need to normalize the theta?
    //ompl_goal->as<SE2State>(1)->setYaw(base_goal.theta());
    return true;
}

// takes in an ompl state and returns a proper robot state that represents the
// same state.
bool OMPLPR2Planner::convertFullState(ompl::base::State* state, RobotState& robot_state){
    ContObjectState obj_state;
    // fix the l_arm angles
    LeftContArmState l_arm;
    RightContArmState r_arm;
    ContBaseState base;
    const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);
    obj_state.x((*(s->as<VectorState>(0)))[0]);
    obj_state.y((*(s->as<VectorState>(0)))[1]);
    obj_state.z((*(s->as<VectorState>(0)))[2]);
    obj_state.roll((*(s->as<VectorState>(0)))[3]);
    obj_state.pitch((*(s->as<VectorState>(0)))[4]);
    obj_state.yaw((*(s->as<VectorState>(0)))[5]);
    r_arm.setUpperArmRoll((*(s->as<VectorState>(0)))[6]);
    l_arm.setUpperArmRoll((*(s->as<VectorState>(0)))[7]);
    base.z((*(s->as<VectorState>(0)))[8]);
    base.x(s->as<SE2State>(1)->getX());
    base.y(s->as<SE2State>(1)->getY());
    base.theta(s->as<SE2State>(1)->getYaw());

    RobotState seed_state(base, r_arm, l_arm);
    RobotPosePtr final_state;

    if (!RobotState::computeRobotPose(obj_state, seed_state, final_state))
        return false;

    robot_state = *final_state;
    return true;
}

bool OMPLPR2Planner::planPathCallback(monolithic_pr2_planner_node::GetMobileArmPlan::Request& req, monolithic_pr2_planner_node::GetMobileArmPlan::Response& res){
    ROS_INFO("running ompl planner!");
    FullState ompl_start(fullBodySpace);
    FullState ompl_goal(fullBodySpace);
    createStartGoal(ompl_start, ompl_goal, req);
    pdef->setStartAndGoalStates(ompl_start,ompl_goal);
    ompl::base::GoalState* temp_goal = new ompl::base::GoalState(planner->getSpaceInformation());
    temp_goal->setState(ompl_goal);
    ompl::base::GoalPtr temp_goal2(temp_goal);

    // something about different planner types here
    //if(planner_id_==2 || planner_id_==3){
        pdef->setGoal(temp_goal2);
    //}
    planner->solve(60.0);
    ompl::base::PathPtr path = planner->getProblemDefinition()->getSolutionPath();
    if (path){
        ROS_INFO("OMPL found a solution!");
        ompl::geometric::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);
        double t2 = ros::Time::now().toSec();
        bool b1 = pathSimplifier->reduceVertices(geo_path);
        bool b2 = pathSimplifier->collapseCloseVertices(geo_path);
        ROS_ERROR("reduce:%d collapse:%d\n",b1,b2);
        //bool b3 = pathSimplifier->shortcutPath(geo_path);
        //ROS_ERROR("shortcut:%d\n",b3);
        double t3 = ros::Time::now().toSec();
        //filename = "/tmp/ompl_stats_" + planner_string_ + ".csv";
        //FILE* stat_out = fopen(filename.c_str(),"a");
        //fprintf(stat_out,"%f, %f\n",t1-t0,t3-t2);
        //fclose(stat_out);
        geo_path.interpolate();
        ROS_INFO("path size of %lu", geo_path.getStateCount());
        for(unsigned int i=0; i<geo_path.getStateCount(); i++){
            ompl::base::State* state = geo_path.getState(i);
            RobotState robot_state;
            if (!convertFullState(state, robot_state))
                ROS_ERROR("ik failed on path reconstruction!");
            robot_state.visualize();
            usleep(10000);

        }
    }

    return true;
}
    //    // do something with the solution
    //    fstream path_stream;
    //    std::string filename = "/tmp/ompl_stats_" + planner_string_ + ".txt";
    //    path_stream.open(filename.c_str(),fstream::out | fstream::app);
    //    path->print(path_stream);
    //    path_stream.close();

    //    ompl::geometric::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);
    //    double t2 = ros::Time::now().toSec();
    //    bool b1 = pathSimplifier->reduceVertices(geo_path);
    //    bool b2 = pathSimplifier->collapseCloseVertices(geo_path);
    //    ROS_ERROR("reduce:%d collapse:%d\n",b1,b2);
    //    //bool b3 = pathSimplifier->shortcutPath(geo_path);
    //    //ROS_ERROR("shortcut:%d\n",b3);
    //    double t3 = ros::Time::now().toSec();
    //    filename = "/tmp/ompl_stats_" + planner_string_ + ".csv";
    //    FILE* stat_out = fopen(filename.c_str(),"a");
    //    fprintf(stat_out,"%f, %f\n",t1-t0,t3-t2);
    //    fclose(stat_out);
    //    geo_path.interpolate();

    //    char buf[64];
    //    filename = "/tmp/ompl_paths_" + planner_string_ + "/%.4d.csv";

    //    sprintf(buf,filename.c_str(),filenum_);
    //    filenum_++;
    //    FILE* traj_file = fopen(buf, "w");
    //    for(unsigned int i=0; i<geo_path.getStateCount(); i++){
    //        ompl::base::State* state = geo_path.getState(i);
    //        const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);

    //        vector<double> wpose(12,0);
    //        wpose[0] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1];
    //        wpose[1] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2];
    //        wpose[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3];
    //        wpose[3] = 0;
    //        wpose[4] = 0;
    //        wpose[5] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4];
    //        wpose[6] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6];
    //        wpose[7] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5];
    //        wpose[8] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getX();
    //        wpose[9] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getY();
    //        wpose[10] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0];
    //        wpose[11] = s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw();

    //        vector<double> arm0(7,0); //right arm angles
    //        vector<double> arm1(7,0); //left arm angles
    //        arm0[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6];
    //        arm1[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5];

    //        if(sbpl_arm_env_.convertWorldPoseToAngles(wpose, arm0, arm1, false)){
    //            ROS_DEBUG("IK is valid");
    //            BodyPose body;
    //            body.x = s->as<ompl::base::SE2StateSpace::StateType>(1)->getX();
    //            body.y = s->as<ompl::base::SE2StateSpace::StateType>(1)->getY();
    //            body.theta = s->as<ompl::base::SE2StateSpace::StateType>(1)->getYaw();
    //            body.z = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0];

    //            /*
    //               short unsigned int blah_x, blah_y, blah_z, blah_yaw;
    //               sbpl_arm_env_.computeObjectPose(body,arm0,blah_x,blah_y,blah_z,blah_yaw);
    //               geometry_msgs::Point p;
    //               sbpl_arm_env_.discToWorldXYZ(blah_x,blah_y,blah_z,p.x,p.y,p.z,true);
    //               marker.points.push_back(p);
    //               pviz_.visualizeRobot(arm0, arm1, body, i*color_inc, "waypoint_"+boost::lexical_cast<std::string>(i),i);
    //               usleep(3000);
    //               */

    //            fprintf(traj_file, "%0.4f, ", body.x);
    //            fprintf(traj_file, "%0.4f, ", body.y);
    //            fprintf(traj_file, "%0.4f, ", body.theta);
    //            fprintf(traj_file, "%0.4f, ", body.z);
    //            for(size_t j=0; j<arm0.size(); ++j)
    //                fprintf(traj_file, "%0.4f, ", arm0[j]);
    //            for(size_t j=0; j<arm1.size(); ++j)
    //                fprintf(traj_file, "%0.4f, ", arm1[j]);
    //            fprintf(traj_file,"\n");
    //        }
    //        else{
    //            ROS_ERROR("IK is not valid");
    //        }
    //    }
    //    fclose(traj_file);
    //    //marker_pub_.publish(marker);

    //    planner->getProblemDefinition()->clearSolutionPaths();

    //    if(planner_id_==0 || planner_id_==2 || planner_id_==3)
    //        planner->clear();
    //    else
    //        planner->as<ompl::geometric::PRM>()->clearQuery();
    //    return true;
    //}
