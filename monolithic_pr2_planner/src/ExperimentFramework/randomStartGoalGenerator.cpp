#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/ExperimentFramework/randomStartGoalGenerator.h>

using namespace monolithic_pr2_planner;

StartGoalGenerator::StartGoalGenerator(monolithic_pr2_planner::CSpaceMgrPtr cspace):
    m_cspace(cspace){}

RobotState StartGoalGenerator::generateRandomState(Region* region_ptr){
    bool foundState = false;
    RobotPosePtr final_state;
    while (!foundState){
        ContObjectState obj_state;
        ContBaseState base;
        base.z(randomDouble(0.0, 0.3));
        obj_state.x(randomDouble(0.35, 1.2));
        obj_state.y(randomDouble(-0.6, 0.6));
        obj_state.z(randomDouble(-0.6, 0.6));

        // TODO find some better values here
        obj_state.roll(0);
        obj_state.pitch(randomDouble(0, 1.5));
        obj_state.yaw(randomDouble(-1.396, 1.396));

        //ROS_INFO("random object state is");
        //obj_state.printToDebug(HEUR_LOG);

        RightContArmState r_arm;
        r_arm.setUpperArmRoll(randomDouble(-3.75, .65));
        
        LeftContArmState l_arm;
        l_arm.setShoulderPan(0.038946287971107774);
        l_arm.setShoulderLift(1.2146697069025374);
        l_arm.setUpperArmRoll(1.3963556492780154);
        l_arm.setElbowFlex(-1.1972269899800325);
        l_arm.setForearmRoll(-4.616317135720829);
        l_arm.setWristFlex(-0.9887266887318599);
        l_arm.setWristRoll(1.1755681069775656);

        // if region_id = -1, then we do a random sample across the entire state space
        double x_lower_bound = 0;
        double x_upper_bound = X_MAX;
        double y_lower_bound = 0;
        double y_upper_bound = Y_MAX;

        if (region_ptr != NULL){
            Region region = *region_ptr;
            double padding = 0.0;
            if(region_ptr != &m_start_region)
                padding = 1.0;
            else
                padding = 0.0;
            x_lower_bound = (region.x_min-padding < 0) ? 0 : region.x_min-padding;
            x_upper_bound = (region.x_max+padding > X_MAX) ? X_MAX : region.x_max+padding;
            y_lower_bound = (region.y_min-padding < 0) ? 0 : region.y_min-padding;
            y_upper_bound = (region.y_max+padding > Y_MAX) ? Y_MAX : region.y_max+padding;
        }
        base.x(randomDouble(x_lower_bound, x_upper_bound));
        base.y(randomDouble(y_lower_bound, y_upper_bound));
        base.theta(randomDouble(-M_PI, M_PI));
        
        vector<double> init_l_arm(7,0);
        init_l_arm[0] = (0.038946287971107774);
        init_l_arm[1] = (1.2146697069025374);
        init_l_arm[2] = (1.3963556492780154);
        init_l_arm[3] = -1.1972269899800325;
        init_l_arm[4] = (-4.616317135720829);
        init_l_arm[5] = -0.9887266887318599;
        init_l_arm[6] = 1.1755681069775656;
        LeftContArmState init_l_arm_v(init_l_arm);
        RobotState seed_state(base, r_arm, init_l_arm_v);

        //ROS_INFO("printing seed state");
        //seed_state.printToInfo(HEUR_LOG);

        foundState = RobotState::computeRobotPose(DiscObjectState(obj_state), seed_state, 
                                                  final_state);
        if (!foundState){
            //ROS_ERROR("ik failed, trying again");
        } else {
            //ROS_INFO("found a state");
            //final_state->printToInfo(HEUR_LOG);
        }
    }
    return *final_state;
}

// generates a bunch of states while making sure they're not in collision and
// they are within user defined m_goal_regions.
bool StartGoalGenerator::generateRandomValidState(RobotState& generated_state,
                                                  int region_id, bool
                                                  is_start_state){
    int counter = 0;
    while(1){
        counter++;
        if (counter % 1000 == 0)
            ROS_INFO("up to iteration %d while searching for valid state", counter);

        if(region_id==-1)  //if -1, we sample uniformly
            generated_state = generateRandomState(NULL);
        else if(is_start_state)  //If we're generating a start state within region bounds
            generated_state = generateRandomState(&m_start_region);
        else{
            generated_state = generateRandomState(&m_goal_regions[region_id]);
            generated_state.visualize();
        }

        if (m_cspace->isValid(generated_state)){
            ContObjectState obj = generated_state.getObjectStateRelMap();

            //iterate through our desired m_goal_regions
            //if the object location is in one of our m_goal_regions then we found a valid state
            if (region_id != -1 && !is_start_state){
                Region region = m_goal_regions[region_id];
                bool isWithinRegion = (obj.x() >= region.x_min && obj.x() <= region.x_max &&
                        obj.y() >= region.y_min && obj.y() <= region.y_max &&
                        obj.z() >= region.z_min && obj.z() <= region.z_max);
                if (isWithinRegion){
                    ROS_INFO("found one in region");
                    return true;
                }
            }
            else if (region_id!=-1 && is_start_state){
                // Nothing to check. Just return true.
                return true;
            }
            else if (region_id == -1){
            // else, uniform sampling across entire map
                if(obj.x() >= X_MIN && obj.x() <= X_MAX &&
                   obj.y() >= Y_MIN && obj.y() <= Y_MAX &&
                   obj.z() >= Z_MIN && obj.z() <= Z_MAX){
                    return true;
                } else {
                    ROS_ERROR("not in map");
                }
            }
        } else {
            // ROS_ERROR("not valid: In collision");
        }
    }
    // only reaches here if no state generated
    ROS_WARN("Couldn't find state for region %d", region_id);
    return false;
}

bool StartGoalGenerator::generateUniformPairs(int num_pairs, 
                                              vector<pair<RobotState, RobotState> >& pairs){
    // int counter = 0;
    for (int i=0; i < num_pairs; i++){
        ROS_INFO("generating pair %d", i);
        // counter++;
        int set_uniform_sampling = -1;
        if(!m_goal_regions.empty()){
            set_uniform_sampling = rand()%static_cast<int>(m_goal_regions.size());
        }
        RobotState start_state;
        RobotState goal_state;
        ROS_DEBUG_NAMED(HEUR_LOG, "generated the following start goal");
        // Will set start region if setting a goal region
        generateRandomValidState(start_state, set_uniform_sampling, true);
        generateRandomValidState(goal_state, set_uniform_sampling);
        pairs.push_back(pair<RobotState, RobotState>(start_state, goal_state));
    }
    return true;
}

/*
void omplFullBodyCollisionChecker::readFile(char filename[], std::vector<std::pair<State, State> >& pairs){
    std::ifstream file(filename);
    std::string line;
    while(std::getline(file, line)){
        istringstream iss(line);
        std::pair<State, State> point;
        vector<float> pts;
        float value;
        while (iss >> value){
            pts.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
        point.first.torso = pts[0];
        point.first.arm_x = pts[1];
        point.first.arm_y = pts[2];
        point.first.arm_z = pts[3];
        point.first.arm_yaw = pts[4];
        point.first.free_angle_left = pts[5];
        point.first.free_angle_right = pts[6];
        point.first.base_x = pts[7];
        point.first.base_y = pts[8];
        point.first.base_yaw = pts[9];
        point.second.torso = pts[10];
        point.second.arm_x = pts[11];
        point.second.arm_y = pts[12];
        point.second.arm_z = pts[13];
        point.second.arm_yaw = pts[14];
        point.second.free_angle_left = pts[15];
        point.second.free_angle_right = pts[16];
        point.second.base_x = pts[17];
        point.second.base_y = pts[18];
        point.second.base_yaw = pts[19];
        pairs.push_back(point);
    }
}
*/

void StartGoalGenerator::initializeRegions(){
    // Initialize from Param server.
    ros::NodeHandle nh;
    int number_of_regions;
    nh.getParam("/monolithic_pr2_planner_node/experiments/number_of_regions",
        number_of_regions);
    ROS_INFO("[randomStartGoalGenerator] Initializing %d regions", number_of_regions);
    for (int i = 0; i < number_of_regions; ++i)
    {
        Region region;
        double X, Y, Z, dimX, dimY, dimZ;
        nh.getParam("/monolithic_pr2_planner_node/experiments/goal_region_x_" + std::to_string(i), X);
        nh.getParam("/monolithic_pr2_planner_node/experiments/goal_region_y_" + std::to_string(i), Y);
        nh.getParam("/monolithic_pr2_planner_node/experiments/goal_region_z_" + std::to_string(i), Z);
        nh.getParam("/monolithic_pr2_planner_node/experiments/goal_region_dimx_" + std::to_string(i),
            dimX);
        nh.getParam("/monolithic_pr2_planner_node/experiments/goal_region_dimy_" + std::to_string(i),
            dimY);
        nh.getParam("/monolithic_pr2_planner_node/experiments/goal_region_dimz_" + std::to_string(i),
            dimZ);
        
        region.x_min = X;
        region.y_min = Y;
        region.z_min = dimZ;    //We want to goal to be *above* the table

        region.x_max = region.x_min + dimX;
        region.y_max = region.y_min + dimY;
        region.z_max = region.z_min + 0.4;  //Defined by the obstacle size
        m_goal_regions.push_back(region);
    }

    // Start region : Only one for now.
    double X, Y, Z, dimX, dimY, dimZ;
    nh.getParam("/monolithic_pr2_planner_node/experiments/start_region_x", X);
    nh.getParam("/monolithic_pr2_planner_node/experiments/start_region_y", Y);
    nh.getParam("/monolithic_pr2_planner_node/experiments/start_region_z", Z);
    nh.getParam("/monolithic_pr2_planner_node/experiments/start_region_dimx",
        dimX);
    nh.getParam("/monolithic_pr2_planner_node/experiments/start_region_dimy",
        dimY);
    nh.getParam("/monolithic_pr2_planner_node/experiments/start_region_dimz",
        dimZ);
    m_start_region.x_min = X;
    m_start_region.y_min = Y;
    m_start_region.z_min = Z;    //We want to goal to be *above* the table

    m_start_region.x_max = m_start_region.x_min + dimX;
    m_start_region.y_max = m_start_region.y_min + dimY;
    m_start_region.z_max = m_start_region.z_min;  //Defined by the obstacle size
}