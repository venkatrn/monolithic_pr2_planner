#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/filesystem.hpp>
#include <log4cxx/logger.h>

using namespace boost::filesystem;
using namespace monolithic_pr2_planner;
using namespace std;


ParameterCatalog::ParameterCatalog() {
}

void ParameterCatalog::fetch(ros::NodeHandle nh){
    ROS_INFO_NAMED(CONFIG_LOG, "fetching parameters from namespace %s", 
                                nh.getNamespace().c_str());
    m_nodehandle = nh;
    // TODO clean this up, setmotionprimitive needs to be run before parse
    // stuff!
    setMotionPrimitiveParams(m_motion_primitive_params);
    setRobotResolutionParams(m_motion_primitive_params, 
                             m_robot_resolution_params);
    setOccupancyGridParams(m_occupancy_grid_params);
    setLeftArmParams(m_left_arm_params);
    setRightArmParams(m_right_arm_params);
    setVisualizationParams(m_visualization_params);
    
}

void ParameterCatalog::setMotionPrimitiveParams(MotionPrimitiveParams& params){
    setFileNameFromParamServer("planner/motion_primitive_file", 
            &params.arm_motion_primitive_file);
    setFileNameFromParamServer("planner/base_motion_primitive_file", 
            &params.base_motion_primitive_file);
    m_nodehandle.param("planner/nominalvel_mpersecs", params.nominal_vel, 
             0.5);
    m_nodehandle.param("planner/timetoturn45degsinplace_secs",
             params.angular_vel,
             0.5);
    params.angular_vel = 1/(params.angular_vel/45 * 180/M_PI); // rad/s
}

void ParameterCatalog::setLeftArmParams(ArmDescriptionParams& params){
    m_nodehandle.param("collision_space/resolution", params.env_resolution, 0.02);
    setFileNameFromParamServer("planner/left_arm_description_file", 
            &params.arm_file);
    std::string robot_urdf_param;
    if(!m_nodehandle.searchParam("robot_description",robot_urdf_param)){
        ROS_ERROR_NAMED(CONFIG_LOG, "Can't find description on param server "
                                "(/robot_description not set).");
    } else {
        m_nodehandle.param<std::string>(robot_urdf_param, params.robot_description_string, 
                                        "robot_description");
    }
}

void ParameterCatalog::setRightArmParams(ArmDescriptionParams& params){
    m_nodehandle.param("collision_space/resolution", params.env_resolution, 0.02);
    setFileNameFromParamServer("planner/right_arm_description_file", 
            &params.arm_file);
    std::string robot_urdf_param;
    if(!m_nodehandle.searchParam("robot_description",robot_urdf_param)){
        ROS_ERROR_NAMED(CONFIG_LOG, "Can't find description on param server "
                                "(/robot_description not set).");
    } else {
        m_nodehandle.param<std::string>(robot_urdf_param, params.robot_description_string, 
                                        "robot_description");
    }
}

void ParameterCatalog::setOccupancyGridParams(OccupancyGridParams& params){
    m_nodehandle.param("collision_space/resolution", params.env_resolution, 0.02);
    m_nodehandle.param("collision_space/reference_frame", params.reference_frame, 
                            std::string("base_link"));
    m_nodehandle.param("collision_space/occupancy_grid/origin_x", params.origin.x,-0.6);
    m_nodehandle.param("collision_space/occupancy_grid/origin_y", params.origin.y,-1.15);
    m_nodehandle.param("collision_space/occupancy_grid/origin_z", params.origin.z,-0.05);
    m_nodehandle.param("collision_space/occupancy_grid/size_x", params.max_point.x,1.6);
    m_nodehandle.param("collision_space/occupancy_grid/size_y", params.max_point.y,1.8);
    m_nodehandle.param("collision_space/occupancy_grid/size_z", params.max_point.z,1.4);
    
    ROS_DEBUG_NAMED(CONFIG_LOG, "Occupancy grid parameter");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tsize: %f %f %f ",
                          params.max_point.x, 
                          params.max_point.y,
                          params.max_point.z);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tresolution: %f ",
                          params.env_resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\torigin: %f %f %f",  
                          params.origin.x, 
                          params.origin.y,
                          params.origin.z);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\treference frame: %s", 
                    params.reference_frame.c_str());
}

void ParameterCatalog::setVisualizationParams(VisualizationParams& params){
    m_nodehandle.param("visualizations/expansions", params.expansions, false);
    m_nodehandle.param("visualizations/final_path", params.final_path, false);
    
    ROS_DEBUG_NAMED(CONFIG_LOG, "Setting VisualizationParams");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\texpansions: %d", 
                    params.expansions);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tfinal_path: %d", 
                    params.final_path);
}

// currently just hard code these...maybe someone will want to retrieve them
// from param server at some point.
void ParameterCatalog::setRobotResolutionParams(const MotionPrimitiveParams& mprims,
                                                RobotResolutionParams& params){
    parseArmMPrimFileHeader(mprims.arm_motion_primitive_file, params);
    parseBaseMPrimFileHeader(mprims.base_motion_primitive_file, params);
    m_nodehandle.param("planner/gripper_sphere_radius/", params.gripper_sphere_radius,0.08);
    ROS_DEBUG_NAMED(CONFIG_LOG, "gripper sphere is %f", params.gripper_sphere_radius);
    //params.obj_xyz_resolution = 0.02;
    //params.obj_rpy_resolution = 2*M_PI/180;
    //params.arm_free_angle_resolution = 3*M_PI/180;
    //params.base_theta_resolution = 22.5*M_PI/180;
}


void ParameterCatalog::getNextLine(ifstream& file, stringstream& ss, 
                                   string& line){
    getline(file, line);
    ss.str(line);
    ss.clear();
}

void ParameterCatalog::parseArmMPrimFileHeader(const std::string& mprim_file,
                             RobotResolutionParams& params){
    ifstream file;
    file.open(mprim_file.c_str(), ios_base::in);
    string line, label;
    double dvalue;
    int ivalue;
    stringstream ss;

    getNextLine(file, ss, line);
    ss >> label >> ivalue;
    if (label == "degrees_of_freedom:"){
        ROS_DEBUG_NAMED(CONFIG_LOG, "DOF set to %d", ivalue);
    } 
    getNextLine(file, ss, line);
    ss >> label >> dvalue;
    if (label == "xyz_resolution(meters):"){
        params.obj_xyz_resolution = dvalue*M_PI/180;
        ROS_DEBUG_NAMED(CONFIG_LOG, "obj_xyz_resolution set to %f", 
                        params.obj_xyz_resolution);
    } 

    getNextLine(file, ss, line);
    ss >> label >> dvalue;
    if (label == "rpy_resolution(degrees):"){
        params.obj_rpy_resolution = dvalue*M_PI/180;
        ROS_DEBUG_NAMED(CONFIG_LOG, "obj_rpy_resolution set to %f", 
                        params.obj_rpy_resolution);
        params.num_rpy_angles = static_cast<int>(360 / dvalue + 0.5);
        ROS_DEBUG_NAMED(CONFIG_LOG, "obj_num_rpy_angles set to %d", 
                        params.num_rpy_angles);
    } 

    getNextLine(file, ss, line);
    ss >> label >> dvalue;
    if (label == "free_angle_resolution(degrees):"){
        params.arm_free_angle_resolution = dvalue*M_PI/180;
        ROS_DEBUG_NAMED(CONFIG_LOG, "free_angle_resolution set to %f", 
                        params.arm_free_angle_resolution);
        params.num_free_angle_angles = static_cast<int>(360 / dvalue + 0.5);
        ROS_DEBUG_NAMED(CONFIG_LOG, "num_free_angle_angles set to %d", 
                        params.num_free_angle_angles);
    } 
    file.close();
}

bool ParameterCatalog::parseBaseMPrimFileHeader(const std::string& mprim_file, 
                                                RobotResolutionParams& params){
    ifstream file;
    file.open(mprim_file.c_str(), ios_base::in);
    string line, label;
    int ivalue;
    stringstream ss;

    getNextLine(file, ss, line);
    getNextLine(file, ss, line);
    ss >> label >> ivalue;
    if (label == "numberofangles:"){
        params.num_base_angles = ivalue;
        params.base_theta_resolution = 2*M_PI/ivalue;

        ROS_DEBUG_NAMED(CONFIG_LOG, "number of base angles set to %d", 
                        params.num_base_angles);
        ROS_DEBUG_NAMED(CONFIG_LOG, "base_theta_resolution set to %f", 
                        params.base_theta_resolution);
    } 
    return true;
}

bool ParameterCatalog::setFileNameFromParamServer(const std::string param_name, 
                                                  std::string* parameter){
    std::string filename;
    m_nodehandle.param<std::string>(param_name, filename, "");
    path input_path(filename.c_str());
    if (exists(input_path)){
       *parameter = filename;
        ROS_DEBUG_NAMED(CONFIG_LOG, "Pulling in data from %s", filename.c_str());
    } else {
       *parameter = filename;
        ROS_ERROR_NAMED(CONFIG_LOG, "Failed to find file '%s' to load in parameters for %s", 
                        filename.c_str(), param_name.c_str());
        return false;
    }
    return true;
}
