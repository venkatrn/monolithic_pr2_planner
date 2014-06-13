#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>

using namespace monolithic_pr2_planner;

bool DiscObjectState::operator==(const DiscObjectState& other){
    return m_coord == other.m_coord;
}

bool DiscObjectState::operator!=(const DiscObjectState& other){
    return !(*this == other);
}

DiscObjectState::DiscObjectState():m_coord(6,0){}

DiscObjectState::DiscObjectState(int x, int y, 
                                 int z, int roll, 
                                 int pitch, int yaw) : m_coord(6){
    m_coord[ObjectPose::X] = x; 
    m_coord[ObjectPose::Y] = y; 
    m_coord[ObjectPose::Z] = z; 
    m_coord[ObjectPose::ROLL] = roll;
    m_coord[ObjectPose::PITCH] = pitch;
    m_coord[ObjectPose::YAW] = yaw;
}

DiscObjectState::DiscObjectState(ContObjectState obj_state): m_coord(6){
    int x, y, z;
    m_occupancy_grid->worldToGrid(obj_state.x(), 
                        obj_state.y(), 
                        obj_state.z(),
                        x, y, z);
    double rpy_res = m_resolution_params.obj_rpy_resolution;
    double roll = static_cast<int>((normalize_angle_positive(obj_state.roll() + rpy_res*0.5))/rpy_res);
    double pitch = static_cast<int>((normalize_angle_positive(obj_state.pitch() + rpy_res*0.5))/rpy_res);
    double yaw = static_cast<int>((normalize_angle_positive(obj_state.yaw() + rpy_res*0.5))/rpy_res);
    m_coord[ObjectPose::X] = x; 
    m_coord[ObjectPose::Y] = y; 
    m_coord[ObjectPose::Z] = z; 
    m_coord[ObjectPose::ROLL] = roll;
    m_coord[ObjectPose::PITCH] = pitch;
    m_coord[ObjectPose::YAW] = yaw;
}

ContObjectState DiscObjectState::getContObjectState() const {
    return ContObjectState(*this);
}

void DiscObjectState::printToInfo(char* log_level){
    ROS_INFO_NAMED(log_level, "object state");
    ROS_INFO_NAMED(log_level, "\t%d %d %d %d %d %d",
                   m_coord[ObjectPose::X],
                   m_coord[ObjectPose::Y],
                   m_coord[ObjectPose::Z],
                   m_coord[ObjectPose::ROLL],
                   m_coord[ObjectPose::PITCH],
                   m_coord[ObjectPose::YAW]);
}

void DiscObjectState::printToDebug(char* log_level){
    ROS_DEBUG_NAMED(log_level, "object state");
    ROS_DEBUG_NAMED(log_level, "\t%d %d %d %d %d %d",
                    m_coord[ObjectPose::X],
                    m_coord[ObjectPose::Y],
                    m_coord[ObjectPose::Z],
                    m_coord[ObjectPose::ROLL],
                    m_coord[ObjectPose::PITCH],
                    m_coord[ObjectPose::YAW]);
}

void DiscObjectState::visualize(std::string name) {
    std::vector <double> pose;
    pose.assign(m_coord.begin(), m_coord.end());
    Visualizer::pviz->visualizePose(pose, name);
}