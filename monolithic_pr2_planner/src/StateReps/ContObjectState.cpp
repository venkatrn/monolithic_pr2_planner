#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

using namespace monolithic_pr2_planner;
using namespace angles;

ContObjectState::ContObjectState():m_coord(6,0){};
ContObjectState::ContObjectState(double vx, double vy, double vz,
                                 double vroll, double vpitch, double vyaw):
    m_coord(6,0){
    m_coord[ObjectPose::X] = vx;
    m_coord[ObjectPose::Y] = vy;
    m_coord[ObjectPose::Z] = vz;
    roll(vroll);
    pitch(vpitch);
    yaw(vyaw);
}

ContObjectState::ContObjectState(DiscObjectState obj_state):
    m_coord(6,0){
        m_occupancy_grid->gridToWorld(obj_state.x(),
                                      obj_state.y(),
                                      obj_state.z(),
                                      m_coord[ObjectPose::X],
                                      m_coord[ObjectPose::Y],
                                      m_coord[ObjectPose::Z]);
        double rpy_res = m_resolution_params.obj_rpy_resolution;
        roll(obj_state.roll()*rpy_res);
        pitch(obj_state.pitch()*rpy_res);
        yaw(obj_state.yaw()*rpy_res);
}

ContObjectState::ContObjectState(const geometry_msgs::PoseStamped& obj_pose):
    m_coord(6,0){
    tf::Pose pose;
    tf::poseMsgToTF(obj_pose.pose, pose);
    double vroll, vpitch, vyaw;
    pose.getBasis().getRPY(vroll, vpitch, vyaw);
    m_coord[ObjectPose::X] = obj_pose.pose.position.x;
    m_coord[ObjectPose::Y] = obj_pose.pose.position.y; 
    m_coord[ObjectPose::Z] = obj_pose.pose.position.z; 
    roll(vroll);
    pitch(vpitch);
    yaw(vyaw);
}

DiscObjectState ContObjectState::getDiscObjectState(){
    return ContObjectState(*this); 
}

void ContObjectState::printToInfo(char* log_level) const {
    ROS_INFO_NAMED(log_level, "object state");
    ROS_INFO_NAMED(log_level, "\t%f %f %f %f %f %f",
                   m_coord[ObjectPose::X],
                   m_coord[ObjectPose::Y],
                   m_coord[ObjectPose::Z],
                   m_coord[ObjectPose::ROLL],
                   m_coord[ObjectPose::PITCH],
                   m_coord[ObjectPose::YAW]);
}

void ContObjectState::printToDebug(char* log_level) const {
    ROS_DEBUG_NAMED(log_level, "object state");
    ROS_DEBUG_NAMED(log_level, "\t%f %f %f %f %f %f",
                    m_coord[ObjectPose::X],
                    m_coord[ObjectPose::Y],
                    m_coord[ObjectPose::Z],
                    m_coord[ObjectPose::ROLL],
                    m_coord[ObjectPose::PITCH],
                    m_coord[ObjectPose::YAW]);
}


// returns <num_steps> number of interpolated points, with the start and end
// included in this count. That's why we subtract 1 from the input number.
vector<ContObjectState> ContObjectState::interpolate(const ContObjectState& start,
                                                     const ContObjectState& end,
                                                     int num_steps){
    vector<ContObjectState> interp_steps;
    if (num_steps < 2){
        interp_steps.push_back(start);
        interp_steps.push_back(end);
        return interp_steps;
    }
    num_steps--;
    double step = 1/static_cast<double>(num_steps);
    double dx = end.x()-start.x();
    double dy = end.y()-start.y();
    double dz = end.z()-start.z();
    double droll = shortest_angular_distance(start.roll(), end.roll());
    double dpitch = shortest_angular_distance(start.pitch(), end.pitch());
    double dyaw = shortest_angular_distance(start.yaw(), end.yaw());

    for (int i=0; i <= num_steps; i++){
        ContObjectState state(start.x() + i*dx*step,
                              start.y() + i*dy*step,
                              start.z() + i*dz*step,
                              start.roll() + i*droll*step,
                              start.pitch() + i*dpitch*step,
                              start.yaw() + i*dyaw*step);

        interp_steps.push_back(state);
    }
    return interp_steps;
}


double ContObjectState::distance(const ContObjectState& start, const ContObjectState& end){
    double dX = end.x() - start.x();
    double dY = end.y() - start.y();
    double dZ = end.z() - start.z();
    return pow((pow(dX,2) + pow(dY,2) + pow(dZ,2)),.5);
}
