#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <angles/angles.h>
using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

bool ContBaseState::operator==(const ContBaseState& other) const {
    return (m_pose == other.m_pose);
}

bool ContBaseState::operator!=(const ContBaseState& other) const {
    return !(*this == other);
}

ContBaseState::ContBaseState():m_pose(4,0){};

ContBaseState::ContBaseState(double vx, double vy, double vz, double vtheta):
    m_pose(4){
    x(vx); y(vy); z(vz); theta(vtheta);
}

ContBaseState::ContBaseState(vector<double> base_pose) : 
    m_pose(base_pose){
}
ContBaseState::ContBaseState(const DiscBaseState& base_pose) :
    m_pose(4){
    m_occupancy_grid->gridToWorld(base_pose.x(),
                                  base_pose.y(),
                                  base_pose.z(),
                                  m_pose[BodyDOF::X],
                                  m_pose[BodyDOF::Y],
                                  m_pose[BodyDOF::Z]);
    double theta_res = m_resolution_params.base_theta_resolution;
    m_pose[BodyDOF::THETA] = normalize_angle_positive(static_cast<double>(base_pose.theta())*theta_res);
}

ContBaseState::ContBaseState(const BodyPose body_pose) :
    m_pose(4) {
        x(body_pose.x);
        y(body_pose.y);
        z(body_pose.z);
        theta(body_pose.theta);
}

// returns <num_steps> number of interpolated points, with the start and end
// included in this count. That's why we subtract 1 from the input number.
vector<ContBaseState> ContBaseState::interpolate(const ContBaseState& start, 
                                                 const ContBaseState& end,
                                                 int num_steps){
    vector<ContBaseState> interp_steps;
    if (num_steps < 2){
        interp_steps.push_back(start);
        interp_steps.push_back(end);
        return interp_steps;
    }
    num_steps--;
    double dX = end.x() - start.x();
    double dY = end.y() - start.y();
    double dZ = end.z() - start.z();
    double dTheta = angles::shortest_angular_distance(start.theta(), end.theta());
    double step_mult = 1/static_cast<double>(num_steps);

    for (int i=0; i <= num_steps; i++){
        ContBaseState state(start.x() + i*dX*step_mult,
                            start.y() + i*dY*step_mult,
                            start.z() + i*dZ*step_mult,
                            start.theta() + i*dTheta*step_mult);
        interp_steps.push_back(state);
    }
    return interp_steps;
}

double ContBaseState::distance(const ContBaseState& start, const ContBaseState& end){
    double dX = end.x() - start.x();
    double dY = end.y() - start.y();
    double dZ = end.z() - start.z();
    return pow((pow(dX,2) + pow(dY,2) + pow(dZ,2)),.5);
}

void ContBaseState::printToDebug(char* logger){
    ROS_DEBUG_NAMED(logger, "%f %f %f %f", x(),y(),z(),theta());
}

BodyPose ContBaseState::body_pose() const {
    BodyPose bp;
    bp.x = x();
    bp.y = y();
    bp.z = z();
    bp.theta = theta();
    return bp;
}
