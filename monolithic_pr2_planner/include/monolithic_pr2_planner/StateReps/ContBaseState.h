#pragma once
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <vector>
namespace monolithic_pr2_planner {
    class DiscBaseState;
    /*! \brief The ContinuousBaseState object represents the location of the PR2
     * base in the continuous coordinates of the map. Also implements an
     * interpolation function for the base.
     */
    class ContBaseState : public OccupancyGridUser {
        public:
            ContBaseState();
            ContBaseState(double x, double y, double z, double theta);
            ContBaseState(std::vector<double> base_pose);
            ContBaseState(const BodyPose body_pose);
            ContBaseState(const DiscBaseState& base_pose);


            bool operator==(const ContBaseState& other) const ;
            bool operator!=(const ContBaseState& other) const ;

            void getValues(std::vector<double>* values) { *values = m_pose; };
            double x() const { return m_pose[BodyDOF::X]; };
            double y() const { return m_pose[BodyDOF::Y]; };
            double z() const { return m_pose[BodyDOF::Z]; };
            double theta() const { return m_pose[BodyDOF::THETA]; };

            void x(double x) { m_pose[BodyDOF::X] = x; };
            void y(double y) { m_pose[BodyDOF::Y] = y; };
            void z(double z) { m_pose[BodyDOF::Z] = z; };
            void theta(double theta) { m_pose[BodyDOF::THETA] = normalize_angle_positive(theta); };

            DiscBaseState getDiscBaseState();
            BodyPose body_pose() const;
            void printToDebug(char* logger);
            static double getThetaResolution(){ return m_resolution_params.base_theta_resolution; };
            static double getXYZResolution(){ return m_occupancy_grid->getResolution(); };

            static std::vector<ContBaseState> interpolate(const ContBaseState& start, 
                                                          const ContBaseState& end, 
                                                          int num_steps);
            static double distance(const ContBaseState& start, const ContBaseState& end);
        private:
            std::vector<double> m_pose;
    };
}
