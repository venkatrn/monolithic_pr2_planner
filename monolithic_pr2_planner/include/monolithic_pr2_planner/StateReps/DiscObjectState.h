#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>

namespace monolithic_pr2_planner {
    class ContObjectState;
    class DiscObjectState : public OccupancyGridUser {
        public:
            bool operator==(const DiscObjectState& other);
            bool operator!=(const DiscObjectState& other);
            DiscObjectState();
            DiscObjectState(ContObjectState obj_state);
            DiscObjectState(int x, int y, int z, int roll, int pitch, int yaw);
            std::vector<unsigned int>::const_iterator getCoordBegin(){ return m_coord.begin(); };
            std::vector<unsigned int>::const_iterator getCoordEnd(){ return m_coord.end(); };

            int x() const { return m_coord[ObjectPose::X]; };
            int y() const { return m_coord[ObjectPose::Y]; };
            int z() const { return m_coord[ObjectPose::Z]; };
            int roll() const { return m_coord[ObjectPose::ROLL]; };
            int pitch() const { return m_coord[ObjectPose::PITCH]; };
            int yaw() const { return m_coord[ObjectPose::YAW]; };

            void x(int value) { m_coord[ObjectPose::X] = value; };
            void y(int value) { m_coord[ObjectPose::Y] = value; };
            void z(int value) { m_coord[ObjectPose::Z] = value; };
            void roll(int value) { m_coord[ObjectPose::ROLL] = normalizeRPY(value); };
            void pitch(int value) { m_coord[ObjectPose::PITCH] = normalizeRPY(value); };
            void yaw(int value) { m_coord[ObjectPose::YAW] = normalizeRPY(value); };


            ContObjectState getContObjectState() const;

            void printToInfo(char* log_level);
            void printToDebug(char* log_level);

            void visualize(std::string name);
            
        private:
            inline int normalizeRPY(int theta){
                int num_thetas = m_resolution_params.num_rpy_angles;
                if (theta >=0){
                    return (theta % num_thetas);
                } else {
                    return (theta % num_thetas + num_thetas) % num_thetas;
                }
            }
            std::vector<unsigned int> m_coord;
    };
}
