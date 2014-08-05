#pragma once
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

namespace monolithic_pr2_planner {
    typedef std::vector<int> GraphStateMotion;
    class GraphState {
        public:
            GraphState(std::vector<double> cont_state);
            GraphState(RobotState robot_pose);
            GraphState(DiscObjectState obj_state, RobotState robot_pose);
            // equality of graphstates is defined as:
            //      same discrete base state
            //      same discrete object state
            //      same left and right arm discrete free angles
            bool operator==(const GraphState& other);
            bool operator!=(const GraphState& other);
            int id() const { return m_id; };
            void id(int id) { m_id = id; };
            RobotState robot_pose() const { return m_robot_pose; };

            void robot_pose(RobotState robot_state) { 
                m_robot_pose = robot_state; 
                updateStateFromRobotState(); 
            };

            bool applyMPrim(const GraphStateMotion& mprim);

            void printToInfo(char* logger) const;
            void printToDebug(char* logger) const ;
            void printContToDebug(char* logger) const ;
            std::vector<int> getCoords() { return m_coord; };
            std::vector<double> getContCoords();

            DiscObjectState getObjectStateRelMap() const;
            DiscObjectState getObjectStateRelBody() const;

            int obj_x(){ return m_coord[GraphStateElement::OBJ_X]; };
            int obj_y(){ return m_coord[GraphStateElement::OBJ_Y]; };
            int obj_z(){ return m_coord[GraphStateElement::OBJ_Z]; };
            int obj_roll(){ return m_coord[GraphStateElement::OBJ_ROLL]; };
            int obj_pitch(){ return m_coord[GraphStateElement::OBJ_PITCH]; };
            int obj_yaw(){ return m_coord[GraphStateElement::OBJ_YAW]; };
            int obj_right_fa(){ return m_coord[GraphStateElement::R_FA]; };
            int obj_left_fa(){ return m_coord[GraphStateElement::L_FA]; };
            int base_x(){return m_coord[GraphStateElement::BASE_X]; };
            int base_y(){return m_coord[GraphStateElement::BASE_Y]; };
            int base_z(){return m_coord[GraphStateElement::BASE_Z]; };
            int base_theta(){return m_coord[GraphStateElement::BASE_THETA]; };

            DiscObjectState getObjectStateRelMapFromState() ;
            bool updateRobotStateFromGraphState(double r_arm, double l_arm);

            void updateStateFromRobotState();
            void lazyApplyMPrim(const GraphStateMotion& mprim);
        private:
            int m_id;
            RobotState m_robot_pose;
            bool m_check_simple;
            std::vector<int> m_coord;
    };
    typedef boost::shared_ptr<GraphState> GraphStatePtr;
};
