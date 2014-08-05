#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/TransitionData.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/Constants.h>
#include <assert.h>

namespace monolithic_pr2_planner {
    /*! \brief Base class for motion primitives. Motion primitives contain user
     * defined motions for generating successors in a search. 
     */
    class MotionPrimitive {
        public:
            MotionPrimitive();
            void setID(int id) { m_id = id; };
            int getID() const { return m_id; };
            virtual void setIntermSteps(IntermSteps& coord) { m_interm_steps = coord; };
            virtual IntermSteps getIntermSteps(){ return m_interm_steps; };
            virtual void setEndCoord(GraphStateMotion& coord); 
            virtual bool apply(const GraphState& graph_state, 
                               GraphStatePtr& successor,
                               TransitionData& t_data) = 0;
            virtual void print() const = 0;
            virtual int motion_type() const = 0;
            virtual void computeCost(const MotionPrimitiveParams& params) = 0;
            virtual void printIntermSteps() const;
            virtual void printEndCoord() const;
            virtual int cost() const { return m_cost; };
            virtual void setAdditionalCostMult(double cost) { m_additional_cost = cost; };
            virtual int getAdditionalCostMult() { return m_additional_cost; };

            GraphStateMotion getEndCoord(){ return m_end_coord; };

        protected:
            double dist(DiscObjectState s1, DiscObjectState s2);
            int m_id;
            int m_cost;
            int m_additional_cost;
            GraphStateMotion m_end_coord;
            IntermSteps m_interm_steps;

    };
    typedef boost::shared_ptr<MotionPrimitive> MotionPrimitivePtr;
}
