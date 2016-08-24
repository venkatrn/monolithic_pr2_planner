#pragma once
#include <ros/ros.h>
#include <esp_planner/esp_environment.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <stdexcept>
#include <vector>
#include <memory>

#define NUM_SMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
#define NUM_IMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
// This should include the Anchor search -> Total number of searches.

namespace monolithic_pr2_planner {
    /*! \brief Implements a complete environment used by the SBPL planner.
     * Contains everything from managing state IDs to collision space
     * information.
     */
    typedef std::pair<int, int> Edge;
    class Environment : public EnvironmentESP {
        public:
            Environment(ros::NodeHandle nh);
            CSpaceMgrPtr getCollisionSpace(){ return m_cspace_mgr; };
            HeuristicMgrPtr getHeuristicMgr(){ return m_heur_mgr; };
            bool configureRequest(SearchRequestParamsPtr search_request_params,
                                  int& start_id, int& goal_id);
            void GetSuccs(int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs);
            void GetSuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs);

            void GetLazySuccs(int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs, std::vector<bool>* isTrueCost);
            void GetLazySuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs, std::vector<bool>* isTrueCost);

            int GetTrueCost(int parentID, int childID);
            std::vector<FullBodyState> reconstructPath(std::vector<int> 
                state_ids);
            void reset();
            void setPlannerType(int planner_type);
            void setUseNewHeuristics(bool use_new_heuristics){m_use_new_heuristics = use_new_heuristics;};

            // ESP-specific.
            // Return a list of successors for parent_id, along with associated
            // edge costs, probability of edge existence, and time it would
            // take to evaluate the edge (if the edge is not a deterministic one).
             void GetSuccs(int parent_id, std::vector<int> *succ_ids,
                        std::vector<int> *costs, std::vector<double> *edge_probabilites,
                        std::vector<double> *edge_eval_times,
                        std::vector<int> *edge_groups = nullptr) override;
            int GetGoalHeuristic(int state_id) override;
            bool EvaluateEdge(int parent_id, int child_id) override;


        protected:
            bool setStartGoal(SearchRequestPtr search_request, 
                              int& start_id, int& goal_id);
            int saveFakeGoalState(const GraphStatePtr& graph_state);
            void configurePlanningDomain();
            void configureQuerySpecificParams(SearchRequestPtr search_request);
            void generateStartState(SearchRequestPtr search_request);

            ParameterCatalog m_param_catalog;
            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
            ros::NodeHandle m_nodehandle;
            GoalStatePtr m_goal;
            bool m_using_lazy;
            MotionPrimitivesMgr m_mprims;
            HeuristicMgrPtr m_heur_mgr;

            int m_planner_type;
            bool m_use_new_heuristics;
            std::map<Edge, int> m_true_cost_cache;

        // SBPL interface stuff
        public:
            bool InitializeEnv(const char* sEnvFile){return false;};
            bool InitializeMDPCfg(MDPConfig *MDPCfg){ return true; };
            int  GetFromToHeuristic(int FromStateID, int ToStateID){ throw std::runtime_error("unimplement");  };
            int  GetGoalHeuristic(int heuristic_id, int stateID);
            int  GetStartHeuristic(int stateID) { throw std::runtime_error("unimplement"); };
            void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){};
            void SetAllActionsandAllOutcomes(CMDPSTATE* state){};
            void SetAllPreds(CMDPSTATE* state){};
            int  SizeofCreatedEnv(){ return m_hash_mgr->size(); };
            void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL){};
            void PrintEnv_Config(FILE* fOut){};
            std::map<Edge, MotionPrimitivePtr> m_edges;
    };
}
