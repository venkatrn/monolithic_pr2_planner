#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <boost/foreach.hpp>
#include <stdexcept>
#include <assert.h>

#define HASH_TABLE_SIZE (32*1024)
#define NUMOFINDICES_STATEID2IND 2
using namespace monolithic_pr2_planner;

HashManager::HashManager(std::vector<int*>* stateID2Mapping) : 
    m_stateID2Mapping(stateID2Mapping),
    m_coord_to_state_id_table(HASH_TABLE_SIZE){
}

// Max's magic function for generating good hashes
unsigned int HashManager::intHash(unsigned int key){
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4); 
    key ^= (key >> 9); 
    key += (key << 10);
    key ^= (key >> 2); 
    key += (key << 7); 
    key ^= (key >> 12);
    return key;
}


unsigned int HashManager::hash(const GraphStatePtr& graph_state){
    int val = 0;
    int counter = 0;
    auto obj_state = graph_state->getObjectStateRelBody();
    for (auto it=obj_state.getCoordBegin(); it!=obj_state.getCoordEnd();++it){
        val += intHash(*it) << counter;
        counter++;
    }

    auto robot_pose = graph_state->robot_pose();
    auto base_state = robot_pose.base_state();
    for (auto it=base_state.getCoordBegin(); it!=base_state.getCoordEnd();++it){
        val += intHash(*it) << counter;
        counter++;
    }

    val += intHash(robot_pose.right_free_angle()) << counter;
    counter++;

    val += intHash(robot_pose.left_free_angle()) << counter;
    counter++;

    // should have used 12 values during the hash function
    // xyzrpy(obj pose) fa1 fa2 (arm) xyzyaw(base)
    int hash_table_size = HASH_TABLE_SIZE;
    return intHash(val) & (hash_table_size-1);
}

GraphStatePtr HashManager::getGraphState(int state_id){
    //assert(state_id != 1);
    return m_state_id_to_graph_table[state_id];
}

unsigned int HashManager::getStateID(const GraphStatePtr& graph_state){
    unsigned int bin_idx = hash(graph_state);
    BOOST_FOREACH(auto g_s, m_coord_to_state_id_table[bin_idx]){
        if (*g_s == *graph_state){
            return g_s->id();
        }
    }
    ROS_DEBUG_NAMED(HASH_LOG, "State looking for:");
    graph_state->printToDebug(HASH_LOG);
    graph_state->robot_pose().printToDebug(HASH_LOG);
    throw std::out_of_range("Graph state does not exist in heap");
}

bool HashManager::exists(const GraphStatePtr& graph_state, int& id){
    unsigned int bin_idx = hash(graph_state);
    //ROS_DEBUG_NAMED(HASH_LOG, "Hashed state to %d", bin_idx);
    m_coord_to_state_id_table[bin_idx];
    BOOST_FOREACH(auto g_s, m_coord_to_state_id_table[bin_idx]){
        if (*g_s == *graph_state){
            //ROS_DEBUG_NAMED(HASH_LOG, "exists! the following two match at %d",g_s->id());
            id = g_s->id();
            //g_s->printToDebug(HASH_LOG);
            graph_state->printToDebug(HASH_LOG);

            return true;
        }
    }
    return false;
}

bool HashManager::save(GraphStatePtr& graph_state){
    // this may not be the desired behavior...
    //ROS_DEBUG_NAMED(HASH_LOG, "Saving graph state");
    int potential_id;
    if (exists(graph_state, potential_id)){
        graph_state->id(potential_id);
        return false;
    }
    //ROS_DEBUG_NAMED(HASH_LOG, "This is a new graph state, adding to table");
    unsigned int bin_idx = hash(graph_state);
    graph_state->id(m_state_id_to_graph_table.size());
    m_state_id_to_graph_table.push_back(graph_state);
    m_coord_to_state_id_table[bin_idx].push_back(graph_state);
    //ROS_DEBUG_NAMED(HASH_LOG, "Saved new entry with id %d", graph_state->id());

    // the planner needs this to happen. i have no idea what it's supposed to
    // do.
    int* entry = new int [NUMOFINDICES_STATEID2IND];
    m_stateID2Mapping->push_back(entry);
    for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++){
        (*m_stateID2Mapping)[graph_state->id()][i] = -1;
    }

    return true;
}


