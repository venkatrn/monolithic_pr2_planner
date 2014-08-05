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
    vector<int> coords = graph_state->getCoords();
    for (auto coord_value : coords){
        val += coord_value << counter;
        counter++;
    }

    int hash_table_size = HASH_TABLE_SIZE;
    return intHash(val) & (hash_table_size-1);
}

GraphStatePtr HashManager::getGraphState(int state_id){
    //assert(state_id != 1);
    return m_state_id_to_graph_table[state_id];
}

unsigned int HashManager::getStateID(const GraphStatePtr& graph_state){
    int id;
    if (exists(graph_state, id)){
        return id;
    }
    std::vector<int> tmp = graph_state->getCoords();
    ROS_ERROR("Can't find state %d in hashmanager! %d %d %d %d %d %d %d %d %d %d %d %d",
              graph_state->id(),
              tmp[0],
              tmp[1],
              tmp[2],
              tmp[3],
              tmp[4],
              tmp[5],
              tmp[6],
              tmp[7],
              tmp[8],
              tmp[9],
              tmp[10],
              tmp[11]);
    graph_state->printToDebug(HASH_LOG);
    unsigned int bin_idx = hash(graph_state);
    ROS_ERROR("bin_idx %u has %lu items", bin_idx, 
              m_coord_to_state_id_table[bin_idx].size());
    for (auto& item : m_coord_to_state_id_table[bin_idx]){
        item->printToDebug(HASH_LOG);
    }
    assert(false);
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
            return true;
        }
    }
    return false;
}

// saves a graph state. if it already exists, this returns false, and it fills
// in the input graph state with the found id WITHOUT overwriting it.
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

    ROS_DEBUG_NAMED(HASH_LOG, "Saved new entry with id %d and hash %u", 
                              graph_state->id(), bin_idx);
    graph_state->printToDebug(HASH_LOG);

    // the planner needs this to happen. i have no idea what it's supposed to
    // do.
    int* entry = new int [NUMOFINDICES_STATEID2IND];
    m_stateID2Mapping->push_back(entry);
    for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++){
        (*m_stateID2Mapping)[graph_state->id()][i] = -1;
    }

    return true;
}


