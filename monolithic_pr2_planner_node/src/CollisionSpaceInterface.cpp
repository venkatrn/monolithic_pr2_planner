#include <monolithic_pr2_planner_node/CollisionSpaceInterface.h>
#include <monolithic_pr2_planner/LoggerNames.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;

CollisionSpaceInterface::CollisionSpaceInterface(CSpaceMgrPtr cspace_mgr, 
                                                 HeuristicMgrPtr heur_mgr):
    m_cspace_mgr(cspace_mgr),
    m_heur_mgr(heur_mgr) {
    m_cmap_pub = m_nodehandle.advertise<arm_navigation_msgs::CollisionMap>("environment", 1);
}


bool CollisionSpaceInterface::bindCollisionSpaceToTopic(string topic_name, 
                                                        tf::TransformListener& tf,
                                                        string target_frame){
    m_collision_map_subscriber.subscribe(m_nodehandle, topic_name, 1);
    ROS_INFO_NAMED(INIT_LOG, "binding collision space to topic %s "
                              "and transforming to %s!", 
                              topic_name.c_str(), target_frame.c_str());
    m_ref_frame = target_frame;
    m_collision_map_filter = make_shared<CollisionMapMsgFilter>(m_collision_map_subscriber, 
                                                                tf, target_frame, 1);
    m_collision_map_filter->registerCallback(boost::bind(
            &CollisionSpaceInterface::mapCallback, this, _1));
    return true;
}

void CollisionSpaceInterface::mapCallback(
        const arm_navigation_msgs::CollisionMapConstPtr &map){

    if(mutex)
      boost::unique_lock<boost::mutex> lock(*mutex);

    ROS_INFO_NAMED(INIT_LOG, "map callback!");
    if(map->header.frame_id.compare(m_ref_frame) != 0)
    {
        // TODO: fix this warning
        //ROS_WARN_NAMED(INIT_LOG, "collision_map_occ is in %s not in %s", 
        //               map->header.frame_id.c_str(), m_ref_frame.c_str());
        ROS_DEBUG_NAMED(INIT_LOG,"the collision map has %i cubic obstacles", 
                        int(map->boxes.size()));
    }
    m_cspace_mgr->updateMap(*map);
    ROS_DEBUG_NAMED(INIT_LOG, "publishing map");
    m_cmap_pub.publish(*map);
    //setArmToMapTransform(map_frame_);
    return;
}

void CollisionSpaceInterface::update3DHeuristicMaps(){
    m_heur_mgr->update3DHeuristicMaps();
}

void CollisionSpaceInterface::update2DHeuristicMaps(std::vector<unsigned char>& data){
    m_heur_mgr->update2DHeuristicMaps(data);
}
