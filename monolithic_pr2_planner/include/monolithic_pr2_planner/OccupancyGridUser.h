#pragma once
#include <pr2_collision_checker/pr2_collision_space.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    /*! \brief Contains information about the collision map. This tends to be
     * used as a singleton for objects that need discretization information.
     */
    class OccupancyGridUser {
        public:
            static void init(OccupancyGridParams& og_params, RobotResolutionParams& r_params);
            static boost::shared_ptr<sbpl_arm_planner::OccupancyGrid> m_occupancy_grid;
            static RobotResolutionParams m_resolution_params;
            void getOccupancyGridSize(int& dimX, int& dimY, int&dimZ){m_occupancy_grid->getGridSize(dimX, dimY, dimZ);};
    };
}
