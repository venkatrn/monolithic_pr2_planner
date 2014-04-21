#pragma once
#include <monolithic_pr2_planner/StatsWriter.h>
#include <vector>

namespace monolithic_pr2_planner {

struct Cuboid {
    double x, y, z, dimX, dimY, dimZ;
};

class ExperimentEnvironment {
    public:
        void addCuboid(Cuboid cuboid);
        inline std::vector<Cuboid> getCuboids() { return m_cuboids; }

        void writeEnvironment() {
            
        }
    private:
        std::vector<Cuboid> m_cuboids;
        StatsWriter m_stats_writer;
};

}