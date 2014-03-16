#include <monolithic_pr2_planner/Visualizer.h>

using namespace monolithic_pr2_planner;

std::unique_ptr<PViz> Visualizer::pviz;

void Visualizer::setReferenceFrame(std::string frame){
        pviz->setReferenceFrame(frame);
}

void Visualizer::createPVizInstance(){
    pviz.reset(new PViz());
}
