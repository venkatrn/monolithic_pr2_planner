#include <monolithic_pr2_planner/ExperimentFramework/ExpInterface.h>
using namespace monolithic_pr2_planner;

ExpInterface::ExpInterface(CSpaceMgrPtr cspace):m_generator(cspace){}

void ExpInterface::generatePairs(){
    m_generator.generateUniformPairs(10);
}
