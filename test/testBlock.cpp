#include <FootstepPlannerLJH/Block/Block.h>
#include <conio.h>
int main()
{
    // prepare input 
    int key = 0;
    double bodyPos[3];
    double bodyAng[3];
    double footPosL[3];
    double footAngL[3];
    double footPosR[3];
    double footAngR[3];
    int supFlag;
    ljh::path::footstep_planner::PlaneData<> planeData;


    ljh::path::footstep_planner::Block footplannerBlock;

    auto pBlockPlanner = &footplannerBlock;
    std::vector<Location> Outcome;
    pBlockPlanner->setInput({
        &key,
        bodyPos,
        bodyAng,
        footPosL,
        footAngL,
        footPosR,
        footAngR,
        &supFlag,
        &planeData,
    });

    pBlockPlanner->setOutput({Outcome});


    pBlockPlanner->init();
    
    key = '3';
    pBlockPlanner->run();
    

    pBlockPlanner->print();

    




}
