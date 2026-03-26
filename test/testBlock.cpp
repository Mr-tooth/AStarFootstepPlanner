// Copyright 2026 Junhang Lai
// SPDX-License-Identifier: Apache-2.0

#include <FootstepPlannerLJH/Block/Block.h>
#ifdef _WIN32
#include <conio.h>
#endif
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

    // Initialize planeData to prevent garbage numOfVertices
    planeData.numOfVertices = 0;
    planeData.clockwiseOrdered = true;
    planeData.alignEdgeEndpoints[0] = 0;
    planeData.alignEdgeEndpoints[1] = 1;

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

    // Use fast stair collision check for CI performance:
    // block_test's stair polygon is a large simplified region (1m×1m), not a precise
    // obstacle. Vertex-inside check is sufficient here and avoids the expensive
    // polygon-polygon intersection. Demo/real scenarios use precise mode (default).
    pBlockPlanner->setUseFastStairCheck(true);

    key = '3';
    pBlockPlanner->run();
    

    pBlockPlanner->print();

    




}
