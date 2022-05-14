#include <FootstepPlannerLJH\AStarFootstepPlanner.h>
//#include <FootstepPlannerLJH\parameters.h>
#include <iostream>
int main()
{
    // define the initial and final pose

    // test 1
    // double startX = 1.440;
    // double startY = 0.0;
    // double startZ = 0.0;
    // double startYaw = 0.0/180.0 * 3.1415926535;

    // double goalX = 3.44;
    // double goalY = 1.3;
    // double goalZ = 0.0;
    // double goalYaw = 60.0/180.0 * 3.1415926535;
    

    //test 2
    // double startX = 0.0;
    // double startY = 0.0;
    // double startZ = 0.0;
    // double startYaw = 140.0/180.0 * 3.1415926535;

    // double goalX = -4;
    // double goalY = 4;
    // double goalZ = 0.0;
    // double goalYaw = 140.0/180.0 * 3.1415926535;

    double startX = 0.0;
    double startY = 0.0;
    double startZ = 0.0;
    double startYaw = 45.0/180.0 * 3.1415926535;

    double goalX = 4;
    double goalY = -2.8;
    double goalZ = 0.0;
    double goalYaw = -140.0/180.0 * 3.1415926535;

    ljh::mathlib::Pose2D<double> goalPose2D(goalX,goalY,goalYaw);
    ljh::mathlib::Pose3D<double> goalPose(goalX,goalY,goalZ,goalYaw,0.0,0.0);
    ljh::mathlib::Pose3D<double> startPose(startX,startY,startZ,startYaw,0.0,0.0);


    


    // ljh::path::footstep_planner::FootstepCostCalculator stepCostCalculator;
    // stepCostCalculator.initialize(goalPose,startPose);

    // ljh::path::footstep_planner::RobotSide sideL(stepL);
    // ljh::path::footstep_planner::RobotSide sideR(stepR);
    // ljh::path::footstep_planner::DiscreteFootstep firststep(1.0,2.0,0.0,sideL);
    // ljh::path::footstep_planner::DiscreteFootstep secondstep(2.0,1.8,0.0,sideR);
    // ljh::path::footstep_planner::FootstepGraphNode Node(firststep,secondstep);

    // std::cout<<stepCostCalculator.computeTotalCost(Node,)

    std::cout<<"initilize start! "<<std::endl;
    ljh::path::footstep_planner::AStarFootstepPlanner footstepPlanner;
    std::cout<<"initilize start 2! "<<std::endl;
    footstepPlanner.initialize(goalPose2D,goalPose,startPose);
    std::cout<<"initilize over! "<<std::endl;




    footstepPlanner.doAStarSearch();
    footstepPlanner.calFootstepSeries();
    std::vector<ljh::path::footstep_planner::FootstepGraphNode> Out = footstepPlanner.getFootstepSeries();
    std::cout<< "First x y yaw"<<Out.at(0).getSecondStep().getX() <<" "<<Out.at(0).getSecondStep().getY() <<" "<<Out.at(0).getSecondStep().getYaw()<<std::endl;
    
    ljh::path::footstep_planner::PlotChecker pltChecker;
    pltChecker.plotSearchOutcome(Out,goalPose,startPose);
    
    std::cout<<"Quit"<<std::endl;
    return 0;
}