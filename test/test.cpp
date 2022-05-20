#include <FootstepPlannerLJH\AStarFootstepPlanner.h>
#include <FootstepPlannerLJH\parameters.h>
#include <FootstepPlannerLJH\StepConstraints\StepConstraintCheck.h>
#include <FootstepPlannerLJH\StepConstraints\StepConstraintCheck.h>
#include <iostream>
int main()
{
    ljh::path::footstep_planner::StepConstraintCheck checker;
    //set parameters!
    ljh::path::footstep_planner::LatticePoint latticepoint;
    ljh::path::footstep_planner::parameters param;

    latticepoint.setGridSizeXY(latticepoint,0.02);
    latticepoint.setYawDivision(latticepoint, 72);

    param.SetEdgeCostDistance(param,4.0);
    param.SetEdgeCostYaw(param, 4.0);
    param.SetEdgeCostStaticPerStep(param,1.4);
    param.SetDebugFlag(param,true);
    param.SetMaxStepYaw(param,pi/8);
    param.SetMinStepYaw(param,-pi/8);        

    param.SetFinalTurnProximity(param,0.3);
    param.SetGoalDistanceProximity(param,0.02);
    param.SetGoalYawProximity(param,2.0/180.0 * pi);
    param.SetFootPolygonExtendedLength(param,0.02);
    

    std:: cout<< "gridSizeXY is "<<latticepoint.getGridSizeXY(latticepoint)<<std::endl;
    std:: cout<< "gridSizeYaw is "<<latticepoint.getGridSizeYaw(latticepoint)<<std::endl;

    std:: cout<< "EdgeCost Weight Distance is "<<param.getEdgeCostDistance(param)<<std::endl;
    std:: cout<< "EdgeCost Weight Yaw is "<<param.getEdgeCostYaw(param)<<std::endl; 
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

    //test 3
    // double startX = 0.0;
    // double startY = 0.0;
    // double startZ = 0.0;
    // double startYaw = 45.0/180.0 * 3.1415926535;

    // double goalX = 4;
    // double goalY = -2.8;
    // double goalZ = 0.0;
    // double goalYaw = -140.0/180.0 * 3.1415926535;

    //test 4
    // double startX = 0.0;
    // double startY = 0.0;
    // double startZ = 0.0;
    // double startYaw = 0.0/180.0 * pi;

    // double goalX = 1.3;//0.8;
    // double goalY = 0.0;//-0.25;
    // double goalZ = 0.0;
    // double goalYaw = 40.0/180.0 * pi;

    //test 5
    // double startX = 0.0;
    // double startY = 0.0;
    // double startZ = 0.0;
    // double startYaw = 0.0/180.0 * pi;

    // double goalX = 0.8;
    // double goalY = -0.25;
    // double goalZ = 0.0;
    // double goalYaw = 40.0/180.0 * pi;

    //test 6
    // double startX = 0.0;
    // double startY = 0.0;
    // double startZ = 0.0;
    // double startYaw = 0.0/180.0 * pi;

    // double goalX = 0.8;
    // double goalY = -0.15;
    // double goalZ = 0.0;
    // double goalYaw = -25.0/180.0 * pi;

    //test 7
    double startX = 0.0;
    double startY = 0.0;
    double startZ = 0.0;
    double startYaw = 0.0/180.0 * pi;

    double goalX = 0.8;
    double goalY = 0.0;
    double goalZ = 0.0;
    double goalYaw = -60.0/180.0 * pi;

    ljh::mathlib::Pose2D<double> goalPose2D(goalX,goalY,goalYaw);
    ljh::mathlib::Pose3D<double> goalPose(goalX,goalY,goalZ,goalYaw,0.0,0.0);
    ljh::mathlib::Pose3D<double> startPose(startX,startY,startZ,startYaw,0.0,0.0);

    double xFromGoalToStair = 0.16+0.005;
    double xLenOfStair = 0.5;
    double yLenOfStair = 0.5;

    Point2D<double> p0(xFromGoalToStair,yLenOfStair/2);
    Point2D<double> p1(xFromGoalToStair+xLenOfStair,yLenOfStair/2);
    Point2D<double> p2(xFromGoalToStair+xLenOfStair,-yLenOfStair/2);
    Point2D<double> p3(xFromGoalToStair,-yLenOfStair/2);

    std::vector<Point2D<double> > stairBuffer({p0,p1,p2,p3});
    for(int i=0;i<4;i++)
    {
        stairBuffer[i].setPoint2D(goalX + cos(goalYaw)*stairBuffer[i].getX() - sin(goalYaw)*stairBuffer[i].getY(),
                                  goalY + sin(goalYaw)*stairBuffer[i].getX() + cos(goalYaw)*stairBuffer[i].getY());
    }
    param.SetStairAlignMode(param,true);
    param.SetStairPolygon(param,stairBuffer,4,1);
    


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
    


    // a second time search
    //test 5
    startX = 0.0;
    startY = 0.0;
    startZ = 0.0;
    startYaw = 0.0/180.0 * pi;
    goalX = 0.8;
    goalY = -0.35;
    goalZ = 0.0;
    goalYaw = 40.0/180.0 * pi;

    goalPose2D.setPosition(goalX,goalY);
    goalPose2D.setOrientation(goalYaw);

    goalPose.setYawPitchRoll(goalYaw,0.0,0.0);
    goalPose.setX(goalX);goalPose.setY(goalY);goalPose.setZ(goalZ);

    startPose.setYawPitchRoll(startYaw,0.0,0.0);
    startPose.setX(startX);startPose.setY(startY);startPose.setZ(startZ);

    std::vector<Point2D<double> > stairBuffer2({p0,p1,p2,p3});
    for(int i=0;i<4;i++)
    {
        stairBuffer2[i].setPoint2D(goalX + cos(goalYaw)*stairBuffer2[i].getX() - sin(goalYaw)*stairBuffer2[i].getY(),
                                  goalY + sin(goalYaw)*stairBuffer2[i].getX() + cos(goalYaw)*stairBuffer2[i].getY());
    }
    param.SetStairPolygon(param,stairBuffer2,4,1);

    footstepPlanner.initialize(goalPose2D,goalPose,startPose);
    footstepPlanner.doAStarSearch();
    footstepPlanner.calFootstepSeries();
    std::vector<ljh::path::footstep_planner::FootstepGraphNode> Out2 = footstepPlanner.getFootstepSeries();
    pltChecker.plotSearchOutcome(Out2,goalPose,startPose);
    //std::cout<<"Collide :"<<checker.isTwoFootCollided(Out.at(7))<<std::endl;
    std::cout<<"Quit"<<std::endl;
    return 0;
}