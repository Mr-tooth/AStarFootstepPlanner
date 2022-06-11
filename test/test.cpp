#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>
#include <iostream>

#include <FootstepPlannerLJH/Block/Block.h>
int main()
{
    // // CHECK the problem with scatter
    // ljh::path::footstep_planner::PlotChecker pltChecker2;

    // Location ha1,ha2,ha3,ha4,ha5;
    // std::vector<Location> ha;
    // ha.push_back(ha2);
    // ha.push_back(ha3);
    // ha.push_back(ha4);
    // ha.push_back(ha5);
    // pltChecker2.plotExpansion(ha1,ha);
    // std::vector<double> expanX,expanY,expanYaw;
    // for(int i=0;i<1000;i++)
    // {
    //     expanX.push_back(i*1.5);
    //     expanY.push_back(i*2);
    //     expanYaw.push_back(i*2.5);
    //     plt::annotate(std::to_string(i),i*1.5,i*2+0.02);
    // }
    // plt::figure(1);
    // //plt::clf();
    // plt::scatter(expanX,expanY);
    // plt::show();


    ljh::path::footstep_planner::StepConstraintCheck checker;
    //set parameters!
    ljh::path::footstep_planner::LatticePoint latticepoint;
    ljh::path::footstep_planner::parameters param;

    latticepoint.setGridSizeXY(latticepoint,0.01);
    latticepoint.setYawDivision(latticepoint, 72);

    param.SetEdgeCostDistance(param,4.0);
    param.SetEdgeCostYaw(param, 4.0);
    param.SetEdgeCostStaticPerStep(param,1.4);
    param.SetDebugFlag(param,true);
    param.SetMaxStepYaw(param,pi/8);
    param.SetMinStepYaw(param,-pi/8);        

    param.SetFinalTurnProximity(param,0.3);
    param.SetGoalDistanceProximity(param,0.04);
    param.SetGoalYawProximity(param,2.0/180.0 * pi);
    param.SetFootPolygonExtendedLength(param,0.03);

    param.SetHWPOfWalkDistacne(param,1.30);

    param.SetMaxStepLength(param, 0.15);
    param.SetMinStepLength(param,-0.15);
    param.SetMaxStepWidth(param,0.26);
    param.SetMinStepWidth(param,0.16);
    param.SetMaxStepReach(param,sqrt((param.MaxStepWidth-param.MinStepWidth) * (param.MaxStepWidth-param.MinStepWidth) + param.MaxStepLength * param.MaxStepLength));
    

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
    // double startX = 0.0;
    // double startY = 0.0;
    // double startZ = 0.0;
    // double startYaw = 0.0/180.0 * pi;

    // double goalX = 0.8;
    // double goalY = 0.0;
    // double goalZ = 0.0;
    // double goalYaw = -60.0/180.0 * pi;
    double startX = 0.01;
    double startY = 0.0;
    double startZ = 0.0;
    double startYaw = 0.0/180.0 * pi;

    // double goalX = 0.292897;
    // double goalY = -0.175203;
    // double goalZ = 0.0;
    // double goalYaw = -0.184148;
    double goalX = 0.428922;
    double goalY = 0.117461;
    double goalZ = 0.0;
    double goalYaw = 0.164328;

    ljh::mathlib::Pose2D<double> goalPose2D(goalX,goalY,goalYaw);
    ljh::mathlib::Pose3D<double> goalPose(goalX,goalY,goalZ,goalYaw,0.0,0.0);
    ljh::mathlib::Pose3D<double> startPose(startX,startY,startZ,startYaw,0.0,0.0);

    double xFromGoalToStair = 0.16+0.005;
    double xLenOfStair = 0.5;
    double yLenOfStair = 0.5;

    // Point2D<double> p0(xFromGoalToStair,yLenOfStair/2);
    // Point2D<double> p1(xFromGoalToStair+xLenOfStair,yLenOfStair/2);
    // Point2D<double> p2(xFromGoalToStair+xLenOfStair,-yLenOfStair/2);
    // Point2D<double> p3(xFromGoalToStair,-yLenOfStair/2);

    // Point2D<double> p0(1.0627,0.183272);
    // Point2D<double> p1(0.569738,0.27509);
    // Point2D<double> p2(0.389632,-0.695077);
    // Point2D<double> p3(0.882591,-0.786894);
    Point2D<double> p0(0.729462,0.743607);
    Point2D<double> p1(0.444198,0.696304);
    Point2D<double> p2(0.630682,-0.425393);
    Point2D<double> p3(0.915946,-0.37809);
    std::cout<<"stair angle is "<<-asin((p0.getX()-p3.getX())/(p0.getY()-p3.getY()))<<std::endl;
    std::vector<Point2D<double> > stairBuffer({p0,p1,p2,p3});
    //std::vector<Point2D<double> > stairBuffer({p3,p2,p1,p0});
    // for(int i=0;i<4;i++)
    // {
    //     stairBuffer[i].setPoint2D(goalX + cos(goalYaw)*stairBuffer[i].getX() - sin(goalYaw)*stairBuffer[i].getY(),
    //                               goalY + sin(goalYaw)*stairBuffer[i].getX() + cos(goalYaw)*stairBuffer[i].getY());
    // }
    param.SetStairAlignMode(param,false);
    param.SetStairPolygon(param,stairBuffer,4,0);
    for(int i=0;i<4;i++)
        std::cout<<"stairPolygon" <<i<<"is"<<param.stairPolygon.getVertexBuffer().at(i).getX()<<" "<<param.stairPolygon.getVertexBuffer().at(i).getY() <<std::endl;


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
    

    auto accurateOut = footstepPlanner.getOrCalAccurateFootstepSeries();
    ljh::path::footstep_planner::PlotChecker pltChecker;
    
    pltChecker.plotSearchOutcome2(Out,goalPose,startPose);
    pltChecker.plotAccurateSearchOutcome(accurateOut,goalPose,startPose);
    // std::cout<<"Collide 1:"<<checker.isTwoFootCollidedAndPlot(Out.at(7))<<std::endl;
    // std::cout<<"Distance of last two: "<<
    // sqrt(pow(Out[Out.size()-1].getSecondStep().getX()-Out[Out.size()-2].getSecondStep().getX(),2)+
    // pow(Out[Out.size()-1].getSecondStep().getY()-Out[Out.size()-2].getSecondStep().getY(),2))<<std::endl;
    param.SetStairAlignMode(param,true);
    pltChecker.plotAccurateSearchOutcome(accurateOut,goalPose,startPose);
    // a second time search
    //test 5
    startX = 0.015;
    startY = 0.0;
    startZ = 0.0;
    startYaw = 0.0/180.0 * pi;
    goalX = 0.815;
    goalY = -0.8/sqrt(3);
    goalZ = 0.0;
    goalYaw = -90.0/180.0 * pi;

    goalPose2D.setPosition(goalX,goalY);
    goalPose2D.setOrientation(goalYaw);

    goalPose.setYawPitchRoll(goalYaw,0.0,0.0);
    goalPose.setX(goalX);goalPose.setY(goalY);goalPose.setZ(goalZ);

    startPose.setYawPitchRoll(startYaw,0.0,0.0);
    startPose.setX(startX);startPose.setY(startY);startPose.setZ(startZ);

    Point2D<double> p10(xFromGoalToStair,yLenOfStair/2);
    Point2D<double> p11(xFromGoalToStair+xLenOfStair,yLenOfStair/2);
    Point2D<double> p12(xFromGoalToStair+xLenOfStair,-yLenOfStair/2);
    Point2D<double> p13(xFromGoalToStair,-yLenOfStair/2);
    std::vector<Point2D<double> > stairBuffer2({p10,p11,p12,p13});
    for(int i=0;i<4;i++)
    {
        stairBuffer2[i].setPoint2D(goalX + cos(goalYaw)*stairBuffer2[i].getX() - sin(goalYaw)*stairBuffer2[i].getY(),
                                  goalY + sin(goalYaw)*stairBuffer2[i].getX() + cos(goalYaw)*stairBuffer2[i].getY());
    }
    param.SetStairPolygon(param,stairBuffer2,4,1);

    param.SetFinalTurnProximity(param,0.3);
    param.SetDebugFlag(param,true);
    footstepPlanner.initialize(goalPose2D,goalPose,startPose);
    footstepPlanner.doAStarSearch();
    footstepPlanner.calFootstepSeries();
    std::vector<ljh::path::footstep_planner::FootstepGraphNode> Out2 = footstepPlanner.getFootstepSeries();
    auto accurateOut2 = footstepPlanner.getOrCalAccurateFootstepSeries();
    pltChecker.plotSearchOutcome2(Out2,goalPose,startPose);
    pltChecker.plotAccurateSearchOutcome(accurateOut2,goalPose,startPose);
    //Location test = Out2.at(7);
    std::vector<ljh::path::footstep_planner::FootstepGraphNode> Out3;
       Out3.push_back(Out2.at(7));
    Out3.push_back(Out2.at(8));
    pltChecker.plotSearchOutcome(Out3,goalPose,startPose);

    std::cout<<"Collide 1:"<<checker.isTwoFootCollided(Out2.at(7))<<std::endl;
    std::cout<<"Collide 2:"<<checker.isTwoFootCollided(Out2.at(8))<<std::endl;
    std::cout<<"Distance of last two: "<<
    sqrt(pow(Out2[Out2.size()-1].getSecondStep().getX()-Out2[Out2.size()-2].getSecondStep().getX(),2)+
    pow(Out2[Out2.size()-1].getSecondStep().getY()-Out2[Out2.size()-2].getSecondStep().getY(),2))<<std::endl;
    std::cout<<"Distance of accurate last two: "<<
    sqrt(pow(accurateOut2[accurateOut2.size()-1].getX()-accurateOut2[accurateOut2.size()-2].getX(),2)+
    pow(accurateOut2[accurateOut2.size()-1].getY()-accurateOut2[accurateOut2.size()-2].getY(),2))<<std::endl;
    std::cout<<"Quit"<<std::endl;
    return 0;
}