
#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <iostream>
// #include <matplotlibcpp.h>
// namespace plt = matplotlibcpp;
_FOOTSTEP_PLANNER_BEGIN

void AStarFootstepPlanner::initialize(Pose2D<double> _goalPose2D, Pose3D<double> _goalPose, Pose3D<double> _startPose)
{
    this->goalPose2D = _goalPose2D;
    this->goalPose = _goalPose;
    this->startPose = _startPose;

    this->stepCostCalculator.initialize(_goalPose,_startPose);
    this->stepExpander.initialize();

    this->frontier = Frontier();
    this->neighbors.clear();
    this->cameFromMap.clear();
    this->costSoFarMap.clear();
    this->wallMap.clear();

    this->calLocationFromPose(_startPose, this->startL, this->startR);
    this->calLocationFromPose(_goalPose, this->goalL, this->goalR);
    // the default startnode is left!
    this->startFlag = false;
    if(this->startFlag)
        this->start = startR;
    else
        this->start = startL;
    
    this->stepOverChecker.initilize(_goalPose2D,this->start,this->param.goalDistanceProximity,
                this->param.goalYawProximity,this->stepCostCalculator.heuristicCalculator);

    this->solutionFoundFlag = GOAL_NO_REACHED;
    this->heuclidCoreTool = HeuclidCoreTool();
    this->path.clear();
    this->accuratePath.clear();
    this->plotChecker = PlotChecker();
    this->countSearch = 0;
}



void AStarFootstepPlanner::doAStarSearch()
{
    
    this->frontier.put(this->start,cost_t(0));
    this->cameFromMap[this->start] = this->start;
    this->costSoFarMap[this->start] = cost_t(0) + this->stepCostCalculator.heuristicCalculator.compute(this->start);

    // plt::figure();
    // plt::ion();
    while(!frontier.empty())
    {   
        if(this->param.getDebugFlag(this->param))
            std::cout<<++countSearch<<std::endl;

        Location current(this->frontier.get());

        // Node Expansion
        this->stepExpander.doFullExpansion(current,this->neighbors);

        if(this->param.getDebugFlag(this->param))
        {
            this->plotChecker.plotExpansion(current,this->neighbors);
            this->plotChecker.plotFrontier(this->frontier);
            if(this->param.isStairAlignMode)
                this->plotChecker.plotGoalposeAndStair(this->goalPose); 
        }
        

        // Node Stop Check
        this->solutionFoundFlag= this->stepOverChecker.checkIfGoalReached(current,this->neighbors,this->heuclidCoreTool,this->goalL,this->goalR);
        
        // Node Cost Cal and Reorder
        if(this->solutionFoundFlag == GOAL_NO_REACHED)
        { //calculate the cost of the neighbors and reorder the neighbors by the lesser cost
            for(const Location& next : this->neighbors)
            {
                cost_t new_cost = this->costSoFarMap[current] + this->stepCostCalculator.computeEdgeCost(next,current);
                if(this->costSoFarMap.find(next) == this->costSoFarMap.end() 
                    || new_cost < this->costSoFarMap[next] )
                {
                    this->costSoFarMap[next] = new_cost;

                    cost_t priority;
                    if(this->param.isStairAlignMode)
                        priority = new_cost + this->stepCostCalculator.computeHeuristicCostWithEllipsiodPath(next);
                    else
                        priority = new_cost + this->stepCostCalculator.computeHeuristicCost(next);

                    // if(this->param.getDebugFlag(this->param))
                    //     std::cout<< "the heuristic cost: "<<this->stepCostCalculator.getHeuristicCost()<<std::endl;

                    this->frontier.put(next, priority);
                    this->cameFromMap[next] = current;
                }
            }

        }
        else if(this->solutionFoundFlag == GOAL_REACHED_PROXIMITY)
        {
            // Load the Last step
            this->cameFromMap[this->stepOverChecker.getEndNode()] = current;
            // Solution Found and Quit Search
            break;
        }
        else if(this->solutionFoundFlag == GOAL_REACHED_SPECIFIC)  
        {
            // Load the Last Two step
            this->cameFromMap[this->stepOverChecker.getStopNode()] = current;
            this->cameFromMap[this->stepOverChecker.getEndNode()] = this->stepOverChecker.getStopNode();
            // Solution Found and Quit Search
            break;
        }   
        else if(this->solutionFoundFlag == NEIGHBOR_EMPTY)   
        {
            break;
        }
    } 
}

void AStarFootstepPlanner::calLocationFromPose(Pose3D<double> _Pose, Location& Left, Location& Right)
{
    RobotSide sideL(stepL);
    RobotSide sideR(stepR);

    double xL, yL;
    double xR, yR;

    xL = _Pose.getPosition().getX() + -sin(_Pose.getOrientation().getYaw()) * this->param.IdealStepWidth/2;
    yL = _Pose.getPosition().getY() +  cos(_Pose.getOrientation().getYaw()) * this->param.IdealStepWidth/2;
    
    xR = _Pose.getPosition().getX() +  sin(_Pose.getOrientation().getYaw()) * this->param.IdealStepWidth/2;
    yR = _Pose.getPosition().getY() + -cos(_Pose.getOrientation().getYaw()) * this->param.IdealStepWidth/2;

    DiscreteFootstep  leftStep(xL, yL, _Pose.getOrientation().getYaw(),sideL);
    DiscreteFootstep rightStep(xR, yR, _Pose.getOrientation().getYaw(),sideR);

    Left = Location(rightStep,leftStep);
    Right = Location(leftStep,rightStep);
    
}

void AStarFootstepPlanner::calFootstepSeries()
{
    this->path.clear();
    Location current(this->stepOverChecker.getEndNode());
    while(!(current==this->start))
    {
        this->path.push_back(current);
        current = cameFromMap[current];
    }
    this->path.push_back(this->start);
    if(this->startFlag)
        this->path.push_back(this->startL);
    else
        this->path.push_back(this->startR);
        
    std::reverse(this->path.begin(),this->path.end());
    // LatticePoint lat;
    // lat.gridSizeXY = 1e-10;
    // lat.gridSizeYaw = 1e-10;
    // this->calLocationFromPose(this->goalPose,this->goalL,this->goalR);
    // if(this->stepOverChecker.getEndNode().getSecondStepSide().getStepFlag()==stepL)
    // {
    //     this->path.push_back(this->goalR);
    //     this->path.push_back(this->goalL);
    // }
    // else
    // {
    //     this->path.push_back(this->goalL);
    //     this->path.push_back(this->goalR);
    // }
    
}

void AStarFootstepPlanner::calAccurateFootstepSeries()
{
    if(!this->path.empty())
    {   
        int i;
        AccurateFootstep footstep;
        AccurateFootstep footstepL(this->goalPose,stepL);
        AccurateFootstep footstepR(this->goalPose,stepR);
        for(i=0;i<path.size()-2;i++)
        {
            //footstep = AccurateFootstep(this->path.at(i));

            footstep.setFromNode(this->path.at(i));
            this->accuratePath.push_back(footstep);
        }

        if(path.at(i).getSecondStepSide().getStepFlag()==stepL)
        {
            this->accuratePath.push_back(footstepL);
            this->accuratePath.push_back(footstepR);
        }
        else
        {
            this->accuratePath.push_back(footstepR);
            this->accuratePath.push_back(footstepL);
        }
        // load the real last two accurate footsteps from goalpose, drop out the last two search 
        
    }
}

std::vector<AccurateFootstep> AStarFootstepPlanner::getOrCalAccurateFootstepSeries()
{
    if(this->accuratePath.empty())
        this->calAccurateFootstepSeries();
    
    return this->accuratePath;
}


void AStarFootstepPlanner::plotAccurateSearchOutcome()
{
    if(this->accuratePath.empty())
        this->calAccurateFootstepSeries();

    this->plotChecker.plotAccurateSearchOutcome2(this->accuratePath,this->goalPose,this->startPose,this->stepCostCalculator.heuristicCalculator.pathHolder);
}
_FOOTSTEP_PLANNER_END