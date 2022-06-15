

#pragma once
#include <vector>
#include <unordered_map>
#include <functional>
#include <algorithm>

#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/AStarSearch.h>
#include <FootstepPlannerLJH/StepCost/FootstepCostCalculator.h>
#include <FootstepPlannerLJH/StepExpansion/ParameterBasedStepExpansion.h>
#include <FootstepPlannerLJH/StepCheck/FootstepPlannerCompletionChecker.h>
#include <FootstepPlannerLJH/PlotCheck/PlotChecker.h>

_FOOTSTEP_PLANNER_BEGIN

class AStarFootstepPlanner
{
private:
    typedef ::cost_t cost_t;
    typedef ::Location Location;
    typedef FootNodeHash LocationHash;
    // may need a more sophisticated QUEUE
    typedef PriorityQueue<Location,cost_t> Frontier;

    Pose2D<double> goalPose2D;
    Pose3D<double> goalPose;
    Pose3D<double> startPose;

    Location startL;
    Location startR;
    Location goalL;
    Location goalR;
    
    FootstepCostCalculator stepCostCalculator;
    ParameterBasedStepExpansion stepExpander;

    Frontier frontier;
    std::vector<Location> neighbors;
    std::unordered_map<Location,Location,LocationHash> cameFromMap;
    std::unordered_map<Location,cost_t,LocationHash> costSoFarMap;
    std::unordered_map<Location,cost_t,LocationHash> wallMap;

    parameters param;
    bool startFlag;

    Location start;
    FootstepCompletionChecker stepOverChecker;
    int solutionFoundFlag;
    HeuclidCoreTool heuclidCoreTool;

    std::vector<Location> path;
    std::vector<AccurateFootstep> accuratePath;
    int countSearch;
    PlotChecker plotChecker;

public:
    AStarFootstepPlanner():goalPose2D(),goalPose(),startPose(),startL(),startR(),goalL(),goalR(),stepCostCalculator(),stepExpander(),frontier(),neighbors(),cameFromMap(),costSoFarMap(),wallMap(),param(),startFlag(false),
         start(),stepOverChecker(),solutionFoundFlag(GOAL_NO_REACHED),heuclidCoreTool(),path(),accuratePath(),countSearch(0),plotChecker() {};

    AStarFootstepPlanner(Pose2D<double> _goalPose2D, Pose3D<double> _goalPose, Pose3D<double> _startPose)
        :goalPose2D(_goalPose2D),goalPose(_goalPose),startPose(_startPose),startL(),startR(),goalL(),goalR(),stepCostCalculator(_goalPose,_startPose),stepExpander(),frontier(),neighbors(),cameFromMap(),costSoFarMap(),wallMap(),param(),startFlag(false),
         start(),stepOverChecker(),solutionFoundFlag(GOAL_NO_REACHED),heuclidCoreTool(),path(),accuratePath(),countSearch(0),plotChecker()
        {
            this->calLocationFromPose(_startPose, this->startL, this->startR);
            this->calLocationFromPose(_goalPose, this->goalL, this->goalR);

            this->stepExpander.initialize();
            if(this->startFlag)
                this->start = startR;        
            else
                this->start = startL;

            stepOverChecker.initilize(_goalPose2D,this->start,this->param.goalDistanceProximity,
                this->param.goalYawProximity,this->stepCostCalculator.heuristicCalculator);
        };

    void initialize(Pose2D<double> _goalPose2D, Pose3D<double> _goalPose, Pose3D<double> _startPose);
    void doAStarSearch();

    void calLocationFromPose(Pose3D<double> _Pose, Location& Left, Location& Right);

    void calFootstepSeries();
    std::vector<Location> getFootstepSeries() const {return this->path;};

    void calAccurateFootstepSeries();
    std::vector<AccurateFootstep> getAccurateFootstepSeries() const {return this->accuratePath;};
    std::vector<AccurateFootstep> getOrCalAccurateFootstepSeries();

    void plotAccurateSearchOutcome();



};






_FOOTSTEP_PLANNER_END

