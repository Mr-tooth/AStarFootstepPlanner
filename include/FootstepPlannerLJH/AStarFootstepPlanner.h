// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0



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
#include <Heuclid/euclid/tools/HeuclidCoreTool.h>

_FOOTSTEP_PLANNER_BEGIN

/**
 * @class AStarFootstepPlanner
 * @brief A* algorithm-based footstep planner for humanoid robots.
 *
 * This planner implements the core A* graph search algorithm from the IHMC
 * footstep planning framework (Humanoids 2019). It searches for an optimal
 * sequence of footsteps connecting start to goal poses while satisfying
 * kinematic and environmental constraints.
 */
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

    std::vector<Location> path;
    std::vector<AccurateFootstep> accuratePath;
    int countSearch;
    PlotChecker plotChecker;

public:
    /** @brief Access parameters for configuration (e.g. SetUseFastStairCheck). */
    parameters& getParamRef() { return this->param; }

    AStarFootstepPlanner():goalPose2D(),goalPose(),startPose(),startL(),startR(),goalL(),goalR(),stepCostCalculator(),stepExpander(),frontier(),neighbors(),cameFromMap(),costSoFarMap(),wallMap(),param(),startFlag(false),
         start(),stepOverChecker(),solutionFoundFlag(GOAL_NO_REACHED),path(),accuratePath(),countSearch(0),plotChecker() {};

    AStarFootstepPlanner(Pose2D<double> _goalPose2D, Pose3D<double> _goalPose, Pose3D<double> _startPose)
        :goalPose2D(_goalPose2D),goalPose(_goalPose),startPose(_startPose),startL(),startR(),goalL(),goalR(),stepCostCalculator(_goalPose,_startPose),stepExpander(),frontier(),neighbors(),cameFromMap(),costSoFarMap(),wallMap(),param(),startFlag(false),
         start(),stepOverChecker(),solutionFoundFlag(GOAL_NO_REACHED),path(),accuratePath(),countSearch(0),plotChecker()
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

    /**
     * @brief Initialize the planner with start and goal poses.
     * @param _goalPose2D 2D goal pose (x, y, yaw)
     * @param _goalPose 3D goal pose (x, y, z, roll, pitch, yaw)
     * @param _startPose 3D start pose (x, y, z, roll, pitch, yaw)
     */
    void initialize(Pose2D<double> _goalPose2D, Pose3D<double> _goalPose, Pose3D<double> _startPose);

    /**
     * @brief Execute the main A* search loop.
     *
     * Performs graph expansion, cost evaluation, and goal checking until
     * a solution is found or the search is terminated.
     */
    void doAStarSearch();

    /**
     * @brief Calculate left and right foot locations from a body pose.
     * @param _Pose Body pose
     * @param Left Output left foot location
     * @param Right Output right foot location
     */
    void calLocationFromPose(Pose3D<double> _Pose, Location& Left, Location& Right);

    /**
     * @brief Calculate the discrete footstep series from the search result.
     */
    void calFootstepSeries();

    /**
     * @brief Get the discrete footstep series.
     * @return Vector of foot locations
     */
    std::vector<Location> getFootstepSeries() const {return this->path;};

    /**
     * @brief Calculate the accurate footstep series from the discrete solution.
     */
    void calAccurateFootstepSeries();

    /**
     * @brief Get the accurate footstep series.
     * @return Vector of accurate footsteps
     */
    std::vector<AccurateFootstep> getAccurateFootstepSeries() const {return this->accuratePath;};

    /**
     * @brief Calculate and return the accurate footstep series.
     * @return Vector of accurate footsteps
     */
    std::vector<AccurateFootstep> getOrCalAccurateFootstepSeries();

    /**
     * @brief Plot the accurate search outcome for visualization.
     */
    void plotAccurateSearchOutcome();



};






_FOOTSTEP_PLANNER_END
