// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0

// Demo: Flat terrain footstep planning with ellipsoid body path
// Agg-compatible: uses only plt::plot + plt::annotate (no arrow, no scatter, no set_aspect_equal)

#include <matplotlibcpp.h>
#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/PlotCheck/PlotChecker.h>
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>
#include <FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>

namespace plt = matplotlibcpp;
namespace fs = std::filesystem;

void drawEllipsoidPath(ljh::path::footstep_planner::Simple2DBodyPathHolder& pathHolder)
{
    auto waypoints = pathHolder.getWayPointPath();
    if (waypoints.empty()) return;

    std::vector<double> x, y;
    for (size_t i = 0; i < waypoints.size(); i++)
    {
        x.push_back(waypoints[i].getPosition().getX());
        y.push_back(waypoints[i].getPosition().getY());
    }

    plt::plot(x, y, {{"color", "3498db"}, {"linewidth", "1.2"}});
}

void drawFootsteps(const std::vector<ljh::path::footstep_planner::FootstepGraphNode>& steps, int count)
{
    std::vector<double> vx, vy;
    for (int i = 0; i < count && i < (int)steps.size(); i++)
    {
        vx.clear();
        vy.clear();
        getFootVertex2D(steps.at(i), vx, vy);
        if (!vx.empty()) { vx.push_back(vx.front()); vy.push_back(vy.front()); }

        std::string color = (steps.at(i).getSecondStepSide().getStepFlag() == stepL) ? "e74c3c" : "f39c12";
        plt::plot(vx, vy, {{"color", color}, {"linewidth", "1.5"}});

        // Step center dot (instead of scatter)
        double cx = steps.at(i).getSecondStep().getX();
        double cy = steps.at(i).getSecondStep().getY();
        plt::plot(std::vector<double>{cx}, std::vector<double>{cy},
                  {{"color", "3498db"}, {"marker", "."}, {"linestyle", "none"}, {"markersize", "4"}});
    }
}

void drawMarkers(double sx, double sy, double syaw,
                 double gx, double gy, double gyaw)
{
    const double arrowLen = 0.03;

    // Start: green dot + direction line
    plt::plot(std::vector<double>{sx}, std::vector<double>{sy},
              {{"color", "2ecc71"}, {"marker", "s"}, {"linestyle", "none"}, {"markersize", "10"}});
    plt::plot(std::vector<double>{sx, sx + cos(syaw) * arrowLen},
              std::vector<double>{sy, sy + sin(syaw) * arrowLen},
              {{"color", "2ecc71"}, {"linewidth", "3.0"}});

    // Goal: red dot + direction line
    plt::plot(std::vector<double>{gx}, std::vector<double>{gy},
              {{"color", "e74c3c"}, {"marker", "D"}, {"linestyle", "none"}, {"markersize", "10"}});
    plt::plot(std::vector<double>{gx, gx + cos(gyaw) * arrowLen},
              std::vector<double>{gy, gy + sin(gyaw) * arrowLen},
              {{"color", "e74c3c"}, {"linewidth", "3.0"}});
}

int main()
{
    std::cout << "=== AStar Footstep Planner - Flat Terrain Demo ===" << std::endl;

    std::string outDir = "demo/frames_flat";
    fs::create_directories(outDir);

    ljh::path::footstep_planner::LatticePoint latticepoint;
    ljh::path::footstep_planner::parameters param;
    latticepoint.setGridSizeXY(latticepoint, 0.01);
    latticepoint.setYawDivision(latticepoint, 72);
    param.SetEdgeCostDistance(param, 4.0);
    param.SetEdgeCostYaw(param, 4.0);
    param.SetEdgeCostStaticPerStep(param, 1.4);
    param.SetDebugFlag(param, false);
    param.SetMaxStepYaw(param, pi / 12);
    param.SetMinStepYaw(param, -pi / 12);
    param.SetFinalTurnProximity(param, 0.3);
    param.SetGoalDistanceProximity(param, 0.04);
    param.SetGoalYawProximity(param, 4.0 / 180.0 * pi);
    param.SetFootPolygonExtendedLength(param, 0.025);
    param.SetHWPOfWalkDistacne(param, 1.30);
    param.SetHWPOfPathDistance(param, 2.50);
    param.SetHWPOfFinalTurnDistacne(param, 1.30);
    param.SetHWPOfFinalWalkDistacne(param, 1.30);
    param.SetMaxStepLength(param, 0.08);
    param.SetMinStepLength(param, -0.08);
    param.SetMaxStepWidth(param, 0.22);
    param.SetMinStepWidth(param, 0.16);
    param.SetMaxStepReach(param, sqrt(
        (param.MaxStepWidth - param.MinStepWidth) * (param.MaxStepWidth - param.MinStepWidth)
        + param.MaxStepLength * param.MaxStepLength));

    double startX = 0.015, startY = 0.0, startZ = 0.0, startYaw = 0.0;
    double goalX = 0.815, goalY = -0.8, goalZ = 0.0, goalYaw = -90.0 / 180.0 * pi;

    ljh::heuclid::Pose2D<double> goalPose2D(goalX, goalY, goalYaw);
    ljh::heuclid::Pose3D<double> goalPose(goalX, goalY, goalZ, goalYaw, 0.0, 0.0);
    ljh::heuclid::Pose3D<double> startPose(startX, startY, startZ, startYaw, 0.0, 0.0);

    ljh::path::footstep_planner::Simple2DBodyPathHolder pathHolder;
    pathHolder.initialize({startX, startY, startYaw}, goalPose2D);

    std::cout << "Running A* search..." << std::endl;
    ljh::path::footstep_planner::AStarFootstepPlanner planner;
    planner.initialize(goalPose2D, goalPose, startPose);
    planner.doAStarSearch();
    planner.calFootstepSeries();
    auto outcome = planner.getFootstepSeries();

    std::cout << "Footsteps: " << outcome.size() << std::endl;
    if (outcome.empty()) { return 1; }

    int totalSteps = (int)outcome.size();
    int totalFrames = totalSteps + 1;
    std::cout << "Generating " << totalFrames << " frames..." << std::endl;

    for (int n = 0; n < totalFrames; n++)
    {
        int stepsToShow = n;

        plt::figure_size(900, 700);

        // Ellipsoid body path (always visible)
        drawEllipsoidPath(pathHolder);

        // Footsteps up to current step
        if (stepsToShow > 0)
        {
            drawFootsteps(outcome, stepsToShow);
        }

        // Start/goal markers ON TOP
        drawMarkers(startX, startY, startYaw, goalX, goalY, goalYaw);

        std::ostringstream fname;
        fname << outDir << "/frame_" << std::setw(3) << std::setfill('0') << n << ".png";
        plt::save(fname.str());
        plt::close();

        if (n % 5 == 0 || n == totalFrames - 1)
            std::cout << "  Frame " << n << "/" << totalFrames - 1 << std::endl;
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
