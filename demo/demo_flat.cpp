// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0

// Demo: Flat terrain footstep planning visualization
// Generates progressive PNG frames showing A* footstep planning from start to goal

#include <matplotlibcpp.h>
#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/PlotCheck/PlotChecker.h>
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>

namespace plt = matplotlibcpp;
namespace fs = std::filesystem;

void drawFootsteps(const std::vector<ljh::path::footstep_planner::FootstepGraphNode>& steps,
                   int count)
{
    std::vector<double> vx, vy;
    for (int i = 0; i < count && i < (int)steps.size(); i++)
    {
        vx.clear();
        vy.clear();
        getFootVertex2D(steps.at(i), vx, vy);
        if (!vx.empty()) { vx.push_back(vx.front()); vy.push_back(vy.front()); }

        std::string color = (steps.at(i).getSecondStepSide().getStepFlag() == stepL) ? "red" : "orange";
        std::map<std::string, std::string> kw = {{"color", color}, {"linewidth", "1.2"}};
        plt::plot(vx, vy, kw);
    }
}

void drawBodyPath(const std::vector<ljh::path::footstep_planner::FootstepGraphNode>& steps,
                  int count)
{
    std::vector<double> px, py;
    for (int i = 0; i < count && i < (int)steps.size(); i++)
    {
        px.push_back(steps.at(i).getSecondStep().getX());
        py.push_back(steps.at(i).getSecondStep().getY());
    }
    if (px.size() >= 2)
    {
        std::map<std::string, std::string> kw = {{"color", "green"}, {"linewidth", "1.0"}};
        plt::plot(px, py, kw);
    }
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

    std::cout << "Running A* search..." << std::endl;
    ljh::path::footstep_planner::AStarFootstepPlanner planner;
    planner.initialize(goalPose2D, goalPose, startPose);
    planner.doAStarSearch();
    planner.calFootstepSeries();

    auto outcome = planner.getFootstepSeries();
    std::cout << "Discrete footsteps: " << outcome.size() << std::endl;

    if (outcome.empty())
    {
        std::cerr << "No footstep solution found!" << std::endl;
        return 1;
    }

    int totalSteps = (int)outcome.size();
    std::cout << "Generating " << (totalSteps + 1) << " frames..." << std::endl;

    double sx_start = startPose.getPosition().getX();
    double sy_start = startPose.getPosition().getY();
    double gx = goalPose.getPosition().getX();
    double gy = goalPose.getPosition().getY();
    double syaw = startPose.getOrientation().getYaw();
    double gyaw = goalPose.getOrientation().getYaw();
    const double arrowLen = 0.04;

    for (int n = 0; n <= totalSteps; n++)
    {
        plt::figure_size(800, 600);
        plt::axis("off");

        // Start/goal arrows
        plt::arrow(sx_start, sy_start, cos(syaw) * arrowLen, sin(syaw) * arrowLen, "green", "k", 0.015, 0.008);
        plt::arrow(gx, gy, cos(gyaw) * arrowLen, sin(gyaw) * arrowLen, "red", "k", 0.015, 0.008);

        // Start/goal dots
        std::vector<double> start_xv = {sx_start}, start_yv = {sy_start};
        std::vector<double> goal_xv = {gx}, goal_yv = {gy};
        plt::scatter(start_xv, start_yv, 30.0);
        plt::scatter(goal_xv, goal_yv, 30.0);

        if (n > 0)
        {
            drawBodyPath(outcome, n);
            drawFootsteps(outcome, n);

            // Step number annotations
            for (int i = 0; i < n && i < (int)outcome.size(); i++)
            {
                double x = outcome.at(i).getSecondStep().getX();
                double y = outcome.at(i).getSecondStep().getY();
                plt::annotate(std::to_string(i), x, y + 0.025);
            }
        }

        // Step label
        std::string label = n == 0 ? "0 / " + std::to_string(totalSteps)
                                   : std::to_string(n) + " / " + std::to_string(totalSteps);
        plt::text(-0.05, -0.95, label);

        plt::set_aspect_equal();

        std::ostringstream fname;
        fname << outDir << "/frame_" << std::setw(3) << std::setfill('0') << n << ".png";
        plt::save(fname.str());
        plt::close();

        if (n % 5 == 0 || n == totalSteps)
            std::cout << "  Frame " << n << "/" << totalSteps << std::endl;
    }

    std::cout << "All frames saved to " << outDir << "/" << std::endl;
    return 0;
}
