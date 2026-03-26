// Copyright 2026 Junhang Lai
// SPDX-License-Identifier: Apache-2.0
//
// Stepping-stone terrain demo — "base plane + elevated stones" approach.
// A large flat base plane at height=0 covers the full 5m distance.
// Small elevated stepping stones (5~15cm height) are scattered on top.
// A* finds a path using the base plane; stepping onto stones is optional.

#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <Heuclid/geometry/ConvexPolygon2D.h>
#include <Heuclid/euclid/tuple2D/Point2D.h>
#include <Heuclid/geometry/Pose2D.h>
#include <Heuclid/geometry/Pose3D.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace ljh::heuclid;
using namespace ljh::path::footstep_planner;

#ifndef PI
#define PI 3.1415926535
#endif

static unsigned int rng_seed = 42;
static float frand() { rng_seed = rng_seed * 1103515245 + 12345; return (rng_seed & 0x7fffffff) / (float)0x7fffffff; }
static float frand_range(float lo, float hi) { return lo + frand() * (hi - lo); }

struct HeightPatch {
    ConvexPolygon2D poly;
    double height;
    std::string label;  // "base" or "stone_0", "stone_1", etc.
};

static ConvexPolygon2D makeRect(double cx, double cy, double hw, double hh, double angle)
{
    std::vector<Point2D<double>> v(4);
    double cos_a = cos(angle), sin_a = sin(angle);
    double lx[4] = {-hw,  hw,  hw, -hw};
    double ly[4] = {-hh, -hh,  hh,  hh};
    for(int i = 0; i < 4; i++)
        v[i].setPoint2D(cx + cos_a*lx[i] - sin_a*ly[i], cy + sin_a*lx[i] + cos_a*ly[i]);
    ConvexPolygon2D poly(4);
    poly.setVertexBuffer(v);
    poly.setClockwiseOrder(false);
    return poly;
}

static std::vector<AccurateFootstep> planSteps(parameters& param,
    double sx, double sy, double syaw, double gx, double gy, double gyaw)
{
    Pose2D<double> g2d(gx, gy, gyaw);
    Pose3D<double> gp(gx, gy, 0.0, gyaw, 0.0, 0.0);
    Pose3D<double> sp(sx, sy, 0.0, syaw, 0.0, 0.0);
    AStarFootstepPlanner planner;
    planner.initialize(g2d, gp, sp);
    planner.doAStarSearch();
    planner.calFootstepSeries();
    return planner.getOrCalAccurateFootstepSeries();
}

static void setupParams(parameters& param)
{
    param.SetEdgeCostDistance(param, 3.0);
    param.SetEdgeCostYaw(param, 2.0);
    param.SetEdgeCostStaticPerStep(param, 1.0);
    param.SetDebugFlag(param, false);
    param.SetMaxStepYaw(param, PI / 6);
    param.SetMinStepYaw(param, -PI / 6);
    param.SetFinalTurnProximity(param, 0.4);
    param.SetGoalDistanceProximity(param, 0.06);
    param.SetGoalYawProximity(param, 15.0 / 180.0 * PI);
    param.SetFootPolygonExtendedLength(param, 0.01);
    param.SetHWPOfWalkDistacne(param, 1.80);
    param.SetFollowBodyPath(param, false);
    param.SetHWPOfPathDistance(param, 1.0);
    param.SetEdgeCostPathDev(param, 0.0);
    param.SetHWPOfInitialTurnDistacne(param, 1.0);
    param.SetHWPOfFinalTurnDistacne(param, 1.80);
    param.SetHWPOfFinalWalkDistacne(param, 1.80);
    param.SetMaxStepLength(param, 0.55);
    param.SetMinStepLength(param, -0.15);
    param.SetMaxStepWidth(param, 0.40);
    param.SetMinStepWidth(param, 0.10);
    param.SetMaxStepReach(param, 0.65);
}

// Generate base plane + stepping stones with heights
static std::vector<HeightPatch> generateSteppingStones(double sx, double sy, double gx, double gy)
{
    std::vector<HeightPatch> result;
    double dx = gx - sx, dy = gy - sy;
    double pathAngle = atan2(dy, dx);
    double perpAngle = pathAngle + PI / 2;

    // Base plane: large, covers full distance, height = 0
    double base_hw = 2.5;  // covers from -0.5 to 4.0
    double base_hh = 2.0;
    double base_cx = (sx + gx) / 2.0;
    double base_cy = (sy + gy) / 2.0;
    result.push_back({makeRect(base_cx, base_cy, base_hw, base_hh, pathAngle), 0.0, "base"});

    // Stepping stones: 4~6 small elevated patches, evenly spaced along full path
    rng_seed = 42;
    int nStones = 4 + (int)(frand() * 3);  // 4~6 stones
    for (int i = 0; i < nStones; i++)
    {
        double t = 0.15 + 0.70 * (double)i / (nStones - 1);  // evenly spaced 15%~85% of path
        double lateral = frand_range(-0.4, 0.4);              // lateral offset
        double cx = sx + dx * t + cos(perpAngle) * lateral;
        double cy = sy + dy * t + sin(perpAngle) * lateral;
        double hw = frand_range(0.25, 0.40);                  // slightly larger stones
        double hh = frand_range(0.20, 0.35);
        double angle = pathAngle + frand_range(-0.10, 0.10);  // less rotation
        double height = frand_range(0.05, 0.15);              // 5~15cm

        result.push_back({makeRect(cx, cy, hw, hh, angle), height, "stone_" + std::to_string(i)});
    }
    return result;
}

int main()
{
    // 2m straight-line distance
    double sx = 0.0, sy = 0.0, syaw = 0.0;
    double gx = 3.5, gy = 0.0, gyaw = 0.0;

    auto patches = generateSteppingStones(sx, sy, gx, gy);

    // Extract just the 2D polygons for the planner (landings are all patches including base)
    std::vector<ConvexPolygon2D> landingPols;
    for (const auto& hp : patches)
        landingPols.push_back(hp.poly);

    parameters param;
    setupParams(param);
    param.SetLandingZonePolygons(param, landingPols);
    param.SetUseLandingZoneCheck(param, true);
    param.SetStairAlignMode(param, false);

    auto bestSteps = planSteps(param, sx, sy, syaw, gx, gy, gyaw);

    std::cout << "Result: " << bestSteps.size() << " steps, " << patches.size() << " patches" << std::endl;
    for (const auto& hp : patches)
        std::cout << "  " << hp.label << ": h=" << hp.height * 100 << "cm" << std::endl;

    // Export terrain patches (x,y only — height in separate file)
    for (size_t i = 0; i < patches.size(); i++) {
        std::ofstream fout("demo/terrain_" + std::to_string(i) + ".csv");
        fout << "x,y" << std::endl;
        for (const auto& p : patches[i].poly.getVertexBuffer())
            fout << p.getX() << "," << p.getY() << std::endl;
        fout.close();
    }

    // Export height info
    {
        std::ofstream fout("demo/terrain_heights.csv");
        fout << "index,label,height_m" << std::endl;
        for (size_t i = 0; i < patches.size(); i++)
            fout << i << "," << patches[i].label << "," << patches[i].height << std::endl;
    }

    // Body path
    {
        std::ofstream fout("demo/body_path.csv");
        fout << "x,y" << std::endl;
        for (int i = 0; i < 100; i++) {
            double t = (double)i / 99.0;
            fout << sx + (gx - sx) * t << "," << sy + (gy - sy) * t << std::endl;
        }
    }

    // Start/goal
    {
        std::ofstream fout("demo/start_goal.csv");
        fout << "x,y,type" << std::endl;
        fout << sx << "," << sy << ",start" << std::endl;
        fout << gx << "," << gy << ",goal" << std::endl;
    }

    // Footsteps
    {
        std::ofstream fout("demo/footsteps.csv");
        fout << "step,x,y,yaw,side" << std::endl;
        int idx = 0;
        for (const auto& s : bestSteps)
            fout << idx++ << "," << s.getX() << "," << s.getY() << ","
                 << s.getYaw() << "," << (s.getStepFlag() == stepL ? "L" : "R") << std::endl;
    }

    // Foot polygons
    {
        std::ofstream fout("demo/foot_polygons.csv");
        fout << "step,x1,y1,x2,y2,x3,y3,x4,y4" << std::endl;
        int idx = 0;
        for (const auto& s : bestSteps) {
            std::vector<double> vx, vy;
            getExtendedFootVertex2D(Pose2D<double>(s.getX(), s.getY(), s.getYaw()),
                                    s.getStepFlag(), vx, vy, 0.025);
            fout << idx++ << ",";
            for (int i = 0; i < 4; i++)
                fout << vx[i] << "," << vy[i] << (i < 3 ? "," : "");
            fout << std::endl;
        }
    }

    return 0;
}
