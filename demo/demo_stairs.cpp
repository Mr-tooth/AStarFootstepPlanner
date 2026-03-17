// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0
//
// Stepping-stone terrain demo.
// Large random terrain patches, relaxed step constraints.
// A* finds path through patches naturally.

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

static unsigned int rng_seed = 0;
static float frand() { rng_seed = rng_seed * 1103515245 + 12345; return (rng_seed & 0x7fffffff) / (float)0x7fffffff; }
static float frand_range(float lo, float hi) { return lo + frand() * (hi - lo); }

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
    param.SetFollowBodyPath(param, true);
    param.SetHWPOfPathDistance(param, 1.0);
    param.SetEdgeCostPathDev(param, 0.0);
    param.SetHWPOfInitialTurnDistacne(param, 1.0);
    param.SetHWPOfFinalTurnDistacne(param, 1.80);
    param.SetHWPOfFinalWalkDistacne(param, 1.80);
    param.SetMaxStepLength(param, 0.40);
    param.SetMinStepLength(param, -0.15);
    param.SetMaxStepWidth(param, 0.35);
    param.SetMinStepWidth(param, 0.10);
    param.SetMaxStepReach(param, 0.45);
}

static std::vector<ConvexPolygon2D> generatePatches(double sx, double sy, double gx, double gy, int seed)
{
    rng_seed = seed;
    std::vector<ConvexPolygon2D> patches;
    double dx = gx - sx, dy = gy - sy;
    double pathAngle = atan2(dy, dx);
    double perpAngle = pathAngle + PI / 2;
    int n = 5 + (int)(frand() * 3);
    for(int i = 0; i < n; i++)
    {
        double t = (double)i / (n - 1);
        double offset = frand_range(-0.4, 0.4);
        double cx = sx + dx * t + cos(perpAngle) * offset;
        double cy = sy + dy * t + sin(perpAngle) * offset;
        double hw = frand_range(0.4, 1.0);
        double hh = frand_range(0.3, 0.8);
        double angle = pathAngle + frand_range(-0.4, 0.4);
        patches.push_back(makeRect(cx, cy, hw, hh, angle));
    }
    return patches;
}

int main()
{
    double sx = 0.1, sy = 0.1, syaw = 0.0;
    double gx = 1.2, gy = -0.5, gyaw = -PI / 3;

    std::vector<int> seeds = {42, 17, 73, 101, 256, 500, 777, 123, 888, 314};
    int bestSeed = -1;
    std::vector<AccurateFootstep> bestSteps;

    for(int seed : seeds)
    {
        auto patches = generatePatches(sx, sy, gx, gy, seed);
        parameters param;
        setupParams(param);
        param.SetLandingZonePolygons(param, patches);
        param.SetUseLandingZoneCheck(param, true);
        param.SetStairAlignMode(param, false);
        auto steps = planSteps(param, sx, sy, syaw, gx, gy, gyaw);
        std::cout << "Seed " << seed << ": " << steps.size() << " steps" << std::endl;
        if((int)steps.size() >= 8) { bestSeed = seed; bestSteps = steps; break; }
    }

    if(bestSeed < 0)
    {
        std::cerr << "No seed worked, falling back to unconstrained" << std::endl;
        parameters param; setupParams(param);
        param.SetUseLandingZoneCheck(param, false);
        bestSteps = planSteps(param, sx, sy, syaw, gx, gy, gyaw);
        bestSeed = seeds[0];
    }

    auto patches = generatePatches(sx, sy, gx, gy, bestSeed);
    std::cout << "Result: " << bestSteps.size() << " steps, " << patches.size() << " patches" << std::endl;

    for(size_t i = 0; i < patches.size(); i++) {
        std::ofstream fout("demo/terrain_" + std::to_string(i) + ".csv");
        fout << "x,y" << std::endl;
        for(const auto& p : patches[i].getVertexBuffer()) fout << p.getX() << "," << p.getY() << std::endl;
        fout.close();
    }
    {
        std::ofstream fout("demo/body_path.csv"); fout << "x,y" << std::endl;
        for(int i = 0; i < 100; i++) { double t = (double)i/99.0; fout << sx+(gx-sx)*t << "," << sy+(gy-sy)*t << std::endl; }
    }
    {
        std::ofstream fout("demo/start_goal.csv"); fout << "x,y,type" << std::endl;
        fout << sx << "," << sy << ",start" << std::endl << gx << "," << gy << ",goal" << std::endl;
    }
    {
        std::ofstream fout("demo/footsteps.csv"); fout << "step,x,y,yaw,side" << std::endl;
        int idx = 0;
        for(const auto& s : bestSteps)
            fout << idx++ << "," << s.getX() << "," << s.getY() << "," << s.getYaw() << "," << (s.getStepFlag()==stepL?"L":"R") << std::endl;
    }
    {
        std::ofstream fout("demo/foot_polygons.csv"); fout << "step,x1,y1,x2,y2,x3,y3,x4,y4" << std::endl;
        int idx = 0;
        for(const auto& s : bestSteps) {
            std::vector<double> vx, vy;
            getExtendedFootVertex2D(Pose2D<double>(s.getX(),s.getY(),s.getYaw()), s.getStepFlag(), vx, vy, 0.025);
            fout << idx++ << ",";
            for(int i=0;i<4;i++) fout << vx[i] << "," << vy[i] << (i<3?",":"");
            fout << std::endl;
        }
    }
    return 0;
}
