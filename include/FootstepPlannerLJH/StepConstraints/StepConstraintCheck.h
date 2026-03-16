// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/EnumDef.h>
#include <FootstepPlannerLJH/FootstepplannerBasic.h>
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>
#include <Heuclid/geometry/ConvexPolygon2D.h>
#include <Heuclid/geometry/tools/HeuclidPolygonTools.h>
#include <Heuclid/geometry/Pose2D.h>
#include <matplotlibcpp.h>
#include <vector>
_FOOTSTEP_PLANNER_BEGIN

/**
 * @class StepConstraintCheck
 * @brief Step collision and constraint checker for footstep planning.
 *
 * Checks various step constraints including:
 * - Foot Foot collision detection (stance vs swing foot)
 * - Stair region constraints (foot alignment)
 * - Goal pose feasibility
 */
class StepConstraintCheck
{
private:
    parameters param;
    ljh::heuclid::HeuclidGeometryPolygonTools polygonTools;

    ljh::heuclid::Pose2D<double> stepPose;
    std::vector<double> vertexX;
    std::vector<double> vertexY;
    std::vector<double> vertexX8;
    std::vector<double> vertexY8;

    ljh::heuclid::Point2D<double> vertex;
    std::vector<Point2D<double> > stanceBuffer;

public:
    /**
     * @brief Default constructor.
     */
    StepConstraintCheck():param(),polygonTools(),stepPose(),vertexX(),vertexY(),vertexX8(),vertexY8(),vertex(),stanceBuffer(){};

    /**
     * @brief Initialize the constraint checker.
     */
    void initialize();

    /**
     * @brief Check if any foot vertex is inside the stair region.
     * @param stepX Foot x position
     * @param stepY Foot y position
     * @param stepYaw Foot yaw angle
     * @param stepFlag Foot side (left or right)
     * @param stairVertexBuffer Stair region vertices
     * @param numOfVertices Number of stair vertices
     * @param clockwiseOrdered Vertex ordering flag
     * @return true if any vertex is inside, false otherwise
     */
    bool isAnyVertexOfFootInsideStairRegion(double stepX, double stepY, double stepYaw, enum StepFlag stepFlag,
                                            std::vector<Point2D<double> > stairVertexBuffer, int numOfVertices, bool clockwiseOrdered);

    /**
     * @brief Check if any foot vertex is inside the stair region (DiscreteFootstep version).
     * @param stepToCheck Footstep to check
     * @param stairPolygon Stair region polygon
     * @return true if any vertex is inside, false otherwise
     */
    bool isAnyVertexOfFootInsideStairRegion(DiscreteFootstep stepToCheck, ljh::heuclid::ConvexPolygon2D stairPolygon);

    /**
     * @brief Check if two feet collide (pose-based).
     * @param stanceX Stance foot x position
     * @param stanceY Stance foot y position
     * @param stanceYaw Stance foot yaw angle
     * @param stanceFlag Stance foot side
     * @param swingX Swing foot x position
     * @param swingY Swing foot y position
     * @param swingYaw Swing foot yaw angle
     * @param swingFlag Swing foot side
     * @return true if feet collide, false otherwise
     */
    bool isTwoFootCollided(double stanceX, double stanceY, double stanceYaw, enum StepFlag stanceFlag,
                           double swingX,  double swingY,  double swingYaw,  enum StepFlag swingFlag);

    /**
     * @brief Check if two feet collide (DiscreteFootstep version).
     * @param stanceStep Stance footstep
     * @param swingStep Swing footstep
     * @return true if feet collide, false otherwise
     */
    bool isTwoFootCollided(DiscreteFootstep stanceStep, DiscreteFootstep swingStep);

    /**
     * @brief Check if the two feet in a stance node collide.
     * @param nodeToCheck Stance node containing both
     * @return true if feet collide, false otherwise
     */
    bool isTwoFootCollided(FootstepGraphNode nodeToCheck);

    /**
     * @brief Check if two feet collide with visualization (pose-based).
     * @param stanceX Stance foot x position
     * @param stanceY Stance foot y position
     * @param stanceYaw Stance foot yaw angle
     * @param stanceFlag Stance foot side
     * @param swingX Swing foot x position
     * @param swingY Swing foot y position
     * @param swingYaw Swing foot yaw angle
     * @param swingFlag Swing foot side
     * @return true if feet collide, false otherwise
     */
    bool isTwoFootCollidedAndPlot(double stanceX, double stanceY, double stanceYaw, enum StepFlag stanceFlag,
                           double swingX,  double swingY,  double swingYaw,  enum StepFlag swingFlag);

    /**
     * @brief Check if two feet collide with visualization (DiscreteFootstep version).
     * @param stanceStep Stance footstep
     * @param swingStep Swing footstep
     * @return true if feet collide, false otherwise
     */
    bool isTwoFootCollidedAndPlot(DiscreteFootstep stanceStep, DiscreteFootstep swingStep);

    /**
     * @brief Check if the two feet in a stance node collide with visualization.
     * @param nodeToCheck Stance node containing both
     * @return true if feet collide, false otherwise
     */
    bool isTwoFootCollidedAndPlot(FootstepGraphNode nodeToCheck);

    /**
     * @brief Check if goal pose feet collide with stair region.
     * @param _goalPose Goal body pose
     * @param stairPolygon Stair region polygon
     * @return true if collision detected, false otherwise
     */
    bool isGoalPoseCollidedWithStairRegion(ljh::heuclid::Pose3D<double> _goalPose,ljh::heuclid::ConvexPolygon2D stairPolygon);
};





_FOOTSTEP_PLANNER_END
