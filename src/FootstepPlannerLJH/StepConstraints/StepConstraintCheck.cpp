// Copyright 2026 Junhang Lai
// SPDX-License-Identifier: Apache-2.0


#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>

_FOOTSTEP_PLANNER_BEGIN

void StepConstraintCheck::initialize()
{
    this->param = parameters();
    //this->polygonTools = ljh::heuclid::HeuclidGeometryPolygonTools();
    ljh::heuclid::Pose2D<double> Res;
    this->stepPose = Res;
    this->vertexX.clear();
    this->vertexY.clear();
    this->vertexX8.clear();
    this->vertexY8.clear();
    this->vertex.setPoint2D(0.0,0.0);
    this->stanceBuffer.clear();
}


bool StepConstraintCheck::isAnyVertexOfFootInsideStairRegion(double stepX, double stepY, double stepYaw, enum StepFlag stepFlag,
                                            std::vector<Point2D<double> > stairVertexBuffer, int numOfVertices, bool clockwiseOrdered)
{
    this->stepPose.setPosition(stepX,stepY);
    this->stepPose.setOrientation(stepYaw);
    getExtendedFootVertex2D(this->stepPose,stepFlag,this->vertexX,this->vertexY,0.0);
    for(int i=0;i<numOfVertices;i++)
    {
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(this->vertexX.at(i),this->vertexY.at(i),stairVertexBuffer,numOfVertices,clockwiseOrdered))
            return true;
    }

    return false;
}

bool StepConstraintCheck::isAnyVertexOfFootInsideStairRegion(DiscreteFootstep stepToCheck, ljh::heuclid::ConvexPolygon2D stairPolygon)
{
    return this->isAnyVertexOfFootInsideStairRegion(stepToCheck.getX(),stepToCheck.getY(),stepToCheck.getYaw(),stepToCheck.getRobotSide().getStepFlag(),
                                                    stairPolygon.getVertexBuffer(),stairPolygon.getNumOfVertices(),stairPolygon.getClockwiseOrder());
}

// ============================================================================
// Helper: Build foot ConvexPolygon2D from pose + side
// ============================================================================

static ljh::heuclid::ConvexPolygon2D buildFootPolygon(double x, double y, double yaw,
                                                        enum StepFlag flag,
                                                        const parameters& param)
{
    ljh::heuclid::Pose2D<double> pose;
    pose.setPosition(x, y);
    pose.setOrientation(yaw);

    std::vector<double> vx, vy;
    getExtendedFootVertex2D(pose, flag, vx, vy, param.footPolygonExtendedLength);

    std::vector<ljh::heuclid::Point2D<double>> pts(4);
    for(int i = 0; i < 4; i++)
    {
        pts[i].setPoint2D(vx.at(i), vy.at(i));
    }

    ljh::heuclid::ConvexPolygon2D poly(4);
    poly.setVertexBuffer(pts);
    poly.setClockwiseOrder(true);
    return poly;
}

// ============================================================================
// Refactored: Foot-foot collision using Heuclid isConvexPolygonIntersect
// ============================================================================

bool StepConstraintCheck::isTwoFootCollided(double stanceX, double stanceY, double stanceYaw, enum StepFlag stanceFlag,
                        double swingX,  double swingY,  double swingYaw,  enum StepFlag swingFlag)
{
    auto stancePoly = buildFootPolygon(stanceX, stanceY, stanceYaw, stanceFlag, this->param);
    auto swingPoly  = buildFootPolygon(swingX,  swingY,  swingYaw,  swingFlag,  this->param);
    return this->polygonTools.isConvexPolygonIntersect(stancePoly, swingPoly);
}   

bool StepConstraintCheck::isTwoFootCollided(DiscreteFootstep stanceStep, DiscreteFootstep swingStep)
{
    return  this->isTwoFootCollided(stanceStep.getX(),stanceStep.getY(),stanceStep.getYaw(),stanceStep.getRobotSide().getStepFlag(),
                                    swingStep.getX() ,swingStep.getY() ,swingStep.getYaw() ,swingStep.getRobotSide().getStepFlag()); 
}

bool StepConstraintCheck::isTwoFootCollided(FootstepGraphNode nodeToCheck)
{
    return this->isTwoFootCollided(nodeToCheck.getFirstStep(),nodeToCheck.getSecondStep());
}


bool StepConstraintCheck::isTwoFootCollidedAndPlot(double stanceX, double stanceY, double stanceYaw, enum StepFlag stanceFlag,
                           double swingX,  double swingY,  double swingYaw,  enum StepFlag swingFlag)
{
#ifdef HAS_MATPLOTLIB
    namespace plt = matplotlibcpp;
#endif
    //calculate and load the vertex2d(in clockwiseorder) of the stanceStep as polygon
    this->stepPose.setPosition(stanceX,stanceY);
    this->stepPose.setOrientation(stanceYaw);
    getExtendedFootVertex2D(this->stepPose,stanceFlag,this->vertexX8,this->vertexY8,this->param.footPolygonExtendedLength);
    
#ifdef HAS_MATPLOTLIB
    plt::plot(this->vertexX8,this->vertexY8,"r");
#endif

    this->stanceBuffer.resize(4);
    for(int i=0;i<4;i++)
    {
        this->vertex.setPoint2D(this->vertexX8.at(i),this->vertexY8.at(i));
        this->stanceBuffer[i] = this->vertex;
    }
    // calculate the vertex2d of the swingStep
    this->stepPose.setPosition(swingX,swingY);
    this->stepPose.setOrientation(swingYaw);
    getExtendedFootVertex2D(this->stepPose,swingFlag,this->vertexX,this->vertexY,this->param.footPolygonExtendedLength);
#ifdef HAS_MATPLOTLIB
    plt::plot(vertexX,vertexY,"g");
#endif
#ifdef HAS_MATPLOTLIB
    plt::scatter(vertexX,vertexY,20.0);
#endif
    // Four Vertex is not enough, need Eight Vertices
    //this->vertexX8.clear(); this->vertexY8.clear();

#ifdef HAS_MATPLOTLIB
    plt::set_aspect_equal();
#endif
#ifdef HAS_MATPLOTLIB
    plt::      show();
#endif
    // check each vertex of swingStep Whether in stanceStep polygon
    for(int i=0;i<4;i++)
    {
        // getFootVertex2D would make an clockwise order
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(this->vertexX.at(i),this->vertexY.at(i),this->stanceBuffer,4,1))
            return true;
    }
    // check 4 middle points of edge
    for(int i=0;i<4;i++)
    {
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(0.5*(this->vertexX.at(i)+this->vertexX.at((i+1)%4)),0.5*(this->vertexY.at(i)+this->vertexY.at((i+1)%4)),this->stanceBuffer,4,1))
            return true;
    } 

    // check each vertex of stanceStep Whether in swingStep polygon
    this->stanceBuffer.resize(4);
    for(int i=0;i<4;i++)
    {
        this->vertex.setPoint2D(this->vertexX.at(i),this->vertexY.at(i));
        this->stanceBuffer[i] = this->vertex;
    }

    for(int i=0;i<4;i++)
    {
        // getFootVertex2D would make an clockwise order
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(this->vertexX8.at(i),this->vertexY8.at(i),this->stanceBuffer,4,1))
            return true;
    }

    // check 4 middle points of edge
    for(int i=0;i<4;i++)
    {
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(0.5*(this->vertexX8.at(i)+this->vertexX8.at((i+1)%4)),0.5*(this->vertexY8.at(i)+this->vertexY8.at((i+1)%4)),this->stanceBuffer,4,1))
            return true;
    }

    return false;
}

bool StepConstraintCheck::isTwoFootCollidedAndPlot(DiscreteFootstep stanceStep, DiscreteFootstep swingStep)
{
    return this->isTwoFootCollidedAndPlot(stanceStep.getX(),stanceStep.getY(),stanceStep.getYaw(),stanceStep.getRobotSide().getStepFlag(),
                                          swingStep.getX() ,swingStep.getY() ,swingStep.getYaw() ,swingStep.getRobotSide().getStepFlag()); 
}

bool StepConstraintCheck::isTwoFootCollidedAndPlot(FootstepGraphNode nodeToCheck)
{
    return this->isTwoFootCollidedAndPlot(nodeToCheck.getFirstStep(),nodeToCheck.getSecondStep());
}


bool StepConstraintCheck::isGoalPoseCollidedWithStairRegion(ljh::heuclid::Pose3D<double> _goalPose,ljh::heuclid::ConvexPolygon2D stairPolygon)
{
    ljh::heuclid::Point2D<double> centralPoint;
    std::vector<ljh::heuclid::Point2D<double> > stairBuffer = stairPolygon.getVertexBuffer();
    double x = 0.0;
    double y = 0.0;
    for(int i=0;i<4;i++)
    {
        x +=stairBuffer.at(i).getX();
        y +=stairBuffer.at(i).getY();
    }
    centralPoint.setX(x/4.0);
    centralPoint.setY(y/4.0);

    double length = std::sqrt(std::pow(centralPoint.getX()-_goalPose.getPosition().getX(),2) + std::pow(centralPoint.getX()-_goalPose.getPosition().getX(),2));
    return false;   
}

// ============================================================================
// AABB helper: compute axis-aligned bounding box of a convex polygon
// ============================================================================
static void computeAABB(const ljh::heuclid::ConvexPolygon2D& poly,
                        double& minX, double& minY, double& maxX, double& maxY)
{
    const auto& verts = poly.getVertexBuffer();
    minX = maxX = verts[0].getX();
    minY = maxY = verts[0].getY();
    for(size_t i = 1; i < verts.size(); i++)
    {
        double x = verts[i].getX(), y = verts[i].getY();
        if(x < minX) minX = x; if(x > maxX) maxX = x;
        if(y < minY) minY = y; if(y > maxY) maxY = y;
    }
}

// ============================================================================
// New: Foot-obstacle collision (full polygon-polygon intersection with AABB fast-reject)
// ============================================================================

bool StepConstraintCheck::isFootPolygonCollidedWithPolygon(double stepX, double stepY, double stepYaw, enum StepFlag stepFlag,
                                                           ljh::heuclid::ConvexPolygon2D obstaclePolygon)
{
    // Degenerate polygon: need at least 3 vertices for a valid polygon
    if(obstaclePolygon.getNumOfVertices() < 3)
        return false;

    auto footPoly = buildFootPolygon(stepX, stepY, stepYaw, stepFlag, this->param);

    // AABB fast-reject: if bounding boxes don't overlap, polygons can't intersect
    double fMinX, fMinY, fMaxX, fMaxY, oMinX, oMinY, oMaxX, oMaxY;
    computeAABB(footPoly, fMinX, fMinY, fMaxX, fMaxY);
    computeAABB(obstaclePolygon, oMinX, oMinY, oMaxX, oMaxY);
    if(fMaxX < oMinX || oMaxX < fMinX || fMaxY < oMinY || oMaxY < fMinY)
        return false;

    return this->polygonTools.isConvexPolygonIntersect(footPoly, obstaclePolygon);
}

bool StepConstraintCheck::isFootPolygonCollidedWithPolygon(DiscreteFootstep stepToCheck, ljh::heuclid::ConvexPolygon2D obstaclePolygon)
{
    return this->isFootPolygonCollidedWithPolygon(stepToCheck.getX(), stepToCheck.getY(), stepToCheck.getYaw(),
                                                   stepToCheck.getRobotSide().getStepFlag(), obstaclePolygon);
}

// ============================================================================
// New: Foot-terrain containment (all foot vertices must be inside terrain)
// ============================================================================

bool StepConstraintCheck::isFootPolygonContainedInPolygon(double stepX, double stepY, double stepYaw, enum StepFlag stepFlag,
                                                          ljh::heuclid::ConvexPolygon2D terrainPolygon)
{
    auto footPoly = buildFootPolygon(stepX, stepY, stepYaw, stepFlag, this->param);
    return this->polygonTools.isConvexPolygonContained(footPoly, terrainPolygon);
}

bool StepConstraintCheck::isFootPolygonContainedInPolygon(DiscreteFootstep stepToCheck, ljh::heuclid::ConvexPolygon2D terrainPolygon)
{
    return this->isFootPolygonContainedInPolygon(stepToCheck.getX(), stepToCheck.getY(), stepToCheck.getYaw(),
                                                  stepToCheck.getRobotSide().getStepFlag(), terrainPolygon);
}

_FOOTSTEP_PLANNER_END
