
## 2026-03-17: Demo + Body Path Following Debugging

### Issue 1: Body path heuristic tied to `isStairAlignMode`
- **Problem**: `computeFollowEllipsoidPath` only called when `isStairAlignMode = true`, which also enables stair polygon constraints
- **Fix**: Added `followBodyPath` static parameter — independent flag for body path heuristic
- **Lesson**: Decouple orthogonal features. A flag controlling 2 unrelated behaviors is an anti-pattern.

### Issue 2: `pfp.distance` is squared, not Euclidean
- **Problem**: `Simple2DBodyPathHolder::getClosestdPointsYawfromPathToGivenPoint` stores `(X-x)²+(Y-y)²` in `pfp.distance`, but code compared it directly to `IdealStepWidth`
- **Fix**: `std::abs(std::sqrt(this->pfp.distance) - IdealStepWidth)`
- **Lesson**: Always check if a distance is squared. `sqrt()` is easy to miss in calling code.

### Issue 3: Duplicate parameter override in demo
- **Problem**: `SetFollowBodyPath(param, true)` on line 65, then `SetFollowBodyPath(param, false)` on line 80
- **Root cause**: Incremental editing left duplicate code blocks
- **Fix**: Remove duplicate section
- **Lesson**: Static members are set once, later calls silently override. Grep for all occurrences before debugging.

### Issue 4: Edge cost penalty too weak
- **Problem**: `edgecost_w_pathdev = 3.0` added only ~0.18 per step vs walk cost of ~0.32 — negligible
- **Insight**: Edge cost penalty has limited leverage. The heuristic is what truly guides the search.
- **Lesson**: Fix the heuristic, not the edge cost, for path following behavior.

### Issue 5: `isAnyVertexOfFootInsideStairRegion` incomplete for obstacle avoidance
- **Problem**: Only checks if foot vertices are inside the obstacle polygon. Misses:
  1. Obstacle vertices inside foot rectangle (small obstacle near foot edge)
  2. Edge-edge intersection (foot straddles obstacle boundary — no vertex inside either polygon)
- **Impact**: Foot and obstacle visually overlap in demo GIF but planner doesn't reject the step
- **User requirement**: Full polygon-polygon collision detection (similar to `isTwoFootCollided` algorithm)
- **Solution proposed**: Add `isPolygonCollided()` in `StepConstraintCheck` using 16-point mutual containment test (4 vertices + 4 edge midpoints × 2 directions)
- **Lesson**: Point-in-polygon is necessary but NOT sufficient for convex polygon intersection. Need mutual containment + edge intersection for robustness.

### Phase 3: Terrain Patch Sizing for Landing Zone Demo
- **Issue**: Landing zone patches centered on footsteps still fail containment check
- **Root cause**: Foot polygon dimensions (ForwardLength=0.16m + BackwardLength=0.11m = 0.27m total length) exceed patch width when patch HW=0.15m
- **Key insight**: `ForwardLength`, `BackwardLength`, `WideWidth`, `NarrowWidth` are single-side half-dimensions from foot center — total foot size = sum of both sides
- **Rule**: patch half-width must be ≥ max(ForwardLength, BackwardLength) + footPolygonExtendedLength (≥ 0.185m)
- **"Shoot first, aim later"**: User suggested defining ideal footstep positions first, then building patches around them — simpler than tuning patches to make planner succeed
- **Lesson**: For demo purposes, "center patches on known-good footsteps" is much easier than "tune patch dimensions until planner finds path"
