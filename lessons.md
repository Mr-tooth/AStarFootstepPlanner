
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
