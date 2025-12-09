# Context for Rotation Lock Investigation

## SOLVED

The root-only object rotation lock issue has been resolved.

### The Problem
Root-only objects (like workbench) would spin when pitch was applied and the wand moved. This was because `SetAngle()` uses Euler angles which have gimbal lock issues when combining pitch with yaw tracking.

### The Solution
Use direct scene graph manipulation for root-only objects, same as pivot objects:

1. **Set `root3D->local.translate` and `root3D->local.rotate` directly** instead of using `SetPosition()`/`SetAngle()`
2. **Use `root3D->Update(updateData)` instead of `Update3DPosition(true)`** - the latter recalculates node rotation from stored Euler angles, overwriting our direct matrix changes
3. **In `ApplyPreviewTransforms`**, copy `local.translate` and `local.rotate` directly from preview to final scene graph nodes

### Key Discovery
`Update3DPosition(true)` overwrites `local.rotate` from the reference's stored Euler angles. Even `Update3DPosition(false)` does this. The solution is to bypass it entirely and use `NiAVObject::Update()` which propagates local transforms to world transforms without recalculating from angles.

### Code Changes Made

**Placement.cpp - Root case frame update:**
```cpp
// Set both position and rotation directly on the scene graph node
root3D->local.translate = originPos;
root3D->local.rotate = final;

// Update scene graph to propagate local -> world transforms
// This avoids Update3DPosition which overwrites rotation from stored Euler angles
RE::NiUpdateData updateData;
root3D->Update(updateData);
```

**Placement.cpp - ApplyPreviewTransforms for root-only:**
```cpp
// root-only / no pivot: copy transforms directly from scene graph
final3D->local.translate = preview3D->local.translate;
final3D->local.rotate = preview3D->local.rotate;

// Update scene graph to propagate local -> world
RE::NiUpdateData updateData;
final3D->Update(updateData);
```

**Also removed delta negation** - both root and pivot modes now use non-negated wand yaw delta since both use direct matrix manipulation.

## Object Types (from nifs.md)
1. **Anvil** - Has child pivot node `BlackSmithAnvil01` under root `BlackSmithMarker`
2. **Workbench** - Geometry directly at root `BlacksmithWorkBench01`, no child pivot

## Files Modified
- `src/Placement.cpp` - Main placement logic
- `src/Placement.h` - PlacementState struct

---

## Outstanding Issue: Pivot Node Selection

### Problem
Some objects grab the wrong mesh as the pivot node - appears to be selecting a node that's too deep in the hierarchy.

### Current Pivot Detection Logic (Placement.cpp:117-167)
The `FindPivotNode` function:
1. Gets root3D and casts to NiNode
2. Iterates over **immediate children** of root only (not recursive for pivot selection)
3. Skips non-NiNode children (pure geometry)
4. Skips nodes with "Collision" in the name
5. Returns **first** NiNode child that has geometry somewhere in its subtree

**The issue**: It returns the FIRST matching NiNode child. If the hierarchy has multiple NiNode children, or if the intended pivot is nested differently, it may pick the wrong one.

```cpp
for (auto& child : rootNode->children) {
    auto* node = obj->AsNode();
    if (!node) continue;  // skip geometry
    if (name contains "Collision") continue;
    if (hasGeometryRecursive(node)) {
        return node;  // Returns FIRST match!
    }
}
```

### Potential Approaches to Investigate
1. **Better heuristics for "main" pivot** - Instead of first match, look for largest bounding box, most geometry, or specific naming patterns
2. **Prefer nodes with non-identity local transforms** - A true pivot node likely has a local offset from root
3. **Check if root has geometry directly** - If root itself has geometry children (not just NiNode children), treat as root-only
4. **Multiple pivot candidates** - Log all candidates and their properties to understand patterns
5. **Fallback to root-only** - If multiple ambiguous candidates, safer to use root-only mode which now works correctly

### To Investigate
- Examine NIFs of problematic objects to understand their hierarchy
- Log the full node tree when pivot selection happens to see what candidates exist
- Compare working (anvil) vs problematic object hierarchies
- Check if problematic objects have geometry at multiple levels (root + children)
