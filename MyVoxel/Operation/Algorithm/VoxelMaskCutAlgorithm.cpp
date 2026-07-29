#include "VoxelMaskCutAlgorithm.h"

#include <cassert>
#include <cstddef>

#include "MyVoxel/Core/Storage/VoxelLeafBlock.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeConstCursor.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeEditor.h"

namespace
{

// 对两个位于同一逻辑地址的节点执行同步体素布尔减。
MyVoxel::Operation::Algorithm::VoxelMaskCutResult cutAlignedCell(
    MyVoxel::VoxelPackedTreeEditor objectEditor,
    MyVoxel::VoxelPackedTreeConstCursor toolCursor,
    MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics* statistics)
{
    using namespace MyVoxel;
    using namespace MyVoxel::Operation::Algorithm;

    if (statistics)
    {
        ++statistics->visitedCellCount;
    }

    const VoxelState objectState = objectEditor.state();
    const VoxelState toolState = toolCursor.state();

    if (objectState == VoxelState::Empty)
    {
        if (statistics)
        {
            ++statistics->emptyObjectSkipCount;
        }

        return VoxelMaskCutResult(VoxelState::Empty, false);
    }

    if (toolState == VoxelState::Empty)
    {
        if (statistics)
        {
            ++statistics->emptyToolSkipCount;
        }

        return VoxelMaskCutResult(objectState, false);
    }

    if (toolState == VoxelState::Material)
    {
        assert(objectState == VoxelState::Material || objectState == VoxelState::Subdivided);

        if (statistics)
        {
            ++statistics->materialToolRemoveCount;
        }

        objectEditor.setState(VoxelState::Empty);
        return VoxelMaskCutResult(VoxelState::Empty, true);
    }

    assert(toolState == VoxelState::Subdivided);
    assert(toolCursor.canAccessChildren());

    const bool objectWasMaterial = objectState == VoxelState::Material;
    bool createdStructure = false;

    if (objectWasMaterial && !objectEditor.canAccessChildren())
    {
        if (toolCursor.isMaskLeaf())
        {
            createdStructure = objectEditor.makeMaskLeaf();

            if (createdStructure && statistics)
            {
                ++statistics->createdMaskLeafCount;
            }
        }
        else
        {
            createdStructure = objectEditor.split();

            if (createdStructure && statistics)
            {
                ++statistics->createdBranchCount;
            }
        }

        assert(createdStructure);
    }

    assert(objectEditor.canAccessChildren());

    if (objectEditor.isMaskLeaf() && toolCursor.isMaskLeaf())
    {
        if (statistics)
        {
            ++statistics->maskOperationCount;
        }

        const bool changed = objectEditor.maskLeaf().cut(toolCursor.maskLeaf());

        if (!changed)
        {
            if (createdStructure)
            {
                objectEditor.setState(VoxelState::Material);
                return VoxelMaskCutResult(VoxelState::Material, false);
            }

            return VoxelMaskCutResult(VoxelState::Subdivided, false);
        }

        if (statistics)
        {
            ++statistics->maskChangedCount;
            ++statistics->mergeAttemptCount;
        }

        const VoxelState mergedState = objectEditor.merge();

        if (mergedState != VoxelState::Subdivided && statistics)
        {
            ++statistics->mergeSuccessCount;
        }

        return VoxelMaskCutResult(mergedState, true);
    }

    if (statistics)
    {
        ++statistics->recursiveNodeCount;
    }

    bool changed = false;

    for (std::size_t cornerIndex = 0; cornerIndex < static_cast<std::size_t>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelMaskCutResult childResult = cutAlignedCell(objectEditor.child(corner), toolCursor.child(corner), statistics);

        changed = childResult.changed || changed;
    }

    if (!changed)
    {
        if (createdStructure)
        {
            objectEditor.setState(VoxelState::Material);
            return VoxelMaskCutResult(VoxelState::Material, false);
        }

        return VoxelMaskCutResult(objectEditor.state(), false);
    }

    if (statistics)
    {
        ++statistics->mergeAttemptCount;
    }

    const VoxelState mergedState = objectEditor.merge();

    if (mergedState != VoxelState::Subdivided && statistics)
    {
        ++statistics->mergeSuccessCount;
    }

    return VoxelMaskCutResult(mergedState, true);
}

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

void VoxelMaskCutStatistics::reset()
{
    visitedCellCount = 0;
    emptyObjectSkipCount = 0;
    emptyToolSkipCount = 0;
    materialToolRemoveCount = 0;
    createdBranchCount = 0;
    createdMaskLeafCount = 0;
    recursiveNodeCount = 0;
    maskOperationCount = 0;
    maskChangedCount = 0;
    mergeAttemptCount = 0;
    mergeSuccessCount = 0;
}

void VoxelMaskCutStatistics::accumulate(const VoxelMaskCutStatistics& other)
{
    visitedCellCount += other.visitedCellCount;
    emptyObjectSkipCount += other.emptyObjectSkipCount;
    emptyToolSkipCount += other.emptyToolSkipCount;
    materialToolRemoveCount += other.materialToolRemoveCount;
    createdBranchCount += other.createdBranchCount;
    createdMaskLeafCount += other.createdMaskLeafCount;
    recursiveNodeCount += other.recursiveNodeCount;
    maskOperationCount += other.maskOperationCount;
    maskChangedCount += other.maskChangedCount;
    mergeAttemptCount += other.mergeAttemptCount;
    mergeSuccessCount += other.mergeSuccessCount;
}

VoxelMaskCutResult::VoxelMaskCutResult(VoxelState stateValue, bool changedValue)
    : state(stateValue)
    , changed(changedValue)
{
}

VoxelMaskCutResult cutAlignedVoxelTree(VoxelPackedRootTree& objectTree, const VoxelPackedRootTree& toolTree, VoxelMaskCutStatistics* statistics)
{
    if (statistics)
    {
        statistics->reset();
    }

    assert(objectTree.isValid());
    assert(toolTree.isValid());

    VoxelPackedTreeEditor objectEditor(objectTree);
    VoxelPackedTreeConstCursor toolCursor(toolTree);
    const VoxelMaskCutResult result = cutAlignedCell(objectEditor, toolCursor, statistics);

    assert(objectTree.isValid());
    return result;
}

}
}
}