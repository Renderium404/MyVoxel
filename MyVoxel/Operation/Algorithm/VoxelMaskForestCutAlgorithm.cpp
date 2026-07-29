#include "VoxelMaskForestCutAlgorithm.h"

#include <cassert>
#include <cstddef>

#include "MyVoxel/Core/Tree/VoxelPackedTreeConstCursor.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 检查两个同地址逻辑节点是否存在材料交集。
bool hasAlignedMaterialIntersection(
    MyVoxel::VoxelPackedTreeConstCursor objectCursor,
    MyVoxel::VoxelPackedTreeConstCursor toolCursor,
    MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics* statistics)
{
    using namespace MyVoxel;

    if (statistics)
    {
        ++statistics->intersectionVisitedCellCount;
    }

    const VoxelState objectState = objectCursor.state();
    const VoxelState toolState = toolCursor.state();

    if (objectState == VoxelState::Empty || toolState == VoxelState::Empty)
    {
        return false;
    }

    if (objectState == VoxelState::Material && toolState == VoxelState::Material)
    {
        return true;
    }

    if (objectCursor.isMaskLeaf() && toolCursor.isMaskLeaf())
    {
        if (statistics)
        {
            ++statistics->intersectionMaskTestCount;
        }

        return (objectCursor.maskLeaf().materialMask & toolCursor.maskLeaf().materialMask) != 0;
    }

    if (objectCursor.isMaskLeaf() && toolState == VoxelState::Material)
    {
        if (statistics)
        {
            ++statistics->intersectionMaskTestCount;
        }

        return !objectCursor.maskLeaf().isEmpty();
    }

    if (objectState == VoxelState::Material && toolCursor.isMaskLeaf())
    {
        if (statistics)
        {
            ++statistics->intersectionMaskTestCount;
        }

        return !toolCursor.maskLeaf().isEmpty();
    }

    MYVOXEL_ASSERT_MESSAGE(
        objectState == VoxelState::Subdivided || toolState == VoxelState::Subdivided,
        "Aligned mask intersection requires at least one subdivided node.");

    for (std::size_t cornerIndex = 0; cornerIndex < static_cast<std::size_t>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);

        if (hasAlignedMaterialIntersection(objectCursor.child(corner), toolCursor.child(corner), statistics))
        {
            return true;
        }
    }

    return false;
}

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{
bool canUseAlignedVoxelMaskCut(const VoxelShape& object, const VoxelShape& tool)
{
    assert(object.grid().isValid());
    assert(tool.grid().isValid());
    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());

    const VoxelGrid& objectGrid = object.grid();
    const VoxelGrid& toolGrid = tool.grid();

    if (objectGrid.maximumLevel() != toolGrid.maximumLevel())
    {
        return false;
    }

    const VoxelCellAddress zeroRootAddress(VoxelCellIndex(0, 0, 0), BaseVoxelLevel);
    const Bounds3 objectRootBounds = objectGrid.cellBounds(zeroRootAddress);
    const Bounds3 toolRootBounds = toolGrid.cellBounds(zeroRootAddress);
    const MyMath::Vector3& objectMinimum = objectRootBounds.minimum();
    const MyMath::Vector3& objectMaximum = objectRootBounds.maximum();
    const MyMath::Vector3& toolMinimum = toolRootBounds.minimum();
    const MyMath::Vector3& toolMaximum = toolRootBounds.maximum();

    const bool sameGrid =
        objectMinimum.x() == toolMinimum.x() &&
        objectMinimum.y() == toolMinimum.y() &&
        objectMinimum.z() == toolMinimum.z() &&
        objectMaximum.x() == toolMaximum.x() &&
        objectMaximum.y() == toolMaximum.y() &&
        objectMaximum.z() == toolMaximum.z();

    return sameGrid && object.transform().isEqualTo(tool.transform(), 0.0);
}

VoxelShape cutAlignedVoxelShapes(
    const VoxelShape& object,
    const VoxelShape& tool,
    VoxelChangeSet* changes,
    VoxelMaskForestCutStatistics* statistics)
{
    assert(canUseAlignedVoxelMaskCut(object, tool));

    if (changes)
    {
        changes->clear();
    }

    if (statistics)
    {
        statistics->reset();
    }

    if (object.isEmpty() || tool.isEmpty())
    {
        return object;
    }

    VoxelShape result = object;
    VoxelForest& resultForest = result.editForest();

    const bool changed =
        cutAlignedVoxelForest(
            resultForest,
            tool.forest(),
            changes,
            statistics);

    return changed ? result : object;
}
void VoxelMaskForestCutStatistics::reset()
{
    rootCandidateCount = 0;
    existingObjectRootCount = 0;
    intersectionVisitedCellCount = 0;
    intersectionMaskTestCount = 0;
    intersectingRootCount = 0;
    detachedRootCount = 0;
    directRootEraseCount = 0;
    emptiedRootEraseCount = 0;
    modifiedRootCount = 0;
    treeStatistics.reset();
}

bool cutAlignedVoxelForest(
    VoxelPackedForest& objectForest,
    const VoxelPackedForest& toolForest,
    VoxelChangeSet* changes,
    VoxelMaskForestCutStatistics* statistics)
{
    MYVOXEL_ASSERT_MESSAGE(&objectForest != &toolForest, "Aligned forest cut requires distinct object and tool forest instances.");

    if (changes)
    {
        changes->clear();
    }

    if (statistics)
    {
        statistics->reset();
    }

    if (objectForest.isEmpty() || toolForest.isEmpty())
    {
        return false;
    }

    bool changed = false;

    toolForest.forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelCellIndex& rootIndex = rootAddress.index;

            if (statistics)
            {
                ++statistics->rootCandidateCount;
            }

            const VoxelPackedRootTree* objectTree = objectForest.rootTree(rootIndex);
            const VoxelPackedRootTree* toolTree = toolForest.rootTree(rootIndex);

            MYVOXEL_ASSERT_MESSAGE(toolTree != nullptr, "Tool root traversal returned a missing Packed root tree.");

            if (!objectTree)
            {
                return;
            }

            if (statistics)
            {
                ++statistics->existingObjectRootCount;
            }

            VoxelPackedTreeConstCursor objectCursor(*objectTree);
            VoxelPackedTreeConstCursor toolCursor(*toolTree);

            if (!hasAlignedMaterialIntersection(objectCursor, toolCursor, statistics))
            {
                return;
            }

            if (statistics)
            {
                ++statistics->intersectingRootCount;
            }

            if (toolTree->rootState == VoxelState::Material)
            {
                const bool erased = objectForest.eraseRootTree(rootIndex);

                MYVOXEL_ASSERT_MESSAGE(erased, "Aligned mask cut failed to erase a fully covered object root.");

                if (statistics)
                {
                    ++statistics->directRootEraseCount;
                    ++statistics->modifiedRootCount;
                }

                if (changes)
                {
                    changes->addModifiedRoot(rootIndex);
                }

                changed = true;
                return;
            }

            VoxelPackedRootTree* editableObjectTree = objectForest.detachRootTree(rootIndex);

            MYVOXEL_ASSERT_MESSAGE(editableObjectTree != nullptr, "Aligned mask cut failed to detach an existing object root.");

            if (statistics)
            {
                ++statistics->detachedRootCount;
            }

            VoxelMaskCutStatistics rootStatistics;
            const VoxelMaskCutResult rootResult = cutAlignedVoxelTree(*editableObjectTree, *toolTree, &rootStatistics);

            if (statistics)
            {
                statistics->treeStatistics.accumulate(rootStatistics);
            }

            MYVOXEL_ASSERT_MESSAGE(rootResult.changed, "Aligned material intersection did not produce a mask cut change.");

            if (!rootResult.changed)
            {
                return;
            }

            if (rootResult.state == VoxelState::Empty)
            {
                const bool erased = objectForest.eraseRootTree(rootIndex);

                MYVOXEL_ASSERT_MESSAGE(erased, "Aligned mask cut failed to erase an empty result root.");

                if (statistics)
                {
                    ++statistics->emptiedRootEraseCount;
                }
            }

            if (statistics)
            {
                ++statistics->modifiedRootCount;
            }

            if (changes)
            {
                changes->addModifiedRoot(rootIndex);
            }

            changed = true;
        });

    return changed;
}

}
}
}