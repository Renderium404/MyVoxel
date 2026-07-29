#include "BooleanCutAlgorithm.h"

#include <cassert>
#include <cstddef>
#include <vector>

#include "BooleanAlgorithmCommon.h"
#include "ShapeCutContext.h"
#include "ShapeCutPlan.h"
#include "MyVoxel/Operation/BooleanOperation.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

VoxelShape cutWithShapeTool(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet* changes)
{
    if (changes)
    {
        changes->clear();
    }

    assert(object.grid().isValid());
    assert(object.transform().isRigidTransform());
    assert(tool.isValid());
    assert(tool.localToWorld().isRigidTransform());

    if (object.isEmpty())
    {
        return object;
    }

    const ShapeToolContext context = createShapeToolContext(object, tool);
    const std::vector<VoxelCellAddress> rootCells = collectRootCellsInRange(object, context.minimumRootIndex, context.maximumRootIndex);

    if (rootCells.empty())
    {
        return object;
    }

    const unsigned int planningWorkerCount = resolveParallelWorkerCount(options, rootCells.size());
    std::vector<RootCutPlan> plans(rootCells.size());

    buildShapeCutPlans(object, tool, context, rootCells, planningWorkerCount, plans);

    if (!hasShapeCutPlanModification(plans))
    {
        return object;
    }

    VoxelShape result = object;
    VoxelForest& resultForest = result.editForest();
    std::vector<RootPlanApplication> applications;

    applications.reserve(rootCells.size());

    for (std::size_t rootIndex = 0; rootIndex < rootCells.size(); ++rootIndex)
    {
        const VoxelCellAddress& rootAddress = rootCells[rootIndex];
        const CutPlanNode& rootPlan = plans[rootIndex].root;

        if (rootPlan.action == CutPlanAction::Keep)
        {
            continue;
        }

        if (rootPlan.action == CutPlanAction::Remove)
        {
            const bool removed = resultForest.setState(rootAddress, VoxelState::Empty);

            assert(removed);

            if (removed && changes)
            {
                changes->addModifiedRoot(rootAddress.index);
            }

            continue;
        }

        assert(rootPlan.action == CutPlanAction::Subdivide);

        if (changes)
        {
            changes->addModifiedRoot(rootAddress.index);
        }

        applications.push_back(RootPlanApplication(resultForest, rootAddress, plans[rootIndex]));
    }

    const unsigned int applicationWorkerCount = resolveParallelWorkerCount(options, applications.size());

    applyShapeCutPlans(applications, applicationWorkerCount);
    return result;
}

}
}
}