#include <cstdlib>
#include <iostream>
#include <string>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Display/Adapter/Display_VoxelSurfaceAdapter.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

void check(bool condition, const std::string& name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++g_failedCount;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

MyVoxel::VoxelLevel surfaceTrackingLevel(const MyVoxel::VoxelGrid& grid)
{
    const unsigned int maximumLevel = static_cast<unsigned int>(grid.maximumLevel());
    return maximumLevel >= 2U ? static_cast<MyVoxel::VoxelLevel>(maximumLevel - 2U) : MyVoxel::BaseVoxelLevel;
}

MyVoxel::VoxelChangeSet setMaterialRoot(MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellIndex& rootIndex)
{
    MyVoxel::VoxelShapeSession session(shape, surfaceTrackingLevel(shape.grid()));
    session.setTree(rootIndex, MyVoxel::VoxelTree(MyVoxel::VoxelState::Material));
    return session.takeChanges();
}

void countOperations(const MyVoxel::Display_MeshUpdate& update, std::size_t& replacementCount, std::size_t& removalCount)
{
    replacementCount = 0;
    removalCount = 0;

    for (std::size_t index = 0; index < update.parts.size(); ++index)
    {
        if (update.parts[index].operation == MyVoxel::Display_MeshPartOperation::Replace)
        {
            ++replacementCount;
        }
        else
        {
            ++removalCount;
        }
    }
}

}

int main()
{
    std::cout << "MyVoxel surface display adapter test" << std::endl;

    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(3));
    MyVoxel::VoxelShape shape(grid);
    const MyVoxel::VoxelCellIndex firstRoot(0, 0, 0);
    const MyVoxel::VoxelCellIndex secondRoot(1, 0, 0);
    const MyVoxel::Display_Color initialColor(0.6, 0.6, 0.6, 1.0);
    const MyVoxel::Display_Color addedColor(0.9, 0.3, 0.2, 1.0);

    setMaterialRoot(shape, firstRoot);

    MyVoxel::VoxelSurfaceCache cache;
    cache.rebuild(shape, initialColor);

    MyVoxel::Display_VoxelSurfaceAdapter adapter(1);
    const MyVoxel::Display_MeshObjectSnapshot firstSnapshot =
        adapter.buildSnapshot(cache, MyMath::Matrix4::identity(), true);

    check(firstSnapshot.isValid(), "initial snapshot is valid");
    check(firstSnapshot.parts.size() == 6, "single root creates six display parts");
    check(firstSnapshot.triangleCount() == 12, "single root snapshot triangle count");
    check(adapter.isInitialized(), "adapter initialized after snapshot");
    check(adapter.activePartCount() == 6, "single root active part count");

    const MyVoxel::Display_MeshPartId stablePartId =
        adapter.partId(firstRoot, MyVoxel::VoxelFaceDirection::NegativeX);

    check(stablePartId != 0, "root direction receives stable part id");

    const MyVoxel::VoxelChangeSet changes = setMaterialRoot(shape, secondRoot);
    const MyVoxel::VoxelSurfaceCacheUpdate surfaceUpdate = cache.update(shape, changes, addedColor);
    const MyVoxel::Display_MeshUpdate displayUpdate = adapter.buildUpdate(cache, surfaceUpdate);

    std::size_t replacementCount = 0;
    std::size_t removalCount = 0;
    countOperations(displayUpdate, replacementCount, removalCount);

    check(displayUpdate.isValid(), "incremental display update is valid");
    check(displayUpdate.parts.size() == 6, "adjacent root update emits six changed directions");
    check(replacementCount == 5, "adjacent root update emits five replacements");
    check(removalCount == 1, "adjacent root update emits one removal");
    check(adapter.activePartCount() == 10, "adjacent roots retain ten visible directions");
    check(adapter.partId(firstRoot, MyVoxel::VoxelFaceDirection::NegativeX) == stablePartId,
          "unchanged root direction keeps stable part id");

    const MyVoxel::Display_MeshUpdate repeatedUpdate = adapter.buildUpdate(cache, surfaceUpdate);
    check(repeatedUpdate.parts.empty(), "repeated source versions produce no display update");

    const MyVoxel::Display_MeshObjectSnapshot secondSnapshot =
        adapter.buildSnapshot(cache, MyMath::Matrix4::identity(), true);

    check(secondSnapshot.isValid(), "updated full snapshot is valid");
    check(secondSnapshot.parts.size() == 10, "adjacent roots full snapshot has ten parts");
    check(secondSnapshot.triangleCount() == 20, "adjacent roots full snapshot triangle count");

    const MyVoxel::VoxelSurfaceCache::RootIndexSet recoloredRoots =
        cache.setDefaultColor(shape, MyVoxel::Display_Color(0.2, 0.5, 0.9, 1.0));
    const MyVoxel::Display_MeshUpdate recolorUpdate = adapter.buildUpdate(cache, recoloredRoots);

    countOperations(recolorUpdate, replacementCount, removalCount);

    check(recolorUpdate.isValid(), "default color change creates valid display update");
    check(replacementCount == 10 && removalCount == 0, "default color change replaces all visible directions");

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
