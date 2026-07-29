#include <cstddef>
#include <iostream>

#include "BooleanOperationTestCommon.h"
#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Operation/BooleanOperation.h"

#if defined(MYVOXEL_TEST)
#error boolean_operation_release_test must not define MYVOXEL_TEST.
#endif

namespace
{

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 测试正式库连续刀具切削。
bool testContinuousTool()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const BooleanOperationTest::OccupancySignature sourceSignature = BooleanOperationTest::occupancySignature(workpiece);
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeContinuousTool();
    MyVoxel::VoxelChangeSet changes;

    const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes);
    const BooleanOperationTest::OccupancySignature resultSignature = BooleanOperationTest::occupancySignature(result);

    const bool passed =
        resultSignature.voxelCount < sourceSignature.voxelCount &&
        resultSignature.voxelCount > 0 &&
        changes.hasChanges() &&
        !result.sharesDataWith(workpiece) &&
        BooleanOperationTest::occupancySignature(workpiece).isEqualTo(sourceSignature) &&
        result.transform().isEqualTo(workpiece.transform());

    return check(passed, "Release continuous tool");
}

// 测试正式库体素刀具切削。
// 测试正式库体素刀具切削。
bool testVoxelTool()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const BooleanOperationTest::OccupancySignature sourceSignature = BooleanOperationTest::occupancySignature(workpiece);
    const MyVoxel::VoxelShape tool = BooleanOperationTest::makeVoxelTool(workpiece);
    const BooleanOperationTest::OccupancySignature toolSignature = BooleanOperationTest::occupancySignature(tool);
    MyVoxel::VoxelChangeSet changes;

    const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes);
    const BooleanOperationTest::OccupancySignature resultSignature = BooleanOperationTest::occupancySignature(result);
    const BooleanOperationTest::OccupancySignature sourceAfterSignature = BooleanOperationTest::occupancySignature(workpiece);

    const bool removedMaterial = resultSignature.voxelCount < sourceSignature.voxelCount;
    const bool retainedMaterial = resultSignature.voxelCount > 0;
    const bool toolNotEmpty = toolSignature.voxelCount > 0;
    const bool changesRecorded = changes.hasChanges();
    const bool resultDetached = !result.sharesDataWith(workpiece);
    const bool sourcePreserved = sourceAfterSignature.isEqualTo(sourceSignature);
    const bool transformPreserved = result.transform().isEqualTo(workpiece.transform());

    std::cout << "  Source voxels      : " << sourceSignature.voxelCount << std::endl;
    std::cout << "  Tool voxels        : " << toolSignature.voxelCount << std::endl;
    std::cout << "  Result voxels      : " << resultSignature.voxelCount << std::endl;
    std::cout << "  Modified roots     : " << changes.modifiedRootCount() << std::endl;
    std::cout << "  Removed material   : " << (removedMaterial ? "Yes" : "No") << std::endl;
    std::cout << "  Retained material  : " << (retainedMaterial ? "Yes" : "No") << std::endl;
    std::cout << "  Tool not empty     : " << (toolNotEmpty ? "Yes" : "No") << std::endl;
    std::cout << "  Changes recorded   : " << (changesRecorded ? "Yes" : "No") << std::endl;
    std::cout << "  Result detached    : " << (resultDetached ? "Yes" : "No") << std::endl;
    std::cout << "  Source preserved   : " << (sourcePreserved ? "Yes" : "No") << std::endl;
    std::cout << "  Transform preserved: " << (transformPreserved ? "Yes" : "No") << std::endl;

    return check(
        removedMaterial &&
        retainedMaterial &&
        toolNotEmpty &&
        changesRecorded &&
        resultDetached &&
        sourcePreserved &&
        transformPreserved,
        "Release voxel tool");
}

// 测试完全分离的连续刀具保持原始数据共享。
bool testOutsideTool()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeOutsideTool();
    MyVoxel::VoxelChangeSet changes;

    const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes);

    const bool passed =
        result.sharesDataWith(workpiece) &&
        !changes.hasChanges() &&
        BooleanOperationTest::occupancySignature(result).isEqualTo(BooleanOperationTest::occupancySignature(workpiece));

    return check(passed, "Release outside tool");
}

// 测试不同线程数量的无统计连续切削结果一致。
bool testWorkerCountConsistency()
{
    const MyVoxel::VoxelShape workpiece = BooleanOperationTest::makeWorkpiece();
    const MyVoxel::Geometry::ShapeInstance tool = BooleanOperationTest::makeContinuousTool();

    MyVoxel::Operation::BooleanOperationOptions singleThreadOptions;
    singleThreadOptions.workerCount = 1;
    singleThreadOptions.minimumParallelRootCount = 1;

    MyVoxel::Operation::BooleanOperationOptions fourThreadOptions;
    fourThreadOptions.workerCount = 4;
    fourThreadOptions.minimumParallelRootCount = 1;

    const MyVoxel::VoxelShape singleThreadResult = MyVoxel::Operation::cut(workpiece, tool, singleThreadOptions);
    const MyVoxel::VoxelShape fourThreadResult = MyVoxel::Operation::cut(workpiece, tool, fourThreadOptions);

    return check(
        BooleanOperationTest::occupancySignature(singleThreadResult).isEqualTo(BooleanOperationTest::occupancySignature(fourThreadResult)),
        "Release worker count consistency");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testContinuousTool(),
        testVoxelTool(),
        testOutsideTool(),
        testWorkerCountConsistency()
    };

    const std::size_t testCount = sizeof(results) / sizeof(results[0]);

    for (std::size_t testIndex = 0; testIndex < testCount; ++testIndex)
    {
        if (results[testIndex])
        {
            ++passedCount;
        }
        else
        {
            ++failedCount;
        }
    }

    std::cout << std::endl;
    std::cout << "Boolean operation release tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}