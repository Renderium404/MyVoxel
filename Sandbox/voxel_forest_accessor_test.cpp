#include <cstddef>
#include <iostream>

#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/Tree/VoxelForestConstAccessor.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelTypes.h"

namespace
{

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 创建指定索引和层级的体素地址。
MyVoxel::VoxelCellAddress makeAddress(MyVoxel::VoxelIndex x, MyVoxel::VoxelIndex y, MyVoxel::VoxelIndex z, MyVoxel::VoxelLevel level)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(x, y, z), level);
}

// 测试不存在根树时的空状态和根缓存。
bool testMissingRootCache()
{
    const MyVoxel::VoxelForest forest;
    MyVoxel::VoxelForestConstAccessor accessor(forest);

    const MyVoxel::VoxelCellAddress first = makeAddress(1, 1, 1, 2);
    const MyVoxel::VoxelCellAddress second = makeAddress(3, 2, 0, 2);

    const MyVoxel::VoxelState firstState = accessor.state(first);
    const MyVoxel::VoxelState secondState = accessor.state(second);
    const MyVoxel::VoxelForestConstAccessorStatistics& statistics = accessor.statistics();

    const bool passed =
        firstState == MyVoxel::VoxelState::Empty &&
        secondState == MyVoxel::VoxelState::Empty &&
        statistics.rootCacheMissCount == 1 &&
        statistics.rootCacheHitCount == 1 &&
        statistics.pathReuseCount == 0 &&
        statistics.reusedPathLevelCount == 0 &&
        statistics.nodeVisitCount == 0;

    return check(passed, "VoxelForestConstAccessor missing root cache");
}

// 测试材料叶节点查询只访问实际存在的叶节点。
bool testMaterialLeafQueries()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress first = makeAddress(1, 2, 3, 3);
    const MyVoxel::VoxelCellAddress second = makeAddress(7, 5, 4, 3);

    forest.setState(root, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelForestConstAccessor accessor(forest);

    const MyVoxel::VoxelState firstState = accessor.state(first);
    const MyVoxel::VoxelState secondState = accessor.state(second);
    const MyVoxel::VoxelForestConstAccessorStatistics& statistics = accessor.statistics();

    const bool passed =
        firstState == MyVoxel::VoxelState::Material &&
        secondState == MyVoxel::VoxelState::Material &&
        statistics.rootCacheMissCount == 1 &&
        statistics.rootCacheHitCount == 1 &&
        statistics.pathReuseCount == 0 &&
        statistics.reusedPathLevelCount == 0 &&
        statistics.nodeVisitCount == 2;

    return check(passed, "VoxelForestConstAccessor material leaf queries");
}

// 测试同一实际分支下连续查询时复用公共路径。
bool testPathReuse()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumXYZ);
    const MyVoxel::VoxelCellAddress first = MyVoxel::childCellAddress(child, MyVoxel::VoxelCorner::Minimum);
    const MyVoxel::VoxelCellAddress second = MyVoxel::childCellAddress(child, MyVoxel::VoxelCorner::MaximumX);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.split(child);
    forest.setStateDeferred(first, MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelForestConstAccessor accessor(forest);

    const MyVoxel::VoxelState firstState = accessor.state(first);
    const MyVoxel::VoxelState secondState = accessor.state(second);
    const MyVoxel::VoxelForestConstAccessorStatistics& statistics = accessor.statistics();

    const bool passed =
        firstState == MyVoxel::VoxelState::Empty &&
        secondState == MyVoxel::VoxelState::Material &&
        statistics.rootCacheMissCount == 1 &&
        statistics.rootCacheHitCount == 1 &&
        statistics.pathReuseCount == 1 &&
        statistics.reusedPathLevelCount == 1 &&
        statistics.nodeVisitCount == 5;

    return check(passed, "VoxelForestConstAccessor path reuse");
}

// 测试切换根树时重新查询根索引。
bool testRootSwitching()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress firstRoot = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress secondRoot = makeAddress(1, 0, 0, 0);

    forest.setState(firstRoot, MyVoxel::VoxelState::Material);
    forest.setState(secondRoot, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelForestConstAccessor accessor(forest);

    accessor.state(firstRoot);
    accessor.state(secondRoot);
    accessor.state(firstRoot);

    const MyVoxel::VoxelForestConstAccessorStatistics& statistics = accessor.statistics();

    const bool passed =
        statistics.rootCacheMissCount == 3 &&
        statistics.rootCacheHitCount == 0 &&
        statistics.pathReuseCount == 0 &&
        statistics.nodeVisitCount == 3;

    return check(passed, "VoxelForestConstAccessor root switching");
}

// 测试查询同一个实际节点时复用完整路径。
bool testCompletePathReuse()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumYZ);
    const MyVoxel::VoxelCellAddress target = MyVoxel::childCellAddress(child, MyVoxel::VoxelCorner::MaximumX);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.split(child);

    MyVoxel::VoxelForestConstAccessor accessor(forest);

    const MyVoxel::VoxelState firstState = accessor.state(target);
    const MyVoxel::VoxelState secondState = accessor.state(target);
    const MyVoxel::VoxelForestConstAccessorStatistics& statistics = accessor.statistics();

    const bool passed =
        firstState == MyVoxel::VoxelState::Material &&
        secondState == MyVoxel::VoxelState::Material &&
        statistics.rootCacheMissCount == 1 &&
        statistics.rootCacheHitCount == 1 &&
        statistics.pathReuseCount == 1 &&
        statistics.reusedPathLevelCount == 2 &&
        statistics.nodeVisitCount == 4;

    return check(passed, "VoxelForestConstAccessor complete path reuse");
}

// 测试清除缓存和统计数据。
bool testClear()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);

    forest.setState(root, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelForestConstAccessor accessor(forest);

    accessor.state(root);
    accessor.state(root);
    accessor.clear();

    const MyVoxel::VoxelForestConstAccessorStatistics& clearedStatistics = accessor.statistics();

    const bool clearPassed =
        clearedStatistics.rootCacheHitCount == 0 &&
        clearedStatistics.rootCacheMissCount == 0 &&
        clearedStatistics.pathReuseCount == 0 &&
        clearedStatistics.reusedPathLevelCount == 0 &&
        clearedStatistics.nodeVisitCount == 0;

    const MyVoxel::VoxelState state = accessor.state(root);
    const MyVoxel::VoxelForestConstAccessorStatistics& finalStatistics = accessor.statistics();

    const bool passed =
        clearPassed &&
        state == MyVoxel::VoxelState::Material &&
        finalStatistics.rootCacheMissCount == 1 &&
        finalStatistics.rootCacheHitCount == 0 &&
        finalStatistics.nodeVisitCount == 1;

    return check(passed, "VoxelForestConstAccessor clear");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testMissingRootCache(),
        testMaterialLeafQueries(),
        testPathReuse(),
        testRootSwitching(),
        testCompletePathReuse(),
        testClear()
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
    std::cout << "Voxel forest accessor tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}