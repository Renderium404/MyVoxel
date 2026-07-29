#include <cstddef>
#include <iostream>
#include <vector>

#include "MyVoxel/Core/Tree/VoxelPackedForest.h"
#include "MyVoxel/Core/Tree/VoxelPackedForestConstAccessor.h"

namespace
{

int g_passedCount = 0;
int g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[Passed] " << name << std::endl;
        return;
    }

    ++g_failedCount;
    std::cout << "[Failed] " << name << std::endl;
}

// 返回指定数值对应的体素角点。
MyVoxel::VoxelCorner corner(unsigned int value)
{
    return static_cast<MyVoxel::VoxelCorner>(value);
}

// 返回指定第0层索引对应的根地址。
MyVoxel::VoxelCellAddress rootAddress(int x, int y = 0, int z = 0)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(x, y, z), MyVoxel::BaseVoxelLevel);
}

// 返回沿指定三个角点建立的第3层地址。
MyVoxel::VoxelCellAddress level3Address(const MyVoxel::VoxelCellAddress& root, unsigned int corner0, unsigned int corner1, unsigned int corner2)
{
    MyVoxel::VoxelCellAddress address = MyVoxel::childCellAddress(root, corner(corner0));
    address = MyVoxel::childCellAddress(address, corner(corner1));
    address = MyVoxel::childCellAddress(address, corner(corner2));
    return address;
}

// 创建访问器测试使用的两个根树和多个深层材料节点。
MyVoxel::VoxelPackedForest makeForest()
{
    MyVoxel::VoxelPackedForest forest;

    const MyVoxel::VoxelCellAddress firstRoot = rootAddress(0);
    const MyVoxel::VoxelCellAddress secondRoot = rootAddress(1);

    forest.setState(level3Address(firstRoot, 1, 2, 3), MyVoxel::VoxelState::Material);
    forest.setState(level3Address(firstRoot, 1, 2, 4), MyVoxel::VoxelState::Material);
    forest.setState(level3Address(firstRoot, 1, 5, 6), MyVoxel::VoxelState::Material);
    forest.setState(secondRoot, MyVoxel::VoxelState::Material);

    return forest;
}

// 验证访问器状态查询与森林普通查询完全一致。
bool testStateConsistency()
{
    const MyVoxel::VoxelPackedForest forest = makeForest();
    MyVoxel::VoxelPackedForestConstAccessor accessor(forest);
    std::vector<MyVoxel::VoxelCellAddress> addresses;

    const MyVoxel::VoxelCellAddress firstRoot = rootAddress(0);
    const MyVoxel::VoxelCellAddress secondRoot = rootAddress(1);

    addresses.push_back(level3Address(firstRoot, 1, 2, 3));
    addresses.push_back(level3Address(firstRoot, 1, 2, 4));
    addresses.push_back(level3Address(firstRoot, 1, 2, 5));
    addresses.push_back(level3Address(firstRoot, 1, 5, 6));
    addresses.push_back(secondRoot);
    addresses.push_back(level3Address(secondRoot, 7, 7, 7));
    addresses.push_back(rootAddress(2));
    addresses.push_back(level3Address(rootAddress(2), 0, 0, 0));

    for (std::size_t addressIndex = 0; addressIndex < addresses.size(); ++addressIndex)
    {
        if (accessor.state(addresses[addressIndex]) != forest.state(addresses[addressIndex]))
        {
            return false;
        }
    }

    return true;
}

// 验证连续查询能够命中根缓存并复用公共角点路径。
bool testRootCacheAndPathReuse()
{
    const MyVoxel::VoxelPackedForest forest = makeForest();
    MyVoxel::VoxelPackedForestConstAccessor accessor(forest);

    const MyVoxel::VoxelCellAddress firstRoot = rootAddress(0);
    const MyVoxel::VoxelCellAddress secondRoot = rootAddress(1);

    accessor.state(level3Address(firstRoot, 1, 2, 3));
    accessor.state(level3Address(firstRoot, 1, 2, 4));
    accessor.state(level3Address(firstRoot, 1, 2, 5));
    accessor.state(level3Address(firstRoot, 1, 5, 6));
    accessor.state(secondRoot);
    accessor.state(level3Address(secondRoot, 7, 7, 7));
    accessor.state(rootAddress(2));
    accessor.state(level3Address(rootAddress(2), 0, 0, 0));

    const MyVoxel::VoxelPackedForestConstAccessorStatistics& statistics = accessor.statistics();

    return statistics.rootCacheMissCount == 3 &&
           statistics.rootCacheHitCount == 5 &&
           statistics.pathReuseCount >= 2 &&
           statistics.reusedPathLevelCount >= 4 &&
           statistics.nodeVisitCount > 0;
}

// 验证不存在的根树会被缓存，连续空区域查询不会重复查找根表。
bool testMissingRootCache()
{
    const MyVoxel::VoxelPackedForest forest = makeForest();
    MyVoxel::VoxelPackedForestConstAccessor accessor(forest);

    const MyVoxel::VoxelCellAddress missingRoot = rootAddress(10);
    const MyVoxel::VoxelCellAddress missingChild = level3Address(missingRoot, 1, 2, 3);

    if (accessor.state(missingRoot) != MyVoxel::VoxelState::Empty ||
        accessor.state(missingChild) != MyVoxel::VoxelState::Empty)
    {
        return false;
    }

    const MyVoxel::VoxelPackedForestConstAccessorStatistics& statistics = accessor.statistics();

    return statistics.rootCacheMissCount == 1 &&
           statistics.rootCacheHitCount == 1 &&
           statistics.nodeVisitCount == 0;
}

// 验证clear会同时清除缓存和全部统计。
bool testClear()
{
    const MyVoxel::VoxelPackedForest forest = makeForest();
    MyVoxel::VoxelPackedForestConstAccessor accessor(forest);

    const MyVoxel::VoxelCellAddress address = level3Address(rootAddress(0), 1, 2, 3);

    accessor.state(address);
    accessor.state(address);

    if (accessor.statistics().rootCacheMissCount != 1 ||
        accessor.statistics().rootCacheHitCount != 1)
    {
        return false;
    }

    accessor.clear();

    const MyVoxel::VoxelPackedForestConstAccessorStatistics& clearedStatistics = accessor.statistics();

    if (clearedStatistics.rootCacheHitCount != 0 ||
        clearedStatistics.rootCacheMissCount != 0 ||
        clearedStatistics.pathReuseCount != 0 ||
        clearedStatistics.reusedPathLevelCount != 0 ||
        clearedStatistics.nodeVisitCount != 0)
    {
        return false;
    }

    accessor.state(address);

    return accessor.statistics().rootCacheMissCount == 1 &&
           accessor.statistics().rootCacheHitCount == 0;
}

// 验证材料根节点查询能够提前终止，不需要访问完整目标层级路径。
bool testMaterialEarlyExit()
{
    MyVoxel::VoxelPackedForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress(0);
    const MyVoxel::VoxelCellAddress deepAddress = level3Address(root, 7, 6, 5);

    forest.setState(root, MyVoxel::VoxelState::Material);

    MyVoxel::VoxelPackedForestConstAccessor accessor(forest);

    if (accessor.state(deepAddress) != MyVoxel::VoxelState::Material)
    {
        return false;
    }

    return accessor.statistics().nodeVisitCount == 1;
}

}

int main()
{
    check(testStateConsistency(), "State consistency");
    check(testRootCacheAndPathReuse(), "Root cache and path reuse");
    check(testMissingRootCache(), "Missing root cache");
    check(testClear(), "Accessor clear");
    check(testMaterialEarlyExit(), "Material early exit");

    std::cout << std::endl;
    std::cout << "Voxel packed forest accessor tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}