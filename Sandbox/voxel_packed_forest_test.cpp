#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "MyVoxel/Core/Tree/VoxelPackedForest.h"

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

// 返回指定角点值。
MyVoxel::VoxelCorner corner(unsigned int value)
{
    return static_cast<MyVoxel::VoxelCorner>(value);
}

// 返回测试使用的第0层根地址。
MyVoxel::VoxelCellAddress rootAddress(int x = 0, int y = 0, int z = 0)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(x, y, z), MyVoxel::BaseVoxelLevel);
}

// 验证根状态设置、删除和根节点数量。
bool testRootState()
{
    MyVoxel::VoxelPackedForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress();

    if (forest.state(root) != MyVoxel::VoxelState::Empty || forest.hasNode(root))
    {
        return false;
    }

    if (!forest.setState(root, MyVoxel::VoxelState::Material))
    {
        return false;
    }

    if (forest.state(root) != MyVoxel::VoxelState::Material || !forest.hasNode(root) || forest.rootCount() != 1)
    {
        return false;
    }

    if (!forest.setState(root, MyVoxel::VoxelState::Empty))
    {
        return false;
    }

    return forest.state(root) == MyVoxel::VoxelState::Empty &&
           !forest.hasNode(root) &&
           forest.rootCount() == 0;
}

// 验证材料根节点细分、子节点修改和合并。
bool testSplitAndMerge()
{
    MyVoxel::VoxelPackedForest forest;
    const MyVoxel::VoxelCellAddress root = rootAddress();
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, corner(3));

    forest.setState(root, MyVoxel::VoxelState::Material);

    if (!forest.split(root))
    {
        return false;
    }

    if (forest.state(root) != MyVoxel::VoxelState::Subdivided ||
        forest.state(child) != MyVoxel::VoxelState::Material ||
        !forest.hasNode(child))
    {
        return false;
    }

    forest.setStateDeferred(child, MyVoxel::VoxelState::Empty);

    if (forest.mergeDeferred(root) != MyVoxel::VoxelState::Subdivided)
    {
        return false;
    }

    forest.setStateDeferred(child, MyVoxel::VoxelState::Material);

    return forest.merge(root) &&
           forest.state(root) == MyVoxel::VoxelState::Material &&
           forest.rootCount() == 1;
}

// 验证从空森林直接创建深层材料节点。
bool testDeepMaterialCreation()
{
    MyVoxel::VoxelPackedForest forest;
    MyVoxel::VoxelCellAddress address = rootAddress(1, 2, 3);

    address = MyVoxel::childCellAddress(address, corner(0));
    address = MyVoxel::childCellAddress(address, corner(5));
    address = MyVoxel::childCellAddress(address, corner(7));
    address = MyVoxel::childCellAddress(address, corner(2));

    if (!forest.setState(address, MyVoxel::VoxelState::Material))
    {
        return false;
    }

    if (forest.state(address) != MyVoxel::VoxelState::Material || !forest.hasNode(address))
    {
        return false;
    }

    MyVoxel::VoxelCellAddress parent = MyVoxel::parentCellAddress(address);

    while (MyVoxel::hasParentCell(parent))
    {
        if (forest.state(parent) != MyVoxel::VoxelState::Subdivided)
        {
            return false;
        }

        parent = MyVoxel::parentCellAddress(parent);
    }

    return forest.state(parent) == MyVoxel::VoxelState::Subdivided &&
           forest.rootCount() == 1;
}

// 验证深层材料删除后空分支会逐级清理并移除根树。
bool testEmptyBranchPruning()
{
    MyVoxel::VoxelPackedForest forest;
    MyVoxel::VoxelCellAddress address = rootAddress();

    address = MyVoxel::childCellAddress(address, corner(1));
    address = MyVoxel::childCellAddress(address, corner(2));
    address = MyVoxel::childCellAddress(address, corner(3));
    address = MyVoxel::childCellAddress(address, corner(4));

    forest.setState(address, MyVoxel::VoxelState::Material);

    if (forest.rootCount() != 1)
    {
        return false;
    }

    forest.setState(address, MyVoxel::VoxelState::Empty);

    return forest.rootCount() == 0 &&
           forest.state(rootAddress()) == MyVoxel::VoxelState::Empty &&
           !forest.hasNode(rootAddress());
}

// 验证材料遍历只返回当前树中压缩后的Material叶节点。
bool testMaterialTraversal()
{
    MyVoxel::VoxelPackedForest forest;
    const MyVoxel::VoxelCellAddress firstRoot = rootAddress(0, 0, 0);
    const MyVoxel::VoxelCellAddress secondRoot = rootAddress(1, 0, 0);

    forest.setState(firstRoot, MyVoxel::VoxelState::Material);
    forest.setState(secondRoot, MyVoxel::VoxelState::Material);
    forest.split(firstRoot);
    forest.setState(MyVoxel::childCellAddress(firstRoot, corner(0)), MyVoxel::VoxelState::Empty);

    std::vector<MyVoxel::VoxelCellAddress> materialCells;

    forest.forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            materialCells.push_back(address);
        });

    if (materialCells.size() != 8)
    {
        return false;
    }

    std::size_t firstRootMaterialCount = 0;
    std::size_t secondRootMaterialCount = 0;

    for (std::size_t cellIndex = 0; cellIndex < materialCells.size(); ++cellIndex)
    {
        MyVoxel::VoxelCellAddress root = materialCells[cellIndex];

        while (MyVoxel::hasParentCell(root))
        {
            root = MyVoxel::parentCellAddress(root);
        }

        if (root.index.x == 0)
        {
            ++firstRootMaterialCount;
        }
        else if (root.index.x == 1)
        {
            ++secondRootMaterialCount;
        }
    }

    return firstRootMaterialCount == 7 && secondRootMaterialCount == 1;
}

// 验证森林复制共享根树，修改副本时只深复制目标根树。
bool testRootCopyOnWrite()
{
    MyVoxel::VoxelPackedForest source;

    source.setState(rootAddress(0, 0, 0), MyVoxel::VoxelState::Material);
    source.setState(rootAddress(1, 0, 0), MyVoxel::VoxelState::Material);

    MyVoxel::VoxelPackedForest copy = source;

    MyVoxel::VoxelPackedRootTree* sourceRoot = source.detachRootTree(MyVoxel::VoxelCellIndex(0, 0, 0));
    MyVoxel::VoxelPackedRootTree* copyRoot = copy.detachRootTree(MyVoxel::VoxelCellIndex(0, 0, 0));

    if (!sourceRoot || !copyRoot || sourceRoot == copyRoot)
    {
        return false;
    }

    copy.setState(rootAddress(0, 0, 0), MyVoxel::VoxelState::Empty);

    return source.state(rootAddress(0, 0, 0)) == MyVoxel::VoxelState::Material &&
           copy.state(rootAddress(0, 0, 0)) == MyVoxel::VoxelState::Empty &&
           source.state(rootAddress(1, 0, 0)) == MyVoxel::VoxelState::Material &&
           copy.state(rootAddress(1, 0, 0)) == MyVoxel::VoxelState::Material;
}

// 验证根索引范围遍历。
bool testRootRangeTraversal()
{
    MyVoxel::VoxelPackedForest forest;

    forest.setState(rootAddress(-1, 0, 0), MyVoxel::VoxelState::Material);
    forest.setState(rootAddress(0, 0, 0), MyVoxel::VoxelState::Material);
    forest.setState(rootAddress(1, 0, 0), MyVoxel::VoxelState::Material);
    forest.setState(rootAddress(2, 0, 0), MyVoxel::VoxelState::Material);

    std::size_t visitedCount = 0;

    const std::size_t returnedCount =
        forest.forEachRootCellInRange(
            MyVoxel::VoxelCellIndex(0, 0, 0),
            MyVoxel::VoxelCellIndex(1, 0, 0),
            [&](const MyVoxel::VoxelCellAddress&)
            {
                ++visitedCount;
            });

    return returnedCount == 2 && visitedCount == 2;
}

}

int main()
{
    check(testRootState(), "Root state");
    check(testSplitAndMerge(), "Split and merge");
    check(testDeepMaterialCreation(), "Deep material creation");
    check(testEmptyBranchPruning(), "Empty branch pruning");
    check(testMaterialTraversal(), "Material traversal");
    check(testRootCopyOnWrite(), "Root copy-on-write");
    check(testRootRangeTraversal(), "Root range traversal");

    std::cout << std::endl;
    std::cout << "Voxel packed forest tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}