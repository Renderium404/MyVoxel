#include <cstddef>
#include <iostream>
#include <set>
#include <vector>

#include "MyVoxel/Core/Tree/VoxelForest.h"
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

// 判断地址集合是否包含指定体素地址。
bool containsAddress(const std::set<MyVoxel::VoxelCellAddress>& addresses, const MyVoxel::VoxelCellAddress& address)
{
    return addresses.find(address) != addresses.end();
}

// 测试空森林遍历。
bool testEmptyTraversal()
{
    const MyVoxel::VoxelForest forest;
    std::size_t rootVisitorCount = 0;
    std::size_t materialVisitorCount = 0;

    const std::size_t rootCount = forest.forEachRootCellInRange(
        MyVoxel::VoxelCellIndex(-1, -1, -1),
        MyVoxel::VoxelCellIndex(1, 1, 1),
        [&rootVisitorCount](const MyVoxel::VoxelCellAddress&)
        {
            ++rootVisitorCount;
        });

    forest.forEachMaterialCell(
        [&materialVisitorCount](const MyVoxel::VoxelCellAddress&)
        {
            ++materialVisitorCount;
        });

    const bool passed =
        rootCount == 0 &&
        rootVisitorCount == 0 &&
        materialVisitorCount == 0;

    return check(passed, "VoxelForest empty traversal");
}

// 测试较小XY候选范围使用Z区间扫描。
bool testRootRangeTraversal()
{
    MyVoxel::VoxelForest forest;

    const MyVoxel::VoxelCellAddress first = makeAddress(0, 0, -2, 0);
    const MyVoxel::VoxelCellAddress second = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress third = makeAddress(0, 0, 3, 0);
    const MyVoxel::VoxelCellAddress fourth = makeAddress(1, 0, 0, 0);
    const MyVoxel::VoxelCellAddress fifth = makeAddress(2, 0, 0, 0);

    forest.setState(first, MyVoxel::VoxelState::Material);
    forest.setState(second, MyVoxel::VoxelState::Material);
    forest.setState(third, MyVoxel::VoxelState::Material);
    forest.setState(fourth, MyVoxel::VoxelState::Material);
    forest.setState(fifth, MyVoxel::VoxelState::Material);

    std::set<MyVoxel::VoxelCellAddress> visited;

    const std::size_t rootCount = forest.forEachRootCellInRange(
        MyVoxel::VoxelCellIndex(0, 0, -1),
        MyVoxel::VoxelCellIndex(1, 0, 1),
        [&visited](const MyVoxel::VoxelCellAddress& address)
        {
            visited.insert(address);
        });

    const bool passed =
        rootCount == 2 &&
        visited.size() == 2 &&
        containsAddress(visited, second) &&
        containsAddress(visited, fourth) &&
        !containsAddress(visited, first) &&
        !containsAddress(visited, third) &&
        !containsAddress(visited, fifth);

    return check(passed, "VoxelForest root range traversal");
}

// 测试大范围查询直接过滤稀疏现有根树。
bool testLargeSparseRootRange()
{
    MyVoxel::VoxelForest forest;

    const MyVoxel::VoxelCellAddress first = makeAddress(-500, 20, 30, 0);
    const MyVoxel::VoxelCellAddress second = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress third = makeAddress(700, -40, 10, 0);

    forest.setState(first, MyVoxel::VoxelState::Material);
    forest.setState(second, MyVoxel::VoxelState::Material);
    forest.setState(third, MyVoxel::VoxelState::Material);

    std::set<MyVoxel::VoxelCellAddress> visited;

    const std::size_t rootCount = forest.forEachRootCellInRange(
        MyVoxel::VoxelCellIndex(-100000, -100000, -100000),
        MyVoxel::VoxelCellIndex(100000, 100000, 100000),
        [&visited](const MyVoxel::VoxelCellAddress& address)
        {
            visited.insert(address);
        });

    const bool passed =
        rootCount == 3 &&
        visited.size() == 3 &&
        containsAddress(visited, first) &&
        containsAddress(visited, second) &&
        containsAddress(visited, third);

    return check(passed, "VoxelForest large sparse root range");
}

// 测试全部材料叶节点递归遍历。
bool testMaterialCellTraversal()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress emptyChild = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::Minimum);
    const MyVoxel::VoxelCellAddress subdividedChild = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumXYZ);
    const MyVoxel::VoxelCellAddress emptyGrandchild = MyVoxel::childCellAddress(subdividedChild, MyVoxel::VoxelCorner::Minimum);
    const MyVoxel::VoxelCellAddress materialChild = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumX);
    const MyVoxel::VoxelCellAddress materialGrandchild = MyVoxel::childCellAddress(subdividedChild, MyVoxel::VoxelCorner::MaximumX);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(emptyChild, MyVoxel::VoxelState::Empty);
    forest.split(subdividedChild);
    forest.setStateDeferred(emptyGrandchild, MyVoxel::VoxelState::Empty);

    std::set<MyVoxel::VoxelCellAddress> materialCells;

    forest.forEachMaterialCell(
        [&materialCells](const MyVoxel::VoxelCellAddress& address)
        {
            materialCells.insert(address);
        });

    const bool passed =
        materialCells.size() == 13 &&
        containsAddress(materialCells, materialChild) &&
        containsAddress(materialCells, materialGrandchild) &&
        !containsAddress(materialCells, root) &&
        !containsAddress(materialCells, emptyChild) &&
        !containsAddress(materialCells, subdividedChild) &&
        !containsAddress(materialCells, emptyGrandchild);

    return check(passed, "VoxelForest material cell traversal");
}

// 测试指定根范围内的材料叶节点遍历。
bool testMaterialCellRootRangeTraversal()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress firstRoot = makeAddress(-1, 0, 0, 0);
    const MyVoxel::VoxelCellAddress secondRoot = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress thirdRoot = makeAddress(2, 0, 0, 0);
    const MyVoxel::VoxelCellAddress emptyChild = MyVoxel::childCellAddress(secondRoot, MyVoxel::VoxelCorner::Minimum);
    const MyVoxel::VoxelCellAddress materialChild = MyVoxel::childCellAddress(secondRoot, MyVoxel::VoxelCorner::MaximumXYZ);

    forest.setState(firstRoot, MyVoxel::VoxelState::Material);
    forest.setState(secondRoot, MyVoxel::VoxelState::Material);
    forest.split(secondRoot);
    forest.setStateDeferred(emptyChild, MyVoxel::VoxelState::Empty);
    forest.setState(thirdRoot, MyVoxel::VoxelState::Material);

    std::set<MyVoxel::VoxelCellAddress> materialCells;

    const std::size_t rootCount = forest.forEachMaterialCellInRootRange(
        MyVoxel::VoxelCellIndex(-1, 0, 0),
        MyVoxel::VoxelCellIndex(0, 0, 0),
        [&materialCells](const MyVoxel::VoxelCellAddress& address)
        {
            materialCells.insert(address);
        });

    const bool passed =
        rootCount == 2 &&
        materialCells.size() == 8 &&
        containsAddress(materialCells, firstRoot) &&
        containsAddress(materialCells, materialChild) &&
        !containsAddress(materialCells, emptyChild) &&
        !containsAddress(materialCells, thirdRoot);

    return check(passed, "VoxelForest material root range traversal");
}

// 测试材料遍历使用稳定的根索引顺序。
bool testTraversalOrder()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress positiveRoot = makeAddress(1, 0, 0, 0);
    const MyVoxel::VoxelCellAddress negativeRoot = makeAddress(-1, 0, 0, 0);

    forest.setState(positiveRoot, MyVoxel::VoxelState::Material);
    forest.setState(negativeRoot, MyVoxel::VoxelState::Material);

    std::vector<MyVoxel::VoxelCellAddress> materialCells;

    forest.forEachMaterialCell(
        [&materialCells](const MyVoxel::VoxelCellAddress& address)
        {
            materialCells.push_back(address);
        });

    const bool passed =
        materialCells.size() == 2 &&
        materialCells[0] == negativeRoot &&
        materialCells[1] == positiveRoot;

    return check(passed, "VoxelForest traversal order");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testEmptyTraversal(),
        testRootRangeTraversal(),
        testLargeSparseRootRange(),
        testMaterialCellTraversal(),
        testMaterialCellRootRangeTraversal(),
        testTraversalOrder()
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
    std::cout << "Voxel forest traversal tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}