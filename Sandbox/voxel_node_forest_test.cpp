#include <cstddef>
#include <iostream>

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

// 测试空森林默认状态。
bool testDefaultState()
{
    const MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress address = makeAddress(0, 0, 0, 0);

    const bool passed =
        forest.isEmpty() &&
        forest.rootCount() == 0 &&
        forest.state(address) == MyVoxel::VoxelState::Empty &&
        !forest.hasNode(address);

    return check(passed, "VoxelForest default state");
}

// 测试第0层根节点状态。
bool testRootState()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(2, -1, 3, 0);
    const MyVoxel::VoxelCellAddress descendant = makeAddress(17, -2, 27, 3);

    const bool firstChange = forest.setState(root, MyVoxel::VoxelState::Material);
    const bool repeatedChange = forest.setState(root, MyVoxel::VoxelState::Material);

    const bool passed =
        firstChange &&
        !repeatedChange &&
        forest.rootCount() == 1 &&
        forest.hasNode(root) &&
        !forest.hasNode(descendant) &&
        forest.state(root) == MyVoxel::VoxelState::Material &&
        forest.state(descendant) == MyVoxel::VoxelState::Material;

    return check(passed, "VoxelForest root state");
}

// 测试深层节点路径创建。
bool testDeepNodeCreation()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress target = makeAddress(5, -2, 7, 3);
    const MyVoxel::VoxelCellAddress parent = MyVoxel::parentCellAddress(target);
    const MyVoxel::VoxelCellAddress sibling = MyVoxel::childCellAddress(parent, MyVoxel::VoxelCorner::Minimum);

    const bool changed = forest.setState(target, MyVoxel::VoxelState::Material);

    const bool passed =
        changed &&
        forest.rootCount() == 1 &&
        forest.hasNode(target) &&
        forest.state(target) == MyVoxel::VoxelState::Material &&
        forest.state(sibling) == MyVoxel::VoxelState::Empty;

    return check(passed, "VoxelForest deep node creation");
}

// 测试材料节点细分。
bool testSplit()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);

    forest.setState(root, MyVoxel::VoxelState::Material);
    const bool split = forest.split(root);

    bool childrenMaterial = true;

    for (std::size_t cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, static_cast<MyVoxel::VoxelCorner>(cornerIndex));

        childrenMaterial =
            childrenMaterial &&
            forest.hasNode(child) &&
            forest.state(child) == MyVoxel::VoxelState::Material;
    }

    const bool passed =
        split &&
        forest.state(root) == MyVoxel::VoxelState::Subdivided &&
        childrenMaterial &&
        !forest.split(root);

    return check(passed, "VoxelForest split");
}

// 测试统一材料子节点合并。
bool testUniformMerge()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumXYZ);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);

    const bool merged = forest.merge(root);

    const bool passed =
        merged &&
        forest.state(root) == MyVoxel::VoxelState::Material &&
        !forest.hasNode(child);

    return check(passed, "VoxelForest uniform merge");
}

// 测试混合子节点不能合并。
bool testMixedMerge()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::Minimum);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(child, MyVoxel::VoxelState::Empty);

    const MyVoxel::VoxelState mergedState = forest.mergeDeferred(root);

    const bool passed =
        mergedState == MyVoxel::VoxelState::Subdivided &&
        forest.state(root) == MyVoxel::VoxelState::Subdivided &&
        forest.state(child) == MyVoxel::VoxelState::Empty;

    return check(passed, "VoxelForest mixed merge");
}

// 测试延迟设置为空和统一空分支清理。
bool testDeferredEmptyPruning()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress target = makeAddress(5, -2, 7, 3);

    forest.setState(target, MyVoxel::VoxelState::Material);
    const bool changed = forest.setStateDeferred(target, MyVoxel::VoxelState::Empty);

    const bool deferredPassed =
        changed &&
        forest.rootCount() == 1 &&
        forest.hasNode(target) &&
        forest.state(target) == MyVoxel::VoxelState::Empty;

    forest.pruneEmptyBranch(target);

    const bool prunedPassed =
        forest.rootCount() == 0 &&
        forest.isEmpty() &&
        !forest.hasNode(target);

    return check(deferredPassed && prunedPassed, "VoxelForest deferred empty pruning");
}

// 测试设置叶节点时释放全部下级结构。
bool testReplaceSubtree()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumXYZ);
    const MyVoxel::VoxelCellAddress grandchild = MyVoxel::childCellAddress(child, MyVoxel::VoxelCorner::MaximumXYZ);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.split(child);

    const bool changed = forest.setState(root, MyVoxel::VoxelState::Material);

    const bool passed =
        changed &&
        forest.state(root) == MyVoxel::VoxelState::Material &&
        !forest.hasNode(child) &&
        !forest.hasNode(grandchild);

    return check(passed, "VoxelForest replace subtree");
}

// 测试森林复制后的根级写时复制。
bool testCopyOnWrite()
{
    MyVoxel::VoxelForest original;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::Minimum);

    original.setState(root, MyVoxel::VoxelState::Material);
    original.split(root);

    MyVoxel::VoxelForest copy = original;
    copy.setState(child, MyVoxel::VoxelState::Empty);

    const bool passed =
        original.state(root) == MyVoxel::VoxelState::Subdivided &&
        copy.state(root) == MyVoxel::VoxelState::Subdivided &&
        original.state(child) == MyVoxel::VoxelState::Material &&
        copy.state(child) == MyVoxel::VoxelState::Empty &&
        original.rootCount() == 1 &&
        copy.rootCount() == 1;

    return check(passed, "VoxelForest root copy-on-write");
}

// 测试负索引深层地址。
bool testNegativeAddress()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress target = makeAddress(-9, -6, -3, 3);
    const MyVoxel::VoxelCellAddress root = MyVoxel::rootCellAddress(target);

    forest.setState(target, MyVoxel::VoxelState::Material);

    const bool passed =
        root.level == MyVoxel::BaseVoxelLevel &&
        forest.rootCount() == 1 &&
        forest.state(target) == MyVoxel::VoxelState::Material &&
        forest.hasNode(target);

    return check(passed, "VoxelForest negative address");
}

// 测试根节点删除和森林清空。
bool testRootDeletionAndClear()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress first = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress second = makeAddress(1, 0, 0, 0);

    forest.setState(first, MyVoxel::VoxelState::Material);
    forest.setState(second, MyVoxel::VoxelState::Material);

    const bool erased = forest.setState(first, MyVoxel::VoxelState::Empty);

    const bool erasePassed =
        erased &&
        forest.rootCount() == 1 &&
        forest.state(first) == MyVoxel::VoxelState::Empty &&
        forest.state(second) == MyVoxel::VoxelState::Material;

    forest.clear();

    const bool clearPassed =
        forest.isEmpty() &&
        forest.rootCount() == 0 &&
        forest.state(second) == MyVoxel::VoxelState::Empty;

    return check(erasePassed && clearPassed, "VoxelForest root deletion and clear");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testDefaultState(),
        testRootState(),
        testDeepNodeCreation(),
        testSplit(),
        testUniformMerge(),
        testMixedMerge(),
        testDeferredEmptyPruning(),
        testReplaceSubtree(),
        testCopyOnWrite(),
        testNegativeAddress(),
        testRootDeletionAndClear()
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
    std::cout << "Voxel node forest tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}