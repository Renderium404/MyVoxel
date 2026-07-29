#include <cstdint>
#include <iostream>

#include "MyVoxel/Core/Tree/VoxelPackedRootTree.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeConstCursor.h"
#include "MyVoxel/Core/Tree/VoxelPackedTreeEditor.h"

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

// 验证根节点状态和虚拟材料子节点访问。
bool testInitialStateAndVirtualMaterial()
{
    MyVoxel::VoxelPackedRootTree tree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedTreeConstCursor cursor(tree);

    if (!tree.isValid() || cursor.state() != MyVoxel::VoxelState::Material)
    {
        return false;
    }

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        if (cursor.child(corner(cornerIndex)).state() != MyVoxel::VoxelState::Material)
        {
            return false;
        }
    }

    return tree.blockPool.allocatedGroupCount() == 0;
}

// 验证根节点和内部节点细分后的掩码、连续物理节点及状态访问。
bool testSplitAndChildAddressing()
{
    MyVoxel::VoxelPackedRootTree tree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);

    rootEditor.split();

    if (rootEditor.state() != MyVoxel::VoxelState::Subdivided ||
        tree.rootBlock.childMask != 0 ||
        tree.rootBlock.leafMask != 0xFF ||
        tree.blockPool.allocatedGroupCount() != 0)
    {
        return false;
    }

    const MyVoxel::VoxelCorner childCorner = corner(2);
    MyVoxel::VoxelPackedTreeEditor childEditor = rootEditor.child(childCorner);

    childEditor.split();

    if (childEditor.state() != MyVoxel::VoxelState::Subdivided ||
        tree.blockPool.allocatedGroupCount() != 1 ||
        !tree.rootBlock.isValid() ||
        !tree.blockPool.isChildrenAddressAligned(tree.rootBlock.firstChildIndex))
    {
        return false;
    }

    const MyVoxel::VoxelNodeIndex childNodeIndex = tree.rootBlock.childNodeIndex(childCorner);
    const MyVoxel::VoxelNodeBlock& childBlock = tree.blockPool.node(childNodeIndex);

    if (childBlock.leafMask != 0xFF || childBlock.childMask != 0)
    {
        return false;
    }

    const MyVoxel::VoxelCorner grandchildCorner = corner(5);
    childEditor.child(grandchildCorner).setState(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelPackedTreeConstCursor rootCursor(tree);
    const MyVoxel::VoxelPackedTreeConstCursor childCursor = rootCursor.child(childCorner);

    return childCursor.state() == MyVoxel::VoxelState::Subdivided &&
           childCursor.child(grandchildCorner).state() == MyVoxel::VoxelState::Empty &&
           childCursor.child(corner(0)).state() == MyVoxel::VoxelState::Material;
}

// 验证内部节点和根节点合并，并在最后一个细分子节点消失时回收物理节点组。
bool testMergeAndGroupRelease()
{
    MyVoxel::VoxelPackedRootTree tree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);

    rootEditor.split();

    const MyVoxel::VoxelCorner childCorner = corner(3);
    const MyVoxel::VoxelCorner grandchildCorner = corner(6);
    MyVoxel::VoxelPackedTreeEditor childEditor = rootEditor.child(childCorner);

    childEditor.split();
    childEditor.child(grandchildCorner).setState(MyVoxel::VoxelState::Empty);

    if (childEditor.merge() != MyVoxel::VoxelState::Subdivided)
    {
        return false;
    }

    childEditor.child(grandchildCorner).setState(MyVoxel::VoxelState::Material);

    if (childEditor.merge() != MyVoxel::VoxelState::Material ||
        tree.blockPool.allocatedGroupCount() != 0 ||
        tree.rootBlock.childMask != 0 ||
        tree.rootBlock.leafMask != 0xFF)
    {
        return false;
    }

    return rootEditor.merge() == MyVoxel::VoxelState::Material &&
           tree.rootState == MyVoxel::VoxelState::Material &&
           tree.isValid();
}

// 验证将高层节点直接设置为叶状态时会迭代释放全部后代物理节点组。
bool testRecursiveRelease()
{
    MyVoxel::VoxelPackedRootTree tree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);

    rootEditor.split();

    MyVoxel::VoxelPackedTreeEditor level1Editor = rootEditor.child(corner(0));
    level1Editor.split();

    MyVoxel::VoxelPackedTreeEditor level2Editor = level1Editor.child(corner(1));
    level2Editor.split();

    MyVoxel::VoxelPackedTreeEditor level3Editor = level2Editor.child(corner(2));
    level3Editor.split();

    if (tree.blockPool.allocatedGroupCount() != 3)
    {
        return false;
    }

    rootEditor.setState(MyVoxel::VoxelState::Empty);

    return tree.rootState == MyVoxel::VoxelState::Empty &&
           tree.blockPool.allocatedGroupCount() == 0 &&
           tree.isValid();
}

// 验证Packed根树复制时执行节点池深复制，副本修改不影响源树。
bool testDeepCopy()
{
    MyVoxel::VoxelPackedRootTree source(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedTreeEditor sourceRootEditor(source);

    sourceRootEditor.split();

    const MyVoxel::VoxelCorner childCorner = corner(4);
    MyVoxel::VoxelPackedTreeEditor sourceChildEditor = sourceRootEditor.child(childCorner);

    sourceChildEditor.split();
    sourceChildEditor.child(corner(7)).setState(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelPackedRootTree copy = source;

    if (copy.rootState != source.rootState ||
        copy.rootBlock.firstChildIndex != source.rootBlock.firstChildIndex ||
        copy.blockPool.allocatedGroupCount() != source.blockPool.allocatedGroupCount())
    {
        return false;
    }

    const MyVoxel::VoxelNodeIndex firstChildIndex = source.rootBlock.firstChildIndex;

    if (&copy.blockPool.node(firstChildIndex) == &source.blockPool.node(firstChildIndex))
    {
        return false;
    }

    MyVoxel::VoxelPackedTreeEditor copyRootEditor(copy);
    copyRootEditor.setState(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelPackedTreeConstCursor sourceCursor(source);
    MyVoxel::VoxelPackedTreeConstCursor copyCursor(copy);

    return sourceCursor.state() == MyVoxel::VoxelState::Subdivided &&
           sourceCursor.child(childCorner).state() == MyVoxel::VoxelState::Subdivided &&
           copyCursor.state() == MyVoxel::VoxelState::Empty &&
           source.blockPool.allocatedGroupCount() == 1 &&
           copy.blockPool.allocatedGroupCount() == 0;
}

// 验证核心结构操作不修改仍然存活节点块的用户数据。
bool testUserDataPreserved()
{
    MyVoxel::VoxelPackedRootTree tree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);

    rootEditor.split();

    const MyVoxel::VoxelCorner childCorner = corner(1);
    MyVoxel::VoxelPackedTreeEditor childEditor = rootEditor.child(childCorner);

    childEditor.split();

    const MyVoxel::VoxelNodeIndex childNodeIndex = tree.rootBlock.childNodeIndex(childCorner);
    MyVoxel::VoxelNodeBlock& childBlock = tree.blockPool.node(childNodeIndex);

    childBlock.userData = 54321;
    childEditor.child(corner(0)).setState(MyVoxel::VoxelState::Empty);
    childEditor.child(corner(1)).setState(MyVoxel::VoxelState::Empty);

    return childBlock.userData == 54321;
}

}

int main()
{
    check(testInitialStateAndVirtualMaterial(), "Initial state and virtual material");
    check(testSplitAndChildAddressing(), "Split and child addressing");
    check(testMergeAndGroupRelease(), "Merge and group release");
    check(testRecursiveRelease(), "Recursive descendant release");
    check(testDeepCopy(), "Packed root tree deep copy");
    check(testUserDataPreserved(), "User data preservation");

    std::cout << std::endl;
    std::cout << "Voxel packed tree tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}