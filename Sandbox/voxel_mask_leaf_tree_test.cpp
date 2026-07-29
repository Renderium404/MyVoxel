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

// 创建一个根节点已经细分的材料树。
MyVoxel::VoxelPackedRootTree makeSplitMaterialTree()
{
    MyVoxel::VoxelPackedRootTree tree(MyVoxel::VoxelState::Material);
    MyVoxel::VoxelPackedTreeEditor editor(tree);

    editor.split();
    return tree;
}

// 验证普通材料子节点能够转换为完整材料掩码叶块。
bool testCreateMaterialMaskLeaf()
{
    MyVoxel::VoxelPackedRootTree tree = makeSplitMaterialTree();
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);
    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(2));

    if (!leafEditor.makeMaskLeaf())
    {
        return false;
    }

    return leafEditor.isMaskLeaf() &&
           leafEditor.state() == MyVoxel::VoxelState::Subdivided &&
           leafEditor.maskLeaf().isFull() &&
           tree.blockPool.allocatedGroupCount() == 1 &&
           tree.rootBlock.childStorageState(corner(2)) == MyVoxel::VoxelStorageState::MaskLeaf;
}

// 验证游标能够透明访问掩码叶块内部两级逻辑节点。
bool testMaskLeafCursorTraversal()
{
    MyVoxel::VoxelPackedRootTree tree = makeSplitMaterialTree();
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);
    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(3));

    leafEditor.makeMaskLeaf();
    leafEditor.child(corner(4)).child(corner(6)).setState(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelPackedTreeConstCursor rootCursor(tree);
    const MyVoxel::VoxelPackedTreeConstCursor leafCursor = rootCursor.child(corner(3));
    const MyVoxel::VoxelPackedTreeConstCursor octantCursor = leafCursor.child(corner(4));

    return leafCursor.isMaskLeaf() &&
           leafCursor.state() == MyVoxel::VoxelState::Subdivided &&
           octantCursor.state() == MyVoxel::VoxelState::Subdivided &&
           octantCursor.child(corner(6)).state() == MyVoxel::VoxelState::Empty &&
           octantCursor.child(corner(5)).state() == MyVoxel::VoxelState::Material;
}

// 验证完整八分区能够直接作为材料或空节点访问。
bool testUniformOctantTraversal()
{
    MyVoxel::VoxelPackedRootTree tree = makeSplitMaterialTree();
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);
    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(1));

    leafEditor.makeMaskLeaf();
    leafEditor.child(corner(5)).setState(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelPackedTreeConstCursor rootCursor(tree);
    const MyVoxel::VoxelPackedTreeConstCursor leafCursor = rootCursor.child(corner(1));

    if (leafCursor.child(corner(5)).state() != MyVoxel::VoxelState::Empty ||
        leafCursor.child(corner(4)).state() != MyVoxel::VoxelState::Material)
    {
        return false;
    }

    for (unsigned int fineIndex = 0; fineIndex < MyVoxel::VoxelCornerCount; ++fineIndex)
    {
        if (leafCursor.child(corner(5)).child(corner(fineIndex)).state() != MyVoxel::VoxelState::Empty)
        {
            return false;
        }
    }

    return true;
}

// 验证掩码叶块只有完全为空或完全为材料时才折叠。
bool testMaskLeafMerge()
{
    MyVoxel::VoxelPackedRootTree tree = makeSplitMaterialTree();
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);
    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(6));

    leafEditor.makeMaskLeaf();
    leafEditor.child(corner(2)).child(corner(3)).setState(MyVoxel::VoxelState::Empty);

    if (leafEditor.merge() != MyVoxel::VoxelState::Subdivided)
    {
        return false;
    }

    leafEditor.child(corner(2)).child(corner(3)).setState(MyVoxel::VoxelState::Material);

    return leafEditor.merge() == MyVoxel::VoxelState::Material &&
           tree.rootBlock.childStorageState(corner(6)) == MyVoxel::VoxelStorageState::Material &&
           tree.blockPool.allocatedGroupCount() == 0;
}

// 验证全部位清空后掩码叶块折叠为空节点。
bool testEmptyMaskLeafMerge()
{
    MyVoxel::VoxelPackedRootTree tree = makeSplitMaterialTree();
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);
    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(0));

    leafEditor.makeMaskLeaf();
    leafEditor.maskLeaf().reset(MyVoxel::VoxelState::Empty);

    return leafEditor.merge() == MyVoxel::VoxelState::Empty &&
           tree.rootBlock.childStorageState(corner(0)) == MyVoxel::VoxelStorageState::Empty &&
           tree.blockPool.allocatedGroupCount() == 0;
}

// 验证掩码叶块与普通分支能够共存于同一八槽组。
bool testMaskLeafAndBranchSiblings()
{
    MyVoxel::VoxelPackedRootTree tree = makeSplitMaterialTree();
    MyVoxel::VoxelPackedTreeEditor rootEditor(tree);
    MyVoxel::VoxelPackedTreeEditor branchEditor = rootEditor.child(corner(1));
    MyVoxel::VoxelPackedTreeEditor leafEditor = rootEditor.child(corner(7));

    branchEditor.split();
    leafEditor.makeMaskLeaf();
    leafEditor.child(corner(3)).child(corner(4)).setState(MyVoxel::VoxelState::Empty);

    return tree.blockPool.allocatedGroupCount() == 1 &&
           tree.rootBlock.childStorageState(corner(1)) == MyVoxel::VoxelStorageState::Branch &&
           tree.rootBlock.childStorageState(corner(7)) == MyVoxel::VoxelStorageState::MaskLeaf &&
           branchEditor.state() == MyVoxel::VoxelState::Subdivided &&
           leafEditor.state() == MyVoxel::VoxelState::Subdivided;
}

// 验证根树深复制能够完整复制掩码叶块。
bool testMaskLeafDeepCopy()
{
    MyVoxel::VoxelPackedRootTree source = makeSplitMaterialTree();
    MyVoxel::VoxelPackedTreeEditor sourceRootEditor(source);
    MyVoxel::VoxelPackedTreeEditor sourceLeafEditor = sourceRootEditor.child(corner(4));

    sourceLeafEditor.makeMaskLeaf();
    sourceLeafEditor.child(corner(2)).child(corner(5)).setState(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelPackedRootTree copy = source;
    MyVoxel::VoxelPackedTreeEditor copyRootEditor(copy);
    MyVoxel::VoxelPackedTreeEditor copyLeafEditor = copyRootEditor.child(corner(4));

    copyLeafEditor.child(corner(2)).child(corner(5)).setState(MyVoxel::VoxelState::Material);

    MyVoxel::VoxelPackedTreeConstCursor sourceCursor(source);
    MyVoxel::VoxelPackedTreeConstCursor copyCursor(copy);

    return sourceCursor.child(corner(4)).child(corner(2)).child(corner(5)).state() == MyVoxel::VoxelState::Empty &&
           copyCursor.child(corner(4)).child(corner(2)).child(corner(5)).state() == MyVoxel::VoxelState::Material;
}

}

int main()
{
    check(testCreateMaterialMaskLeaf(), "Create material mask leaf");
    check(testMaskLeafCursorTraversal(), "Mask leaf cursor traversal");
    check(testUniformOctantTraversal(), "Uniform octant traversal");
    check(testMaskLeafMerge(), "Mask leaf material merge");
    check(testEmptyMaskLeafMerge(), "Mask leaf empty merge");
    check(testMaskLeafAndBranchSiblings(), "Mask leaf and branch siblings");
    check(testMaskLeafDeepCopy(), "Mask leaf deep copy");

    std::cout << std::endl;
    std::cout << "Voxel mask leaf tree tests" << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? 0 : 1;
}