#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>

#include "MyVoxel/Core/Mask/MaskUtils.h"
#include "MyVoxel/Core/Storage/LeafBlock.h"
#include "MyVoxel/Core/Storage/MaskBlock.h"
#include "MyVoxel/Core/Storage/NodeBlock.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/Tree/VoxelTreeCursor.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"

namespace
{

const float BackgroundDistance = 1.0f; // 综合测试统一使用1.0作为TSDF截断背景距离B。
const float FloatTolerance = 1.0e-6f; // 测试只比较简单有限float写入值，使用1e-6作为输出比较容差。

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果并累计统计。
void check(bool condition, const char* name)
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

// 比较两个float是否在测试容差内一致。
bool equalFloat(float first, float second)
{
    return std::fabs(first - second) <= FloatTolerance;
}

// 创建指定根索引的第0层地址。
MyVoxel::VoxelCellAddress rootAddress(const MyVoxel::VoxelCellIndex& rootIndex)
{
    return MyVoxel::VoxelCellAddress(rootIndex, MyVoxel::BaseVoxelLevel);
}

// 沿三个角点构造第3层地址，用于MaskLeaf最高层距离样本测试。
MyVoxel::VoxelCellAddress level3Address(const MyVoxel::VoxelCellIndex& rootIndex, MyVoxel::VoxelCorner first,
                                       MyVoxel::VoxelCorner second, MyVoxel::VoxelCorner third)
{
    MyVoxel::VoxelCellAddress address = rootAddress(rootIndex);
    address = MyVoxel::childCellAddress(address, first);
    address = MyVoxel::childCellAddress(address, second);
    address = MyVoxel::childCellAddress(address, third);
    return address;
}

/// 冻结基础布局

void testFrozenStorageLayout()
{
    check(sizeof(MyVoxel::NodeData) == 4, "NodeData size is 4 bytes");
    check(sizeof(MyVoxel::NodeBlock) == 8, "NodeBlock size is 8 bytes");
    check(sizeof(MyVoxel::IndexBlock) == 8, "IndexBlock size is 8 bytes");
    check(sizeof(MyVoxel::VoxelBlock) == 8, "VoxelBlock size is 8 bytes");
    check(sizeof(MyVoxel::MaskBlock) == 8, "MaskBlock size is 8 bytes");
    check(sizeof(MyVoxel::LeafBlock) == sizeof(float) * 64U, "LeafBlock contains exactly 64 floats");
    check(static_cast<unsigned int>(MyVoxel::VoxelCorner::Minimum) == 0U &&
          static_cast<unsigned int>(MyVoxel::VoxelCorner::MaximumXYZ) == 7U,
          "VoxelCorner remains 0 to 7 index encoding");
    check(MyVoxel::cornerBit(MyVoxel::VoxelCorner::Minimum) == 0x01U &&
          MyVoxel::cornerBit(MyVoxel::VoxelCorner::MaximumXY) == 0x08U &&
          MyVoxel::cornerBit(MyVoxel::VoxelCorner::MaximumXYZ) == 0x80U,
          "VoxelCorner converts to one-hot mask only through cornerBit");
}

/// VoxelTree终止Value

void testTreeTerminalValue()
{
    MyVoxel::VoxelTree emptyTree(MyVoxel::VoxelState::Empty, 0.25f);
    check(emptyTree.isValid(), "Empty root Tile Value tree is valid");
    check(emptyTree.state() == MyVoxel::VoxelState::Empty && equalFloat(emptyTree.value(), 0.25f), "Empty root stores positive Tile Value");
    check(emptyTree.cursor().isTerminal() && emptyTree.cursor().hasDistance() && equalFloat(emptyTree.cursor().distance(), 0.25f),
          "Root cursor exposes Empty Tile Value");
    check(emptyTree.isTsdfValid(BackgroundDistance), "Positive root Tile Value satisfies TSDF range");

    MyVoxel::VoxelTree materialTree(MyVoxel::VoxelState::Material, -0.5f);
    check(materialTree.isValid(), "Material root Tile Value tree is valid");
    check(materialTree.state() == MyVoxel::VoxelState::Material && equalFloat(materialTree.value(), -0.5f), "Material root stores negative Tile Value");
    check(materialTree.cursor().isTerminal() && equalFloat(materialTree.cursor().distance(), -0.5f), "Root cursor exposes Material Tile Value");
    check(materialTree.isTsdfValid(BackgroundDistance), "Negative root Tile Value satisfies TSDF range");
}

/// Tile细分和Node八槽Value继承

void testTreeSubdivisionInheritance()
{
    MyVoxel::VoxelTree tree(MyVoxel::VoxelState::Empty, 0.25f);
    MyVoxel::VoxelTreeEditor editor = tree.editor(BackgroundDistance);

    check(editor.subdivide(), "Root terminal Tile subdivides");
    check(tree.state() == MyVoxel::VoxelState::Subdivided, "Root becomes Subdivided after split");
    check(tree.allocatedGroupCount() == 1, "Root split allocates one complete eight-slot group");

    const MyVoxel::VoxelTreeCursor cursor = tree.cursor();
    bool allInherited = true;
    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelTreeCursor child = cursor.child(static_cast<MyVoxel::VoxelCorner>(cornerIndex));
        allInherited = allInherited && child.isEmpty() && child.hasDistance() && equalFloat(child.distance(), 0.25f);
    }
    check(allInherited, "All eight child Tiles inherit parent Value");
    check(tree.isValid() && tree.isTsdfValid(BackgroundDistance), "Subdivided inherited tree remains valid");
}

/// Node只有Value完全相同时才能无损折叠

void testTreeValueAwarePrune()
{
    MyVoxel::VoxelTree tree(MyVoxel::VoxelState::Empty, 0.5f);
    MyVoxel::VoxelTreeEditor editor = tree.editor(BackgroundDistance);
    MyVoxel::VoxelTreeEditor changedChild = editor.child(MyVoxel::VoxelCorner::MaximumXYZ);

    check(changedChild.setValue(0.25f), "Child Tile accepts a different positive Value");
    check(tree.cursor().child(MyVoxel::VoxelCorner::MaximumXYZ).isEmpty() &&
          equalFloat(tree.cursor().child(MyVoxel::VoxelCorner::MaximumXYZ).distance(), 0.25f),
          "Changed child keeps Empty state with independent Value");
    check(!tree.prune(BackgroundDistance), "Prune does not collapse same-state children with different Values");
    check(tree.state() == MyVoxel::VoxelState::Subdivided && tree.allocatedGroupCount() == 1, "Different child Value keeps root group alive");

    MyVoxel::VoxelTreeEditor restoreEditor = tree.editor(BackgroundDistance);
    check(restoreEditor.child(MyVoxel::VoxelCorner::MaximumXYZ).setValue(0.5f), "Child Tile restores inherited Value");
    check(tree.prune(BackgroundDistance), "Prune collapses eight identical child Values");
    check(tree.state() == MyVoxel::VoxelState::Empty && equalFloat(tree.value(), 0.5f), "Collapsed root preserves actual Tile Value");
    check(tree.allocatedGroupCount() == 0 && tree.allocatedLeafCount() == 0, "Collapsed root releases all physical descendants");
}

/// MaskLeaf继承Tile Value并按真实距离无损折叠

void testTreeLeafValueInheritance()
{
    MyVoxel::VoxelTree tree(MyVoxel::VoxelState::Empty, 0.25f);
    MyVoxel::VoxelTreeEditor rootEditor = tree.editor(BackgroundDistance);
    MyVoxel::VoxelTreeEditor leafEntryEditor = rootEditor.child(MyVoxel::VoxelCorner::Minimum);

    check(leafEntryEditor.touchLeaf(), "Terminal child materializes as MaskLeaf");
    check(tree.allocatedGroupCount() == 1 && tree.allocatedLeafCount() == 1, "MaskLeaf owns one node group and one leaf block");

    const MyVoxel::VoxelTreeCursor leafCursor = tree.cursor().child(MyVoxel::VoxelCorner::Minimum);
    bool allInherited = leafCursor.hasLeafData();
    if (allInherited)
    {
        const MyVoxel::LeafBlock& block = leafCursor.leafBlock();
        for (unsigned int sampleIndex = 0; sampleIndex < MyVoxel::LeafBlockSampleCount; ++sampleIndex)
        {
            allInherited = allInherited && equalFloat(block.distances[sampleIndex], 0.25f);
        }
        allInherited = allInherited && leafCursor.materialMask() == static_cast<std::uint64_t>(0);
    }
    check(allInherited, "MaskLeaf inherits Tile Value into all 64 distances and material mask");

    MyVoxel::VoxelTreeEditor sampleEditor = tree.editor(BackgroundDistance)
        .child(MyVoxel::VoxelCorner::Minimum)
        .child(MyVoxel::VoxelCorner::Minimum)
        .child(MyVoxel::VoxelCorner::MaximumXYZ);

    check(sampleEditor.setDistance(-0.5f), "Leaf sample accepts independent signed distance");
    const MyVoxel::VoxelTreeCursor sampleCursor = tree.cursor()
        .child(MyVoxel::VoxelCorner::Minimum)
        .child(MyVoxel::VoxelCorner::Minimum)
        .child(MyVoxel::VoxelCorner::MaximumXYZ);
    check(sampleCursor.hasLeafDistance() && sampleCursor.isMaterial() && equalFloat(sampleCursor.distance(), -0.5f),
          "Leaf sample distance and material bit stay synchronized");
    check(!tree.prune(BackgroundDistance), "Non-uniform MaskLeaf cannot be pruned");

    MyVoxel::VoxelTreeEditor restoreSampleEditor = tree.editor(BackgroundDistance)
        .child(MyVoxel::VoxelCorner::Minimum)
        .child(MyVoxel::VoxelCorner::Minimum)
        .child(MyVoxel::VoxelCorner::MaximumXYZ);
    check(restoreSampleEditor.setDistance(0.25f), "Leaf sample restores original inherited Value");
    check(tree.prune(BackgroundDistance), "Uniform MaskLeaf and parent Node prune bottom-up");
    check(tree.state() == MyVoxel::VoxelState::Empty && equalFloat(tree.value(), 0.25f), "Leaf prune preserves non-background positive Tile Value");
    check(tree.allocatedGroupCount() == 0 && tree.allocatedLeafCount() == 0, "Leaf prune releases group and leaf storage");
}

/// VoxelTree复制后的BlockPool写时复制

void testTreeCopyOnWrite()
{
    MyVoxel::VoxelTree original(MyVoxel::VoxelState::Empty, 0.5f);
    original.editor(BackgroundDistance).child(MyVoxel::VoxelCorner::MaximumX).setValue(0.25f);

    MyVoxel::VoxelTree copy = original;
    check(original.isValid() && copy.isValid(), "Copied trees are valid before modification");

    MyVoxel::VoxelTreeEditor copyEditor = copy.editor(BackgroundDistance);
    check(copyEditor.child(MyVoxel::VoxelCorner::MaximumX).setValue(-0.25f), "Copied tree child Value can be modified");

    const float originalValue = original.cursor().child(MyVoxel::VoxelCorner::MaximumX).distance();
    const float copyValue = copy.cursor().child(MyVoxel::VoxelCorner::MaximumX).distance();
    check(equalFloat(originalValue, 0.25f), "Original tree keeps child Value after copy modification");
    check(equalFloat(copyValue, -0.25f) && copy.cursor().child(MyVoxel::VoxelCorner::MaximumX).isMaterial(),
          "Copied tree owns modified child Value after COW");
    check(original.isValid() && copy.isValid(), "Both trees remain valid after COW");
}

/// Forest缺失Root与显式Root Value

void testForestRootValue()
{
    const MyVoxel::VoxelCellIndex rootIndex(2, -1, 3);
    const MyVoxel::VoxelCellAddress root = rootAddress(rootIndex);
    MyVoxel::VoxelForest forest(BackgroundDistance);

    check(forest.isValid() && forest.isEmpty(), "New Forest is valid and sparse-empty");
    check(forest.state(root) == MyVoxel::VoxelState::Empty && equalFloat(forest.distance(root), BackgroundDistance),
          "Missing Forest root resolves to positive background B");
    check(!forest.hasNode(root) && forest.rootCount() == 0, "Missing background root has no explicit node");

    forest.setTree(rootIndex, MyVoxel::VoxelTree(MyVoxel::VoxelState::Empty, 0.5f));
    check(forest.rootCount() == 1 && forest.hasNode(root), "Non-background Empty root is stored explicitly");
    check(forest.state(root) == MyVoxel::VoxelState::Empty && equalFloat(forest.distance(root), 0.5f),
          "Explicit Empty root preserves non-background Tile Value");
    check(forest.isValid(), "Forest accepts explicit non-background Empty Tile");

    check(forest.setState(root, MyVoxel::VoxelState::Empty), "Setting explicit Empty root to +B removes redundant root");
    check(forest.rootCount() == 0 && equalFloat(forest.distance(root), BackgroundDistance), "Removed root falls back to +B background");

    check(forest.setState(root, MyVoxel::VoxelState::Material), "Missing root can be set to Material");
    check(forest.rootCount() == 1 && forest.state(root) == MyVoxel::VoxelState::Material &&
          equalFloat(forest.distance(root), -BackgroundDistance),
          "Material setState writes -B Tile Value");
    check(forest.setState(root, MyVoxel::VoxelState::Empty) && forest.rootCount() == 0, "Root setEmpty returns Forest to sparse background");
}

/// Forest split/merge必须保留非背景Tile Value

void testForestSplitMergeValue()
{
    const MyVoxel::VoxelCellIndex rootIndex(4, 0, -2);
    const MyVoxel::VoxelCellAddress root = rootAddress(rootIndex);
    MyVoxel::VoxelForest forest(BackgroundDistance);

    forest.setTree(rootIndex, MyVoxel::VoxelTree(MyVoxel::VoxelState::Empty, 0.5f));
    check(forest.split(root), "Forest split accepts explicit terminal Tile");
    check(forest.state(root) == MyVoxel::VoxelState::Subdivided && equalFloat(forest.distance(MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumYZ)), 0.5f),
          "Forest split preserves inherited non-background Value");

    check(forest.merge(root), "Forest merge collapses eight identical direct Tile Values");
    check(forest.state(root) == MyVoxel::VoxelState::Empty && equalFloat(forest.distance(root), 0.5f), "Forest merge preserves actual merged Value");
    check(forest.rootCount() == 1, "Non-background Empty root remains explicitly stored after merge");
}

/// Forest最高层距离显式化与恢复背景

void testForestDistanceMaterialization()
{
    const MyVoxel::VoxelCellIndex rootIndex(0, 0, 0);
    const MyVoxel::VoxelCellAddress sample = level3Address(rootIndex, MyVoxel::VoxelCorner::Minimum,
                                                           MyVoxel::VoxelCorner::MaximumX, MyVoxel::VoxelCorner::MaximumXYZ);
    const MyVoxel::VoxelCellAddress neighbor = level3Address(rootIndex, MyVoxel::VoxelCorner::Minimum,
                                                             MyVoxel::VoxelCorner::MaximumX, MyVoxel::VoxelCorner::Minimum);
    MyVoxel::VoxelForest forest(BackgroundDistance);

    check(forest.setDistance(sample, 0.25f), "Forest materializes one highest-level TSDF sample");
    check(forest.rootCount() == 1 && equalFloat(forest.distance(sample), 0.25f), "Materialized sample keeps requested Value");
    check(equalFloat(forest.distance(neighbor), BackgroundDistance), "Unmodified sample inherits previous +B field");
    check(forest.isValid(), "Forest remains valid after sample materialization");

    check(forest.setDistance(sample, BackgroundDistance), "Highest-level sample can restore background Value");
    check(forest.pruneTree(rootIndex), "Prune removes now-redundant background hierarchy");
    check(forest.rootCount() == 0 && forest.isEmpty(), "Background-equivalent root is erased after prune");
    check(equalFloat(forest.distance(sample), BackgroundDistance), "Erased root still resolves to +B");
}

/// Shape和Session的Shape级COW

void testShapeSessionCopyOnWrite()
{
    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(3));
    const MyVoxel::VoxelCellIndex rootIndex(0, 0, 0);
    const MyVoxel::VoxelCellAddress sample = level3Address(rootIndex, MyVoxel::VoxelCorner::MaximumX,
                                                           MyVoxel::VoxelCorner::MaximumY, MyVoxel::VoxelCorner::MaximumZ);

    MyVoxel::VoxelShape original(grid, BackgroundDistance);
    MyVoxel::VoxelShape copy = original;

    check(original.isValid() && copy.isValid(), "VoxelShape instances are valid");
    check(original.sharesDataWith(copy) && original.isDataShared() && copy.isDataShared(), "Copied VoxelShape initially shares SharedData");
    check(equalFloat(original.distance(sample), BackgroundDistance), "Original empty Shape resolves to +B");

    {
        MyVoxel::VoxelShapeSession session = copy.session(static_cast<MyVoxel::VoxelLevel>(1));
        check(!copy.sharesDataWith(original) && !copy.isDataShared(), "Creating write session detaches copied Shape data");
        check(session.setDistance(sample, -0.25f), "Shape session writes highest-level TSDF distance");
        check(session.hasChanges(), "Shape session records field change");
        check(equalFloat(session.distance(sample), -0.25f), "Shape session reads its modified distance");
    }

    check(equalFloat(original.distance(sample), BackgroundDistance), "Original Shape remains unchanged after copied Shape session");
    check(equalFloat(copy.distance(sample), -0.25f), "Modified Shape keeps detached TSDF value");
    check(original.isEmpty() && !copy.isEmpty(), "Shape COW isolates sparse Forest root creation");
    check(original.isValid() && copy.isValid(), "Both Shapes remain valid after COW modification");
}

}

int main()
{
    std::cout << "MyVoxel Tile Value / Tree / Forest / Shape regression test" << std::endl << std::endl;

    testFrozenStorageLayout();
    testTreeTerminalValue();
    testTreeSubdivisionInheritance();
    testTreeValueAwarePrune();
    testTreeLeafValueInheritance();
    testTreeCopyOnWrite();
    testForestRootValue();
    testForestSplitMergeValue();
    testForestDistanceMaterialization();
    testShapeSessionCopyOnWrite();

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}