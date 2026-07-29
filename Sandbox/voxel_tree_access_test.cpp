#include <cstddef>
#include <iostream>

#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/Tree/VoxelTreeConstCursor.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelTypes.h"
#include "MyVoxel/Foundation/RefPtr.h"

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

// 测试不存在体素树时的虚拟空节点游标。
bool testMissingTreeCursor()
{
    const MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress address = makeAddress(5, -3, 7, 3);
    const MyVoxel::VoxelTreeConstCursor cursor(forest, address);
    const MyVoxel::VoxelTreeConstCursor child = cursor.child(MyVoxel::VoxelCorner::MaximumXYZ);

    const bool passed =
        cursor.isVirtual() &&
        !cursor.hasActualNode() &&
        cursor.isEmpty() &&
        cursor.isLeaf() &&
        child.isVirtual() &&
        child.isEmpty();

    return check(passed, "VoxelTreeConstCursor missing tree");
}

// 测试材料叶节点向下形成虚拟材料节点。
bool testVirtualMaterialCursor()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress target = makeAddress(7, 5, 3, 3);

    forest.setState(root, MyVoxel::VoxelState::Material);

    const MyVoxel::VoxelTreeConstCursor rootCursor(forest, root);
    const MyVoxel::VoxelTreeConstCursor targetCursor(forest, target);
    const MyVoxel::VoxelTreeConstCursor childCursor = targetCursor.child(MyVoxel::VoxelCorner::Minimum);

    const bool passed =
        rootCursor.hasActualNode() &&
        !rootCursor.isVirtual() &&
        rootCursor.isMaterial() &&
        targetCursor.isVirtual() &&
        targetCursor.isMaterial() &&
        childCursor.isVirtual() &&
        childCursor.isMaterial();

    return check(passed, "VoxelTreeConstCursor virtual material");
}

// 测试游标访问实际细分节点。
bool testActualNodeCursor()
{
    MyVoxel::VoxelForest forest;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress childAddress = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::MaximumXYZ);

    forest.setState(root, MyVoxel::VoxelState::Material);
    forest.split(root);
    forest.setStateDeferred(childAddress, MyVoxel::VoxelState::Empty);

    const MyVoxel::VoxelTreeConstCursor rootCursor(forest, root);
    const MyVoxel::VoxelTreeConstCursor childCursor = rootCursor.child(MyVoxel::VoxelCorner::MaximumXYZ);

    const bool passed =
        rootCursor.hasActualNode() &&
        rootCursor.isSubdivided() &&
        childCursor.hasActualNode() &&
        !childCursor.isVirtual() &&
        childCursor.isEmpty();

    return check(passed, "VoxelTreeConstCursor actual node");
}

// 测试编辑器拆分空叶节点并保持子节点初始状态。
bool testEmptyNodeSplit()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::VoxelTree> tree =
        MyVoxel::Foundation::makeRef<MyVoxel::VoxelTree>(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelTreeEditor editor(*tree);

    const bool split = editor.split();
    const MyVoxel::VoxelTreeConstCursor cursor(*tree);

    bool childrenEmpty = true;

    for (std::size_t cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        childrenEmpty =
            childrenEmpty &&
            cursor.child(static_cast<MyVoxel::VoxelCorner>(cornerIndex)).isEmpty();
    }

    const bool passed =
        split &&
        editor.isSubdivided() &&
        tree->nodePool.allocatedGroupCount() == 1 &&
        childrenEmpty;

    return check(passed, "VoxelTreeEditor empty node split");
}

// 测试编辑器拆分材料叶节点并保持子节点初始状态。
bool testMaterialNodeSplit()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::VoxelTree> tree =
        MyVoxel::Foundation::makeRef<MyVoxel::VoxelTree>(MyVoxel::VoxelState::Material);

    MyVoxel::VoxelTreeEditor editor(*tree);

    const bool split = editor.split();
    const MyVoxel::VoxelTreeConstCursor cursor(*tree);

    bool childrenMaterial = true;

    for (std::size_t cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        childrenMaterial =
            childrenMaterial &&
            cursor.child(static_cast<MyVoxel::VoxelCorner>(cornerIndex)).isMaterial();
    }

    const bool passed =
        split &&
        editor.isSubdivided() &&
        tree->nodePool.allocatedGroupCount() == 1 &&
        childrenMaterial;

    return check(passed, "VoxelTreeEditor material node split");
}

// 测试ensureChild创建路径并修改目标子节点。
bool testEnsureChild()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::VoxelTree> tree =
        MyVoxel::Foundation::makeRef<MyVoxel::VoxelTree>(MyVoxel::VoxelState::Empty);

    MyVoxel::VoxelTreeEditor rootEditor(*tree);
    MyVoxel::VoxelTreeEditor childEditor = rootEditor.ensureChild(MyVoxel::VoxelCorner::MaximumXYZ);

    const bool changed = childEditor.setState(MyVoxel::VoxelState::Material);
    const MyVoxel::VoxelTreeConstCursor rootCursor(*tree);
    const MyVoxel::VoxelTreeConstCursor targetCursor = rootCursor.child(MyVoxel::VoxelCorner::MaximumXYZ);
    const MyVoxel::VoxelTreeConstCursor otherCursor = rootCursor.child(MyVoxel::VoxelCorner::Minimum);

    const bool passed =
        changed &&
        rootEditor.isSubdivided() &&
        targetCursor.isMaterial() &&
        otherCursor.isEmpty();

    return check(passed, "VoxelTreeEditor ensure child");
}

// 测试统一子节点合并。
bool testEditorMerge()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::VoxelTree> tree =
        MyVoxel::Foundation::makeRef<MyVoxel::VoxelTree>(MyVoxel::VoxelState::Material);

    MyVoxel::VoxelTreeEditor editor(*tree);

    editor.split();

    const bool canMerge = editor.canMerge();
    const MyVoxel::VoxelState mergedState = editor.merge();

    const bool passed =
        canMerge &&
        mergedState == MyVoxel::VoxelState::Material &&
        editor.isMaterial() &&
        tree->nodePool.isEmpty();

    return check(passed, "VoxelTreeEditor merge");
}

// 测试混合子节点不能合并。
bool testMixedEditorMerge()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::VoxelTree> tree =
        MyVoxel::Foundation::makeRef<MyVoxel::VoxelTree>(MyVoxel::VoxelState::Material);

    MyVoxel::VoxelTreeEditor editor(*tree);

    editor.split();
    editor.child(MyVoxel::VoxelCorner::Minimum).setState(MyVoxel::VoxelState::Empty);

    const bool passed =
        !editor.canMerge() &&
        editor.merge() == MyVoxel::VoxelState::Subdivided &&
        editor.isSubdivided();

    return check(passed, "VoxelTreeEditor mixed merge");
}

// 测试设置叶节点状态时递归释放全部下级节点组。
bool testEditorReplaceSubtree()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::VoxelTree> tree =
        MyVoxel::Foundation::makeRef<MyVoxel::VoxelTree>(MyVoxel::VoxelState::Material);

    MyVoxel::VoxelTreeEditor rootEditor(*tree);

    MyVoxel::VoxelTreeEditor childEditor = rootEditor.ensureChild(MyVoxel::VoxelCorner::MaximumXYZ);
    childEditor.ensureChild(MyVoxel::VoxelCorner::MaximumXYZ);

    const bool beforePassed = tree->nodePool.allocatedGroupCount() == 2;
    const bool changed = rootEditor.setState(MyVoxel::VoxelState::Empty);

    const bool passed =
        beforePassed &&
        changed &&
        rootEditor.isEmpty() &&
        tree->nodePool.isEmpty();

    return check(passed, "VoxelTreeEditor replace subtree");
}

// 测试通过森林创建编辑器时执行根级写时复制。
bool testForestCopyOnWriteEditor()
{
    MyVoxel::VoxelForest original;
    const MyVoxel::VoxelCellAddress root = makeAddress(0, 0, 0, 0);
    const MyVoxel::VoxelCellAddress child = MyVoxel::childCellAddress(root, MyVoxel::VoxelCorner::Minimum);

    original.setState(root, MyVoxel::VoxelState::Material);
    original.split(root);

    MyVoxel::VoxelForest copy = original;
    MyVoxel::VoxelTreeEditor editor(copy, child);
    editor.setState(MyVoxel::VoxelState::Empty);

    const bool passed =
        original.state(child) == MyVoxel::VoxelState::Material &&
        copy.state(child) == MyVoxel::VoxelState::Empty &&
        original.state(root) == MyVoxel::VoxelState::Subdivided &&
        copy.state(root) == MyVoxel::VoxelState::Subdivided;

    return check(passed, "VoxelTreeEditor forest copy-on-write");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testMissingTreeCursor(),
        testVirtualMaterialCursor(),
        testActualNodeCursor(),
        testEmptyNodeSplit(),
        testMaterialNodeSplit(),
        testEnsureChild(),
        testEditorMerge(),
        testMixedEditorMerge(),
        testEditorReplaceSubtree(),
        testForestCopyOnWriteEditor()
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
    std::cout << "Voxel tree access tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}