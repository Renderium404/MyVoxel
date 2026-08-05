#include "VoxelTreeBooleanAlgorithm.h"

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/Mask/VoxelNodeMask.h"
#include "MyVoxel/Core/Tree/VoxelTreeCursor.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

namespace
{

class DisabledStatisticsRecorder
{
public:
    void visitVoxel() {}
    void leafMaskOperation() {}
    void clearChildren(std::uint8_t) {}
    void fillChildren(std::uint8_t) {}
    void copySubtree() {}
    void invertSubtree() {}
    void recurseChildren(std::uint8_t) {}
    void collapseVoxel() {}
};

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

class EnabledStatisticsRecorder
{
public:
    explicit EnabledStatisticsRecorder(VoxelTreeBooleanStatistics& statistics)
        : m_statistics(statistics)
    {
    }

    void visitVoxel()
    {
        ++m_statistics.visitedVoxelCount;
    }

    void leafMaskOperation()
    {
        ++m_statistics.leafMaskOperationCount;
    }

    void clearChildren(std::uint8_t mask)
    {
        m_statistics.directlyClearedChildCount += nodeMaskBitCount(mask);
    }

    void fillChildren(std::uint8_t mask)
    {
        m_statistics.directlyFilledChildCount += nodeMaskBitCount(mask);
    }

    void copySubtree()
    {
        ++m_statistics.copiedSubtreeCount;
    }

    void invertSubtree()
    {
        ++m_statistics.invertedSubtreeCount;
    }

    void recurseChildren(std::uint8_t mask)
    {
        m_statistics.recursiveChildCount += nodeMaskBitCount(mask);
    }

    void collapseVoxel()
    {
        ++m_statistics.collapsedVoxelCount;
    }

private:
    VoxelTreeBooleanStatistics& m_statistics;
};

using StatisticsRecorder = EnabledStatisticsRecorder;

#else

using StatisticsRecorder = DisabledStatisticsRecorder;

#endif

// 尝试将八个状态完全一致的子体素折叠为当前终止体素。
bool collapseVoxel(VoxelTreeEditor& editor, StatisticsRecorder& statistics)
{
    if (!editor.isSubdivided())
    {
        return false;
    }

    const VoxelChildStateMasks states = editor.childStateMasks();

    if (states.empty == FullVoxelNodeMask)
    {
        editor.setEmpty();
        statistics.collapseVoxel();
        return true;
    }

    if (states.material == FullVoxelNodeMask)
    {
        editor.setMaterial();
        statistics.collapseVoxel();
        return true;
    }

    return false;
}

// 将source逻辑子树复制到target，可选择在复制过程中反转空和材料。
bool copySubtree(VoxelTreeEditor& target, const VoxelTreeCursor& source, bool inverted, StatisticsRecorder& statistics)
{
    statistics.copySubtree();

    if (source.isTerminal())
    {
        const VoxelState sourceState = source.state();
        const VoxelState targetState = inverted ?
            (sourceState == VoxelState::Material ? VoxelState::Empty : VoxelState::Material) :
            sourceState;

        if (target.state() == targetState)
        {
            return false;
        }

        if (targetState == VoxelState::Material)
        {
            target.setMaterial();
        }
        else
        {
            target.setEmpty();
        }

        return true;
    }

    if (source.hasMaterialMask())
    {
        const std::uint64_t sourceMask = source.materialMask();
        const std::uint64_t targetMask = inverted ? ~sourceMask : sourceMask;

        statistics.leafMaskOperation();
        assert(target.canSetMaterialMask());
        return target.setMaterialMask(targetMask);
    }

    const VoxelChildStateMasks sourceStates = source.childStateMasks();
    const std::uint8_t targetEmptyMask = inverted ? sourceStates.material : sourceStates.empty;
    const std::uint8_t targetMaterialMask = inverted ? sourceStates.empty : sourceStates.material;

    bool changed = false;

    if (targetEmptyMask != EmptyVoxelNodeMask)
    {
        statistics.clearChildren(targetEmptyMask);
        changed = target.setChildrenState(targetEmptyMask, VoxelState::Empty) || changed;
    }

    if (targetMaterialMask != EmptyVoxelNodeMask)
    {
        statistics.fillChildren(targetMaterialMask);
        changed = target.setChildrenState(targetMaterialMask, VoxelState::Material) || changed;
    }

    std::uint8_t recursiveMask = sourceStates.subdivided;

    statistics.recurseChildren(recursiveMask);

    while (recursiveMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(recursiveMask);
        VoxelTreeEditor targetChild = target.child(corner);
        const VoxelTreeCursor sourceChild = source.child(corner);

        changed = copySubtree(targetChild, sourceChild, inverted, statistics) || changed;
    }

    collapseVoxel(target, statistics);
    return changed;
}

// 原地反转当前逻辑子树的全部空和材料状态。
bool invertSubtree(VoxelTreeEditor& editor, const VoxelTreeCursor& cursor, StatisticsRecorder& statistics)
{
    statistics.invertSubtree();

    if (cursor.isEmpty())
    {
        editor.setMaterial();
        return true;
    }

    if (cursor.isMaterial())
    {
        editor.setEmpty();
        return true;
    }

    if (cursor.hasMaterialMask())
    {
        statistics.leafMaskOperation();
        assert(editor.canSetMaterialMask());
        return editor.setMaterialMask(~cursor.materialMask());
    }

    const VoxelChildStateMasks states = cursor.childStateMasks();
    bool changed = false;

    if (states.empty != EmptyVoxelNodeMask)
    {
        statistics.fillChildren(states.empty);
        changed = editor.setChildrenState(states.empty, VoxelState::Material) || changed;
    }

    if (states.material != EmptyVoxelNodeMask)
    {
        statistics.clearChildren(states.material);
        changed = editor.setChildrenState(states.material, VoxelState::Empty) || changed;
    }

    std::uint8_t recursiveMask = states.subdivided;

    statistics.recurseChildren(recursiveMask);

    while (recursiveMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(recursiveMask);
        const VoxelTreeCursor childCursor = cursor.child(corner);
        VoxelTreeEditor childEditor = editor.child(corner);

        changed = invertSubtree(childEditor, childCursor, statistics) || changed;
    }

    collapseVoxel(editor, statistics);
    return changed;
}

bool applySubtree(VoxelTreeEditor& leftEditor, const VoxelTreeCursor& leftCursor,
                  const VoxelTreeCursor& rightCursor, VoxelBooleanType type,
                  StatisticsRecorder& statistics);

// 对两个已细分逻辑体素执行子层布尔运算。
bool applySubdivided(VoxelTreeEditor& leftEditor, const VoxelTreeCursor& leftCursor,
                     const VoxelTreeCursor& rightCursor, VoxelBooleanType type,
                     StatisticsRecorder& statistics)
{
    assert(leftCursor.isSubdivided());
    assert(rightCursor.isSubdivided());

    if (leftCursor.hasMaterialMask() && rightCursor.hasMaterialMask())
    {
        const VoxelLeafBooleanResult result = leafBooleanResult(type, leftCursor.materialMask(), rightCursor.materialMask());

        statistics.leafMaskOperation();

        if (!result.changed)
        {
            return false;
        }

        assert(leftEditor.canSetMaterialMask());
        return leftEditor.setMaterialMask(result.materialMask);
    }

    const VoxelChildStateMasks leftStates = leftCursor.childStateMasks();
    const VoxelChildStateMasks rightStates = rightCursor.childStateMasks();
    const VoxelBooleanActionMasks actions = childBooleanActionMasks(type, leftStates, rightStates);

    bool changed = false;

    if (actions.setEmpty != EmptyVoxelNodeMask)
    {
        statistics.clearChildren(actions.setEmpty);
        changed = leftEditor.setChildrenState(actions.setEmpty, VoxelState::Empty) || changed;
    }

    if (actions.setMaterial != EmptyVoxelNodeMask)
    {
        statistics.fillChildren(actions.setMaterial);
        changed = leftEditor.setChildrenState(actions.setMaterial, VoxelState::Material) || changed;
    }

    std::uint8_t copyMask = actions.copyRight;

    while (copyMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(copyMask);
        VoxelTreeEditor leftChildEditor = leftEditor.child(corner);
        const VoxelTreeCursor rightChildCursor = rightCursor.child(corner);

        changed = copySubtree(leftChildEditor, rightChildCursor, false, statistics) || changed;
    }

    std::uint8_t invertLeftMask = actions.invertLeft;

    while (invertLeftMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(invertLeftMask);
        const VoxelTreeCursor leftChildCursor = leftCursor.child(corner);
        VoxelTreeEditor leftChildEditor = leftEditor.child(corner);

        changed = invertSubtree(leftChildEditor, leftChildCursor, statistics) || changed;
    }

    std::uint8_t copyInvertedMask = actions.copyInvertedRight;

    while (copyInvertedMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(copyInvertedMask);
        VoxelTreeEditor leftChildEditor = leftEditor.child(corner);
        const VoxelTreeCursor rightChildCursor = rightCursor.child(corner);

        changed = copySubtree(leftChildEditor, rightChildCursor, true, statistics) || changed;
    }

    std::uint8_t recursiveMask = actions.recurse;

    statistics.recurseChildren(recursiveMask);

    while (recursiveMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(recursiveMask);
        const VoxelTreeCursor leftChildCursor = leftCursor.child(corner);
        const VoxelTreeCursor rightChildCursor = rightCursor.child(corner);
        VoxelTreeEditor leftChildEditor = leftEditor.child(corner);

        changed = applySubtree(leftChildEditor, leftChildCursor, rightChildCursor, type, statistics) || changed;
    }

    collapseVoxel(leftEditor, statistics);
    return changed;
}

// 对两个任意逻辑体素执行布尔运算。
bool applySubtree(VoxelTreeEditor& leftEditor, const VoxelTreeCursor& leftCursor,
                  const VoxelTreeCursor& rightCursor, VoxelBooleanType type,
                  StatisticsRecorder& statistics)
{
    statistics.visitVoxel();

    const VoxelBooleanAction action = booleanAction(type, leftCursor.state(), rightCursor.state());

    switch (action)
    {
    case VoxelBooleanAction::KeepLeft:
        return false;

    case VoxelBooleanAction::SetEmpty:
        leftEditor.setEmpty();
        return true;

    case VoxelBooleanAction::SetMaterial:
        leftEditor.setMaterial();
        return true;

    case VoxelBooleanAction::CopyRight:
        return copySubtree(leftEditor, rightCursor, false, statistics);

    case VoxelBooleanAction::InvertLeft:
        return invertSubtree(leftEditor, leftCursor, statistics);

    case VoxelBooleanAction::CopyInvertedRight:
        return copySubtree(leftEditor, rightCursor, true, statistics);

    case VoxelBooleanAction::Recurse:
        return applySubdivided(leftEditor, leftCursor, rightCursor, type, statistics);
    }

    assert(false);
    return false;
}

// 处理left和right引用同一棵树的确定结果。
bool applyAliasedTree(VoxelTree& tree, VoxelBooleanType type)
{
    switch (type)
    {
    case VoxelBooleanType::Union:
    case VoxelBooleanType::Intersection:
        return false;

    case VoxelBooleanType::Difference:
    case VoxelBooleanType::ExclusiveOr:
        if (tree.state() == VoxelState::Empty)
        {
            return false;
        }

        tree.editor().setEmpty();
        return true;
    }

    assert(false);
    return false;
}

bool applyRoot(VoxelTree& left, const VoxelTree& right, VoxelBooleanType type, StatisticsRecorder& statistics)
{
    assert(left.isValid());
    assert(right.isValid());

    if (&left == &right)
    {
        return applyAliasedTree(left, type);
    }

    const VoxelBooleanAction action = booleanAction(type, left.state(), right.state());

    switch (action)
    {
    case VoxelBooleanAction::KeepLeft:
        return false;

    case VoxelBooleanAction::SetEmpty:
        left.editor().setEmpty();
        return true;

    case VoxelBooleanAction::SetMaterial:
        left.editor().setMaterial();
        return true;

    case VoxelBooleanAction::CopyRight:
        left = right;
        return true;

    case VoxelBooleanAction::InvertLeft:
    {
        VoxelTreeEditor leftEditor = left.editor();
        const VoxelTreeCursor leftCursor = left.cursor();
        return invertSubtree(leftEditor, leftCursor, statistics);
    }

    case VoxelBooleanAction::CopyInvertedRight:
    {
        left = right;

        VoxelTreeEditor leftEditor = left.editor();
        const VoxelTreeCursor leftCursor = left.cursor();

        return invertSubtree(leftEditor, leftCursor, statistics);
    }

    case VoxelBooleanAction::Recurse:
    {
        VoxelTreeEditor leftEditor = left.editor();
        const VoxelTreeCursor leftCursor = left.cursor();
        const VoxelTreeCursor rightCursor = right.cursor();

        return applySubdivided(leftEditor, leftCursor, rightCursor, type, statistics);
    }
    }

    assert(false);
    return false;
}

}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

bool VoxelTreeBooleanAlgorithm::apply(VoxelTree& left, const VoxelTree& right,
                                      VoxelBooleanType type, VoxelTreeBooleanStatistics& statistics)
{
    statistics = VoxelTreeBooleanStatistics();

    StatisticsRecorder recorder(statistics);
    return applyRoot(left, right, type, recorder);
}

#else

bool VoxelTreeBooleanAlgorithm::apply(VoxelTree& left, const VoxelTree& right, VoxelBooleanType type)
{
    StatisticsRecorder recorder;
    return applyRoot(left, right, type, recorder);
}

#endif

}
}
}