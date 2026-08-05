#include "VoxelForestBooleanAlgorithm.h"

#include <cassert>
#include <utility>
#include <vector>

#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/ParallelOptions.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

namespace
{

const std::size_t MinimumParallelPairedRootCount = 4; // 暂定四个配对根开始并行，后续通过离散布尔运算基准重新标定。

class DisabledForestStatisticsRecorder
{
public:
    void candidateRoot() {}
    void pairedRoot() {}
    void copiedRoot() {}
    void erasedRoot() {}
    void modifiedRoot() {}
    void skippedRoot() {}
    void accumulateTree(const VoxelTreeBooleanStatistics&) {}
};

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 将一次树级统计累加到森林级统计。
void accumulateTreeStatistics(VoxelTreeBooleanStatistics& target, const VoxelTreeBooleanStatistics& source)
{
    target.visitedVoxelCount += source.visitedVoxelCount;
    target.leafMaskOperationCount += source.leafMaskOperationCount;
    target.directlyClearedChildCount += source.directlyClearedChildCount;
    target.directlyFilledChildCount += source.directlyFilledChildCount;
    target.copiedSubtreeCount += source.copiedSubtreeCount;
    target.invertedSubtreeCount += source.invertedSubtreeCount;
    target.recursiveChildCount += source.recursiveChildCount;
    target.collapsedVoxelCount += source.collapsedVoxelCount;
}

class EnabledForestStatisticsRecorder
{
public:
    explicit EnabledForestStatisticsRecorder(VoxelForestBooleanStatistics& statistics)
        : m_statistics(statistics)
    {
    }

    void candidateRoot()
    {
        ++m_statistics.candidateRootCount;
    }

    void pairedRoot()
    {
        ++m_statistics.pairedRootCount;
    }

    void copiedRoot()
    {
        ++m_statistics.copiedRootCount;
    }

    void erasedRoot()
    {
        ++m_statistics.erasedRootCount;
    }

    void modifiedRoot()
    {
        ++m_statistics.modifiedRootCount;
    }

    void skippedRoot()
    {
        ++m_statistics.skippedRootCount;
    }

    void accumulateTree(const VoxelTreeBooleanStatistics& statistics)
    {
        accumulateTreeStatistics(m_statistics.treeStatistics, statistics);
    }

private:
    VoxelForestBooleanStatistics& m_statistics;
};

using ForestStatisticsRecorder = EnabledForestStatisticsRecorder;

#else

using ForestStatisticsRecorder = DisabledForestStatisticsRecorder;

#endif

// 保存需要从右森林直接复制到左森林的根。
struct CopiedRootItem
{
    CopiedRootItem(const VoxelCellIndex& rootIndexValue, const VoxelTree* rightTreeValue)
        : rootIndex(rootIndexValue)
        , rightTree(rightTreeValue)
    {
        assert(rightTreeValue);
    }

    VoxelCellIndex rootIndex; // 当前需要复制的第0层根索引。
    const VoxelTree* rightTree; // 当前需要复制的右侧只读根树。
};

// 保存一个左右配对根的独立计算状态。
struct PairedRootItem
{
    PairedRootItem(const VoxelCellIndex& rootIndexValue, const VoxelTree& leftTree, const VoxelTree* rightTreeValue)
        : rootIndex(rootIndexValue)
        , resultTree(leftTree)
        , rightTree(rightTreeValue)
        , changed(false)
        , removed(false)
    {
        assert(rightTreeValue);
    }

    VoxelCellIndex rootIndex; // 当前左右配对根的第0层索引。
    VoxelTree resultTree; // 当前根从左树复制得到的独立计算结果。
    const VoxelTree* rightTree; // 当前根对应的右侧只读树。
    bool changed; // 当前根是否发生实际变化。
    bool removed; // 当前根运算结果是否为空并应从左森林删除。

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    VoxelTreeBooleanStatistics treeStatistics; // 当前配对根独立产生的树级统计。
#endif
};

// 保存一次森林布尔运算的串行直接操作和并行根任务。
struct ForestBooleanPlan
{
    std::vector<CopiedRootItem> copiedRoots; // 需要从右森林直接复制的根。
    std::vector<VoxelCellIndex> erasedRoots; // 需要从左森林直接删除的根。
    std::vector<PairedRootItem> pairedRoots; // 需要执行树级布尔运算的左右配对根。
};

// 收集森林当前全部根索引，允许后续安全清空森林。
void collectRootIndices(const VoxelForest& forest, std::vector<VoxelCellIndex>& rootIndices)
{
    rootIndices.clear();
    rootIndices.reserve(forest.rootCount());

    forest.forEachRootCell(
        [&rootIndices](const VoxelCellAddress& rootAddress)
        {
            rootIndices.push_back(rootAddress.index);
        });
}

// 记录一个发生实际变化的根。
void recordChangedRoot(ForestStatisticsRecorder& statistics)
{
    statistics.modifiedRoot();
}

// 处理left和right引用同一森林的确定结果。
bool applyAliasedForest(VoxelShapeSession& left, VoxelBooleanType type,
                        ForestStatisticsRecorder& statistics)
{
    switch (type)
    {
    case VoxelBooleanType::Union:
    case VoxelBooleanType::Intersection:
        return false;

    case VoxelBooleanType::Difference:
    case VoxelBooleanType::ExclusiveOr:
        break;
    }

    if (left.isEmpty())
    {
        return false;
    }

    std::vector<VoxelCellIndex> rootIndices;
    collectRootIndices(left.forest(), rootIndices);

    for (std::size_t rootPosition = 0; rootPosition < rootIndices.size(); ++rootPosition)
    {
        const VoxelCellIndex& rootIndex = rootIndices[rootPosition];

        statistics.candidateRoot();
        statistics.erasedRoot();
        recordChangedRoot(statistics);
    }

    left.clear();
    return true;
}

// 准备森林并集计划。
void prepareUnionPlan(const VoxelForest& left, const VoxelForest& right,
                      ForestBooleanPlan& plan, ForestStatisticsRecorder& statistics)
{
    plan.copiedRoots.reserve(right.rootCount());
    plan.pairedRoots.reserve(right.rootCount());

    right.forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelCellIndex& rootIndex = rootAddress.index;
            const VoxelTree* rightTree = right.getTree(rootIndex);

            assert(rightTree);
            assert(rightTree->state() != VoxelState::Empty);

            statistics.candidateRoot();

            const VoxelTree* leftTree = left.getTree(rootIndex);

            if (!leftTree)
            {
                plan.copiedRoots.push_back(CopiedRootItem(rootIndex, rightTree));
                return;
            }

            statistics.pairedRoot();
            plan.pairedRoots.push_back(PairedRootItem(rootIndex, *leftTree, rightTree));
        });
}

// 准备森林交集计划。
void prepareIntersectionPlan(const VoxelForest& left, const VoxelForest& right,
                             ForestBooleanPlan& plan, ForestStatisticsRecorder& statistics)
{
    plan.erasedRoots.reserve(left.rootCount());
    plan.pairedRoots.reserve(left.rootCount());

    left.forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelCellIndex& rootIndex = rootAddress.index;
            const VoxelTree* leftTree = left.getTree(rootIndex);

            assert(leftTree);
            assert(leftTree->state() != VoxelState::Empty);

            statistics.candidateRoot();

            const VoxelTree* rightTree = right.getTree(rootIndex);

            if (!rightTree)
            {
                plan.erasedRoots.push_back(rootIndex);
                return;
            }

            statistics.pairedRoot();
            plan.pairedRoots.push_back(PairedRootItem(rootIndex, *leftTree, rightTree));
        });
}

// 准备森林差集计划。
void prepareDifferencePlan(const VoxelForest& left, const VoxelForest& right,
                           ForestBooleanPlan& plan, ForestStatisticsRecorder& statistics)
{
    plan.pairedRoots.reserve((std::min)(left.rootCount(), right.rootCount()));

    right.forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelCellIndex& rootIndex = rootAddress.index;
            const VoxelTree* rightTree = right.getTree(rootIndex);

            assert(rightTree);
            assert(rightTree->state() != VoxelState::Empty);

            statistics.candidateRoot();

            const VoxelTree* leftTree = left.getTree(rootIndex);

            if (!leftTree)
            {
                statistics.skippedRoot();
                return;
            }

            statistics.pairedRoot();
            plan.pairedRoots.push_back(PairedRootItem(rootIndex, *leftTree, rightTree));
        });
}

// 准备森林异或计划。
void prepareExclusiveOrPlan(const VoxelForest& left, const VoxelForest& right,
                            ForestBooleanPlan& plan, ForestStatisticsRecorder& statistics)
{
    plan.copiedRoots.reserve(right.rootCount());
    plan.pairedRoots.reserve(right.rootCount());

    right.forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelCellIndex& rootIndex = rootAddress.index;
            const VoxelTree* rightTree = right.getTree(rootIndex);

            assert(rightTree);
            assert(rightTree->state() != VoxelState::Empty);

            statistics.candidateRoot();

            const VoxelTree* leftTree = left.getTree(rootIndex);

            if (!leftTree)
            {
                plan.copiedRoots.push_back(CopiedRootItem(rootIndex, rightTree));
                return;
            }

            statistics.pairedRoot();
            plan.pairedRoots.push_back(PairedRootItem(rootIndex, *leftTree, rightTree));
        });
}

// 根据布尔运算类型建立完整根级执行计划。
void prepareForestBooleanPlan(const VoxelForest& left, const VoxelForest& right,
                              VoxelBooleanType type, ForestBooleanPlan& plan,
                              ForestStatisticsRecorder& statistics)
{
    switch (type)
    {
    case VoxelBooleanType::Union:
        prepareUnionPlan(left, right, plan, statistics);
        return;

    case VoxelBooleanType::Intersection:
        prepareIntersectionPlan(left, right, plan, statistics);
        return;

    case VoxelBooleanType::Difference:
        prepareDifferencePlan(left, right, plan, statistics);
        return;

    case VoxelBooleanType::ExclusiveOr:
        prepareExclusiveOrPlan(left, right, plan, statistics);
        return;
    }

    assert(false);
}

// 在独立左根副本上执行一次树级布尔运算。
void processPairedRoot(PairedRootItem& item, VoxelBooleanType type)
{
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    item.changed = VoxelTreeBooleanAlgorithm::apply(
        item.resultTree,
        *item.rightTree,
        type,
        item.treeStatistics);

#else

    item.changed = VoxelTreeBooleanAlgorithm::apply(
        item.resultTree,
        *item.rightTree,
        type);

#endif

    item.removed = item.changed && item.resultTree.state() == VoxelState::Empty;

    assert(item.resultTree.isValid());
}

// 并行处理全部左右配对根。
void executePairedRoots(ForestBooleanPlan& plan, VoxelBooleanType type)
{
    if (plan.pairedRoots.empty())
    {
        return;
    }

    Foundation::ParallelOptions options;
    options.minimumParallelTaskCount = MinimumParallelPairedRootCount;

    Foundation::ParallelExecutor::global().execute(
        0,
        plan.pairedRoots.size(),
        [&plan, type](std::size_t blockBegin, std::size_t blockEnd)
        {
            for (std::size_t itemIndex = blockBegin; itemIndex < blockEnd; ++itemIndex)
            {
                processPairedRoot(plan.pairedRoots[itemIndex], type);
            }
        },
        options);
}

// 串行提交从右森林直接复制的根。
bool commitCopiedRoots(VoxelShapeSession& left, const std::vector<CopiedRootItem>& items,
                       ForestStatisticsRecorder& statistics)
{
    bool changed = false;

    for (std::size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
    {
        const CopiedRootItem& item = items[itemIndex];

        assert(item.rightTree);
        assert(!left.tree(item.rootIndex));

        left.setTree(item.rootIndex, *item.rightTree);

        statistics.copiedRoot();
        recordChangedRoot(statistics);
        changed = true;
    }

    return changed;
}

// 串行提交需要直接删除的左侧根。
bool commitErasedRoots(VoxelShapeSession& left, const std::vector<VoxelCellIndex>& rootIndices,
                       ForestStatisticsRecorder& statistics)
{
    bool changed = false;

    for (std::size_t rootPosition = 0; rootPosition < rootIndices.size(); ++rootPosition)
    {
        const VoxelCellIndex& rootIndex = rootIndices[rootPosition];
        const bool erased = left.eraseTree(rootIndex);

        assert(erased);

        statistics.erasedRoot();
        recordChangedRoot(statistics);
        changed = true;
    }

    return changed;
}

// 串行提交全部配对根计算结果。
bool commitPairedRoots(VoxelShapeSession& left, std::vector<PairedRootItem>& items,
                       ForestStatisticsRecorder& statistics)
{
    bool changed = false;

    for (std::size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
    {
        PairedRootItem& item = items[itemIndex];

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
        statistics.accumulateTree(item.treeStatistics);
#endif

        if (!item.changed)
        {
            statistics.skippedRoot();
            continue;
        }

        if (item.removed)
        {
            const bool erased = left.eraseTree(item.rootIndex);

            assert(erased);
            statistics.erasedRoot();
        }
        else
        {
            assert(item.resultTree.isValid());
            left.setTree(item.rootIndex, std::move(item.resultTree));
        }

        recordChangedRoot(statistics);
        changed = true;
    }

    return changed;
}

// 串行提交完整森林布尔运算计划。
bool commitForestBooleanPlan(VoxelShapeSession& left, ForestBooleanPlan& plan,
                             ForestStatisticsRecorder& statistics)
{
    bool changed = false;

    changed = commitCopiedRoots(left, plan.copiedRoots, statistics) || changed;
    changed = commitErasedRoots(left, plan.erasedRoots, statistics) || changed;
    changed = commitPairedRoots(left, plan.pairedRoots, statistics) || changed;

    return changed;
}

// 执行事务式森林布尔运算。
bool applyForest(VoxelShapeSession& left, const VoxelForest& right, VoxelBooleanType type,
                 ForestStatisticsRecorder& statistics)
{
    if (&left.forest() == &right)
    {
        return applyAliasedForest(left, type, statistics);
    }

    ForestBooleanPlan plan;

    prepareForestBooleanPlan(left.forest(), right, type, plan, statistics);
    executePairedRoots(plan, type);

    // 任意并行任务抛出异常时不会进入提交阶段，left保持原始内容不变。
    return commitForestBooleanPlan(left, plan, statistics);
}

}

void VoxelForestBooleanStatistics::reset()
{
    candidateRootCount = 0;
    pairedRootCount = 0;
    copiedRootCount = 0;
    erasedRootCount = 0;
    modifiedRootCount = 0;
    skippedRootCount = 0;
    treeStatistics = VoxelTreeBooleanStatistics();
}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

bool VoxelForestBooleanAlgorithm::apply(VoxelShapeSession& left, const VoxelForest& right,
                                        VoxelBooleanType type,
                                        VoxelForestBooleanStatistics& statistics)
{
    statistics.reset();

    ForestStatisticsRecorder recorder(statistics);
    return applyForest(left, right, type, recorder);
}

#else

bool VoxelForestBooleanAlgorithm::apply(VoxelShapeSession& left, const VoxelForest& right,
                                        VoxelBooleanType type)
{
    ForestStatisticsRecorder recorder;
    return applyForest(left, right, type, recorder);
}

#endif

}
}
}