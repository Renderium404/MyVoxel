#include "ShapeVoxelization.h"
#include "VoxelizationWorkspace.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include <array>
#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Mask/VoxelChildMask.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/ParallelOptions.h"
#include "MyVoxel/Geometry/ShapeQuery.h"
#include "MyVoxel/Geometry/ShapeRelation.h"

namespace MyVoxel
{
namespace Modeling
{

// 仅向ShapeVoxelization.cpp开放工作区内部资源管理入口。
struct VoxelizationWorkspaceAccess
{
    static void prepare(VoxelizationWorkspace& workspace, const VoxelShape& reference)
    {
        workspace.prepare(reference);
    }

    static VoxelTree acquireRootTree(VoxelizationWorkspace& workspace, const VoxelCellIndex& rootIndex)
    {
        return workspace.acquireRootTree(rootIndex);
    }

    static void recycleRootTree(VoxelizationWorkspace& workspace, const VoxelCellIndex& rootIndex, VoxelTree&& tree)
    {
        workspace.recycleRootTree(rootIndex, std::move(tree));
    }

    static VoxelShape& result(VoxelizationWorkspace& workspace)
    {
        return workspace.m_result;
    }
};

}
}

namespace
{

const double CellCenterScale = 0.5; // 包围盒二分和中心计算使用的固定比例。
const unsigned int XChildMask = 1; // VoxelCorner第0位控制子包围盒的X方向。
const unsigned int YChildMask = 2; // VoxelCorner第1位控制子包围盒的Y方向。
const unsigned int ZChildMask = 4; // VoxelCorner第2位控制子包围盒的Z方向。
const unsigned int MaskLeafCoveredLevelCount = 2; // 一个64位掩码叶块固定覆盖当前体素下面两级。
const std::size_t MinimumParallelIntersectingRootCount = 4; // 四个及以上边界相交根进入并行体素化。
static_assert(
    static_cast<unsigned int>(MyVoxel::Geometry::ShapeQuery::OctantCount) ==
    static_cast<unsigned int>(MyVoxel::VoxelCornerCount),
    "ShapeQuery octant order must match VoxelCorner order.");
// 保存体素在查询空间中的中心和半尺寸，递归过程不再构造Bounds3。
struct CellBounds
{
    CellBounds(const MyMath::Vector3& centerValue, const MyMath::Vector3& extentValue)
        : center(centerValue)
        , extent(extentValue)
    {
        MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "CellBounds center must be finite.");
        MYVOXEL_ASSERT_MESSAGE(extent.isFinite() &&
                               extent.x() >= 0.0 &&
                               extent.y() >= 0.0 &&
                               extent.z() >= 0.0,
                               "CellBounds extent must be finite and non-negative.");
    }

    MyMath::Vector3 center; // 当前体素中心。
    MyMath::Vector3 extent; // 当前体素三个方向的半尺寸。
};
// 正式路径使用的空统计记录器。
class DisabledVoxelizationRecorder
{
public:
    DisabledVoxelizationRecorder() = default;

    explicit DisabledVoxelizationRecorder(MyVoxel::Modeling::VoxelizationStatistics&)
    {
    }

    void setRootCandidateCount(std::uint64_t)
    {
    }

    void createdRoot()
    {
    }

    void gridCellBounds()
    {
    }

    void derivedChildBounds()
    {
    }

    void relation(MyVoxel::Geometry::ShapeRelation)
    {
    }

    void centerSample()
    {
    }

    void split()
    {
    }

    void maskLeafBuild()
    {
    }

    void maskLeafStored()
    {
    }

    void mergeSuccess()
    {
    }

    void accumulate(const MyVoxel::Modeling::VoxelizationStatistics&)
    {
    }
};

// 带统计路径使用的体素化记录器。
class EnabledVoxelizationRecorder
{
public:
    explicit EnabledVoxelizationRecorder(MyVoxel::Modeling::VoxelizationStatistics& statistics)
        : m_statistics(statistics)
    {
    }

    void setRootCandidateCount(std::uint64_t count)
    {
        m_statistics.rootCandidateCount = count;
    }

    void createdRoot()
    {
        ++m_statistics.createdRootCount;
    }

    void gridCellBounds()
    {
        ++m_statistics.gridCellBoundsCount;
    }

    void derivedChildBounds()
    {
        ++m_statistics.derivedChildBoundsCount;
    }

    void relation(MyVoxel::Geometry::ShapeRelation relation)
    {
        ++m_statistics.visitedCellCount;

        switch (relation)
        {
        case MyVoxel::Geometry::ShapeRelation::Outside:
            ++m_statistics.outsideCellCount;
            return;

        case MyVoxel::Geometry::ShapeRelation::Inside:
            ++m_statistics.insideCellCount;
            return;

        case MyVoxel::Geometry::ShapeRelation::Intersecting:
            ++m_statistics.intersectingCellCount;
            return;
        }

        MYVOXEL_ASSERT_MESSAGE(false, "Voxelization encountered an unknown ShapeRelation.");
    }

    void centerSample()
    {
        ++m_statistics.centerSampleCount;
    }

    void split()
    {
        ++m_statistics.splitCount;
    }

    void maskLeafBuild()
    {
        ++m_statistics.maskLeafBuildCount;
    }

    void maskLeafStored()
    {
        ++m_statistics.maskLeafStoredCount;
    }

    void mergeSuccess()
    {
        ++m_statistics.mergeSuccessCount;
    }

    void accumulate(const MyVoxel::Modeling::VoxelizationStatistics& statistics)
    {
        m_statistics.accumulate(statistics);
    }

private:
    MyVoxel::Modeling::VoxelizationStatistics& m_statistics;
};

// 保存一个需要递归体素化的边界相交根。
struct RootVoxelizationItem
{
    RootVoxelizationItem(const MyVoxel::VoxelCellIndex& rootIndexValue, const CellBounds& rootBoundsValue)
        : rootIndex(rootIndexValue)
        , rootBounds(rootBoundsValue)
        , resultTree(MyVoxel::VoxelState::Empty)
        , resultState(MyVoxel::VoxelState::Empty)
    {
    }

    RootVoxelizationItem(const MyVoxel::VoxelCellIndex& rootIndexValue,
                         const CellBounds& rootBoundsValue,
                         MyVoxel::VoxelTree&& reusableTree)
        : rootIndex(rootIndexValue)
        , rootBounds(rootBoundsValue)
        , resultTree(std::move(reusableTree))
        , resultState(MyVoxel::VoxelState::Empty)
    {
        MYVOXEL_ASSERT_MESSAGE(resultTree.isValid(), "Reusable voxelization root tree must be valid.");
        MYVOXEL_ASSERT_MESSAGE(resultTree.state() == MyVoxel::VoxelState::Empty, "Reusable voxelization root tree must be empty.");
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前第0层根索引。
    CellBounds rootBounds; // 当前第0层根在查询空间中的中心和半尺寸。
    MyVoxel::VoxelTree resultTree; // 当前根独立生成的体素树。
    MyVoxel::VoxelState resultState; // 当前根最终状态。
    MyVoxel::Modeling::VoxelizationStatistics statistics; // 当前根独立产生的统计。
};

// 执行无符号64位饱和乘法，溢出时返回最大值。
std::uint64_t saturatedMultiply(std::uint64_t first, std::uint64_t second)
{
    if (first == 0 || second == 0)
    {
        return 0;
    }

    const std::uint64_t maximumValue = (std::numeric_limits<std::uint64_t>::max)();

    if (first > maximumValue / second)
    {
        return maximumValue;
    }

    return first * second;
}

// 将普通轴对齐包围盒转换为中心和半尺寸表示。
CellBounds makeCellBounds(const MyVoxel::Bounds3& bounds)
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Cannot create CellBounds from invalid Bounds3.");

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    return CellBounds(
        MyMath::Vector3(
            (minimum.x() + maximum.x()) * CellCenterScale,
            (minimum.y() + maximum.y()) * CellCenterScale,
            (minimum.z() + maximum.z()) * CellCenterScale),
        MyMath::Vector3(
            (maximum.x() - minimum.x()) * CellCenterScale,
            (maximum.y() - minimum.y()) * CellCenterScale,
            (maximum.z() - minimum.z()) * CellCenterScale));
}

// 根据父中心、子半尺寸和角点方向生成子体素。
CellBounds makeChildBounds(const MyMath::Vector3& parentCenter,
                           const MyMath::Vector3& childExtent,
                           MyVoxel::VoxelCorner corner)
{
    const unsigned int cornerValue = static_cast<unsigned int>(corner);

    MYVOXEL_ASSERT_MESSAGE(
        cornerValue < static_cast<unsigned int>(MyVoxel::VoxelCornerCount),
        "Voxel child corner must be in range [0, 7].");

    const MyMath::Vector3 childCenter(
        parentCenter.x() + ((cornerValue & XChildMask) != 0 ? childExtent.x() : -childExtent.x()),
        parentCenter.y() + ((cornerValue & YChildMask) != 0 ? childExtent.y() : -childExtent.y()),
        parentCenter.z() + ((cornerValue & ZChildMask) != 0 ? childExtent.z() : -childExtent.z()));

    return CellBounds(childCenter, childExtent);
}

// 批量分类当前体素的八个等尺寸子体素，并返回子体素半尺寸。
template<typename Recorder>
MyMath::Vector3 classifyChildBoundsFast(
    const MyVoxel::Geometry::ShapeQuery& query,
    const CellBounds& parentBounds,
    std::array<MyVoxel::Geometry::ShapeRelation, MyVoxel::Geometry::ShapeQuery::OctantCount>& relations,
    Recorder& recorder)
{
    const MyMath::Vector3 childExtent = parentBounds.extent * CellCenterScale;

    query.classifyOctantBoundsFast(
        parentBounds.center,
        childExtent,
        relations);

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        recorder.derivedChildBounds();
    }

    return childExtent;
}

// 判断当前逻辑体素是否可以直接使用覆盖下面两级的MaskLeaf。
bool shouldBuildMaskLeaf(MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel)
{
    if (level == MyVoxel::BaseVoxelLevel || level > maximumLevel)
    {
        return false;
    }

    const unsigned int currentLevel = static_cast<unsigned int>(level);
    const unsigned int finalLevel = static_cast<unsigned int>(maximumLevel);

    return finalLevel - currentLevel == MaskLeafCoveredLevelCount;
}

// 对最高层相交体素执行中心采样。
template<typename Recorder>
bool sampleCellCenter(const MyVoxel::Geometry::ShapeQuery& query,
                      const CellBounds& bounds,
                      Recorder& recorder)
{
    recorder.centerSample();
    return query.containsPoint(bounds.center);
}

// 返回一个最高层体素的材料状态。
template<typename Recorder>
bool maximumLevelCellMaterial(const MyVoxel::Geometry::ShapeQuery& query,
                              const CellBounds& bounds,
                              MyVoxel::Geometry::ShapeRelation relation,
                              Recorder& recorder)
{
    switch (relation)
    {
    case MyVoxel::Geometry::ShapeRelation::Outside:
        return false;

    case MyVoxel::Geometry::ShapeRelation::Inside:
        return true;

    case MyVoxel::Geometry::ShapeRelation::Intersecting:
        return sampleCellCenter(query, bounds, recorder);
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Voxelization encountered an unknown ShapeRelation.");
    return false;
}


// 直接计算当前体素下面两级对应的64位材料掩码。
template<typename Recorder>
std::uint64_t buildMaskLeafMaterialMask(const MyVoxel::Geometry::ShapeQuery& query,
                                        const CellBounds& cellBounds,
                                        MyVoxel::VoxelLevel level,
                                        MyVoxel::VoxelLevel maximumLevel,
                                        Recorder& recorder)
{
    MYVOXEL_ASSERT_MESSAGE(
        shouldBuildMaskLeaf(level, maximumLevel),
        "MaskLeaf build level must cover exactly two remaining levels.");

    recorder.maskLeafBuild();

    std::uint64_t materialMask = MyVoxel::EmptyVoxelLeafMask;

    std::array<
        MyVoxel::Geometry::ShapeRelation,
        MyVoxel::Geometry::ShapeQuery::OctantCount> coarseRelations;

    const MyMath::Vector3 coarseExtent =
        classifyChildBoundsFast(
            query,
            cellBounds,
            coarseRelations,
            recorder);

    for (unsigned int coarseIndex = 0; coarseIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++coarseIndex)
    {
        const MyVoxel::VoxelCorner coarseCorner = static_cast<MyVoxel::VoxelCorner>(coarseIndex);
        const MyVoxel::Geometry::ShapeRelation coarseRelation = coarseRelations[coarseIndex];

        recorder.relation(coarseRelation);

        if (coarseRelation == MyVoxel::Geometry::ShapeRelation::Outside)
        {
            continue;
        }

        if (coarseRelation == MyVoxel::Geometry::ShapeRelation::Inside)
        {
            materialMask |= MyVoxel::leafGroupMask(coarseCorner);
            continue;
        }

        const CellBounds coarseBounds =
            makeChildBounds(
                cellBounds.center,
                coarseExtent,
                coarseCorner);

        std::array<
            MyVoxel::Geometry::ShapeRelation,
            MyVoxel::Geometry::ShapeQuery::OctantCount> fineRelations;

        const MyMath::Vector3 fineExtent =
            classifyChildBoundsFast(
                query,
                coarseBounds,
                fineRelations,
                recorder);

        std::uint8_t groupBits = 0;

        for (unsigned int fineIndex = 0; fineIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++fineIndex)
        {
            const MyVoxel::VoxelCorner fineCorner = static_cast<MyVoxel::VoxelCorner>(fineIndex);
            const MyVoxel::Geometry::ShapeRelation fineRelation = fineRelations[fineIndex];

            recorder.relation(fineRelation);

            if (fineRelation == MyVoxel::Geometry::ShapeRelation::Outside)
            {
                continue;
            }

            if (fineRelation == MyVoxel::Geometry::ShapeRelation::Inside)
            {
                groupBits = static_cast<std::uint8_t>(groupBits | MyVoxel::nodeCornerMask(fineCorner));
                continue;
            }

            const CellBounds fineBounds =
                makeChildBounds(
                    coarseBounds.center,
                    fineExtent,
                    fineCorner);

            if (sampleCellCenter(query, fineBounds, recorder))
            {
                groupBits = static_cast<std::uint8_t>(groupBits | MyVoxel::nodeCornerMask(fineCorner));
            }
        }

        materialMask |= static_cast<std::uint64_t>(groupBits) << MyVoxel::leafMaskOffset(coarseCorner);
    }

    return materialMask;
}



// 尝试将当前已细分体素折叠为空或材料。
template<typename Recorder>
MyVoxel::VoxelState collapseVoxel(MyVoxel::VoxelTreeEditor& editor, Recorder& recorder)
{
    const MyVoxel::VoxelChildStateMasks states = editor.childStateMasks();

    if (states.empty == MyVoxel::FullVoxelNodeMask)
    {
        editor.setEmpty();
        recorder.mergeSuccess();
        return MyVoxel::VoxelState::Empty;
    }

    if (states.material == MyVoxel::FullVoxelNodeMask)
    {
        editor.setMaterial();
        recorder.mergeSuccess();
        return MyVoxel::VoxelState::Material;
    }

    return MyVoxel::VoxelState::Subdivided;
}

// 按已经获得的几何关系和中心半尺寸递归生成当前逻辑体素。
template<typename Recorder>
MyVoxel::VoxelState voxelizeCell(MyVoxel::VoxelTreeEditor editor,
                                 const MyVoxel::Geometry::ShapeQuery& query,
                                 MyVoxel::VoxelLevel level,
                                 MyVoxel::VoxelLevel maximumLevel,
                                 const CellBounds& cellBounds,
                                 MyVoxel::Geometry::ShapeRelation relation,
                                 Recorder& recorder)
{
    recorder.relation(relation);

    if (relation == MyVoxel::Geometry::ShapeRelation::Outside)
    {
        editor.setEmpty();
        return MyVoxel::VoxelState::Empty;
    }

    if (relation == MyVoxel::Geometry::ShapeRelation::Inside)
    {
        editor.setMaterial();
        return MyVoxel::VoxelState::Material;
    }

    MYVOXEL_ASSERT_MESSAGE(relation == MyVoxel::Geometry::ShapeRelation::Intersecting,
                           "Voxelization requires a valid ShapeRelation.");

    if (level == maximumLevel)
    {
        if (sampleCellCenter(query, cellBounds, recorder))
        {
            editor.setMaterial();
            return MyVoxel::VoxelState::Material;
        }

        editor.setEmpty();
        return MyVoxel::VoxelState::Empty;
    }

    MYVOXEL_ASSERT_MESSAGE(level < maximumLevel, "Voxelization level exceeds the VoxelGrid maximum level.");

    if (shouldBuildMaskLeaf(level, maximumLevel))
    {
        const std::uint64_t materialMask =
            buildMaskLeafMaterialMask(
                query,
                cellBounds,
                level,
                maximumLevel,
                recorder);

        const MyVoxel::VoxelState resultState = MyVoxel::leafMaskState(materialMask);

        MYVOXEL_ASSERT_MESSAGE(editor.canSetMaterialMask(),"MaskLeaf fast path requires a normal node-child editor.");

        editor.setMaterialMask(materialMask);

        if (resultState == MyVoxel::VoxelState::Subdivided)
        {
            recorder.maskLeafStored();
        }
        else
        {
            recorder.mergeSuccess();
        }

        return resultState;
    }

    if (!editor.isSubdivided())
    {
        editor.subdivide();
        recorder.split();
    }

    const MyVoxel::VoxelLevel childLevel = static_cast<MyVoxel::VoxelLevel>(level + 1);

    std::array<
        MyVoxel::Geometry::ShapeRelation,
        MyVoxel::Geometry::ShapeQuery::OctantCount> childRelations;

    const MyMath::Vector3 childExtent =
        classifyChildBoundsFast(
            query,
            cellBounds,
            childRelations,
            recorder);

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);

        const CellBounds childBounds =
            makeChildBounds(
                cellBounds.center,
                childExtent,
                corner);

        voxelizeCell(
            editor.child(corner),
            query,
            childLevel,
            maximumLevel,
            childBounds,
            childRelations[cornerIndex],
            recorder);
    }

    return collapseVoxel(editor, recorder);

}






// 根据几何包围盒收集完整材料根和边界相交根。
template<typename Recorder>
void collectRootVoxelizationItems(const MyVoxel::Geometry::ShapeQuery& query,
                                  const MyVoxel::VoxelGrid& grid,
                                  const MyVoxel::VoxelCellRange& rootRange,
                                  std::vector<MyVoxel::VoxelCellIndex>& materialRootIndices,
                                  std::vector<RootVoxelizationItem>& intersectingItems,
                                  Recorder& recorder,
                                  MyVoxel::Modeling::VoxelizationWorkspace* workspace)
{
    materialRootIndices.clear();
    intersectingItems.clear();

    const std::uint64_t xyCandidateCount = saturatedMultiply(rootRange.countX(), rootRange.countY());
    const std::uint64_t rootCandidateCount = saturatedMultiply(xyCandidateCount, rootRange.countZ());

    recorder.setRootCandidateCount(rootCandidateCount);

    if (rootCandidateCount <= static_cast<std::uint64_t>(materialRootIndices.max_size()))
    {
        materialRootIndices.reserve(static_cast<std::size_t>(rootCandidateCount));
    }

    if (rootCandidateCount <= static_cast<std::uint64_t>(intersectingItems.max_size()))
    {
        intersectingItems.reserve(static_cast<std::size_t>(rootCandidateCount));
    }

    for (std::int64_t z = static_cast<std::int64_t>(rootRange.minimum.z);
         z <= static_cast<std::int64_t>(rootRange.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(rootRange.minimum.y);
             y <= static_cast<std::int64_t>(rootRange.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(rootRange.minimum.x);
                 x <= static_cast<std::int64_t>(rootRange.maximum.x);
                 ++x)
            {
                const MyVoxel::VoxelCellIndex rootIndex(
                    static_cast<MyVoxel::VoxelIndex>(x),
                    static_cast<MyVoxel::VoxelIndex>(y),
                    static_cast<MyVoxel::VoxelIndex>(z));

                const MyVoxel::VoxelCellAddress rootAddress(rootIndex, MyVoxel::BaseVoxelLevel);

                recorder.gridCellBounds();

                const CellBounds rootBounds = makeCellBounds(grid.cellBounds(rootAddress));

                const MyVoxel::Geometry::ShapeRelation relation =query.classifyBoundsFast(rootBounds.center, rootBounds.extent);

                switch (relation)
                {
                case MyVoxel::Geometry::ShapeRelation::Outside:
                    recorder.relation(relation);
                    break;

                case MyVoxel::Geometry::ShapeRelation::Inside:
                    recorder.relation(relation);
                    materialRootIndices.push_back(rootIndex);
                    break;

                case MyVoxel::Geometry::ShapeRelation::Intersecting:
                    if (workspace)
                    {
                        MyVoxel::VoxelTree reusableTree =
                            MyVoxel::Modeling::VoxelizationWorkspaceAccess::acquireRootTree(*workspace, rootIndex);

                        intersectingItems.push_back(
                            RootVoxelizationItem(
                                rootIndex,
                                rootBounds,
                                std::move(reusableTree)));
                    }
                    else
                    {
                        intersectingItems.push_back(RootVoxelizationItem(rootIndex, rootBounds));
                    }

                    break;
                }
            }
        }
    }
}

// 独立生成一个边界相交根树。
template<typename Recorder>
void processIntersectingRoot(RootVoxelizationItem& item,
                             const MyVoxel::Geometry::ShapeQuery& query,
                             MyVoxel::VoxelLevel maximumLevel)
{
    Recorder recorder(item.statistics);
    MyVoxel::VoxelTreeEditor editor(item.resultTree);

    item.resultState =
        voxelizeCell(
            editor,
            query,
            MyVoxel::BaseVoxelLevel,
            maximumLevel,
            item.rootBounds,
            MyVoxel::Geometry::ShapeRelation::Intersecting,
            recorder);

    MYVOXEL_ASSERT_MESSAGE(item.resultTree.isValid(), "Voxelization produced an invalid root tree.");
}

// 并行生成全部边界相交根树。
template<typename Recorder>
void executeIntersectingRoots(const MyVoxel::Geometry::ShapeQuery& query,
                              MyVoxel::VoxelLevel maximumLevel,
                              std::vector<RootVoxelizationItem>& items)
{
    if (items.empty())
    {
        return;
    }

    MyVoxel::Foundation::ParallelOptions options;
    options.minimumParallelTaskCount = MinimumParallelIntersectingRootCount;

    MyVoxel::Foundation::ParallelExecutor::global().execute(
        0,
        items.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            // 每个批次复制独立查询上下文，避免查询器未来增加内部缓存后形成共享状态。
            const MyVoxel::Geometry::ShapeQuery threadQuery = query;

            for (std::size_t itemIndex = blockBegin; itemIndex < blockEnd; ++itemIndex)
            {
                processIntersectingRoot<Recorder>(items[itemIndex], threadQuery, maximumLevel);
            }
        },
        options);
}

// 串行提交完整材料根和边界相交根。
template<typename Recorder>
void commitVoxelizationRoots(MyVoxel::VoxelShapeSession& session,
                             const std::vector<MyVoxel::VoxelCellIndex>& materialRootIndices,
                             std::vector<RootVoxelizationItem>& intersectingItems,
                             Recorder& recorder,
                             MyVoxel::Modeling::VoxelizationWorkspace* workspace)
{
    for (std::size_t rootPosition = 0; rootPosition < materialRootIndices.size(); ++rootPosition)
    {
        MyVoxel::VoxelTree materialTree(MyVoxel::VoxelState::Material);

        session.setTree(materialRootIndices[rootPosition], std::move(materialTree));
        recorder.createdRoot();
    }

    for (std::size_t itemIndex = 0; itemIndex < intersectingItems.size(); ++itemIndex)
    {
        RootVoxelizationItem& item = intersectingItems[itemIndex];

        recorder.accumulate(item.statistics);

        if (item.resultState == MyVoxel::VoxelState::Empty)
        {
            if (workspace)
            {
                MyVoxel::Modeling::VoxelizationWorkspaceAccess::recycleRootTree(
                    *workspace,
                    item.rootIndex,
                    std::move(item.resultTree));
            }

            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(item.resultTree.isValid(), "Cannot commit an invalid voxelization root tree.");

        session.setTree(item.rootIndex, std::move(item.resultTree));
        recorder.createdRoot();
    }
}

// 将尚未提交的可复用根树归还工作区。
void recycleUncommittedRootTrees(std::vector<RootVoxelizationItem>& items,
                                 MyVoxel::Modeling::VoxelizationWorkspace& workspace)
{
    for (std::size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
    {
        RootVoxelizationItem& item = items[itemIndex];

        if (item.resultTree.chunkCount() == 0)
        {
            continue;
        }

        MyVoxel::Modeling::VoxelizationWorkspaceAccess::recycleRootTree(
            workspace,
            item.rootIndex,
            std::move(item.resultTree));
    }
}

// 将统一查询体素化到已经准备好的空VoxelShape。
template<typename Recorder>
void voxelizeQueryInto(const MyVoxel::Geometry::ShapeQuery& query,
                       const MyVoxel::VoxelGrid& grid,
                       const MyMath::Matrix4& resultTransform,
                       Recorder& recorder,
                       MyVoxel::Modeling::VoxelizationWorkspace* workspace,
                       MyVoxel::VoxelShape& result)
{
    MYVOXEL_ASSERT_MESSAGE(query.isValid(), "Shape voxelization requires a valid ShapeQuery.");
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "Shape voxelization requires a valid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(resultTransform.isAffine(), "Voxelization result transform must be affine.");
    MYVOXEL_ASSERT_MESSAGE(resultTransform.isInvertible(), "Voxelization result transform must be invertible.");
    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "Voxelization output must be valid.");
    MYVOXEL_ASSERT_MESSAGE(result.isEmpty(), "Voxelization output must be empty before generation.");
    MYVOXEL_ASSERT_MESSAGE(result.grid().isEqualTo(grid, 0.0), "Voxelization output grid must match the requested grid.");

    const MyVoxel::VoxelCellRange rootRange = grid.cellRange(query.queryBounds(), MyVoxel::BaseVoxelLevel);

    std::vector<MyVoxel::VoxelCellIndex> materialRootIndices;
    std::vector<RootVoxelizationItem> intersectingItems;

    try
    {
        collectRootVoxelizationItems(query,grid,rootRange,materialRootIndices,intersectingItems,recorder,workspace);
        executeIntersectingRoots<Recorder>(query,grid.maximumLevel(),intersectingItems);
        // 任意并行任务抛出异常时不会进入提交阶段，返回结果不会包含部分生成的根树。
        {
            MyVoxel::VoxelShapeSession session =result.session(MyVoxel::BaseVoxelLevel);
            commitVoxelizationRoots(session,materialRootIndices,intersectingItems,recorder,workspace);
        }
    }
    catch (...)
    {
        if (workspace)
        {
            recycleUncommittedRootTrees(intersectingItems, *workspace);
        }

        throw;
    }

    result.setTransform(resultTransform);

    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "Shape voxelization produced an invalid VoxelShape.");
}

// 执行一次性统一体素化。
template<typename Recorder>
MyVoxel::VoxelShape voxelizeQueryImpl(const MyVoxel::Geometry::ShapeQuery& query,
                                      const MyVoxel::VoxelGrid& grid,
                                      const MyMath::Matrix4& resultTransform,
                                      Recorder& recorder)
{
    MyVoxel::VoxelShape result(grid);

    voxelizeQueryInto(
        query,
        grid,
        resultTransform,
        recorder,
        nullptr,
        result);

    return result;
}

// 使用可复用工作区执行统一对齐体素化。
template<typename Recorder>
const MyVoxel::VoxelShape& voxelizeQueryImpl(const MyVoxel::Geometry::ShapeQuery& query,
                                             const MyVoxel::VoxelShape& reference,
                                             Recorder& recorder,
                                             MyVoxel::Modeling::VoxelizationWorkspace& workspace)
{
    MyVoxel::Modeling::VoxelizationWorkspaceAccess::prepare(workspace, reference);

    MyVoxel::VoxelShape& result =
        MyVoxel::Modeling::VoxelizationWorkspaceAccess::result(workspace);

    voxelizeQueryInto(
        query,
        reference.grid(),
        reference.transform(),
        recorder,
        &workspace,
        result);

    return result;
}

}

namespace MyVoxel
{
namespace Modeling
{

VoxelizationStatistics::VoxelizationStatistics()
    : rootCandidateCount(0)
    , createdRootCount(0)
    , gridCellBoundsCount(0)
    , derivedChildBoundsCount(0)
    , visitedCellCount(0)
    , outsideCellCount(0)
    , insideCellCount(0)
    , intersectingCellCount(0)
    , centerSampleCount(0)
    , splitCount(0)
    , maskLeafBuildCount(0)
    , maskLeafStoredCount(0)
    , mergeSuccessCount(0)
{
}

void VoxelizationStatistics::reset()
{
    rootCandidateCount = 0;
    createdRootCount = 0;
    gridCellBoundsCount = 0;
    derivedChildBoundsCount = 0;
    visitedCellCount = 0;
    outsideCellCount = 0;
    insideCellCount = 0;
    intersectingCellCount = 0;
    centerSampleCount = 0;
    splitCount = 0;
    maskLeafBuildCount = 0;
    maskLeafStoredCount = 0;
    mergeSuccessCount = 0;
}

void VoxelizationStatistics::accumulate(const VoxelizationStatistics& other)
{
    rootCandidateCount += other.rootCandidateCount;
    createdRootCount += other.createdRootCount;
    gridCellBoundsCount += other.gridCellBoundsCount;
    derivedChildBoundsCount += other.derivedChildBoundsCount;
    visitedCellCount += other.visitedCellCount;
    outsideCellCount += other.outsideCellCount;
    insideCellCount += other.insideCellCount;
    intersectingCellCount += other.intersectingCellCount;
    centerSampleCount += other.centerSampleCount;
    splitCount += other.splitCount;
    maskLeafBuildCount += other.maskLeafBuildCount;
    maskLeafStoredCount += other.maskLeafStoredCount;
    mergeSuccessCount += other.mergeSuccessCount;
}

/// 局部Shape体素化

VoxelShape voxelize(const Geometry::Shape& shape, const VoxelGrid& grid)
{
    const Geometry::ShapeQuery query(shape);
    DisabledVoxelizationRecorder recorder;

    return voxelizeQueryImpl(
        query,
        grid,
        MyMath::Matrix4::identity(),
        recorder);
}

VoxelShape voxelize(const Geometry::Shape& shape,
                    const VoxelGrid& grid,
                    VoxelizationStatistics& statistics)
{
    statistics.reset();

    const Geometry::ShapeQuery query(shape);
    EnabledVoxelizationRecorder recorder(statistics);

    return voxelizeQueryImpl(
        query,
        grid,
        MyMath::Matrix4::identity(),
        recorder);
}

VoxelShape voxelize(const Geometry::Shape& shape,
                    double baseVoxelEdgeLength,
                    VoxelLevel maximumLevel)
{
    return voxelize(
        shape,
        VoxelGrid(
            baseVoxelEdgeLength,
            maximumLevel));
}

VoxelShape voxelize(const Geometry::Shape& shape,
                    double baseVoxelEdgeLength,
                    VoxelLevel maximumLevel,
                    VoxelizationStatistics& statistics)
{
    return voxelize(
        shape,
        VoxelGrid(
            baseVoxelEdgeLength,
            maximumLevel),
        statistics);
}

/// Shape实例局部体素化

VoxelShape voxelize(const Geometry::ShapeInstance& instance,
                    const VoxelGrid& localGrid)
{
    MYVOXEL_ASSERT_MESSAGE(instance.isValid(), "ShapeInstance voxelization requires a valid instance.");

    const Geometry::ShapeQuery query(instance.shape());
    DisabledVoxelizationRecorder recorder;

    return voxelizeQueryImpl(
        query,
        localGrid,
        instance.localToWorld(),
        recorder);
}

VoxelShape voxelize(const Geometry::ShapeInstance& instance,
                    const VoxelGrid& localGrid,
                    VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(instance.isValid(), "ShapeInstance voxelization requires a valid instance.");

    statistics.reset();

    const Geometry::ShapeQuery query(instance.shape());
    EnabledVoxelizationRecorder recorder(statistics);

    return voxelizeQueryImpl(
        query,
        localGrid,
        instance.localToWorld(),
        recorder);
}

/// Shape实例对齐体素化

VoxelShape voxelizeAligned(const Geometry::ShapeInstance& instance,
                           const VoxelShape& reference)
{
    MYVOXEL_ASSERT_MESSAGE(instance.isValid(), "Aligned voxelization requires a valid ShapeInstance.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");

    const Geometry::ShapeQuery query(instance, reference.transform());
    DisabledVoxelizationRecorder recorder;

    return voxelizeQueryImpl(
        query,
        reference.grid(),
        reference.transform(),
        recorder);
}

VoxelShape voxelizeAligned(const Geometry::ShapeInstance& instance,
                           const VoxelShape& reference,
                           VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(instance.isValid(), "Aligned voxelization requires a valid ShapeInstance.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");

    statistics.reset();

    const Geometry::ShapeQuery query(instance, reference.transform());
    EnabledVoxelizationRecorder recorder(statistics);

    return voxelizeQueryImpl(
        query,
        reference.grid(),
        reference.transform(),
        recorder);
}

/// 工作区对齐体素化

const VoxelShape& voxelizeAligned(const Geometry::ShapeInstance& instance,
                                  const VoxelShape& reference,
                                  VoxelizationWorkspace& workspace)
{
    MYVOXEL_ASSERT_MESSAGE(instance.isValid(), "Aligned voxelization requires a valid ShapeInstance.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");

    const Geometry::ShapeQuery query(instance, reference.transform());
    DisabledVoxelizationRecorder recorder;

    return voxelizeQueryImpl(query, reference, recorder, workspace);
}

const VoxelShape& voxelizeAligned(const Geometry::ShapeInstance& instance,
                                  const VoxelShape& reference,
                                  VoxelizationWorkspace& workspace,
                                  VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(instance.isValid(), "Aligned voxelization requires a valid ShapeInstance.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");

    statistics.reset();

    const Geometry::ShapeQuery query(instance, reference.transform());
    EnabledVoxelizationRecorder recorder(statistics);

    return voxelizeQueryImpl(query, reference, recorder, workspace);
}

}
}