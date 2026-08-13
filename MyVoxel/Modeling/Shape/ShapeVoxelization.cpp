#include "ShapeVoxelization.h"
#include "VoxelizationWorkspace.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Storage/LeafBlock.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/ParallelOptions.h"
#include "MyVoxel/Tool/Query/ShapeQuery.h"
#include "MyVoxel/Modeling/Feature/ShapeFeatureExtractor.h"
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
const unsigned int MaskLeafCoveredLevelCount = 2; // 一个MaskLeaf固定显式保存当前NodeChild下面两级的4×4×4共64个最高层样本。
const std::size_t MinimumParallelRootCount = 4; // 四个及以上窄带根任务进入并行体素化。

// 保存体素在查询空间中的中心和半尺寸，递归过程不重复构造Bounds3。
struct CellBounds
{
    CellBounds(const MyMath::Vector3& centerValue, const MyMath::Vector3& extentValue)
        : center(centerValue)
        , extent(extentValue)
    {
        MYVOXEL_ASSERT_MESSAGE(center.isFinite(), "CellBounds center must be finite.");
        MYVOXEL_ASSERT_MESSAGE(extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0, "CellBounds extent must be finite and non-negative.");
    }

    MyMath::Vector3 center; // 当前体素中心。
    MyMath::Vector3 extent; // 当前体素三个方向的半尺寸。
};

// 正式路径使用的空统计记录器。
class DisabledVoxelizationRecorder
{
public:
    DisabledVoxelizationRecorder() = default;
    explicit DisabledVoxelizationRecorder(MyVoxel::Modeling::VoxelizationStatistics&){}
    void setRootCandidateCount(std::uint64_t){}
    void createdRoot(){}
    void gridCellBounds(){}
    void derivedChildBounds(){}
    void cellSample(double, double, float){}
    void split(){}
    void maskLeafBuild(){}
    void maskLeafStored(){}
    void mergeSuccess(){}
    void accumulate(const MyVoxel::Modeling::VoxelizationStatistics&){}
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

    void cellSample(double signedDistance, double cellRadius, float backgroundDistance)
    {
        ++m_statistics.visitedCellCount;
        ++m_statistics.centerSampleCount;

        if (signedDistance - cellRadius >= static_cast<double>(backgroundDistance))
        {
            ++m_statistics.outsideCellCount;
        }
        else if (signedDistance + cellRadius <= -static_cast<double>(backgroundDistance))
        {
            ++m_statistics.insideCellCount;
        }
        else
        {
            ++m_statistics.intersectingCellCount;
        }
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

// 保存一个需要递归生成窄带TSDF的根。
struct RootVoxelizationItem
{
    RootVoxelizationItem(const MyVoxel::VoxelCellIndex& rootIndexValue, const CellBounds& rootBoundsValue,
                         double centerDistanceValue, float backgroundDistance)
        : rootIndex(rootIndexValue)
        , rootBounds(rootBoundsValue)
        , centerDistance(centerDistanceValue)
        , resultTree(MyVoxel::VoxelState::Empty, backgroundDistance)
    {
    }

    RootVoxelizationItem(const MyVoxel::VoxelCellIndex& rootIndexValue, const CellBounds& rootBoundsValue,
                         double centerDistanceValue, float backgroundDistance, MyVoxel::VoxelTree&& reusableTree)
        : rootIndex(rootIndexValue)
        , rootBounds(rootBoundsValue)
        , centerDistance(centerDistanceValue)
        , resultTree(std::move(reusableTree))
    {
        MYVOXEL_ASSERT_MESSAGE(resultTree.isValid(), "Reusable voxelization root tree must be valid.");
        MYVOXEL_ASSERT_MESSAGE(resultTree.state() == MyVoxel::VoxelState::Empty && resultTree.value() == backgroundDistance,
                               "Reusable voxelization root tree must represent current +B background.");
    }

    MyVoxel::VoxelCellIndex rootIndex; // 当前第0层根索引。
    CellBounds rootBounds; // 当前第0层根在查询空间中的中心和半尺寸。
    double centerDistance; // 当前根中心到Shape边界的精确有符号距离。
    MyVoxel::VoxelTree resultTree; // 当前根独立生成的TSDF树。
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
    return first > maximumValue / second ? maximumValue : first * second;
}

// 返回旧公开构造语义使用的默认截断背景距离，即最高层体素边长。
float defaultBackgroundDistance(const MyVoxel::VoxelGrid& grid)
{
    const float result = static_cast<float>(grid.minimumCellEdgeLength());
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(static_cast<double>(result)) && result > 0.0f, "VoxelGrid minimum cell edge length must define a finite positive background distance.");
    return result;
}

// 将普通轴对齐包围盒转换为中心和半尺寸表示。
CellBounds makeCellBounds(const MyVoxel::Bounds3& bounds)
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Cannot create CellBounds from invalid Bounds3.");

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    return CellBounds(
        MyMath::Vector3((minimum.x() + maximum.x()) * CellCenterScale,
                        (minimum.y() + maximum.y()) * CellCenterScale,
                        (minimum.z() + maximum.z()) * CellCenterScale),
        MyMath::Vector3((maximum.x() - minimum.x()) * CellCenterScale,
                        (maximum.y() - minimum.y()) * CellCenterScale,
                        (maximum.z() - minimum.z()) * CellCenterScale));
}

// 根据父中心、子半尺寸和角点方向生成子体素。
CellBounds makeChildBounds(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent, MyVoxel::VoxelCorner corner)
{
    const unsigned int cornerValue = static_cast<unsigned int>(corner);
    MYVOXEL_ASSERT_MESSAGE(cornerValue < static_cast<unsigned int>(MyVoxel::VoxelCornerCount), "Voxel child corner must be in range [0, 7].");

    return CellBounds(
        MyMath::Vector3(parentCenter.x() + ((cornerValue & XChildMask) != 0 ? childExtent.x() : -childExtent.x()),
                        parentCenter.y() + ((cornerValue & YChildMask) != 0 ? childExtent.y() : -childExtent.y()),
                        parentCenter.z() + ((cornerValue & ZChildMask) != 0 ? childExtent.z() : -childExtent.z())),
        childExtent);
}

// 返回当前轴对齐体素从中心到最远角点的欧氏距离。
double cellRadius(const CellBounds& bounds)
{
    const double x = bounds.extent.x();
    const double y = bounds.extent.y();
    const double z = bounds.extent.z();
    return std::sqrt(x * x + y * y + z * z);
}

// 将精确有符号距离截断到[-B,+B]并转换为Leaf/Tile使用的float。
float clampedTsdfValue(double signedDistance, float backgroundDistance)
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(signedDistance), "Signed-distance sample must be finite.");

    if (signedDistance >= static_cast<double>(backgroundDistance))
    {
        return backgroundDistance;
    }

    if (signedDistance <= -static_cast<double>(backgroundDistance))
    {
        return -backgroundDistance;
    }

    const float value = static_cast<float>(signedDistance);
    return (std::max)(-backgroundDistance, (std::min)(backgroundDistance, value));
}

// 根据精确中心SDF和1-Lipschitz性质判断整个体素截断后是否恒为±B。
bool saturatedCellValue(double signedDistance, double radius, float backgroundDistance, float& value)
{
    if (signedDistance - radius >= static_cast<double>(backgroundDistance))
    {
        value = backgroundDistance;
        return true;
    }

    if (signedDistance + radius <= -static_cast<double>(backgroundDistance))
    {
        value = -backgroundDistance;
        return true;
    }

    return false;
}

// 判断当前位置是否可以使用覆盖下面两级的MaskLeaf，Root本身不能直接保存Leaf。
bool shouldBuildMaskLeaf(MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel)
{
    if (level == MyVoxel::BaseVoxelLevel || level > maximumLevel)
    {
        return false;
    }

    return static_cast<unsigned int>(maximumLevel) - static_cast<unsigned int>(level) == MaskLeafCoveredLevelCount;
}

// 判断一棵根树是否与Forest缺失根+B背景完全等价。
bool isBackgroundTree(const MyVoxel::VoxelTree& tree, float backgroundDistance)
{
    return tree.state() == MyVoxel::VoxelState::Empty && tree.value() == backgroundDistance;
}

// 对一个体素中心执行精确signed-distance查询并记录统计。
template<typename Recorder>
double sampleCellDistance(const MyVoxel::ShapeQuery& query, const CellBounds& bounds, float backgroundDistance, Recorder& recorder)
{
    const double distance = query.signedDistanceToPoint(bounds.center);
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(distance), "Shape voxelization signed-distance query must return a finite value.");
    recorder.cellSample(distance, cellRadius(bounds), backgroundDistance);
    return distance;
}

// 直接计算当前体素下面两级对应的64个最高层TSDF样本。
template<typename Recorder>
void buildMaskLeafBlock(const MyVoxel::ShapeQuery& query, const CellBounds& cellBounds, float backgroundDistance,
                        MyVoxel::LeafBlock& block, Recorder& recorder)
{
    recorder.maskLeafBuild();

    const MyMath::Vector3 coarseExtent = cellBounds.extent * CellCenterScale;
    const MyMath::Vector3 fineExtent = coarseExtent * CellCenterScale;

    for (unsigned int coarseIndex = 0; coarseIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++coarseIndex)
    {
        const MyVoxel::VoxelCorner coarseCorner = static_cast<MyVoxel::VoxelCorner>(coarseIndex);
        const CellBounds coarseBounds = makeChildBounds(cellBounds.center, coarseExtent, coarseCorner);
        recorder.derivedChildBounds();

        for (unsigned int fineIndex = 0; fineIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++fineIndex)
        {
            const MyVoxel::VoxelCorner fineCorner = static_cast<MyVoxel::VoxelCorner>(fineIndex);
            const CellBounds fineBounds = makeChildBounds(coarseBounds.center, fineExtent, fineCorner);
            recorder.derivedChildBounds();

            const double distance = query.signedDistanceToPoint(fineBounds.center);
            MYVOXEL_ASSERT_MESSAGE(std::isfinite(distance), "MaskLeaf signed-distance sample must be finite.");
            recorder.cellSample(distance, cellRadius(fineBounds), backgroundDistance);
            block.distances[MyVoxel::leafSampleIndex(coarseCorner, fineCorner)] = clampedTsdfValue(distance, backgroundDistance);
        }
    }
}

// 按已知中心SDF递归生成当前逻辑体素。
template<typename Recorder>
void voxelizeCellWithDistance(MyVoxel::VoxelTreeEditor editor, const MyVoxel::ShapeQuery& query,
                              MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel,
                              const CellBounds& cellBounds, double centerDistance, float backgroundDistance,
                              Recorder& recorder)
{
    const double radius = cellRadius(cellBounds);
    float saturatedValue = 0.0f;

    if (saturatedCellValue(centerDistance, radius, backgroundDistance, saturatedValue))
    {
        editor.setValue(saturatedValue);
        return;
    }

    if (level == maximumLevel)
    {
        editor.setValue(clampedTsdfValue(centerDistance, backgroundDistance));
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(level < maximumLevel, "Voxelization level exceeds the VoxelGrid maximum level.");

    if (shouldBuildMaskLeaf(level, maximumLevel))
    {
        MYVOXEL_ASSERT_MESSAGE(editor.canSetLeafBlock(), "MaskLeaf fast path requires a normal NodeChild editor.");

        MyVoxel::LeafBlock block;
        buildMaskLeafBlock(query, cellBounds, backgroundDistance, block, recorder);
        editor.setLeafBlock(block);

        if (editor.isSubdivided())
        {
            recorder.maskLeafStored();
        }
        else
        {
            recorder.mergeSuccess();
        }

        return;
    }

    if (!editor.isSubdivided())
    {
        editor.subdivide();
        recorder.split();
    }

    const MyVoxel::VoxelLevel childLevel = static_cast<MyVoxel::VoxelLevel>(level + 1);
    const MyMath::Vector3 childExtent = cellBounds.extent * CellCenterScale;

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        const CellBounds childBounds = makeChildBounds(cellBounds.center, childExtent, corner);
        recorder.derivedChildBounds();
        const double childDistance = sampleCellDistance(query, childBounds, backgroundDistance, recorder);

        voxelizeCellWithDistance(editor.child(corner), query, childLevel, maximumLevel, childBounds,
                                 childDistance, backgroundDistance, recorder);
    }
}

// 根据Shape窄带包围盒收集完全-B根和需要递归生成的窄带根。
template<typename Recorder>
void collectRootVoxelizationItems(const MyVoxel::ShapeQuery& query, const MyVoxel::VoxelGrid& grid,
                                  float backgroundDistance, const MyVoxel::VoxelCellRange& rootRange,
                                  std::vector<MyVoxel::VoxelCellIndex>& materialRootIndices,
                                  std::vector<RootVoxelizationItem>& narrowBandItems,
                                  Recorder& recorder, MyVoxel::Modeling::VoxelizationWorkspace* workspace)
{
    materialRootIndices.clear();
    narrowBandItems.clear();

    const std::uint64_t xyCandidateCount = saturatedMultiply(rootRange.countX(), rootRange.countY());
    const std::uint64_t rootCandidateCount = saturatedMultiply(xyCandidateCount, rootRange.countZ());
    recorder.setRootCandidateCount(rootCandidateCount);

    if (rootCandidateCount <= static_cast<std::uint64_t>(materialRootIndices.max_size()))
    {
        materialRootIndices.reserve(static_cast<std::size_t>(rootCandidateCount));
    }

    if (rootCandidateCount <= static_cast<std::uint64_t>(narrowBandItems.max_size()))
    {
        narrowBandItems.reserve(static_cast<std::size_t>(rootCandidateCount));
    }

    for (std::int64_t z = static_cast<std::int64_t>(rootRange.minimum.z); z <= static_cast<std::int64_t>(rootRange.maximum.z); ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(rootRange.minimum.y); y <= static_cast<std::int64_t>(rootRange.maximum.y); ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(rootRange.minimum.x); x <= static_cast<std::int64_t>(rootRange.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex rootIndex(static_cast<MyVoxel::VoxelIndex>(x),
                                                        static_cast<MyVoxel::VoxelIndex>(y),
                                                        static_cast<MyVoxel::VoxelIndex>(z));
                const MyVoxel::VoxelCellAddress rootAddress(rootIndex, MyVoxel::BaseVoxelLevel);

                recorder.gridCellBounds();
                const CellBounds rootBounds = makeCellBounds(grid.cellBounds(rootAddress));
                const double centerDistance = sampleCellDistance(query, rootBounds, backgroundDistance, recorder);
                float saturatedValue = 0.0f;

                if (saturatedCellValue(centerDistance, cellRadius(rootBounds), backgroundDistance, saturatedValue))
                {
                    if (saturatedValue < 0.0f)
                    {
                        materialRootIndices.push_back(rootIndex);
                    }
                    continue;
                }

                if (workspace)
                {
                    MyVoxel::VoxelTree reusableTree = MyVoxel::Modeling::VoxelizationWorkspaceAccess::acquireRootTree(*workspace, rootIndex);
                    narrowBandItems.push_back(RootVoxelizationItem(rootIndex, rootBounds, centerDistance, backgroundDistance, std::move(reusableTree)));
                }
                else
                {
                    narrowBandItems.push_back(RootVoxelizationItem(rootIndex, rootBounds, centerDistance, backgroundDistance));
                }
            }
        }
    }
}

// 独立生成一个窄带根树并执行完整bottom-up无损裁剪。
template<typename Recorder>
void processNarrowBandRoot(RootVoxelizationItem& item, const MyVoxel::ShapeQuery& query,
                           MyVoxel::VoxelLevel maximumLevel, float backgroundDistance)
{
    Recorder recorder(item.statistics);
    MyVoxel::VoxelTreeEditor editor(item.resultTree, backgroundDistance);

    voxelizeCellWithDistance(editor, query, MyVoxel::BaseVoxelLevel, maximumLevel,
                             item.rootBounds, item.centerDistance, backgroundDistance, recorder);

    if (item.resultTree.prune(backgroundDistance))
    {
        recorder.mergeSuccess();
    }

    MYVOXEL_ASSERT_MESSAGE(item.resultTree.isTsdfValid(backgroundDistance), "Voxelization produced an invalid TSDF root tree.");
    MYVOXEL_ASSERT_MESSAGE(item.resultTree.isNormalized(backgroundDistance), "Voxelization root tree must be normalized after prune.");
}

// 并行生成全部窄带根树。
template<typename Recorder>
void executeNarrowBandRoots(const MyVoxel::ShapeQuery& query, MyVoxel::VoxelLevel maximumLevel,
                            float backgroundDistance, std::vector<RootVoxelizationItem>& items)
{
    if (items.empty())
    {
        return;
    }

    MyVoxel::Foundation::ParallelOptions options;
    options.minimumParallelTaskCount = MinimumParallelRootCount;

    MyVoxel::Foundation::ParallelExecutor::global().execute(
        0,
        items.size(),
        [&](std::size_t blockBegin, std::size_t blockEnd)
        {
            // ShapeQuery构造后只读且不可复制，根任务只执行并发只读signed-distance查询。
            for (std::size_t itemIndex = blockBegin; itemIndex < blockEnd; ++itemIndex)
            {
                processNarrowBandRoot<Recorder>(items[itemIndex], query, maximumLevel, backgroundDistance);
            }
        },
        options);
}

// 串行提交完全-B根和窄带根。
template<typename Recorder>
void commitVoxelizationRoots(MyVoxel::VoxelShapeSession& session,
                             const std::vector<MyVoxel::VoxelCellIndex>& materialRootIndices,
                             std::vector<RootVoxelizationItem>& narrowBandItems,
                             float backgroundDistance, Recorder& recorder,
                             MyVoxel::Modeling::VoxelizationWorkspace* workspace)
{
    for (std::size_t rootPosition = 0; rootPosition < materialRootIndices.size(); ++rootPosition)
    {
        MyVoxel::VoxelTree materialTree(MyVoxel::VoxelState::Material, -backgroundDistance);
        session.setTree(materialRootIndices[rootPosition], std::move(materialTree));
        recorder.createdRoot();
    }

    for (std::size_t itemIndex = 0; itemIndex < narrowBandItems.size(); ++itemIndex)
    {
        RootVoxelizationItem& item = narrowBandItems[itemIndex];
        recorder.accumulate(item.statistics);

        if (isBackgroundTree(item.resultTree, backgroundDistance))
        {
            if (workspace)
            {
                MyVoxel::Modeling::VoxelizationWorkspaceAccess::recycleRootTree(*workspace, item.rootIndex, std::move(item.resultTree));
            }
            continue;
        }

        MYVOXEL_ASSERT_MESSAGE(item.resultTree.isTsdfValid(backgroundDistance), "Cannot commit an invalid TSDF voxelization root tree.");
        session.setTree(item.rootIndex, std::move(item.resultTree));
        recorder.createdRoot();
    }
}

// 将尚未提交的可复用根树归还工作区。
void recycleUncommittedRootTrees(std::vector<RootVoxelizationItem>& items, MyVoxel::Modeling::VoxelizationWorkspace& workspace)
{
    for (std::size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
    {
        RootVoxelizationItem& item = items[itemIndex];

        if (item.resultTree.storageCapacityBytes() == 0)
        {
            continue;
        }

        MyVoxel::Modeling::VoxelizationWorkspaceAccess::recycleRootTree(workspace, item.rootIndex, std::move(item.resultTree));
    }
}

// 将统一signed-distance查询体素化到已经准备好的空VoxelShape。
template<typename Recorder>
void voxelizeQueryInto(const MyVoxel::ShapeQuery& query, const MyVoxel::VoxelGrid& grid,
                       const MyMath::Matrix4& resultTransform, Recorder& recorder,
                       MyVoxel::Modeling::VoxelizationWorkspace* workspace, MyVoxel::VoxelShape& result)
{
    MYVOXEL_ASSERT_MESSAGE(query.isValid(), "Shape voxelization requires a valid ShapeQuery.");
    MYVOXEL_REQUIRE_MESSAGE(query.supportsSignedDistance(), "TSDF voxelization requires exact signed-distance support in the current query space.");
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "Shape voxelization requires a valid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(resultTransform.isAffine() && resultTransform.isInvertible(), "Voxelization result transform must be invertible affine.");
    MYVOXEL_ASSERT_MESSAGE(result.isValid() && result.isEmpty(), "Voxelization output must be a valid empty VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(result.grid().isEqualTo(grid, 0.0), "Voxelization output grid must match the requested grid.");
    MYVOXEL_ASSERT_MESSAGE(result.features().isEmpty(), "Voxelization output must not retain explicit Features from a previous result.");
    const float backgroundDistance = result.backgroundDistance();
    const MyVoxel::Bounds3 voxelizationBounds = query.queryBounds().expanded(static_cast<double>(backgroundDistance));
    const MyVoxel::VoxelCellRange rootRange = grid.cellRange(voxelizationBounds, MyVoxel::BaseVoxelLevel);

    std::vector<MyVoxel::VoxelCellIndex> materialRootIndices;
    std::vector<RootVoxelizationItem> narrowBandItems;

    try
    {
        collectRootVoxelizationItems(query, grid, backgroundDistance, rootRange, materialRootIndices, narrowBandItems, recorder, workspace);
        executeNarrowBandRoots<Recorder>(query, grid.maximumLevel(), backgroundDistance, narrowBandItems);

        {
            MyVoxel::VoxelShapeSession session = result.session(MyVoxel::BaseVoxelLevel);
            commitVoxelizationRoots(session, materialRootIndices, narrowBandItems, backgroundDistance, recorder, workspace);

            if (MyVoxel::Modeling::ShapeFeatureExtractor::supports(query))
            {
                session.setFeatures(MyVoxel::Modeling::ShapeFeatureExtractor::extract(query));
            }
            else
            {
                // TSDF仍然有效，但当前Shape类型尚无完整Feature提取规则，不能把空集合误认为“确认无特征”。
                session.invalidateFeatures();
            }
        }
    }
    catch (...)
    {
        if (workspace)
        {
            recycleUncommittedRootTrees(narrowBandItems, *workspace);
        }
        throw;
    }

    result.setTransform(resultTransform);
    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "Shape voxelization produced an invalid VoxelShape.");
}

// 执行一次性统一体素化，显式指定结果B。
template<typename Recorder>
MyVoxel::VoxelShape voxelizeQueryImpl(const MyVoxel::ShapeQuery& query, const MyVoxel::VoxelGrid& grid,
                                      const MyMath::Matrix4& resultTransform, float backgroundDistance, Recorder& recorder)
{
    MyVoxel::VoxelShape result(grid, backgroundDistance);
    voxelizeQueryInto(query, grid, resultTransform, recorder, nullptr, result);
    return result;
}

// 使用可复用工作区执行统一对齐体素化。
template<typename Recorder>
const MyVoxel::VoxelShape& voxelizeQueryImpl(const MyVoxel::ShapeQuery& query, const MyVoxel::VoxelShape& reference,
                                             Recorder& recorder, MyVoxel::Modeling::VoxelizationWorkspace& workspace)
{
    MyVoxel::Modeling::VoxelizationWorkspaceAccess::prepare(workspace, reference);
    MyVoxel::VoxelShape& result = MyVoxel::Modeling::VoxelizationWorkspaceAccess::result(workspace);
    voxelizeQueryInto(query, reference.grid(), reference.transform(), recorder, &workspace, result);
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

/// 局部Topology_Shape体素化

VoxelShape voxelize(const Topology_Shape& topology, const VoxelGrid& grid)
{
    const ShapeQuery query(topology);
    DisabledVoxelizationRecorder recorder;
    return voxelizeQueryImpl(query, grid, MyMath::Matrix4::identity(), defaultBackgroundDistance(grid), recorder);
}

VoxelShape voxelize(const Topology_Shape& topology, const VoxelGrid& grid, VoxelizationStatistics& statistics)
{
    statistics.reset();
    const ShapeQuery query(topology);
    EnabledVoxelizationRecorder recorder(statistics);
    return voxelizeQueryImpl(query, grid, MyMath::Matrix4::identity(), defaultBackgroundDistance(grid), recorder);
}

VoxelShape voxelize(const Topology_Shape& topology, double baseVoxelEdgeLength, VoxelLevel maximumLevel)
{
    return voxelize(topology, VoxelGrid(baseVoxelEdgeLength, maximumLevel));
}

VoxelShape voxelize(const Topology_Shape& topology, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics)
{
    return voxelize(topology, VoxelGrid(baseVoxelEdgeLength, maximumLevel), statistics);
}

/// Shape实例局部体素化

VoxelShape voxelize(const Shape& shape, const VoxelGrid& localGrid)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Shape voxelization requires a valid Shape.");
    const ShapeQuery query(shape.topology());
    DisabledVoxelizationRecorder recorder;
    return voxelizeQueryImpl(query, localGrid, shape.localToWorld(), defaultBackgroundDistance(localGrid), recorder);
}

VoxelShape voxelize(const Shape& shape, const VoxelGrid& localGrid, VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Shape voxelization requires a valid Shape.");
    statistics.reset();
    const ShapeQuery query(shape.topology());
    EnabledVoxelizationRecorder recorder(statistics);
    return voxelizeQueryImpl(query, localGrid, shape.localToWorld(), defaultBackgroundDistance(localGrid), recorder);
}

VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel)
{
    return voxelize(shape, VoxelGrid(baseVoxelEdgeLength, maximumLevel));
}

VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics)
{
    return voxelize(shape, VoxelGrid(baseVoxelEdgeLength, maximumLevel), statistics);
}

/// Shape实例对齐体素化

VoxelShape voxelizeAligned(const Shape& shape, const VoxelShape& reference)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Aligned voxelization requires a valid Shape.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");
    const ShapeQuery query(shape, reference.transform());
    DisabledVoxelizationRecorder recorder;
    return voxelizeQueryImpl(query, reference.grid(), reference.transform(), reference.backgroundDistance(), recorder);
}

VoxelShape voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Aligned voxelization requires a valid Shape.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");
    statistics.reset();
    const ShapeQuery query(shape, reference.transform());
    EnabledVoxelizationRecorder recorder(statistics);
    return voxelizeQueryImpl(query, reference.grid(), reference.transform(), reference.backgroundDistance(), recorder);
}

/// 工作区对齐体素化

const VoxelShape& voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationWorkspace& workspace)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Aligned voxelization requires a valid Shape.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");
    const ShapeQuery query(shape, reference.transform());
    DisabledVoxelizationRecorder recorder;
    return voxelizeQueryImpl(query, reference, recorder, workspace);
}

const VoxelShape& voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationWorkspace& workspace, VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Aligned voxelization requires a valid Shape.");
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "Aligned voxelization requires a valid reference VoxelShape.");
    statistics.reset();
    const ShapeQuery query(shape, reference.transform());
    EnabledVoxelizationRecorder recorder(statistics);
    return voxelizeQueryImpl(query, reference, recorder, workspace);
}

}
}