#ifndef MYVOXEL_SURFACE_VOXELSURFACECACHE_H
#define MYVOXEL_SURFACE_VOXELSURFACECACHE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Mesh/Mesh.h"

#include "VoxelFaceColorMap.h"
#include "VoxelFaceExtractor.h"
#include "VoxelFaceSet.h"
#include "VoxelRootFaceMasks.h"

namespace MyVoxel
{

// 保存一次根级体素表面缓存更新产生的面集合变化。
struct VoxelSurfaceCacheUpdate
{
    using RootIndexSet = VoxelChangeSet::RootIndexSet;

    // 判断本次更新是否实际增加或删除了表面。
    bool hasSurfaceChanges() const;

    RootIndexSet affectedRootIndices; // 根据修改根扩展得到的全部待检查根。
    RootIndexSet changedRootIndices; // 面集合实际发生变化并重新构建网格的根。
    VoxelFaceSet addedFaces; // 更新后新增的最高层单位表面。
    VoxelFaceSet removedFaces; // 更新后消失的最高层单位表面。
};

// 保存一次根级表面缓存增量更新的阶段耗时、任务规模和并行调度摘要。
struct VoxelSurfaceUpdateStatistics
{
    VoxelSurfaceUpdateStatistics();

    // 清空全部阶段耗时、计数和调度状态。
    void clear();

    // 累加另一次缓存更新统计。
    void add(const VoxelSurfaceUpdateStatistics& other);

    double affectedRootCollectionMilliseconds; // 根据修改根收集目标根及六个面邻根的耗时。
    double dirtyRegionBuildMilliseconds; // 将实际变化掩码叶块扩展为待更新Root局部区域的耗时。
    double faceExtractionMilliseconds; // 全部受影响根单位面提取的调用墙钟耗时。
    double occupancyBuildMilliseconds; // 面提取内部收集、并行展开和提交共享根占用的墙钟耗时。
    double occupancyBuildCpuMilliseconds; // 各源根独立展开核心占用的CPU耗时总和。
    double faceGenerationMilliseconds; // 面提取内部并行生成全部目标Root方向面掩码的墙钟耗时。
    double faceGenerationCpuMilliseconds; // 各目标Root独立生成方向面掩码的CPU耗时总和。
    double faceSetBuildMilliseconds; // 保留兼容统计项；热路径不再生成完整Root面集合。
    double oldFaceCopyMilliseconds; // 保留兼容统计项；面掩码路径不再复制旧根面集合。
    double faceComparisonMilliseconds; // 比较新旧Root方向面掩码的累计耗时。
    double faceDifferenceMilliseconds; // 并行生成Root局部差分并通过有序归并构造全局集合的墙钟耗时。
    double faceDifferenceCpuMilliseconds; // 各变化Root生成差分地址并完成局部排序去重的CPU耗时总和。
    double faceDifferenceMergeMilliseconds; // 将多个Root有序差分集合归并为全局有序集合的墙钟耗时。
    double colorUpdateMilliseconds; // 复制并更新独立面颜色映射的耗时。
    double meshBuildWallMilliseconds; // Root颜色预处理、方向并行构建及局部包围盒更新的同步墙钟耗时。
    double rootColorPrepareMilliseconds; // 各变化Root一次性准备方向切片颜色数据的墙钟耗时。
    double facePlaneBuildCpuMilliseconds; // 各方向复制活动位图并叠加预处理颜色的CPU耗时总和。
    double greedyMergeCpuMilliseconds; // 各方向执行活动位行贪心合并并写入临时Mesh的CPU耗时总和。
    double directionFinalizeMilliseconds; // 收集方向任务结果并根据六方向网格更新Root局部包围盒的耗时。
    double directionMeshPlanMilliseconds; // 分析六方向重建、复用和清空动作并准备颜色数据的耗时。
    double cacheCommitMilliseconds; // 分配根条目并提交面掩码、颜色、方向网格和版本的耗时。
    double boundsUpdateMilliseconds; // 重新计算完整缓存包围盒的耗时。
    double totalMilliseconds; // VoxelSurfaceCache::update完整调用耗时。

    std::size_t affectedRootCount; // 本次需要检查的根数量。
    std::size_t changedRootCount; // 本次面集合实际变化的根数量。
    std::size_t colorUpdatedRootCount; // 本次复制并更新局部颜色映射的变化根数量。
    std::size_t colorCopiedEntryCount; // 本次为事务准备复制的Root局部颜色记录数量。
    std::size_t occupancySourceRequestCount; // 各目标根累计请求的源根占用次数。
    std::size_t uniqueOccupancyRootCount; // 批次去重后的源根索引数量，包括空根。
    std::size_t builtOccupancyRootCount; // 实际存在并建立核心占用数据的源根数量。
    std::size_t occupancyIncrementalUpdateCount; // 从旧缓存复制并仅替换变化掩码叶块的Root数量。
    std::size_t occupancyFullRebuildCount; // 缺少兼容旧缓存或脏区域时完整重建占用的Root数量。
    std::size_t occupancyRemovedRootCount; // 当前形体中已经不存在、待从占用缓存删除的Root数量。
    std::size_t dirtyLeafBlockCount; // 实际修改Root中唯一变化掩码叶块数量。
    std::size_t expandedDirtyLeafBlockCount; // 加入六个面邻块后的唯一待更新掩码叶块数量。
    std::size_t occupancyBuildCallCount; // 实际执行源根占用构建调度的次数。
    std::size_t occupancyBuildTaskCount; // 全部调度请求的源根构建任务数量。
    std::size_t occupancyBuildParticipantCount; // 全部调度中实际领取过任务批次的参与线程记录数量。
    std::size_t occupancyBuildBatchSizeTotal; // 全部调度实际批次大小的累计值。
    std::size_t occupancyBuildSerialCallCount; // 退化为单参与线程的占用构建调度次数。
    std::size_t occupancyBuildNestedSerialCallCount; // 因位于同一线程池工作线程而串行执行的调度次数。
    std::size_t faceGenerationCallCount; // 实际执行目标Root面掩码生成调度的次数。
    std::size_t faceGenerationTaskCount; // 全部调度请求的目标Root面掩码任务数量。
    std::size_t faceGenerationParticipantCount; // 全部调度中实际领取过任务批次的参与线程记录数量。
    std::size_t faceGenerationBatchSizeTotal; // 全部调度实际批次大小的累计值。
    std::size_t faceGenerationSerialCallCount; // 面掩码生成退化为单参与线程的调度次数。
    std::size_t faceGenerationNestedSerialCallCount; // 因位于同一线程池工作线程而串行执行的面掩码调度次数。
    std::size_t faceMaskIncrementalRootCount; // 使用Leaf局部字更新面掩码的Root数量。
    std::size_t faceMaskFullRebuildRootCount; // Dirty覆盖达到阈值而完整字级重建面掩码的Root数量。
    std::size_t faceMaskUpdatedWordCount; // 局部更新阶段执行的掩码字覆盖次数。
    std::size_t faceMaskUpdatedCellCount; // 局部更新覆盖的最高层体素数量。

    std::size_t faceDifferenceCallCount; // 实际执行Root面差分调度的次数。
    std::size_t faceDifferenceTaskCount; // 全部调度请求的变化Root差分任务数量。
    std::size_t faceDifferenceParticipantCount; // 全部差分调度中实际领取过任务批次的参与线程记录数量。
    std::size_t faceDifferenceBatchSizeTotal; // 全部差分调度实际批次大小的累计值。
    std::size_t faceDifferenceSerialCallCount; // Root面差分退化为单参与线程的调度次数。
    std::size_t faceDifferenceNestedSerialCallCount; // 因位于同一线程池工作线程而串行执行的差分调度次数。

    std::size_t emittedFaceCount; // 面掩码阶段累计识别出的单位外表面数量。
    std::size_t meshedFaceCount; // 贪心网格阶段累计消费的单位面数量。
    std::size_t nonEmptyPlaneCount; // 贪心网格阶段累计处理的非空方向切片数量。
    std::size_t mergedQuadCount; // 贪心网格阶段累计输出的四边形数量。
    std::size_t directionMeshHintedTaskCount; // 使用历史四边形数量预留方向临时Mesh容量的任务数量。
    std::size_t directionMeshReservedQuadCount; // 全部方向任务根据历史提示预留的四边形容量总和。
    std::size_t directionMeshHintUnderestimateCount; // 实际四边形数量超过预留容量的方向任务数量。
    std::size_t directionMeshUnusedReservedQuadCount; // 预留容量超过实际输出的四边形数量总和。
    std::size_t directionMeshConsideredCount; // 本次Root网格构建累计分析的方向数量。
    std::size_t directionMeshReusedCount; // 几何和颜色均未变化、直接复用旧方向Mesh的数量。
    std::size_t directionMeshClearedCount; // 更新后方向为空、无需执行Mesher的数量。
    std::size_t directionMeshReusedQuadCount; // 直接复用方向Mesh包含的四边形数量总和。

    std::size_t meshBuildCallCount; // 实际执行根网格构建调度的次数。
    std::size_t meshBuildTaskCount; // 全部调度请求的Root方向任务数量。
    std::size_t meshBuildParticipantCount; // 全部调度中实际领取过任务批次的参与线程记录数量。
    std::size_t meshBuildBatchSizeTotal; // 全部调度实际批次大小的累计值。
    std::size_t meshBuildSerialCallCount; // Root方向任务退化为单参与线程的调度次数。
    std::size_t meshBuildNestedSerialCallCount; // 因位于同一线程池工作线程而串行执行的调度次数。
};

// 缓存全部实际Root的最高层材料占用，并按非空表面Root缓存单位面、局部颜色、六方向独立网格及包围盒。
//
// 每个Root独立持有自身单独上色的单位面颜色，完整颜色视图仅在访问faceColors()时按需组合。
// 每次增量更新将新增表面设置为本批指定颜色，同时删除已经消失表面的颜色记录。
// 更新和颜色修改采用两阶段提交，网格构建失败时保留调用前的完整缓存状态。
class VoxelSurfaceCache
{
public:
    using RootIndexSet = VoxelChangeSet::RootIndexSet;
    using DirectionMeshes = std::array<Mesh, VoxelFaceDirectionCount>;
    using DirectionMeshVersions = std::array<std::uint64_t, VoxelFaceDirectionCount>;
    using DirectionQuadCountHints = std::array<std::size_t, VoxelFaceDirectionCount>;

    struct RootEntry
    {
        // 创建空Root缓存项，并将六个方向历史四边形数量初始化为零。
        RootEntry();

        // 标记按需组合Root网格缓存失效，并释放此前组合数据。
        void invalidateCombinedMesh();

        // 按固定方向顺序返回兼容Root网格，仅在显式访问时组合并缓存。
        const Mesh& combinedMesh() const;

        VoxelRootFaceMasks faceMasks; // 当前根全部单位外表面的方向位平面核心表示。
        VoxelFaceColorMap colors; // 当前根中需要单独上色的单位面颜色。
        DirectionMeshes directionMeshes; // 六个方向分别缓存的独立网格，用于增量复用和分段显示。
        DirectionMeshVersions directionMeshVersions; // 六个方向网格的单调版本，用于显示层跳过未变化上传。
        Bounds3 localBounds; // 六个方向网格共同形成的局部轴对齐包围盒。
        DirectionQuadCountHints directionQuadCountHints; // 六个方向上一次成功构建的实际四边形数量。

    private:
        mutable Mesh m_combinedMesh; // 兼容rootMesh()按需生成的临时组合网格。
        mutable bool m_combinedMeshValid; // 兼容组合网格是否与六方向网格一致。
    };

    using RootEntryMap = std::map<VoxelCellIndex, RootEntry>;

    VoxelSurfaceCache();

    /// 缓存管理

    // 清空全部Root占用、表面缓存、面颜色和来源状态。
    void clear();

    // 根据完整体素形体重新建立缓存，并清除此前保存的全部单独面颜色。
    void rebuild(const VoxelShape& shape, const Display_Color& defaultColor);

    // 根据修改Root的变化掩码叶块局部更新占用和面掩码，并将新增表面设置为newFaceColor。
    //
    // 全部受影响根网格成功构建后才提交新面集合和颜色。
    VoxelSurfaceCacheUpdate update(const VoxelShape& shape,
                                   const VoxelChangeSet& changes,
                                   const Display_Color& newFaceColor);

    // 根据修改根增量更新缓存，并返回完整阶段统计。
    VoxelSurfaceCacheUpdate update(const VoxelShape& shape,
                                   const VoxelChangeSet& changes,
                                   const Display_Color& newFaceColor,
                                   VoxelSurfaceUpdateStatistics* statistics);

    /// 颜色修改

    // 为当前仍然存在的指定面设置颜色，并重新构建实际发生颜色变化的根网格。
    //
    // 全部受影响根网格成功构建后才提交新颜色。
    RootIndexSet setFaceColor(const VoxelShape& shape,
                              const VoxelFaceSet& faces,
                              const Display_Color& color);

    // 删除指定面保存的单独颜色，并重新构建实际发生颜色变化的根网格。
    //
    // 全部受影响根网格成功构建后才提交新颜色。
    RootIndexSet eraseFaceColor(const VoxelShape& shape, const VoxelFaceSet& faces);

    // 清空全部单独面颜色，并重新构建此前包含单独颜色的根网格。
    //
    // 全部受影响根网格成功构建后才清空颜色映射。
    RootIndexSet clearFaceColors(const VoxelShape& shape);

    // 修改未单独上色表面的默认颜色，并重新构建全部根网格。
    //
    // 全部根网格成功构建后才提交新默认颜色。
    RootIndexSet setDefaultColor(const VoxelShape& shape, const Display_Color& color);

    /// 缓存状态

    // 判断缓存是否已经根据某个VoxelShape建立。
    bool isInitialized() const;

    // 判断当前缓存是否不包含任何非空根表面。
    bool isEmpty() const;

    // 返回当前缓存的非空根数量。
    std::size_t rootCount() const;

    // 返回全部根包含的最高层单位面数量。
    std::size_t faceCount() const;

    // 返回全部根网格包含的顶点数量。
    std::size_t vertexCount() const;

    // 返回全部根网格包含的三角形数量。
    std::size_t triangleCount() const;

    // 返回当前默认表面颜色。
    const Display_Color& defaultColor() const;

    // 返回全部根表面的局部轴对齐包围盒，空缓存返回无效包围盒。
    const Bounds3& localBounds() const;

    /// 根缓存访问

    // 判断指定根是否具有非空表面缓存。
    bool containsRoot(const VoxelCellIndex& rootIndex) const;

    // 返回指定根缓存项，不存在时返回空指针。
    const RootEntry* rootEntry(const VoxelCellIndex& rootIndex) const;

    // 按需生成指定根的完整单位面集合，不存在时返回空集合。
    VoxelFaceSet rootFaces(const VoxelCellIndex& rootIndex) const;

    // 返回指定根按固定方向顺序组合的兼容网格，不存在时返回空指针。
    //
    // 该接口会按需组合六方向网格，不应在增量显示热路径中调用。
    const Mesh* rootMesh(const VoxelCellIndex& rootIndex) const;

    // 返回指定根的六方向独立网格，不存在时返回空指针。
    const DirectionMeshes* rootDirectionMeshes(const VoxelCellIndex& rootIndex) const;

    // 返回指定根、指定方向的独立网格，不存在时返回空指针。
    const Mesh* rootDirectionMesh(const VoxelCellIndex& rootIndex,
                                            VoxelFaceDirection direction) const;

    // 返回指定根、指定方向的网格版本，不存在时返回0。
    std::uint64_t rootDirectionMeshVersion(const VoxelCellIndex& rootIndex,
                                           VoxelFaceDirection direction) const;

    // 返回指定根网格的局部包围盒，不存在时返回空指针。
    const Bounds3* rootBounds(const VoxelCellIndex& rootIndex) const;

    // 返回全部根缓存项。
    const RootEntryMap& rootEntries() const;

    /// 完整数据访问

    // 返回当前全部单独上色的单位面颜色映射。
    const VoxelFaceColorMap& faceColors() const;

    // 将全部根单位面组合为一个完整面集合。
    VoxelFaceSet combinedFaces() const;

    // 将全部根网格按根索引顺序组合为一个完整网格。
    Mesh combinedMesh() const;

private:
    // 检查指定形体的体素网格是否与当前缓存来源完全一致。
    bool isCompatible(const VoxelShape& shape) const;

    // 重新构建指定根集合中的网格，面集合为空时删除对应缓存项。
    void rebuildRootMeshes(const VoxelShape& shape, const RootIndexSet& rootIndices);

    // 根据全部根缓存的包围盒重新计算完整缓存包围盒。
    void rebuildLocalBounds();

    // 标记完整颜色视图需要在下次访问时重新组合。
    void invalidateCombinedFaceColors();

    // 按根索引顺序重新组合完整颜色视图。
    void rebuildCombinedFaceColors() const;

private:
    bool m_initialized; // 当前缓存是否已经通过rebuild完整建立。
    VoxelGrid m_grid; // 当前缓存对应的体素网格。
    Display_Color m_defaultColor; // 未单独上色表面的默认颜色。
    VoxelRootOccupancyMap m_occupancies; // 全部实际Root展开到最高层后的核心材料占用缓存，包括无外表面的内部Root。
    RootEntryMap m_roots; // 第0层根索引与根级面掩码、颜色和六方向网格缓存项。
    mutable VoxelFaceColorMap m_combinedFaceColors; // 按需组合的全部根单独面颜色只读视图。
    mutable bool m_combinedFaceColorsDirty; // 完整颜色视图是否需要重新组合。
    Bounds3 m_localBounds; // 全部非空根网格组合后的局部轴对齐包围盒。
};

}

#endif // MYVOXEL_SURFACE_VOXELSURFACECACHE_H
