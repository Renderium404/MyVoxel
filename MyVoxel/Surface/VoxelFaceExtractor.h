#ifndef MYVOXEL_SURFACE_VOXELFACEEXTRACTOR_H
#define MYVOXEL_SURFACE_VOXELFACEEXTRACTOR_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "VoxelFaceSet.h"
#include "VoxelRootFaceMasks.h"

namespace MyVoxel
{

// 保存一个第0层根展开到最高层后的材料占用位图。
//
// 每个YZ行独立按照64位字对齐，不保存Halo；根边界邻居由相邻VoxelRootOccupancy提供。
class VoxelRootOccupancy
{
public:
    VoxelRootOccupancy();

    // 为指定根和最高层单轴分辨率创建空或完整材料占用。
    VoxelRootOccupancy(const VoxelCellIndex& rootIndex,
                       std::size_t axisCellCount,
                       bool uniformMaterial);

    VoxelRootOccupancy(const VoxelRootOccupancy&) = default;
    VoxelRootOccupancy& operator=(const VoxelRootOccupancy&) = default;

    // 移动构造和移动赋值手动实现，以兼容Visual Studio 2013。
    VoxelRootOccupancy(VoxelRootOccupancy&& other);
    VoxelRootOccupancy& operator=(VoxelRootOccupancy&& other);

    /// 状态与属性

    // 判断当前占用位图是否已经初始化。
    bool isInitialized() const;

    // 判断当前根是否使用无需显式位图的完整材料状态。
    bool isUniformMaterial() const;

    // 返回当前占用所属的第0层根索引。
    const VoxelCellIndex& rootIndex() const;

    // 返回当前根单轴最高层体素数量。
    std::size_t axisCellCount() const;

    // 返回每个X方向占用行包含的64位字数量。
    std::size_t wordsPerRow() const;

    /// 占用读取

    // 返回指定YZ行中的一个64位占用字。
    std::uint64_t rowWord(std::size_t y,
                          std::size_t z,
                          std::size_t wordIndex) const;

    // 返回指定最高层局部体素是否包含材料。
    bool isMaterial(std::size_t x,
                    std::size_t y,
                    std::size_t z) const;

    /// 内部构建

    // 返回指定占用字中属于当前根最高层体素的有效位掩码。
    std::uint64_t validWordMask(std::size_t wordIndex) const;

    // 将同一YZ行中闭区间[minX, maxX]批量设置为材料。
    void setMaterialXRange(std::size_t minX,
                           std::size_t maxX,
                           std::size_t y,
                           std::size_t z);

    // 使用64位掩码替换一个掩码叶块覆盖的最高层占用。
    void replaceLeafBlock(std::size_t localBlockX,
                          std::size_t localBlockY,
                          std::size_t localBlockZ,
                          std::size_t blockCellScale,
                          std::uint64_t materialMask);

private:
    // 将完整材料表示展开为可局部修改的显式位图。
    void materializeUniformStorage();

    // 返回保存指定YZ行字的线性索引。
    std::size_t rowWordIndex(std::size_t y,
                             std::size_t z,
                             std::size_t wordIndex) const;

private:
    VoxelCellIndex m_rootIndex; // 当前占用位图所属的第0层根索引。
    std::size_t m_axisCellCount; // 当前根单轴最高层体素数量。
    std::size_t m_wordsPerRow; // 一个X方向占用行包含的64位字数量。
    bool m_uniformMaterial; // 当前根是否整体处于Material终止状态。
    std::vector<std::uint64_t> m_words; // 非均匀根按YZ行独立对齐的材料占用位图。
};

using VoxelRootOccupancyMap = std::map<VoxelCellIndex, VoxelRootOccupancy>;


// 保存一个目标Root及其六个面邻Root的只读最高层占用。
struct VoxelRootOccupancyNeighborhood
{
    VoxelRootOccupancyNeighborhood();

    // 使用当前Root及六个面邻Root占用建立只读邻域。
    VoxelRootOccupancyNeighborhood(
        const VoxelRootOccupancy* currentOccupancy,
        const VoxelRootOccupancy* negativeXOccupancy,
        const VoxelRootOccupancy* positiveXOccupancy,
        const VoxelRootOccupancy* negativeYOccupancy,
        const VoxelRootOccupancy* positiveYOccupancy,
        const VoxelRootOccupancy* negativeZOccupancy,
        const VoxelRootOccupancy* positiveZOccupancy);

    const VoxelRootOccupancy* current; // 当前目标Root占用，空Root为nullptr。
    const VoxelRootOccupancy* negativeX; // -X面邻Root占用。
    const VoxelRootOccupancy* positiveX; // +X面邻Root占用。
    const VoxelRootOccupancy* negativeY; // -Y面邻Root占用。
    const VoxelRootOccupancy* positiveY; // +Y面邻Root占用。
    const VoxelRootOccupancy* negativeZ; // -Z面邻Root占用。
    const VoxelRootOccupancy* positiveZ; // +Z面邻Root占用。
};

// 保存同一64位X区间中六个方向的暴露面掩码。
struct VoxelFaceMaskWord
{
    VoxelFaceMaskWord();

    // 返回至少具有一个暴露面的材料体素掩码。
    std::uint64_t exposedCells() const;

    std::uint64_t negativeX; // -X方向暴露面掩码。
    std::uint64_t positiveX; // +X方向暴露面掩码。
    std::uint64_t negativeY; // -Y方向暴露面掩码。
    std::uint64_t positiveY; // +Y方向暴露面掩码。
    std::uint64_t negativeZ; // -Z方向暴露面掩码。
    std::uint64_t positiveZ; // +Z方向暴露面掩码。
};

// 保存一次体素单位面提取的阶段耗时和结果规模。
struct VoxelFaceExtractionStatistics
{
    VoxelFaceExtractionStatistics();

    // 清空全部阶段耗时和计数。
    void clear();

    // 累加另一次面提取统计。
    void add(const VoxelFaceExtractionStatistics& other);

    double occupancyBuildMilliseconds; // 收集源根、并行展开或增量更新核心占用的墙钟耗时。
    double occupancyBuildCpuMilliseconds; // 各Root独立构建或更新核心占用的CPU耗时总和。
    double faceGenerationMilliseconds; // 并行生成或增量更新Root方向面掩码的墙钟耗时。
    double faceGenerationCpuMilliseconds; // 各Root独立生成或更新方向面掩码的CPU耗时总和。
    double faceSetBuildMilliseconds; // 按需将Root方向面掩码转换为兼容VoxelFaceSet的耗时。

    std::size_t occupancySourceRequestCount; // 各目标根累计请求的源根占用次数，重复请求分别计数。
    std::size_t uniqueOccupancyRootCount; // 当前批次去重后的源根索引数量，包括不存在的空根。
    std::size_t builtOccupancyRootCount; // 当前批次实际存在并建立核心占用数据的源根数量。
    std::size_t occupancyBuildCallCount; // 实际执行源根占用构建或更新调度的次数。
    std::size_t occupancyBuildTaskCount; // 全部调度请求的源根占用任务数量。
    std::size_t occupancyBuildParticipantCount; // 全部调度中实际领取过任务批次的参与线程记录数量。
    std::size_t occupancyBuildBatchSizeTotal; // 全部调度实际批次大小的累计值。
    std::size_t occupancyBuildSerialCallCount; // 退化为单参与线程的占用调度次数。
    std::size_t occupancyBuildNestedSerialCallCount; // 因位于同一线程池工作线程而串行执行的占用调度次数。
    std::size_t faceGenerationCallCount; // 实际执行目标Root面掩码生成或更新调度的次数。
    std::size_t faceGenerationTaskCount; // 全部调度请求的目标Root面掩码任务数量。
    std::size_t faceGenerationParticipantCount; // 全部调度中实际领取过任务批次的参与线程记录数量。
    std::size_t faceGenerationBatchSizeTotal; // 全部调度实际批次大小的累计值。
    std::size_t faceGenerationSerialCallCount; // 面掩码调度退化为单参与线程的次数。
    std::size_t faceGenerationNestedSerialCallCount; // 因位于同一线程池工作线程而串行执行的面掩码调度次数。
    std::size_t emittedFaceCount; // 掩码阶段识别出的单位外表面数量。
};

// 保存一个Root提取得到的方向面掩码和兼容面集合视图。
struct VoxelRootSurface
{
    VoxelRootSurface();

    VoxelRootSurface(const VoxelRootSurface&) = default;
    VoxelRootSurface& operator=(const VoxelRootSurface&) = default;

    // 移动构造和移动赋值手动实现，以兼容Visual Studio 2013。
    VoxelRootSurface(VoxelRootSurface&& other);
    VoxelRootSurface& operator=(VoxelRootSurface&& other);

    VoxelRootFaceMasks faceMasks; // 当前Root全部单位外表面的方向位平面。
    VoxelFaceSet faces; // 与faceMasks完全一致的排序单位面集合兼容视图。
};

// 从VoxelShape中提取最高层单位外表面集合。
//
// 每个面地址属于一个最高层材料体素，面方向为该材料体素的外法线方向。
// 根级提取只返回拥有者位于指定第0层根中的面，但会读取六个面邻根判断根边界表面是否暴露。
class VoxelFaceExtractor
{
public:
    /// 完整表面

    // 提取当前体素形体的全部最高层单位外表面。
    static VoxelFaceSet extract(const VoxelShape& shape);

    // 提取当前体素形体的全部最高层单位外表面，并返回阶段统计。
    static VoxelFaceSet extract(const VoxelShape& shape,
                                VoxelFaceExtractionStatistics* statistics);

    /// Root核心占用

    // 建立指定实际Root的最高层核心占用；Root不存在时返回false并清空结果。
    static bool extractRootOccupancy(const VoxelShape& shape,
                                     const VoxelCellIndex& rootIndex,
                                     VoxelRootOccupancy& occupancy);

    // 并行建立指定实际Root的最高层核心占用；不存在的空Root不写入结果映射。
    static void extractRootOccupancies(const VoxelShape& shape,
                                       const std::vector<VoxelCellIndex>& rootIndices,
                                       VoxelRootOccupancyMap& occupancies);

    // 并行建立指定实际Root的最高层核心占用，并返回调度统计。
    static void extractRootOccupancies(const VoxelShape& shape,
                                       const std::vector<VoxelCellIndex>& rootIndices,
                                       VoxelRootOccupancyMap& occupancies,
                                       VoxelFaceExtractionStatistics* statistics);

    // 根据变化掩码叶块从当前VoxelShape局部更新一个已缓存Root占用。
    static void updateRootOccupancy(const VoxelShape& shape,
                                    const VoxelChangeSet::DirtyCellRegion& dirtyRegion,
                                    VoxelRootOccupancy& occupancy);

    /// 根级表面

    // 提取拥有者位于指定第0层根中的全部最高层单位外表面。
    static VoxelFaceSet extractRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex);

    // 提取拥有者位于指定第0层根中的全部最高层单位外表面，并返回阶段统计。
    static VoxelFaceSet extractRoot(const VoxelShape& shape,
                                    const VoxelCellIndex& rootIndex,
                                    VoxelFaceExtractionStatistics* statistics);

    // 批量提取指定第0层根拥有的最高层单位外表面，结果顺序与rootIndices一致。
    //
    // 同一批次所需的源根最高层占用数据只展开一次。
    static void extractRoots(const VoxelShape& shape,
                             const std::vector<VoxelCellIndex>& rootIndices,
                             std::vector<VoxelFaceSet>& faceSets);

    // 批量提取指定第0层根拥有的最高层单位外表面，并返回共享占用和面生成统计。
    static void extractRoots(const VoxelShape& shape,
                             const std::vector<VoxelCellIndex>& rootIndices,
                             std::vector<VoxelFaceSet>& faceSets,
                             VoxelFaceExtractionStatistics* statistics);

    /// Root表面掩码

    // 提取指定Root的方向面掩码，不生成兼容面集合。
    static VoxelRootFaceMasks extractRootMasks(const VoxelShape& shape,
                                               const VoxelCellIndex& rootIndex);

    // 提取指定Root的方向面掩码并返回阶段统计，不生成兼容面集合。
    static VoxelRootFaceMasks extractRootMasks(const VoxelShape& shape,
                                               const VoxelCellIndex& rootIndex,
                                               VoxelFaceExtractionStatistics* statistics);

    // 批量提取Root方向面掩码，结果顺序与rootIndices一致。
    static void extractRootMasks(const VoxelShape& shape,
                                 const std::vector<VoxelCellIndex>& rootIndices,
                                 std::vector<VoxelRootFaceMasks>& faceMasks);

    // 批量提取Root方向面掩码并返回共享占用和掩码生成统计。
    static void extractRootMasks(const VoxelShape& shape,
                                 const std::vector<VoxelCellIndex>& rootIndices,
                                 std::vector<VoxelRootFaceMasks>& faceMasks,
                                 VoxelFaceExtractionStatistics* statistics);

    // 根据已缓存的全部实际Root占用并行生成指定Root方向面掩码。
    static void buildRootMasks(const VoxelRootOccupancyMap& occupancies,
                               const std::vector<VoxelCellIndex>& rootIndices,
                               std::vector<VoxelRootFaceMasks>& faceMasks,
                               VoxelFaceExtractionStatistics* statistics);


    /// Root面掩码字内核

    // 计算一个YZ行中的64位X区间在六个方向上的暴露面。
    static VoxelFaceMaskWord buildFaceMaskWord(
        const VoxelRootOccupancyNeighborhood& neighborhood,
        std::size_t localY,
        std::size_t localZ,
        std::size_t wordIndex);

    // 使用共享字内核完整生成指定Root方向面掩码。
    static VoxelRootFaceMasks buildRootFaceMasks(
        const VoxelCellIndex& rootIndex,
        std::size_t axisCellCount,
        const VoxelRootOccupancyNeighborhood& neighborhood);

    // 提取指定Root的方向面掩码和兼容面集合视图。
    static VoxelRootSurface extractRootSurface(const VoxelShape& shape,
                                               const VoxelCellIndex& rootIndex);

    // 提取指定Root的方向面掩码和兼容面集合视图，并返回阶段统计。
    static VoxelRootSurface extractRootSurface(const VoxelShape& shape,
                                               const VoxelCellIndex& rootIndex,
                                               VoxelFaceExtractionStatistics* statistics);

    // 批量提取Root方向面掩码和兼容面集合视图，结果顺序与rootIndices一致。
    static void extractRootSurfaces(const VoxelShape& shape,
                                    const std::vector<VoxelCellIndex>& rootIndices,
                                    std::vector<VoxelRootSurface>& surfaces);

    // 批量提取Root方向面掩码和兼容面集合视图，并返回共享占用和面生成统计。
    static void extractRootSurfaces(const VoxelShape& shape,
                                    const std::vector<VoxelCellIndex>& rootIndices,
                                    std::vector<VoxelRootSurface>& surfaces,
                                    VoxelFaceExtractionStatistics* statistics);

private:
    VoxelFaceExtractor() = delete;
};

}

#endif // MYVOXEL_SURFACE_VOXELFACEEXTRACTOR_H