#ifndef MYVOXEL_CORE_QUERY_VOXELFORESTQUERY_H
#define MYVOXEL_CORE_QUERY_VOXELFORESTQUERY_H

#include <cstdint>

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelGrid.h"

namespace MyVoxel
{

// 表示查询包围盒与体素材料区域之间的保守空间关系。
enum class VoxelRegionRelation : std::uint8_t
{
    Outside = 0, // 查询包围盒与材料区域不存在体积相交。
    Intersecting = 1, // 查询包围盒与材料区域部分相交，或无法确认是否完全位于材料内部。
    Inside = 2 // 查询包围盒完全位于一个材料叶节点内部。
};

// 记录森林区域查询过程中的根树候选和节点访问数量。
struct VoxelForestQueryStatistics
{
    VoxelForestQueryStatistics();

    // 清空全部查询统计数据。
    void reset();

    // 累加另一次森林查询的全部统计数据。
    void accumulate(const VoxelForestQueryStatistics& other);

    std::uint64_t rootCandidateCount; // 查询包围盒覆盖的第0层候选根体素数量。
    std::uint64_t existingRootCount; // 候选范围内实际存在的体素树数量。
    std::uint64_t nodeBoundsTestCount; // 节点包围盒体积相交测试数量。
    std::uint64_t visitedNodeCount; // 查询过程中实际访问的节点数量。
    std::uint64_t emptyNodeCount; // 查询过程中访问的空节点数量。
    std::uint64_t materialNodeCount; // 查询过程中访问的材料叶节点数量。
    std::uint64_t subdividedNodeCount; // 查询过程中访问的已细分节点数量。
};

// 使用VoxelGrid空间映射和VoxelForest层级结构执行只读包围盒查询。
class VoxelForestQuery
{
public:
    // 绑定指定只读森林并复制其空间查询使用的体素网格。
    VoxelForestQuery(const VoxelForest& forest, const VoxelGrid& grid);

    // 返回指定有体积包围盒与森林材料区域之间的保守关系。
    VoxelRegionRelation classify(const Bounds3& bounds) const;

    // 返回指定有体积包围盒与森林材料区域之间的保守关系，并记录查询统计。
    VoxelRegionRelation classify(const Bounds3& bounds, VoxelForestQueryStatistics& statistics) const;

private:
    // 执行统一查询实现，statistics为空时不记录统计数据。
    VoxelRegionRelation classifyImpl(const Bounds3& bounds, VoxelForestQueryStatistics* statistics) const;

    // 递归查询指定节点及其空间包围盒与目标包围盒之间的保守关系。
    VoxelRegionRelation classifyNode(const Bounds3& bounds, const Bounds3& nodeBounds, const VoxelNode& node, const VoxelNodePool& nodePool, VoxelForestQueryStatistics* statistics) const;

private:
    const VoxelForest* m_forest; // 当前查询使用的只读体素森林。
    VoxelGrid m_grid; // 当前查询使用的体素空间映射。
};

}

#endif // MYVOXEL_CORE_QUERY_VOXELFORESTQUERY_H