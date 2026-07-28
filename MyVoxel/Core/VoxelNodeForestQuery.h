#ifndef MYVOXEL_VOXELNODEFORESTQUERY_H
#define MYVOXEL_VOXELNODEFORESTQUERY_H

#include <cstdint>

#include "VoxelAddress.h"
#include "VoxelTypes.h"

namespace MyVoxel
{

class VoxelNode;
class VoxelNodeForest;
class VoxelNodePool;

// 表示体素形体局部坐标系中的轴对齐包围盒。
struct VoxelLocalBox
{
    VoxelLocalBox() = default;
    VoxelLocalBox(double minimumXValue, double minimumYValue, double minimumZValue, double maximumXValue, double maximumYValue, double maximumZValue);

    double minimumX = 0.0; // X方向最小坐标。
    double minimumY = 0.0; // Y方向最小坐标。
    double minimumZ = 0.0; // Z方向最小坐标。
    double maximumX = 0.0; // X方向最大坐标。
    double maximumY = 0.0; // Y方向最大坐标。
    double maximumZ = 0.0; // Z方向最大坐标。
};

// 表示查询区域与森林材料区域之间的关系。
enum class VoxelRegionRelation
{
    Outside, // 查询区域与材料完全不相交。
    Intersecting, // 查询区域与材料可能部分相交。
    Inside // 查询区域完全位于一个材料节点内。
};

// 记录森林区域查询过程中的节点访问数量。
struct VoxelNodeForestQueryStatistics
{
    // 清空全部查询统计。
    void reset();

    // 累加另一次森林查询的统计数据。
    void accumulate(const VoxelNodeForestQueryStatistics& other);

    std::uint64_t rootCandidateCount = 0; // 查询包围盒覆盖的第0层候选根节点数量。
    std::uint64_t existingRootCount = 0; // 候选范围内实际存在的根节点数量。
    std::uint64_t nodeBoundsTestCount = 0; // 节点包围盒相交测试数量。
    std::uint64_t visitedNodeCount = 0; // 实际访问的相交节点数量。
    std::uint64_t emptyNodeCount = 0; // 查询过程中遇到的空节点数量。
    std::uint64_t materialNodeCount = 0; // 查询过程中遇到的材料节点数量。
    std::uint64_t subdividedNodeCount = 0; // 查询过程中遇到的已细分节点数量。
};

// 使用节点森林原有层级结构执行只读空间区域查询。
class VoxelNodeForestQuery
{
public:
    // 绑定指定节点森林和第0层体素边长。
    VoxelNodeForestQuery(const VoxelNodeForest& forest, double baseVoxelEdgeLength);

    // 返回指定局部包围盒与森林材料区域之间的保守关系。
    VoxelRegionRelation classify(const VoxelLocalBox& bounds) const;

    // 返回指定局部包围盒与森林材料区域之间的保守关系，并记录查询统计。
    VoxelRegionRelation classify(const VoxelLocalBox& bounds, VoxelNodeForestQueryStatistics& statistics) const;

private:
    VoxelRegionRelation classifyImpl(const VoxelLocalBox& bounds, VoxelNodeForestQueryStatistics* statistics) const;
    VoxelRegionRelation classifyNode(const VoxelLocalBox& bounds, const VoxelLocalBox& nodeBounds, const VoxelNode& node, const VoxelNodePool& nodePool, VoxelNodeForestQueryStatistics* statistics) const;

    const VoxelNodeForest* m_forest = nullptr; // 被查询的只读节点森林。
    double m_baseVoxelEdgeLength = 1.0; // 第0层体素边长。
};

}

#endif // MYVOXEL_VOXELNODEFORESTQUERY_H