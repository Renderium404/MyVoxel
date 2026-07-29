#ifndef MYVOXEL_VOXELREGIONQUERY_H
#define MYVOXEL_VOXELREGIONQUERY_H

#include <cstdint>

namespace MyVoxel
{

// 表示查询区域与体素森林材料区域之间的关系。
enum class VoxelRegionRelation
{
    Outside,
    Intersecting,
    Inside
};

// 记录体素森林区域查询过程中的节点访问数量。
struct VoxelForestQueryStatistics
{
    // 清空全部查询统计。
    void reset()
    {
        *this = VoxelForestQueryStatistics();
    }

    // 累加另一次森林查询的统计数据。
    void accumulate(const VoxelForestQueryStatistics& other)
    {
        rootCandidateCount += other.rootCandidateCount;
        existingRootCount += other.existingRootCount;
        nodeBoundsTestCount += other.nodeBoundsTestCount;
        visitedNodeCount += other.visitedNodeCount;
        emptyNodeCount += other.emptyNodeCount;
        materialNodeCount += other.materialNodeCount;
        subdividedNodeCount += other.subdividedNodeCount;
    }

    std::uint64_t rootCandidateCount = 0; // 查询包围盒覆盖的第0层候选根节点数量。
    std::uint64_t existingRootCount = 0; // 候选范围内实际存在的根节点数量。
    std::uint64_t nodeBoundsTestCount = 0; // 节点包围盒相交测试数量。
    std::uint64_t visitedNodeCount = 0; // 实际访问的相交节点数量。
    std::uint64_t emptyNodeCount = 0; // 查询过程中遇到的空节点数量。
    std::uint64_t materialNodeCount = 0; // 查询过程中遇到的材料节点数量。
    std::uint64_t subdividedNodeCount = 0; // 查询过程中遇到的已细分节点数量。
};

}

#endif // MYVOXEL_VOXELREGIONQUERY_H