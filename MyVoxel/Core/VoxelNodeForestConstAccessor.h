#ifndef MYVOXEL_VOXELNODEFORESTCONSTACCESSOR_H
#define MYVOXEL_VOXELNODEFORESTCONSTACCESSOR_H

#include <cstdint>
#include <vector>

#include "VoxelNodeForest.h"

namespace MyVoxel
{

// 记录只读节点访问器的根节点缓存和路径复用统计。
struct VoxelNodeForestConstAccessorStatistics
{
    std::uint64_t rootCacheHitCount = 0; // 查询命中最近根节点缓存的次数。
    std::uint64_t rootCacheMissCount = 0; // 查询未命中最近根节点缓存的次数。
    std::uint64_t pathReuseCount = 0; // 查询复用至少一级下级路径的次数。
    std::uint64_t reusedPathLevelCount = 0; // 累计复用的下级路径层级数量。
    std::uint64_t nodeVisitCount = 0; // 查询实际检查节点状态的次数。
};

// 缓存最近根树和节点路径，用于连续只读点状态查询。
class VoxelNodeForestConstAccessor
{
public:
    // 绑定指定只读节点森林。
    explicit VoxelNodeForestConstAccessor(const VoxelNodeForest& forest);

    // 返回指定地址对应的节点状态，并复用最近查询路径。
    VoxelState state(const VoxelCellAddress& address);

    // 清除缓存和统计数据。
    void clear();

    // 返回当前访问器统计数据。
    const VoxelNodeForestConstAccessorStatistics& statistics() const;

private:
    // 检查两个根节点索引是否相同。
    static bool sameIndex(const VoxelCellIndex& first, const VoxelCellIndex& second);

    const VoxelNodeForest* m_forest; // 被访问的只读节点森林。
    const VoxelRootTree* m_cachedTree; // 最近查询使用的独立根树，不存在根节点时为空。
    VoxelCellIndex m_cachedRootIndex; // 最近查询的第0层根节点索引。
    bool m_hasCachedRoot; // 当前是否保存了根节点查询结果。
    std::vector<VoxelCorner> m_cachedCorners; // 最近查询实际缓存的根到节点角点路径。
    std::vector<const VoxelNode*> m_cachedNodes; // 最近查询缓存的各层节点，首项为根节点。
    VoxelNodeForestConstAccessorStatistics m_statistics; // 当前访问器统计数据。
};

}

#endif // MYVOXEL_VOXELNODEFORESTCONSTACCESSOR_H
