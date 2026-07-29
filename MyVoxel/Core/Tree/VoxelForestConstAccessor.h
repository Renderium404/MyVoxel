#ifndef MYVOXEL_CORE_TREE_VOXELFORESTCONSTACCESSOR_H
#define MYVOXEL_CORE_TREE_VOXELFORESTCONSTACCESSOR_H

#include <cstdint>
#include <vector>

#include "VoxelForest.h"

namespace MyVoxel
{

// 记录只读森林访问器的根树缓存和节点路径复用统计。
struct VoxelForestConstAccessorStatistics
{
    VoxelForestConstAccessorStatistics();

    // 清空全部统计数据。
    void clear();

    std::uint64_t rootCacheHitCount; // 查询命中最近根树缓存的次数。
    std::uint64_t rootCacheMissCount; // 查询未命中最近根树缓存的次数。
    std::uint64_t pathReuseCount; // 查询复用至少一级节点路径的次数。
    std::uint64_t reusedPathLevelCount; // 累计复用的节点路径层级数量。
    std::uint64_t nodeVisitCount; // 查询过程中实际检查节点状态的次数。
};

// 缓存最近根树和节点路径，用于连续执行只读节点状态查询。
class VoxelForestConstAccessor
{
public:
    // 绑定指定只读森林，访问器使用期间森林不能被修改。
    explicit VoxelForestConstAccessor(const VoxelForest& forest);

    /// 节点查询
    // 返回指定地址表示的节点状态，并尝试复用最近根树和节点路径。
    VoxelState state(const VoxelCellAddress& address);

    /// 缓存与统计
    // 清除当前根树缓存、节点路径缓存和全部统计数据。
    void clear();

    // 返回当前访问器统计数据。
    const VoxelForestConstAccessorStatistics& statistics() const;

private:
    const VoxelForest* m_forest; // 当前绑定的只读体素森林。
    const VoxelTree* m_cachedTree; // 最近查询使用的体素树，不存在对应树时为空。
    VoxelCellIndex m_cachedRootIndex; // 最近查询使用的第0层根体素索引。
    bool m_hasCachedRoot; // 当前是否已经缓存过根体素查询结果。
    std::vector<VoxelCorner> m_cachedCorners; // 最近查询缓存的根节点到目标节点角点路径。
    std::vector<const VoxelNode*> m_cachedNodes; // 最近查询缓存的实际节点路径，首项为根节点。
    VoxelForestConstAccessorStatistics m_statistics; // 当前访问器统计数据。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELFORESTCONSTACCESSOR_H