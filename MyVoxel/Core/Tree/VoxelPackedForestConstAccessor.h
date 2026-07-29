#ifndef MYVOXEL_VOXELPACKEDFORESTCONSTACCESSOR_H
#define MYVOXEL_VOXELPACKEDFORESTCONSTACCESSOR_H

#include <cstdint>
#include <vector>

#include "VoxelPackedForest.h"
#include "VoxelPackedTreeConstCursor.h"

namespace MyVoxel
{

// 记录Packed森林只读访问器的根缓存和路径复用情况。
struct VoxelPackedForestConstAccessorStatistics
{
    // 清空全部访问统计。
    void reset()
    {
        *this = VoxelPackedForestConstAccessorStatistics();
    }

    std::uint64_t rootCacheHitCount = 0; // 查询地址与上次查询位于同一根树的次数。
    std::uint64_t rootCacheMissCount = 0; // 查询地址切换根树或首次查询的次数。
    std::uint64_t pathReuseCount = 0; // 查询复用了至少一级已缓存路径的次数。
    std::uint64_t reusedPathLevelCount = 0; // 全部查询累计复用的路径层级数量。
    std::uint64_t nodeVisitCount = 0; // 查询过程中实际读取节点状态的次数。
};

// 通过缓存最近根树和从根到目标节点的游标路径加速连续只读状态查询。
class VoxelPackedForestConstAccessor
{
public:
    // 绑定指定只读Packed森林，访问器生命周期内森林拓扑不得修改。
    explicit VoxelPackedForestConstAccessor(const VoxelPackedForest& forest);

    // 返回指定地址当前对应的节点状态。
    VoxelState state(const VoxelCellAddress& address);

    // 清除全部根缓存、路径缓存和访问统计。
    void clear();

    // 返回当前累计访问统计。
    const VoxelPackedForestConstAccessorStatistics& statistics() const;

private:
    // 判断两个第0层根索引是否一致。
    static bool sameIndex(const VoxelCellIndex& first, const VoxelCellIndex& second);

    const VoxelPackedForest* m_forest; // 当前绑定的只读Packed森林。
    VoxelCellIndex m_cachedRootIndex; // 最近一次查询对应的根索引。
    const VoxelPackedRootTree* m_cachedTree; // 最近一次查询对应的根树，不存在时为空。
    bool m_hasCachedRoot; // 当前是否已经缓存过一个根索引。
    std::vector<VoxelCorner> m_cachedCorners; // 最近查询从根节点到目标节点的角点路径。
    std::vector<VoxelPackedTreeConstCursor> m_cachedCursors; // 最近路径中从根到各层节点的游标。
    VoxelPackedForestConstAccessorStatistics m_statistics; // 当前累计访问统计。
};

}

#endif // MYVOXEL_VOXELPACKEDFORESTCONSTACCESSOR_H