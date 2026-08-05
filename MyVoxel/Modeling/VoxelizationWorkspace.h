#ifndef MYVOXEL_MODELING_VOXELIZATIONWORKSPACE_H
#define MYVOXEL_MODELING_VOXELIZATIONWORKSPACE_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelShape.h"

namespace MyVoxel
{
namespace Modeling
{

struct VoxelizationWorkspaceAccess;

// 保存连续刀位体素化过程中可以重复使用的根树节点池。
//
// 工作区持有当前体素化结果。返回的结果引用在下一次使用该工作区体素化、
// 调用reset或release前有效。一个工作区只能由一个同步执行流使用。
class VoxelizationWorkspace
{
public:
    VoxelizationWorkspace();

    /// 当前结果

    // 返回工作区当前持有的体素化结果。
    const VoxelShape& result() const;

    // 返回工作区是否已经绑定有效体素网格并生成或准备结果。
    bool isInitialized() const;

    /// 资源管理

    // 清空当前逻辑结果，将可复用根树节点池转入缓存并保留全部Chunk。
    void reset();

    // 释放当前结果和全部缓存根树持有的节点池资源。
    void release();

    /// 缓存统计

    // 返回当前处于空闲状态的可复用根树槽数量。
    std::size_t cachedRootSlotCount() const;

    // 返回当前活动结果和空闲缓存合计持有的根树槽数量。
    std::size_t retainedRootSlotCount() const;

    // 返回当前活动结果和空闲缓存合计持有的Chunk数量。
    std::size_t retainedChunkCount() const;

    // 返回当前活动结果和空闲缓存合计持有的节点池容量，单位为字节。
    std::size_t retainedStorageCapacityBytes() const;

    /// 运行统计

    // 返回相同根索引直接命中缓存的累计次数。
    std::size_t exactRootReuseCount() const;

    // 返回没有相同根索引时复用其他根节点池的累计次数。
    std::size_t fallbackRootReuseCount() const;

    // 返回没有可用缓存时新建根树节点池的累计次数。
    std::size_t createdRootTreeCount() const;

    // 清空工作区运行统计，不释放当前结果或缓存资源。
    void resetStatistics();

private:
    friend struct VoxelizationWorkspaceAccess;

    // 为下一次对齐体素化准备空结果，并回收可以安全复用的旧根树。
    void prepare(const VoxelShape& reference);

    // 优先取得上一刀相同根索引的可复用树，否则取得容量最大的空闲树。
    VoxelTree acquireRootTree(const VoxelCellIndex& rootIndex);

    // 将指定根树重置为空并加入可复用缓存。
    void recycleRootTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree);

    // 将当前独占结果中的全部根树移动到缓存。
    void recycleResultRoots();

private:
    bool m_initialized; // 当前结果是否已经绑定有效参考网格。
    VoxelShape m_result; // 当前工作区持有的活动体素化结果。
    std::vector<VoxelForest::TreeEntry> m_cachedRootTrees; // 当前空闲的可复用根树槽。
    std::size_t m_exactRootReuseCount; // 相同根索引直接复用节点池的累计次数。
    std::size_t m_fallbackRootReuseCount; // 不同根索引之间转移节点池的累计次数。
    std::size_t m_createdRootTreeCount; // 无缓存可用时新建根树的累计次数。
};

}
}

#endif // MYVOXEL_MODELING_VOXELIZATIONWORKSPACE_H