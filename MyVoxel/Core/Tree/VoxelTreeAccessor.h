#ifndef MYVOXEL_CORE_TREE_VOXELTREEACCESSOR_H
#define MYVOXEL_CORE_TREE_VOXELTREEACCESSOR_H

#include <cstddef>
#include <vector>

#include "../VoxelAddress.h"
#include "VoxelTreeCursor.h"

namespace MyVoxel
{

// 根据体素地址定位只读逻辑节点，并复用连续查询之间的公共访问路径。
//
// Accessor绑定一棵独立根树和对应的第0层根地址。
// 体素树发生任何修改后，必须调用reset()清除缓存路径。
class VoxelTreeAccessor
{
public:
    // 绑定指定根树、根地址和最高体素层级。
    VoxelTreeAccessor(const VoxelTree& tree, const VoxelCellAddress& rootAddress, VoxelLevel maximumLevel);

    /// 地址定位

    // 定位指定体素地址并返回其逻辑状态。
    //
    // 如果树在目标层级之前已经以Empty或Material结束，则返回继承状态，
    // 此时isExact()为false，resolvedAddress()返回实际终止祖先地址。
    VoxelState seek(const VoxelCellAddress& address);

    // 清除路径缓存并重新定位到根节点。
    void reset();

    /// 定位结果
    // 返回最近一次请求的目标地址。
    const VoxelCellAddress& targetAddress() const;
    // 返回最近一次实际定位到的节点地址。
    const VoxelCellAddress& resolvedAddress() const;

    // 返回当前目标地址的逻辑状态。
    VoxelState state() const;

    // 判断是否实际定位到了目标层级。
    bool isExact() const;
    // 判断目标状态是否继承自更高层的终止节点。
    bool isImplicit() const;
    // 返回当前实际定位到的只读游标。
    const VoxelTreeCursor& cursor() const;

    /// 缓存统计
    // 返回最近一次定位复用到的最高层级。
    VoxelLevel reusedLevel() const;
    // 返回最近一次定位中新访问的逻辑节点数量。
    std::size_t visitedNodeCount() const;
    // 返回当前缓存的逻辑节点数量，包含根节点。
    std::size_t cachedNodeCount() const;

private:
    // 保存一个已定位地址及其游标。
    struct PathEntry
    {
        PathEntry(const VoxelCellAddress& addressValue, const VoxelTreeCursor& cursorValue);

        VoxelCellAddress address; // 当前缓存节点地址。
        VoxelTreeCursor cursor; // 当前缓存节点游标。
    };

private:
    const VoxelTree* m_tree; // 当前绑定的只读体素树。
    VoxelCellAddress m_rootAddress; // 当前体素树对应的第0层根地址。
    VoxelCellAddress m_targetAddress; // 最近一次请求的目标地址。
    VoxelLevel m_maximumLevel; // 当前体素树允许访问的最高层级。

    std::vector<PathEntry> m_path; // 当前实际缓存的根到终止节点路径。
    std::vector<VoxelCorner> m_cachedCorners; // 当前缓存路径中的角点序列。
    std::vector<VoxelCorner> m_targetCorners; // 最近一次目标地址对应的完整角点序列。

    VoxelState m_state; // 最近一次目标地址的逻辑状态。
    VoxelLevel m_reusedLevel; // 最近一次定位复用到的最高层级。
    std::size_t m_visitedNodeCount; // 最近一次定位中新访问的节点数量。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREEACCESSOR_H