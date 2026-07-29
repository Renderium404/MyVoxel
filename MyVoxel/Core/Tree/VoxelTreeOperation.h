#ifndef MYVOXEL_CORE_TREE_VOXELTREEOPERATION_H
#define MYVOXEL_CORE_TREE_VOXELTREEOPERATION_H

#include <cstddef>
#include <cstdint>
#include <functional>

#include "../VoxelAddress.h"
#include "VoxelTreeCursor.h"

namespace MyVoxel
{

// 保存一棵根体素树的压缩逻辑结构统计。
struct VoxelTreeStatistics
{
    std::uint64_t logicalNodeCount = 0; // 实际遍历到的逻辑节点数量。
    std::uint64_t emptyNodeCount = 0; // 压缩树中以Empty终止的逻辑节点数量。
    std::uint64_t materialNodeCount = 0; // 压缩树中以Material终止的逻辑节点数量。
    std::uint64_t subdividedNodeCount = 0; // 压缩树中允许继续访问子节点的逻辑节点数量。
    std::uint64_t finestMaterialVoxelCount = 0; // 展开到最高层后对应的材料体素数量。
    bool finestMaterialVoxelCountSaturated = false; // 材料体素数量是否发生无符号64位饱和。
    VoxelLevel deepestVisitedLevel = BaseVoxelLevel; // 实际访问到的最深体素层级。

    std::size_t allocatedGroupCount = 0; // 节点池当前已分配的八槽组数量。
    std::size_t storageGroupCount = 0; // 节点池曾经创建的八槽组数量。
    std::size_t chunkCount = 0; // 节点池当前Chunk数量。
};

// 提供独立根体素树的通用查询、修改、遍历、统计和规范化操作。
class VoxelTreeOperation
{
public:
    using MaterialCellVisitor = std::function<void(const VoxelCellAddress&)>;

    /// 整树状态

    // 将整棵树设置为空并释放全部物理后代。
    static void clear(VoxelTree& tree);

    // 将整棵树设置为完全包含材料并释放全部物理后代。
    static void fill(VoxelTree& tree);

    /// 地址访问

    // 返回指定地址对应的逻辑状态。
    static VoxelCursorState stateAt(const VoxelTree& tree, const VoxelCellAddress& rootAddress,
                                    VoxelLevel maximumLevel, const VoxelCellAddress& address);

    // 将指定地址设置为Empty或Material，必要时创建中间普通分支。
    //
    // 该接口只负责修改目标节点，不自动执行祖先合并。
    static void setCellState(VoxelTree& tree, const VoxelCellAddress& rootAddress,
                             VoxelLevel maximumLevel, const VoxelCellAddress& address, VoxelState state);

    /// 树规范化

    // 递归合并八个子节点状态完全一致的分支，返回实际合并的节点数量。
    static std::size_t normalize(VoxelTree& tree, VoxelLevel maximumLevel);

    /// 树遍历

    // 遍历压缩树中的全部Material终止节点。
    //
    // 一个Material节点可能表示多个最高层体素，visitor接收的是其压缩地址。
    static void forEachMaterialCell(const VoxelTree& tree, const VoxelCellAddress& rootAddress,
                                    VoxelLevel maximumLevel, const MaterialCellVisitor& visitor);

    /// 树统计

    // 收集压缩逻辑节点、最高层材料体素和节点池统计。
    static VoxelTreeStatistics statistics(const VoxelTree& tree, const VoxelCellAddress& rootAddress,
                                          VoxelLevel maximumLevel);

private:
    VoxelTreeOperation() = delete;
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREEOPERATION_H