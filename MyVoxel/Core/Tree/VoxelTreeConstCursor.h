#ifndef MYVOXEL_VOXELTREECONSTCURSOR_H
#define MYVOXEL_VOXELTREECONSTCURSOR_H

#include "VoxelForest.h"
#include "VoxelPackedTreeConstCursor.h"

namespace MyVoxel
{

// 使用生产接口只读访问Packed体素树节点。
class VoxelTreeConstCursor
{
public:
    // 使用森林中已经存在的节点创建只读游标。
    VoxelTreeConstCursor(const VoxelForest& forest, const VoxelCellAddress& address);

/// 节点状态

// 返回当前节点状态。
VoxelState state() const;

// 检查当前节点是否直接对应一个掩码叶块。
bool isMaskLeaf() const;

// 检查当前节点是否允许继续访问逻辑子节点。
bool canAccessChildren() const;

/// 子节点访问

// 返回指定角点对应的直接子节点游标。
VoxelTreeConstCursor child(VoxelCorner corner) const;

/// 掩码叶块访问

// 返回当前节点直接对应的只读掩码叶块。
const VoxelLeafBlock& maskLeaf() const;


private:
    // 使用已经定位的Packed游标创建生产子节点游标。
    explicit VoxelTreeConstCursor(const VoxelPackedTreeConstCursor& cursor);

    // 返回森林中指定地址对应的Packed游标。
    static VoxelPackedTreeConstCursor createCursor(const VoxelForest& forest, const VoxelCellAddress& address);

    VoxelPackedTreeConstCursor m_cursor; // 当前实际读取节点状态的Packed游标。
};

}

#endif // MYVOXEL_VOXELTREECONSTCURSOR_H