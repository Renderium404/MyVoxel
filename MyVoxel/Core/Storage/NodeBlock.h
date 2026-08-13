#ifndef MYVOXEL_CORE_STORAGE_NODEBLOCK_H
#define MYVOXEL_CORE_STORAGE_NODEBLOCK_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

// 一个四字节节点数据槽保存子表首索引或非分化节点数值，实际语义由父节点记录的状态决定。
union NodeData
{
    std::uint32_t rawValue;     // 原始四字节数据。
    VoxelIndex firstChildIndex; // 分化节点八个连续子VoxelBlock槽的首索引。
    float value;                // 非分化节点保存的数值。
};

static_assert(sizeof(NodeData) == 4, "NodeData must be exactly 4 bytes.");

// 一个NodeBlock既可表示非分化节点，也可表示具有八个直接子项的分化节点。
//
// 当前NodeBlock自身的类型由父节点状态决定：
// Empty / Material：仅nodeData.value有效。
// Node：mChildMask、mValueMask和nodeData.firstChildIndex有效。
//
// Node状态下两个八位掩码保存八个直接子项状态：
//
// mChildMask mValueMask 状态        对应VoxelBlock数据
//     0          0     Empty       NodeBlock::nodeData.value
//     0          1     Material    NodeBlock::nodeData.value
//     1          0     Node        NodeBlock
//     1          1     MaskLeaf    IndexBlock
//
// 分化节点建立子表后始终拥有连续八个VoxelBlock物理槽。
// mChildMask和mValueMask只解释八个子项状态，不负责表示物理槽是否存在。
// 八个直接子项的物理槽连续保存，首索引由nodeData.firstChildIndex记录。
struct NodeBlock
{
    std::uint8_t mChildMask; // 分化节点使用，记录八个直接子项是否继续分化。
    std::uint8_t mValueMask; // 分化节点使用，与mChildMask组合表示四种子项状态。
    std::uint16_t reserved;  // 保留字段。
    NodeData nodeData;       // 分化节点保存子表首索引，非分化节点保存数值。
};

static_assert(sizeof(NodeBlock) == 8, "NodeBlock must be exactly 8 bytes.");

// 保存一个逻辑叶节点对应的统一叶存储索引。
// 同一个leafIndex同时定位对应的MaskBlock和LeafBlock，两者必须具有一致生命周期。
struct IndexBlock
{
    VoxelIndex leafIndex;   // 同时定位对应MaskBlock和LeafBlock的统一叶存储索引。
    std::uint32_t reserved; // 保留字段。
};

static_assert(sizeof(IndexBlock) == 8, "IndexBlock must be exactly 8 bytes.");

// 一个八字节物理槽保存非分化节点、普通分化节点或叶索引，实际类型由父节点记录的状态决定。
union VoxelBlock
{
    VoxelBlock()
        : rawValue(0)
    {
    }

    std::uint64_t rawValue; // 原始八字节数据。
    NodeBlock node;         // Empty、Material或Node状态下使用。
    IndexBlock index;       // MaskLeaf状态下使用。
};

static_assert(sizeof(VoxelBlock) == 8, "VoxelBlock must be exactly 8 bytes.");

/// 块初始化

// 将普通分化节点初始化为八个统一Empty或Material子项，子表尚未分配。
inline void reset(NodeBlock& block, VoxelNodeState childState = VoxelNodeState::Empty)
{
    assert(childState == VoxelNodeState::Empty || childState == VoxelNodeState::Material);
    block.mChildMask = 0;
    block.mValueMask = childState == VoxelNodeState::Material ? static_cast<std::uint8_t>(0xFFU) : static_cast<std::uint8_t>(0);
    block.reserved = 0;
    block.nodeData.firstChildIndex = InvalidVoxelIndex;
}

// 将叶索引块恢复为未引用任何叶数据的状态。
inline void reset(IndexBlock& block)
{
    block.leafIndex = InvalidVoxelIndex;
    block.reserved = 0;
}

// 将物理槽清零，清零结果本身不表示任何逻辑状态。
inline void reset(VoxelBlock& block)
{
    block.rawValue = 0;
}

/// 分化节点子表

// 判断分化节点是否已经分配完整的八槽子表。
inline bool hasChildStorage(const NodeBlock& block)
{
    return block.nodeData.firstChildIndex != InvalidVoxelIndex;
}

// 设置分化节点完整八槽子表的首索引。
inline void setChildStorage(NodeBlock& block, VoxelIndex firstChildIndex)
{
    assert(firstChildIndex != InvalidVoxelIndex);
    block.nodeData.firstChildIndex = firstChildIndex;
}

// 清除分化节点对子表的引用，调用者必须已经释放对应物理存储。
inline void clearChildStorage(NodeBlock& block)
{
    block.nodeData.firstChildIndex = InvalidVoxelIndex;
}

// 返回指定直接子项对应的VoxelBlock物理槽索引。
inline VoxelIndex childStorageIndex(const NodeBlock& block, VoxelCorner corner)
{
    assert(hasChildStorage(block));
    const unsigned int cornerIndex = static_cast<unsigned int>(corner);
    assert(cornerIndex < static_cast<unsigned int>(VoxelCornerCount));
    return block.nodeData.firstChildIndex + static_cast<VoxelIndex>(cornerIndex);
}

/// 非分化节点数值

// 返回非分化节点保存的数值，调用者必须保证当前NodeBlock由父节点定义为Empty或Material。
inline float nodeValue(const NodeBlock& block)
{
    return block.nodeData.value;
}

// 设置非分化节点保存的数值，调用者必须保证当前NodeBlock由父节点定义为Empty或Material。
inline void setNodeValue(NodeBlock& block, float value)
{
    block.nodeData.value = value;
}

/// 叶索引状态

// 判断叶索引块是否引用有效的统一叶存储位置。
inline bool hasLeafStorage(const IndexBlock& block)
{
    return block.leafIndex != InvalidVoxelIndex;
}

// 设置叶索引块引用的统一叶存储位置。
inline void setLeafStorage(IndexBlock& block, VoxelIndex leafIndex)
{
    assert(leafIndex != InvalidVoxelIndex);
    block.leafIndex = leafIndex;
}

// 清除叶索引块对统一叶存储位置的引用，调用者必须已经释放对应叶数据。
inline void clearLeafStorage(IndexBlock& block)
{
    block.leafIndex = InvalidVoxelIndex;
}

}

#endif // MYVOXEL_CORE_STORAGE_NODEBLOCK_H