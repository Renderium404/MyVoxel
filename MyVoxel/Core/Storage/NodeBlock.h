#ifndef MYVOXEL_CORE_STORAGE_NODEBLOCK_H
#define MYVOXEL_CORE_STORAGE_NODEBLOCK_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

// 使用两个八位掩码保存一个普通节点的八个直接子节点状态。
//
// storageMask leafMask 状态
//     0          0     Empty
//     0          1     Material
//     1          0     Branch
//     1          1     Leaf
//
// storageMask位为1时，对应子节点占用VoxelBlock物理槽。
// 八个直接子节点的物理槽连续保存，首索引为firstChildIndex。
struct NodeBlock
{
    std::uint8_t storageMask;   // 各子节点是否占用对应VoxelBlock物理槽。
    std::uint8_t leafMask;      // 与storageMask组合表示各子节点的四种物理状态。
    std::uint16_t reserved;     // 保留字段。
    VoxelIndex firstChildIndex; // 八个连续子VoxelBlock槽的首索引。
};

static_assert(sizeof(NodeBlock) == 8, "NodeBlock must be exactly 8 bytes.");

// 保存一个逻辑叶节点对应的统一叶存储索引。
// 同一个leafIndex同时定位对应的MaskBlock和LeafBlock，两者必须具有一致生命周期。
struct IndexBlock
{
    VoxelIndex leafIndex;     // 同时定位对应MaskBlock和LeafBlock的统一叶存储索引。
    std::uint32_t reserved;   // 保留字段。
};

static_assert(sizeof(IndexBlock) == 8, "IndexBlock must be exactly 8 bytes.");

// 一个8字节物理槽保存普通节点或叶索引，实际类型由父节点记录的状态决定。
union VoxelBlock
{
    VoxelBlock()
        : rawValue(0)
    {
    }

    std::uint64_t rawValue; // 原始八字节数据。
    NodeBlock node;         // Branch状态下使用。
    IndexBlock index;       // Leaf状态下使用。
};

static_assert(sizeof(VoxelBlock) == 8, "VoxelBlock must be exactly 8 bytes.");

/// 块初始化

// 将普通节点的八个直接子节点统一设置为空或材料状态。
inline void reset(NodeBlock& block, VoxelNodeState childState = VoxelNodeState::Empty)
{
    assert(childState == VoxelNodeState::Empty || childState == VoxelNodeState::Material);
    block.storageMask = 0;
    block.leafMask = childState == VoxelNodeState::Material ? static_cast<std::uint8_t>(0xFFU) : static_cast<std::uint8_t>(0);
    block.reserved = 0;
    block.firstChildIndex = InvalidVoxelIndex;
}

// 将叶索引块恢复为未引用任何叶数据的状态。
inline void reset(IndexBlock& block)
{
    block.leafIndex = InvalidVoxelIndex;
    block.reserved = 0;
}

/// 八位子节点掩码

// 返回指定角点在八位子节点掩码中的位。
inline std::uint8_t cornerBit(VoxelCorner corner)
{
    const unsigned int cornerIndex = static_cast<unsigned int>(corner);
    assert(cornerIndex < static_cast<unsigned int>(VoxelCornerCount));
    return static_cast<std::uint8_t>(1U << cornerIndex);
}

// 返回八位掩码中指定角点的值。
inline bool maskBit(std::uint8_t mask, VoxelCorner corner)
{
    return (mask & cornerBit(corner)) != 0;
}

// 设置八位掩码中指定角点的值。
inline void setMaskBit(std::uint8_t& mask, VoxelCorner corner, bool value)
{
    const std::uint8_t bit = cornerBit(corner);
    if (value)
    {
        mask = static_cast<std::uint8_t>(mask | bit);
    }
    else
    {
        mask = static_cast<std::uint8_t>(mask & static_cast<std::uint8_t>(~bit));
    }
}

/// 普通节点状态

// 返回普通节点中指定直接子节点的物理状态。
inline VoxelNodeState nodeState(const NodeBlock& block, VoxelCorner corner)
{
    const std::uint8_t storageValue = maskBit(block.storageMask, corner) ? static_cast<std::uint8_t>(2) : static_cast<std::uint8_t>(0);
    const std::uint8_t leafValue = maskBit(block.leafMask, corner) ? static_cast<std::uint8_t>(1) : static_cast<std::uint8_t>(0);
    return static_cast<VoxelNodeState>(storageValue | leafValue);
}

// 设置普通节点中指定直接子节点的物理状态。
inline void setNodeState(NodeBlock& block, VoxelCorner corner, VoxelNodeState state)
{
    const unsigned int stateValue = static_cast<unsigned int>(state);
    assert(stateValue <= 3U);
    setMaskBit(block.storageMask, corner, (stateValue & 2U) != 0);
    setMaskBit(block.leafMask, corner, (stateValue & 1U) != 0);
}

// 返回指定直接子节点是否占用VoxelBlock物理槽。
inline bool hasChildStorage(const NodeBlock& block, VoxelCorner corner)
{
    return maskBit(block.storageMask, corner);
}

// 返回指定直接子节点对应的VoxelBlock物理槽索引。
inline VoxelIndex childStorageIndex(const NodeBlock& block, VoxelCorner corner)
{
    assert(block.firstChildIndex != InvalidVoxelIndex);
    assert(hasChildStorage(block, corner));
    return block.firstChildIndex + static_cast<VoxelIndex>(corner);
}

/// 叶索引状态

// 判断叶索引块是否引用有效的统一叶存储位置。
inline bool hasLeafStorage(const IndexBlock& block)
{
    return block.leafIndex != InvalidVoxelIndex;
}

}

#endif // MYVOXEL_CORE_STORAGE_NODEBLOCK_H