#ifndef MYVOXEL_CORE_STORAGE_VOXELBLOCK_H
#define MYVOXEL_CORE_STORAGE_VOXELBLOCK_H

#include <cassert>
#include <cstdint>

#include "../VoxelTypes.h"

namespace MyVoxel
{

// 一个VoxelNodeBlock使用两个八位掩码记录八个子节点的状态。
//
// storageMask leafMask 状态
//     0          0     Empty
//     0          1     Material
//     1          0     Branch
//     1          1     MaskLeaf
//
// 指定角点的状态值为：
// (storageMask位 << 1) | leafMask位
//
// storageMask位为1时，对应子节点占用物理存储槽，槽索引为firstChildIndex + 角点值。
// storageMask位为0时，对应子节点状态完全保存在两个掩码中，不访问物理存储槽。
struct VoxelNodeBlock
{
    std::uint8_t storageMask = 0; // 各子节点是否占用对应物理存储槽。
    std::uint8_t leafMask = 0; // 与storageMask组合表示各子节点的四种状态。
    std::uint16_t userData = 0; // 预留给上层模块使用的节点数据。
    VoxelIndex firstChildIndex = InvalidVoxelIndex; // 八个连续子节点槽的首索引。
};

static_assert(sizeof(VoxelNodeBlock) == 8, "VoxelNodeBlock must be exactly 8 bytes.");

// 使用一个64位掩码保存连续两层共4×4×4个最高层体素的材料状态。
//
// 64位掩码按粗层角点划分为八组，每组包含对应粗层体素的八个细层体素：
//
// 位范围    粗层角点
// [0, 7]    Minimum
// [8, 15]   MaximumX
// [16, 23]  MaximumY
// [24, 31]  MaximumXY
// [32, 39]  MaximumZ
// [40, 47]  MaximumXZ
// [48, 55]  MaximumYZ
// [56, 63]  MaximumXYZ
//
// 每组内部仍按照VoxelCorner数值排列，第0位为Minimum，第7位为MaximumXYZ。
// 位为1表示材料存在，位为0表示材料不存在。
// 任意最高层体素对应的位索引为coarseCorner×8+fineCorner。
struct VoxelLeafBlock
{
    std::uint64_t materialMask = 0; // 两级角点按coarseCorner×8+fineCorner映射到64位材料状态。
};

static_assert(sizeof(VoxelLeafBlock) == 8, "VoxelLeafBlock must be exactly 8 bytes.");
// 保存一个8字节普通节点块或掩码叶块，实际类型由父节点状态决定。
union VoxelBlock
{
    // 默认激活原始数据成员并清零。
    VoxelBlock()
        : rawValue(0)
    {
    }
    std::uint64_t rawValue;         // 原始八字节数据。
    VoxelNodeBlock nodeBlock;       // 普通分支节点数据。
    VoxelLeafBlock leafBlock;       // 掩码叶节点数据。
};
static_assert(sizeof(VoxelBlock) == 8, "VoxelBlock must be exactly 8 bytes.");
/// 块初始化

// 将普通节点块的八个子节点统一设置为空或材料状态。
inline void reset(VoxelNodeBlock& block, VoxelState childState = VoxelState::Empty)
{
    assert(childState == VoxelState::Empty || childState == VoxelState::Material);

    block.storageMask = 0;
    block.leafMask = childState == VoxelState::Material ? static_cast<std::uint8_t>(0xFFU) : static_cast<std::uint8_t>(0);
    block.userData = 0;
    block.firstChildIndex = InvalidVoxelIndex;
}

// 将掩码叶块中的64个最高层体素统一设置为空或材料状态。
inline void reset(VoxelLeafBlock& block, VoxelState state = VoxelState::Empty)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    block.materialMask = state == VoxelState::Material ? ~static_cast<std::uint64_t>(0) : static_cast<std::uint64_t>(0);
}

/// 八位掩码操作

// 返回指定角点在八位子节点掩码中对应的位。
inline std::uint8_t cornerBit(VoxelCorner corner)
{
    const unsigned int cornerIndex = static_cast<unsigned int>(corner);

    assert(cornerIndex < static_cast<unsigned int>(VoxelCornerCount));
    return static_cast<std::uint8_t>(1U << cornerIndex);
}

// 从八位掩码中提取指定角点的状态。
inline bool maskBit(std::uint8_t mask, VoxelCorner corner)
{
    return (mask & cornerBit(corner)) != 0;
}

// 设置八位掩码中指定角点的状态。
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

/// 六十四位叶块掩码操作

// 返回粗层角点在64位叶块掩码中对应的八位偏移。
inline unsigned int leafMaskOffset(VoxelCorner coarseCorner)
{
    const unsigned int cornerIndex = static_cast<unsigned int>(coarseCorner);

    assert(cornerIndex < static_cast<unsigned int>(VoxelCornerCount));
    return cornerIndex * static_cast<unsigned int>(VoxelCornerCount);
}

// 返回粗层角点和细层角点对应的64位叶块位。
inline std::uint64_t leafBit(VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    const unsigned int fineIndex = static_cast<unsigned int>(fineCorner);

    assert(fineIndex < static_cast<unsigned int>(VoxelCornerCount));
    return static_cast<std::uint64_t>(1ULL << (leafMaskOffset(coarseCorner) + fineIndex));
}

// 从64位掩码中提取指定粗层角点对应的八位细层体素掩码。
inline std::uint8_t maskBits(std::uint64_t mask, VoxelCorner coarseCorner)
{
    return static_cast<std::uint8_t>((mask >> leafMaskOffset(coarseCorner)) & 0xFFULL);
}

// 从64位掩码中提取指定粗层角点和细层角点对应的材料状态。
inline bool maskBit(std::uint64_t mask, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return (mask & leafBit(coarseCorner, fineCorner)) != 0;
}

// 设置64位掩码中指定粗层角点和细层角点对应的材料状态。
inline void setMaskBit(std::uint64_t& mask, VoxelCorner coarseCorner, VoxelCorner fineCorner, bool value)
{
    const std::uint64_t bit = leafBit(coarseCorner, fineCorner);

    if (value)
    {
        mask |= bit;
    }
    else
    {
        mask &= ~bit;
    }
}

/// 普通节点状态操作

// 返回普通节点块中指定角点记录的子节点状态。
inline VoxelState childState(const VoxelNodeBlock& block, VoxelCorner corner)
{
    const std::uint8_t storageValue = maskBit(block.storageMask, corner) ? static_cast<std::uint8_t>(2) : static_cast<std::uint8_t>(0);
    const std::uint8_t leafValue = maskBit(block.leafMask, corner) ? static_cast<std::uint8_t>(1) : static_cast<std::uint8_t>(0);

    return static_cast<VoxelState>(storageValue | leafValue);
}

// 返回指定角点是否占用物理存储槽。
inline bool hasChildStorage(const VoxelNodeBlock& block, VoxelCorner corner)
{
    return maskBit(block.storageMask, corner);
}

// 返回指定角点对应的物理存储槽索引。
inline VoxelIndex childStorageIndex(const VoxelNodeBlock& block, VoxelCorner corner)
{
    assert(block.firstChildIndex != InvalidVoxelIndex);
    assert(hasChildStorage(block, corner));

    return block.firstChildIndex + static_cast<VoxelIndex>(corner);
}

}

#endif // MYVOXEL_CORE_STORAGE_VOXELBLOCK_H
