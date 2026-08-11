#ifndef MYVOXEL_CORE_MASK_VOXELNODEMASK_H
#define MYVOXEL_CORE_MASK_VOXELNODEMASK_H

#include <cassert>
#include <cstdint>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "MyVoxel/Core/Storage/NodeBlock.h"

namespace MyVoxel
{

static_assert(VoxelCornerCount == 8, "VoxelNodeMask requires exactly eight voxel corners.");

const std::uint8_t EmptyVoxelNodeMask = static_cast<std::uint8_t>(0); // 不包含任何子体素位置的八位掩码。
const std::uint8_t FullVoxelNodeMask = static_cast<std::uint8_t>(0xFFU); // 包含全部八个子体素位置的八位掩码。

// 保存NodeBlock中八个直接子体素的批量物理状态掩码。
struct VoxelNodeStateMasks
{
    std::uint8_t empty = 0; // Empty子体素位置。
    std::uint8_t material = 0; // Material子体素位置。
    std::uint8_t node = 0; // Node子体素位置。
    std::uint8_t maskLeaf = 0; // MaskLeaf子体素位置。
    std::uint8_t terminal = 0; // Empty或Material子体素位置。
    std::uint8_t subdivided = 0; // Node或MaskLeaf子体素位置。

    // 检查四种物理状态是否互不重叠并完整覆盖八个子体素。
    bool isValid() const
    {
        const std::uint8_t masks[] = { empty, material, node, maskLeaf };
        std::uint8_t accumulated = 0;

        for (unsigned int maskIndex = 0; maskIndex < 4; ++maskIndex)
        {
            if ((accumulated & masks[maskIndex]) != 0)
            {
                return false;
            }

            accumulated = static_cast<std::uint8_t>(accumulated | masks[maskIndex]);
        }

        return accumulated == FullVoxelNodeMask &&
               terminal == static_cast<std::uint8_t>(empty | material) &&
               subdivided == static_cast<std::uint8_t>(node | maskLeaf);
    }
};

/// 角点掩码

// 返回指定角点对应的八位位置掩码。
inline std::uint8_t nodeCornerMask(VoxelCorner corner)
{
    const unsigned int cornerIndex = static_cast<unsigned int>(corner);

    assert(cornerIndex < static_cast<unsigned int>(VoxelCornerCount));
    return static_cast<std::uint8_t>(1U << cornerIndex);
}

// 判断八位掩码中指定角点是否置位。
inline bool nodeMaskBit(std::uint8_t mask, VoxelCorner corner)
{
    return (mask & nodeCornerMask(corner)) != 0;
}

/// 单个子节点状态

// 返回NodeBlock中指定角点记录的物理状态。
inline VoxelNodeState nodeState(const NodeBlock& block, VoxelCorner corner)
{
    const std::uint8_t storageValue = nodeMaskBit(block.storageMask, corner) ? static_cast<std::uint8_t>(2) : static_cast<std::uint8_t>(0);
    const std::uint8_t leafValue = nodeMaskBit(block.leafMask, corner) ? static_cast<std::uint8_t>(1) : static_cast<std::uint8_t>(0);
    return static_cast<VoxelNodeState>(storageValue | leafValue);
}

// 判断指定角点是否占用VoxelBlock物理槽。
inline bool hasChildStorage(const NodeBlock& block, VoxelCorner corner)
{
    return nodeMaskBit(block.storageMask, corner);
}

// 返回指定角点对应的VoxelBlock物理槽索引。
inline VoxelIndex childStorageIndex(const NodeBlock& block, VoxelCorner corner)
{
    assert(block.firstChildIndex != InvalidVoxelIndex);
    assert(hasChildStorage(block, corner));
    return block.firstChildIndex + static_cast<VoxelIndex>(corner);
}

/// 状态掩码

// 返回全部Empty子体素位置。
inline std::uint8_t nodeEmptyMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(~static_cast<std::uint8_t>(block.storageMask | block.leafMask));
}

// 返回全部Material子体素位置。
inline std::uint8_t nodeMaterialMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(static_cast<std::uint8_t>(~block.storageMask) & block.leafMask);
}

// 返回全部Node子体素位置。
inline std::uint8_t nodeChildNodeMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(block.storageMask & static_cast<std::uint8_t>(~block.leafMask));
}

// 返回全部MaskLeaf子体素位置。
inline std::uint8_t nodeMaskLeafMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(block.storageMask & block.leafMask);
}

// 返回全部Empty或Material终止子体素位置。
inline std::uint8_t nodeTerminalMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(~block.storageMask);
}

// 返回全部Node或MaskLeaf细分子体素位置。
inline std::uint8_t nodeSubdividedMask(const NodeBlock& block)
{
    return block.storageMask;
}

// 返回指定物理状态对应的全部子体素位置。
inline std::uint8_t nodeStateMask(const NodeBlock& block, VoxelNodeState state)
{
    switch (state)
    {
    case VoxelNodeState::Empty:
        return nodeEmptyMask(block);

    case VoxelNodeState::Material:
        return nodeMaterialMask(block);

    case VoxelNodeState::Node:
        return nodeChildNodeMask(block);

    case VoxelNodeState::MaskLeaf:
        return nodeMaskLeafMask(block);
    }

    assert(false);
    return EmptyVoxelNodeMask;
}

// 一次返回NodeBlock中八个直接子体素的全部状态掩码。
inline VoxelNodeStateMasks nodeStateMasks(const NodeBlock& block)
{
    VoxelNodeStateMasks result;
    result.empty = nodeEmptyMask(block);
    result.material = nodeMaterialMask(block);
    result.node = nodeChildNodeMask(block);
    result.maskLeaf = nodeMaskLeafMask(block);
    result.terminal = nodeTerminalMask(block);
    result.subdivided = nodeSubdividedMask(block);

    assert(result.isValid());
    return result;
}

/// 节点整体状态

// 判断NodeBlock的八个直接子体素是否全部为空。
inline bool isNodeBlockEmpty(const NodeBlock& block)
{
    return block.storageMask == EmptyVoxelNodeMask && block.leafMask == EmptyVoxelNodeMask;
}

// 判断NodeBlock的八个直接子体素是否全部包含材料。
inline bool isNodeBlockMaterial(const NodeBlock& block)
{
    return block.storageMask == EmptyVoxelNodeMask && block.leafMask == FullVoxelNodeMask;
}

// 判断NodeBlock是否包含Node或MaskLeaf物理子节点。
inline bool hasNodeChildStorage(const NodeBlock& block)
{
    return block.storageMask != EmptyVoxelNodeMask;
}

// 判断NodeBlock是否可以整体折叠为空或材料终止体素。
inline bool isNodeBlockCollapsible(const NodeBlock& block)
{
    return isNodeBlockEmpty(block) || isNodeBlockMaterial(block);
}

// 返回NodeBlock八个直接子体素整体对应的逻辑状态。
inline VoxelState nodeBlockState(const NodeBlock& block)
{
    if (isNodeBlockEmpty(block))
    {
        return VoxelState::Empty;
    }

    if (isNodeBlockMaterial(block))
    {
        return VoxelState::Material;
    }

    return VoxelState::Subdivided;
}

/// 掩码逻辑运算

// 返回两个八位掩码的并集。
inline std::uint8_t nodeMaskUnion(std::uint8_t first, std::uint8_t second)
{
    return static_cast<std::uint8_t>(first | second);
}

// 返回两个八位掩码的交集。
inline std::uint8_t nodeMaskIntersection(std::uint8_t first, std::uint8_t second)
{
    return static_cast<std::uint8_t>(first & second);
}

// 返回first中存在而second中不存在的位置。
inline std::uint8_t nodeMaskDifference(std::uint8_t first, std::uint8_t second)
{
    return static_cast<std::uint8_t>(first & static_cast<std::uint8_t>(~second));
}

// 返回两个八位掩码的对称差。
inline std::uint8_t nodeMaskSymmetricDifference(std::uint8_t first, std::uint8_t second)
{
    return static_cast<std::uint8_t>(first ^ second);
}

// 判断两个八位掩码是否存在重叠位置。
inline bool nodeMasksOverlap(std::uint8_t first, std::uint8_t second)
{
    return (first & second) != 0;
}

// 判断container是否包含subset中的全部位置。
inline bool nodeMaskContains(std::uint8_t container, std::uint8_t subset)
{
    return (container & subset) == subset;
}

/// 掩码统计与遍历

// 返回八位掩码中的置位数量。
inline unsigned int nodeMaskBitCount(std::uint8_t mask)
{
    unsigned int value = static_cast<unsigned int>(mask);
    value = value - ((value >> 1U) & 0x55U);
    value = (value & 0x33U) + ((value >> 2U) & 0x33U);
    return (value + (value >> 4U)) & 0x0FU;
}

// 返回非零八位掩码中最低置位的角点索引。
inline unsigned int firstNodeBitIndex(std::uint8_t mask)
{
    assert(mask != EmptyVoxelNodeMask);

#if defined(_MSC_VER)
    unsigned long bitIndex = 0;
    _BitScanForward(&bitIndex, static_cast<unsigned long>(mask));
    return static_cast<unsigned int>(bitIndex);
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<unsigned int>(__builtin_ctz(static_cast<unsigned int>(mask)));
#else
    unsigned int bitIndex = 0;

    while ((mask & static_cast<std::uint8_t>(1U)) == 0)
    {
        mask = static_cast<std::uint8_t>(mask >> 1U);
        ++bitIndex;
    }

    return bitIndex;
#endif
}

// 返回非零八位掩码中最低置位对应的角点。
inline VoxelCorner firstNodeCorner(std::uint8_t mask)
{
    return static_cast<VoxelCorner>(firstNodeBitIndex(mask));
}

// 返回最低置位对应的角点，并从原掩码中删除该位。
inline VoxelCorner takeFirstNodeCorner(std::uint8_t& mask)
{
    const VoxelCorner corner = firstNodeCorner(mask);
    mask = static_cast<std::uint8_t>(mask & static_cast<std::uint8_t>(mask - 1U));
    return corner;
}

/// 状态位修改

// 只修改NodeBlock中指定位置的状态位，不创建、复制或释放对应物理存储。
//
// 将Node或MaskLeaf改为终止状态前，调用者必须先释放物理后代。
// 将终止状态改为Node或MaskLeaf前，调用者必须先初始化对应物理槽。
inline void setNodeStateBits(NodeBlock& block, std::uint8_t cornerMask, VoxelNodeState state)
{
    switch (state)
    {
    case VoxelNodeState::Empty:
        block.storageMask = static_cast<std::uint8_t>(block.storageMask & static_cast<std::uint8_t>(~cornerMask));
        block.leafMask = static_cast<std::uint8_t>(block.leafMask & static_cast<std::uint8_t>(~cornerMask));
        return;

    case VoxelNodeState::Material:
        block.storageMask = static_cast<std::uint8_t>(block.storageMask & static_cast<std::uint8_t>(~cornerMask));
        block.leafMask = static_cast<std::uint8_t>(block.leafMask | cornerMask);
        return;

    case VoxelNodeState::Node:
        block.storageMask = static_cast<std::uint8_t>(block.storageMask | cornerMask);
        block.leafMask = static_cast<std::uint8_t>(block.leafMask & static_cast<std::uint8_t>(~cornerMask));
        return;

    case VoxelNodeState::MaskLeaf:
        block.storageMask = static_cast<std::uint8_t>(block.storageMask | cornerMask);
        block.leafMask = static_cast<std::uint8_t>(block.leafMask | cornerMask);
        return;
    }

    assert(false);
}

// 只修改NodeBlock中指定角点的状态位，不管理对应物理存储。
inline void setNodeStateBits(NodeBlock& block, VoxelCorner corner, VoxelNodeState state)
{
    setNodeStateBits(block, nodeCornerMask(corner), state);
}

// 将指定终止子体素位置统一设置为空或材料。
inline void setTerminalNodeStateBits(NodeBlock& block, std::uint8_t cornerMask, VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    assert((block.storageMask & cornerMask) == 0);

    if (state == VoxelState::Material)
    {
        block.leafMask = static_cast<std::uint8_t>(block.leafMask | cornerMask);
    }
    else
    {
        block.leafMask = static_cast<std::uint8_t>(block.leafMask & static_cast<std::uint8_t>(~cornerMask));
    }
}

// 反转指定终止子体素位置的Empty和Material状态。
inline void invertTerminalNodeStateBits(NodeBlock& block, std::uint8_t cornerMask)
{
    assert((block.storageMask & cornerMask) == 0);
    block.leafMask = static_cast<std::uint8_t>(block.leafMask ^ cornerMask);
}

}

#endif // MYVOXEL_CORE_MASK_VOXELNODEMASK_H