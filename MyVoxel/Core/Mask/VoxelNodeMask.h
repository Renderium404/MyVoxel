#ifndef MYVOXEL_CORE_MASK_VOXELNODEMASK_H
#define MYVOXEL_CORE_MASK_VOXELNODEMASK_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/Mask/MaskUtils.h"
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

/// 单个子项状态

// 返回分化节点中指定直接子项的物理状态。
inline VoxelNodeState nodeState(const NodeBlock& block, VoxelCorner corner)
{
    const std::uint8_t childValue = maskBit(block.mChildMask, corner) ? static_cast<std::uint8_t>(2U) : static_cast<std::uint8_t>(0U);
    const std::uint8_t valueValue = maskBit(block.mValueMask, corner) ? static_cast<std::uint8_t>(1U) : static_cast<std::uint8_t>(0U);
    return static_cast<VoxelNodeState>(childValue | valueValue);
}

// 设置分化节点中指定直接子项的物理状态，只修改掩码，不处理对应物理槽数据和生命周期。
inline void setNodeState(NodeBlock& block, VoxelCorner corner, VoxelNodeState state)
{
    const unsigned int stateValue = static_cast<unsigned int>(state);

    assert(stateValue <= 3U);

    setMaskBit(block.mChildMask, corner, (stateValue & 2U) != 0);
    setMaskBit(block.mValueMask, corner, (stateValue & 1U) != 0);
}

// 返回指定直接子项是否继续分化为Node或MaskLeaf。
inline bool isSubdivided(const NodeBlock& block, VoxelCorner corner)
{
    return maskBit(block.mChildMask, corner);
}

// 返回指定直接子项是否为空。
inline bool isEmptyChild(const NodeBlock& block, VoxelCorner corner)
{
    return !maskBit(block.mChildMask, corner) && !maskBit(block.mValueMask, corner);
}

// 返回指定直接子项是否为材料。
inline bool isMaterialChild(const NodeBlock& block, VoxelCorner corner)
{
    return !maskBit(block.mChildMask, corner) && maskBit(block.mValueMask, corner);
}

// 返回指定直接子项是否为普通节点。
inline bool isNodeChild(const NodeBlock& block, VoxelCorner corner)
{
    return maskBit(block.mChildMask, corner) && !maskBit(block.mValueMask, corner);
}

// 返回指定直接子项是否为MaskLeaf。
inline bool isLeafChild(const NodeBlock& block, VoxelCorner corner)
{
    return maskBit(block.mChildMask, corner) && maskBit(block.mValueMask, corner);
}

/// 状态掩码

// 返回八个直接子项中的Empty状态掩码。
inline std::uint8_t emptyChildMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(static_cast<std::uint8_t>(~block.mChildMask) & static_cast<std::uint8_t>(~block.mValueMask));
}

// 返回八个直接子项中的Material状态掩码。
inline std::uint8_t materialChildMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(static_cast<std::uint8_t>(~block.mChildMask) & block.mValueMask);
}

// 返回八个直接子项中的Node状态掩码。
inline std::uint8_t nodeChildMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(block.mChildMask & static_cast<std::uint8_t>(~block.mValueMask));
}

// 返回八个直接子项中的MaskLeaf状态掩码。
inline std::uint8_t leafChildMask(const NodeBlock& block)
{
    return static_cast<std::uint8_t>(block.mChildMask & block.mValueMask);
}

// 返回指定物理状态对应的全部直接子项位置。
inline std::uint8_t nodeStateMask(const NodeBlock& block, VoxelNodeState state)
{
    switch (state)
    {
    case VoxelNodeState::Empty:
        return emptyChildMask(block);
    case VoxelNodeState::Material:
        return materialChildMask(block);
    case VoxelNodeState::Node:
        return nodeChildMask(block);
    case VoxelNodeState::MaskLeaf:
        return leafChildMask(block);
    }

    assert(false);
    return EmptyVoxelNodeMask;
}

// 一次返回NodeBlock中八个直接子项的全部状态掩码。
inline VoxelNodeStateMasks nodeStateMasks(const NodeBlock& block)
{
    VoxelNodeStateMasks result;
    result.empty = emptyChildMask(block);
    result.material = materialChildMask(block);
    result.node = nodeChildMask(block);
    result.maskLeaf = leafChildMask(block);
    result.terminal = static_cast<std::uint8_t>(~block.mChildMask);
    result.subdivided = block.mChildMask;

    assert(result.isValid());
    return result;
}

/// 节点整体二值状态

// 判断八个直接子项是否全部为空，仅描述二值状态，不表示八个Value相同。
inline bool isNodeBlockEmpty(const NodeBlock& block)
{
    return block.mChildMask == EmptyVoxelNodeMask && block.mValueMask == EmptyVoxelNodeMask;
}

// 判断八个直接子项是否全部包含材料，仅描述二值状态，不表示八个Value相同。
inline bool isNodeBlockMaterial(const NodeBlock& block)
{
    return block.mChildMask == EmptyVoxelNodeMask && block.mValueMask == FullVoxelNodeMask;
}

// 返回八个直接子项整体对应的逻辑二值状态，不判断非分化Value是否一致。
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

/// 掩码遍历

// 返回非零八位节点掩码中最低置位对应的角点。
inline VoxelCorner firstNodeCorner(std::uint8_t mask)
{
    return static_cast<VoxelCorner>(firstMaskBitIndex(mask));
}

// 返回最低置位对应的角点，并从原掩码中删除该位。
inline VoxelCorner takeFirstNodeCorner(std::uint8_t& mask)
{
    return static_cast<VoxelCorner>(takeFirstMaskBitIndex(mask));
}

/// 批量状态修改

// 只修改指定多个子项的状态位，不创建、修改或释放对应VoxelBlock物理数据。
inline void setNodeStateBits(NodeBlock& block, std::uint8_t cornerMask, VoxelNodeState state)
{
    switch (state)
    {
    case VoxelNodeState::Empty:
        block.mChildMask = static_cast<std::uint8_t>(block.mChildMask & static_cast<std::uint8_t>(~cornerMask));
        block.mValueMask = static_cast<std::uint8_t>(block.mValueMask & static_cast<std::uint8_t>(~cornerMask));
        return;

    case VoxelNodeState::Material:
        block.mChildMask = static_cast<std::uint8_t>(block.mChildMask & static_cast<std::uint8_t>(~cornerMask));
        block.mValueMask = static_cast<std::uint8_t>(block.mValueMask | cornerMask);
        return;

    case VoxelNodeState::Node:
        block.mChildMask = static_cast<std::uint8_t>(block.mChildMask | cornerMask);
        block.mValueMask = static_cast<std::uint8_t>(block.mValueMask & static_cast<std::uint8_t>(~cornerMask));
        return;

    case VoxelNodeState::MaskLeaf:
        block.mChildMask = static_cast<std::uint8_t>(block.mChildMask | cornerMask);
        block.mValueMask = static_cast<std::uint8_t>(block.mValueMask | cornerMask);
        return;
    }

    assert(false);
}

// 将指定非分化子项统一设置为空或材料，只修改材料状态位，不修改对应Value。
inline void setTerminalNodeStateBits(NodeBlock& block, std::uint8_t cornerMask, VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    assert((block.mChildMask & cornerMask) == 0);

    block.mValueMask = state == VoxelState::Material ?
        static_cast<std::uint8_t>(block.mValueMask | cornerMask) :
        static_cast<std::uint8_t>(block.mValueMask & static_cast<std::uint8_t>(~cornerMask));
}

// 反转指定非分化子项的Empty和Material状态，只修改材料状态位，调用者必须同步处理对应Value。
inline void invertTerminalNodeStateBits(NodeBlock& block, std::uint8_t cornerMask)
{
    assert((block.mChildMask & cornerMask) == 0);
    block.mValueMask = static_cast<std::uint8_t>(block.mValueMask ^ cornerMask);
}

}

#endif // MYVOXEL_CORE_MASK_VOXELNODEMASK_H