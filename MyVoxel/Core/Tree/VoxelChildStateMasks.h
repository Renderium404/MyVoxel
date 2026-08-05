#ifndef MYVOXEL_CORE_TREE_VOXELCHILDSTATEMASKS_H
#define MYVOXEL_CORE_TREE_VOXELCHILDSTATEMASKS_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

const std::uint8_t EmptyVoxelChildMask = static_cast<std::uint8_t>(0); // 不包含任何直接子体素位置的八位逻辑掩码。
const std::uint8_t FullVoxelChildMask = static_cast<std::uint8_t>(0xFFU); // 包含全部八个直接子体素位置的八位逻辑掩码。

// 保存一个逻辑体素的八个直接子体素状态掩码。
struct VoxelChildStateMasks
{
    std::uint8_t empty = EmptyVoxelChildMask; // Empty子体素位置。
    std::uint8_t material = EmptyVoxelChildMask; // Material子体素位置。
    std::uint8_t subdivided = EmptyVoxelChildMask; // Subdivided子体素位置。

    // 返回Empty或Material终止子体素位置。
    std::uint8_t terminalMask() const
    {
        return static_cast<std::uint8_t>(empty | material);
    }

    // 返回指定逻辑状态对应的子体素位置。
    std::uint8_t stateMask(VoxelState state) const
    {
        switch (state)
        {
        case VoxelState::Empty:
            return empty;

        case VoxelState::Material:
            return material;

        case VoxelState::Subdivided:
            return subdivided;
        }

        assert(false);
        return EmptyVoxelChildMask;
    }

    // 判断八个直接子体素是否可以统一折叠为空或材料终止体素。
    bool isCollapsible() const
    {
        return empty == FullVoxelChildMask ||
               material == FullVoxelChildMask;
    }

    // 返回八个直接子体素统一折叠后的终止状态，当前状态必须可以折叠。
    VoxelState collapsedState() const
    {
        assert(isCollapsible());

        return material == FullVoxelChildMask
                   ? VoxelState::Material
                   : VoxelState::Empty;
    }

    // 检查三个逻辑状态是否互不重叠并完整覆盖八个子体素。
    bool isValid() const
    {
        return (empty & material) == 0 &&
               (empty & subdivided) == 0 &&
               (material & subdivided) == 0 &&
               static_cast<std::uint8_t>(empty | material | subdivided) ==FullVoxelChildMask;
    }
};

/// 状态构造

// 返回八个直接子体素全部继承指定终止状态的逻辑状态掩码。
inline VoxelChildStateMasks uniformChildStateMasks(VoxelState state)
{
    assert(state == VoxelState::Empty ||state == VoxelState::Material);

    VoxelChildStateMasks result;

    if (state == VoxelState::Material)
    {
        result.material = FullVoxelChildMask;
    }
    else
    {
        result.empty = FullVoxelChildMask;
    }

    assert(result.isValid());
    return result;
}

}

#endif // MYVOXEL_CORE_TREE_VOXELCHILDSTATEMASKS_H
