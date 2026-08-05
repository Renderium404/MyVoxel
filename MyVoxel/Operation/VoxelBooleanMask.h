#ifndef MYVOXEL_OPERATION_VOXELBOOLEANMASK_H
#define MYVOXEL_OPERATION_VOXELBOOLEANMASK_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/Mask/VoxelChildMask.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"

namespace MyVoxel
{
namespace Operation
{

// 表示两个体素材料集合之间执行的布尔运算类型。
enum class VoxelBooleanType : std::uint8_t
{
    Union = 0,          // 返回左右体素材料的并集。
    Intersection = 1,   // 返回左右体素材料的交集。
    Difference = 2,     // 从左侧体素材料中减去右侧体素材料。
    ExclusiveOr = 3     // 返回左右体素材料的异或结果。
};

// 表示左侧体素执行布尔运算时应采取的 7 种完备原子动作。
enum class VoxelBooleanAction : std::uint8_t
{
    KeepLeft = 0,          // 1. 保持不变：左侧当前状态符合运算结果，零内存修改直接跳过。
    SetEmpty = 1,          // 2. 直接清空：将左侧设置为Empty，并批量释放下属物理存储槽。
    SetMaterial = 2,       // 3. 填满材料：将左侧设置为Material，并批量释放下属物理存储槽。
    CopyRight = 3,         // 4. 复制右侧：直接将右侧节点的掩码或子树结构挂载/复制到左侧。
    InvertLeft = 4,        // 5. 原地反转：保持左侧树结构，将其叶节点材料状态逐位取反 (XOR 场景)。
    CopyInvertedRight = 5, // 6. 复制并反转：复制右侧树结构的同时，将其内部全部材料状态反转后挂载到左侧 (Difference 场景)。
    Recurse = 6            // 7. 下潜递归：左右侧均已细分，无法在当前层直接裁决，需下潜一层继续运算。
};

// 保存八个直接子体素执行布尔运算时的批量处理掩码。
//
// 七种掩码互不重叠，并完整覆盖全部八个子体素位置。
struct VoxelBooleanActionMasks
{
    std::uint8_t keepLeft = 0; // 保持左侧子体素不变的位置。
    std::uint8_t setEmpty = 0; // 将左侧子体素直接设置为空的位置。
    std::uint8_t setMaterial = 0; // 将左侧子体素直接设置为材料的位置。
    std::uint8_t copyRight = 0; // 将右侧子体素复制到左侧的位置。
    std::uint8_t invertLeft = 0; // 原地反转左侧子体素的位置。
    std::uint8_t copyInvertedRight = 0; // 将反转后的右侧子体素复制到左侧的位置。
    std::uint8_t recurse = 0; // 需要继续执行子层布尔运算的位置。

    // 返回指定处理方式对应的子体素位置。
    std::uint8_t actionMask(VoxelBooleanAction action) const;

    // 返回不需要继续递归的全部子体素位置。
    std::uint8_t directMask() const;

    // 返回能够在当前层直接完成修改的子体素位置。
    std::uint8_t directChangedMask() const;

    // 返回七种处理方式覆盖的全部子体素位置。
    std::uint8_t coveredMask() const;

    // 判断是否包含需要继续递归处理的子体素。
    bool hasRecursiveWork() const;

    // 判断是否包含能够在当前层直接完成的修改。
    bool hasDirectChanges() const;

    // 检查七种处理掩码是否互不重叠并完整覆盖八个子体素。
    bool isValid() const;
};

// 保存一次64位叶块布尔运算结果。
struct VoxelLeafBooleanResult
{
    std::uint64_t materialMask = 0; // 布尔运算后的64位材料掩码。
    VoxelState state = VoxelState::Empty; // 运算结果整体对应的逻辑状态。
    bool changed = false; // 运算结果是否与左侧原材料掩码不同。
};

/// 终止状态运算

// 判断指定逻辑状态是否为空或材料终止状态。
inline bool isTerminalBooleanState(VoxelState state)
{
    return state == VoxelState::Empty || state == VoxelState::Material;
}

// 将终止逻辑状态转换为统一的64位叶块材料掩码。
inline std::uint64_t terminalLeafMaterialMask(VoxelState state)
{
    assert(isTerminalBooleanState(state));
    return state == VoxelState::Material ? FullVoxelLeafMask : EmptyVoxelLeafMask;
}

// 对两个终止逻辑状态执行布尔运算。
inline VoxelState applyTerminalBoolean(VoxelBooleanType type, VoxelState leftState, VoxelState rightState)
{
    assert(isTerminalBooleanState(leftState));
    assert(isTerminalBooleanState(rightState));

    const bool leftMaterial = leftState == VoxelState::Material;
    const bool rightMaterial = rightState == VoxelState::Material;
    bool resultMaterial = false;

    switch (type)
    {
    case VoxelBooleanType::Union:
        resultMaterial = leftMaterial || rightMaterial;
        break;

    case VoxelBooleanType::Intersection:
        resultMaterial = leftMaterial && rightMaterial;
        break;

    case VoxelBooleanType::Difference:
        resultMaterial = leftMaterial && !rightMaterial;
        break;

    case VoxelBooleanType::ExclusiveOr:
        resultMaterial = leftMaterial != rightMaterial;
        break;
    }

    return resultMaterial ? VoxelState::Material : VoxelState::Empty;
}

/// 根节点和单个逻辑体素决策

// 返回左侧体素执行指定布尔运算时应采取的处理方式。
inline VoxelBooleanAction booleanAction(VoxelBooleanType type, VoxelState leftState, VoxelState rightState)
{
    assert(leftState == VoxelState::Empty || leftState == VoxelState::Material || leftState == VoxelState::Subdivided);
    assert(rightState == VoxelState::Empty || rightState == VoxelState::Material || rightState == VoxelState::Subdivided);

    switch (type)
    {
    case VoxelBooleanType::Union:
        if (leftState == VoxelState::Material || rightState == VoxelState::Empty)
        {
            return VoxelBooleanAction::KeepLeft;
        }

        if (rightState == VoxelState::Material)
        {
            return VoxelBooleanAction::SetMaterial;
        }

        if (leftState == VoxelState::Empty)
        {
            return VoxelBooleanAction::CopyRight;
        }

        return VoxelBooleanAction::Recurse;

    case VoxelBooleanType::Intersection:
        if (leftState == VoxelState::Empty || rightState == VoxelState::Material)
        {
            return VoxelBooleanAction::KeepLeft;
        }

        if (rightState == VoxelState::Empty)
        {
            return VoxelBooleanAction::SetEmpty;
        }

        if (leftState == VoxelState::Material)
        {
            return VoxelBooleanAction::CopyRight;
        }

        return VoxelBooleanAction::Recurse;

    case VoxelBooleanType::Difference:
        if (leftState == VoxelState::Empty || rightState == VoxelState::Empty)
        {
            return VoxelBooleanAction::KeepLeft;
        }

        if (rightState == VoxelState::Material)
        {
            return VoxelBooleanAction::SetEmpty;
        }

        if (leftState == VoxelState::Material)
        {
            return VoxelBooleanAction::CopyInvertedRight;
        }

        return VoxelBooleanAction::Recurse;

    case VoxelBooleanType::ExclusiveOr:
        if (rightState == VoxelState::Empty)
        {
            return VoxelBooleanAction::KeepLeft;
        }

        if (rightState == VoxelState::Material)
        {
            if (leftState == VoxelState::Empty)
            {
                return VoxelBooleanAction::SetMaterial;
            }

            if (leftState == VoxelState::Material)
            {
                return VoxelBooleanAction::SetEmpty;
            }

            return VoxelBooleanAction::InvertLeft;
        }

        if (leftState == VoxelState::Empty)
        {
            return VoxelBooleanAction::CopyRight;
        }

        if (leftState == VoxelState::Material)
        {
            return VoxelBooleanAction::CopyInvertedRight;
        }

        return VoxelBooleanAction::Recurse;
    }

    assert(false);
    return VoxelBooleanAction::KeepLeft;
}

/// 八子节点批量决策

// 返回八个直接子体素执行指定布尔运算时的批量处理掩码。
inline VoxelBooleanActionMasks childBooleanActionMasks(VoxelBooleanType type, const VoxelChildStateMasks& left, const VoxelChildStateMasks& right)
{
    assert(left.isValid());
    assert(right.isValid());

    VoxelBooleanActionMasks result;

    switch (type)
    {
    case VoxelBooleanType::Union:
        result.keepLeft = static_cast<std::uint8_t>(left.material | right.empty);
        result.setMaterial = static_cast<std::uint8_t>(right.material & static_cast<std::uint8_t>(~left.material));
        result.copyRight = static_cast<std::uint8_t>(left.empty & right.subdivided);
        result.recurse = static_cast<std::uint8_t>(left.subdivided & right.subdivided);
        break;

    case VoxelBooleanType::Intersection:
        result.keepLeft = static_cast<std::uint8_t>(left.empty | right.material);
        result.setEmpty = static_cast<std::uint8_t>(right.empty & static_cast<std::uint8_t>(~left.empty));
        result.copyRight = static_cast<std::uint8_t>(left.material & right.subdivided);
        result.recurse = static_cast<std::uint8_t>(left.subdivided & right.subdivided);
        break;

    case VoxelBooleanType::Difference:
        result.keepLeft = static_cast<std::uint8_t>(left.empty | right.empty);
        result.setEmpty = static_cast<std::uint8_t>(right.material & static_cast<std::uint8_t>(~left.empty));
        result.copyInvertedRight = static_cast<std::uint8_t>(left.material & right.subdivided);
        result.recurse = static_cast<std::uint8_t>(left.subdivided & right.subdivided);
        break;

    case VoxelBooleanType::ExclusiveOr:
        result.keepLeft = right.empty;
        result.setEmpty = static_cast<std::uint8_t>(left.material & right.material);
        result.setMaterial = static_cast<std::uint8_t>(left.empty & right.material);
        result.copyRight = static_cast<std::uint8_t>(left.empty & right.subdivided);
        result.invertLeft = static_cast<std::uint8_t>(left.subdivided & right.material);
        result.copyInvertedRight = static_cast<std::uint8_t>(left.material & right.subdivided);
        result.recurse = static_cast<std::uint8_t>(left.subdivided & right.subdivided);
        break;
    }

    assert(result.isValid());
    return result;
}

/// 64位叶块布尔运算

// 对两个64位叶块材料掩码执行指定布尔运算。
inline std::uint64_t applyLeafBoolean(VoxelBooleanType type, std::uint64_t leftMask, std::uint64_t rightMask)
{
    switch (type)
    {
    case VoxelBooleanType::Union:
        return leftMask | rightMask;

    case VoxelBooleanType::Intersection:
        return leftMask & rightMask;

    case VoxelBooleanType::Difference:
        return leftMask & ~rightMask;

    case VoxelBooleanType::ExclusiveOr:
        return leftMask ^ rightMask;
    }

    assert(false);
    return leftMask;
}

// 对两个64位材料掩码执行布尔运算并返回完整结果。
inline VoxelLeafBooleanResult leafBooleanResult(VoxelBooleanType type, std::uint64_t leftMask, std::uint64_t rightMask)
{
    VoxelLeafBooleanResult result;

    result.materialMask = applyLeafBoolean(type, leftMask, rightMask);
    result.state = leafMaskState(result.materialMask);
    result.changed = result.materialMask != leftMask;

    return result;
}

// 对两个掩码叶块执行布尔运算并返回完整结果。
inline VoxelLeafBooleanResult leafBooleanResult(VoxelBooleanType type, const VoxelLeafBlock& leftLeaf, const VoxelLeafBlock& rightLeaf)
{
    return leafBooleanResult(type, leftLeaf.materialMask, rightLeaf.materialMask);
}

// 对左侧终止体素和右侧64位叶块执行布尔运算。
inline VoxelLeafBooleanResult leafBooleanResult(VoxelBooleanType type, VoxelState leftState, std::uint64_t rightMask)
{
    assert(isTerminalBooleanState(leftState));
    return leafBooleanResult(type, terminalLeafMaterialMask(leftState), rightMask);
}

// 对左侧64位叶块和右侧终止体素执行布尔运算。
inline VoxelLeafBooleanResult leafBooleanResult(VoxelBooleanType type, std::uint64_t leftMask, VoxelState rightState)
{
    assert(isTerminalBooleanState(rightState));
    return leafBooleanResult(type, leftMask, terminalLeafMaterialMask(rightState));
}

// 对左侧终止体素和右侧掩码叶块执行布尔运算。
inline VoxelLeafBooleanResult leafBooleanResult(VoxelBooleanType type, VoxelState leftState, const VoxelLeafBlock& rightLeaf)
{
    return leafBooleanResult(type, leftState, rightLeaf.materialMask);
}

// 对左侧掩码叶块和右侧终止体素执行布尔运算。
inline VoxelLeafBooleanResult leafBooleanResult(VoxelBooleanType type, const VoxelLeafBlock& leftLeaf, VoxelState rightState)
{
    return leafBooleanResult(type, leftLeaf.materialMask, rightState);
}

/// VoxelBooleanActionMasks

inline std::uint8_t VoxelBooleanActionMasks::actionMask(VoxelBooleanAction action) const
{
    switch (action)
    {
    case VoxelBooleanAction::KeepLeft:
        return keepLeft;

    case VoxelBooleanAction::SetEmpty:
        return setEmpty;

    case VoxelBooleanAction::SetMaterial:
        return setMaterial;

    case VoxelBooleanAction::CopyRight:
        return copyRight;

    case VoxelBooleanAction::InvertLeft:
        return invertLeft;

    case VoxelBooleanAction::CopyInvertedRight:
        return copyInvertedRight;

    case VoxelBooleanAction::Recurse:
        return recurse;
    }

    assert(false);
    return EmptyVoxelNodeMask;
}

inline std::uint8_t VoxelBooleanActionMasks::directMask() const
{
    return static_cast<std::uint8_t>(keepLeft | setEmpty | setMaterial | copyRight | invertLeft | copyInvertedRight);
}

inline std::uint8_t VoxelBooleanActionMasks::directChangedMask() const
{
    return static_cast<std::uint8_t>(setEmpty | setMaterial | copyRight | invertLeft | copyInvertedRight);
}

inline std::uint8_t VoxelBooleanActionMasks::coveredMask() const
{
    return static_cast<std::uint8_t>(directMask() | recurse);
}

inline bool VoxelBooleanActionMasks::hasRecursiveWork() const
{
    return recurse != EmptyVoxelNodeMask;
}

inline bool VoxelBooleanActionMasks::hasDirectChanges() const
{
    return directChangedMask() != EmptyVoxelNodeMask;
}

inline bool VoxelBooleanActionMasks::isValid() const
{
    const std::uint8_t masks[] =
    {
        keepLeft,
        setEmpty,
        setMaterial,
        copyRight,
        invertLeft,
        copyInvertedRight,
        recurse
    };

    std::uint8_t accumulatedMask = EmptyVoxelNodeMask;

    for (unsigned int maskIndex = 0; maskIndex < static_cast<unsigned int>(sizeof(masks) / sizeof(masks[0])); ++maskIndex)
    {
        if ((accumulatedMask & masks[maskIndex]) != EmptyVoxelNodeMask)
        {
            return false;
        }

        accumulatedMask = static_cast<std::uint8_t>(accumulatedMask | masks[maskIndex]);
    }

    return accumulatedMask == FullVoxelNodeMask;
}

}
}

#endif // MYVOXEL_OPERATION_VOXELBOOLEANMASK_H