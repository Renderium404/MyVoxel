#ifndef MYVOXEL_VOXELNODEBLOCK_H
#define MYVOXEL_VOXELNODEBLOCK_H

#include <cassert>
#include <cstdint>
#include <limits>

#include "../VoxelTypes.h"

namespace MyVoxel
{

using VoxelNodeIndex = std::uint32_t; // 节点池中的物理存储槽索引。

const VoxelNodeIndex InvalidVoxelNodeIndex = (std::numeric_limits<VoxelNodeIndex>::max)(); // 表示当前节点块没有物理子存储组。

// 表示节点块中一个直接子单元的底层存储状态。
enum class VoxelStorageState
{
    Empty,
    Material,
    Branch,
    MaskLeaf
};

// 保存一个已细分体素节点的八个直接子单元状态及物理存储槽首索引。
struct VoxelNodeBlock
{
    // 将八个直接子单元初始化为指定叶状态，并清除用户数据和物理存储索引。
    void reset(VoxelState childState = VoxelState::Empty)
    {
        assert(childState == VoxelState::Empty || childState == VoxelState::Material);

        childMask = 0;
        leafMask = childState == VoxelState::Material ? 0xFF : 0x00;
        userData = 0;
        firstChildIndex = InvalidVoxelNodeIndex;
    }

    /// 子单元状态

    // 返回指定角点对应直接子单元的底层存储状态。
    VoxelStorageState childStorageState(VoxelCorner corner) const
    {
        const std::uint8_t bit = cornerBit(corner);
        const bool hasStorage = (childMask & bit) != 0;
        const bool leafBit = (leafMask & bit) != 0;

        if (!hasStorage)
        {
            return leafBit ? VoxelStorageState::Material : VoxelStorageState::Empty;
        }

        return leafBit ? VoxelStorageState::MaskLeaf : VoxelStorageState::Branch;
    }

    // 返回指定角点对应直接子单元的公共体素状态。
    VoxelState childState(VoxelCorner corner) const
    {
        const VoxelStorageState storageState = childStorageState(corner);

        if (storageState == VoxelStorageState::Empty)
        {
            return VoxelState::Empty;
        }

        if (storageState == VoxelStorageState::Material)
        {
            return VoxelState::Material;
        }

        return VoxelState::Subdivided;
    }

    // 将指定直接子单元设置为空或材料状态。
    void setChildLeafState(VoxelCorner corner, VoxelState state)
    {
        assert(state == VoxelState::Empty || state == VoxelState::Material);

        const std::uint8_t bit = cornerBit(corner);
        childMask = static_cast<std::uint8_t>(childMask & static_cast<std::uint8_t>(~bit));

        if (state == VoxelState::Material)
        {
            leafMask = static_cast<std::uint8_t>(leafMask | bit);
        }
        else
        {
            leafMask = static_cast<std::uint8_t>(leafMask & static_cast<std::uint8_t>(~bit));
        }
    }

    // 将指定直接子单元设置为普通树分支。
    void setChildBranch(VoxelCorner corner)
    {
        assert(firstChildIndex != InvalidVoxelNodeIndex);

        const std::uint8_t bit = cornerBit(corner);
        childMask = static_cast<std::uint8_t>(childMask | bit);
        leafMask = static_cast<std::uint8_t>(leafMask & static_cast<std::uint8_t>(~bit));
    }

    // 将指定直接子单元设置为掩码叶块。
    void setChildMaskLeaf(VoxelCorner corner)
    {
        assert(firstChildIndex != InvalidVoxelNodeIndex);

        const std::uint8_t bit = cornerBit(corner);
        childMask = static_cast<std::uint8_t>(childMask | bit);
        leafMask = static_cast<std::uint8_t>(leafMask | bit);
    }

    // 兼容原有接口，将指定直接子单元设置为普通树分支。
    void setChildSubdivided(VoxelCorner corner)
    {
        setChildBranch(corner);
    }

    // 检查指定直接子单元是否具有物理存储槽。
    bool childHasStorage(VoxelCorner corner) const
    {
        return (childMask & cornerBit(corner)) != 0;
    }

    // 检查当前节点块是否包含具有物理存储的直接子单元。
    bool hasStoredChildren() const
    {
        return childMask != 0;
    }

    // 兼容原有接口。
    bool hasSubdividedChildren() const
    {
        return hasStoredChildren();
    }

    /// 合并

    // 检查八个直接子单元是否能够合并为单一叶状态。
    bool canMerge() const
    {
        return childMask == 0 && (leafMask == 0x00 || leafMask == 0xFF);
    }

    // 返回八个一致直接子单元合并后的叶状态。
    VoxelState mergedState() const
    {
        assert(canMerge());
        return leafMask == 0xFF ? VoxelState::Material : VoxelState::Empty;
    }

    /// 物理存储索引

    // 返回指定具有物理存储的直接子单元对应的槽索引。
    VoxelNodeIndex childStorageIndex(VoxelCorner corner) const
    {
        assert(firstChildIndex != InvalidVoxelNodeIndex);
        assert(childHasStorage(corner));

        return firstChildIndex + static_cast<VoxelNodeIndex>(corner);
    }

    // 返回指定普通分支对应的节点槽索引。
    VoxelNodeIndex childNodeIndex(VoxelCorner corner) const
    {
        assert(childStorageState(corner) == VoxelStorageState::Branch);
        return childStorageIndex(corner);
    }

    // 返回指定掩码叶块对应的物理槽索引。
    VoxelNodeIndex childLeafIndex(VoxelCorner corner) const
    {
        assert(childStorageState(corner) == VoxelStorageState::MaskLeaf);
        return childStorageIndex(corner);
    }

    /// 结构验证

    // 检查掩码和物理存储索引是否满足结构约束。
    bool isValid() const
    {
        if (childMask == 0)
        {
            return firstChildIndex == InvalidVoxelNodeIndex;
        }

        return firstChildIndex != InvalidVoxelNodeIndex && (firstChildIndex % VoxelCornerCount) == 0;
    }

    // 返回指定角点对应的单个位掩码。
    static std::uint8_t cornerBit(VoxelCorner corner)
    {
        const unsigned int cornerIndex = static_cast<unsigned int>(corner);

        assert(cornerIndex < static_cast<unsigned int>(VoxelCornerCount));
        return static_cast<std::uint8_t>(1U << cornerIndex);
    }

    std::uint8_t childMask; // bit i为1时，第i个直接子单元具有物理存储槽。
    std::uint8_t leafMask; // 无存储时表示Material，有存储时表示MaskLeaf。
    std::uint16_t userData; // 用户自定义模块使用的原始数据，核心模块不解释。
    VoxelNodeIndex firstChildIndex; // 八个连续物理存储槽的首索引。
};

static_assert(sizeof(VoxelNodeBlock) == 8, "VoxelNodeBlock must be exactly 8 bytes.");

}

#endif // MYVOXEL_VOXELNODEBLOCK_H