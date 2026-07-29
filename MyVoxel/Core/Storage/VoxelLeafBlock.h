#ifndef MYVOXEL_VOXELLEAFBLOCK_H
#define MYVOXEL_VOXELLEAFBLOCK_H

#include <cassert>
#include <cstddef>
#include <cstdint>

#include "../VoxelTypes.h"

namespace MyVoxel
{

// 使用一个64位掩码保存连续两层共4×4×4个最高层体素。
struct VoxelLeafBlock
{
    enum
    {
        OctantCount = 8,
        VoxelsPerOctant = 8,
        VoxelCount = 64
    };

    // 将叶块中的全部体素初始化为指定叶状态。
    void reset(VoxelState state = VoxelState::Empty)
    {
        assert(state == VoxelState::Empty || state == VoxelState::Material);
        materialMask = state == VoxelState::Material ? fullMask() : 0;
    }

    /// 单体素状态

    // 返回两级角点路径对应最高层体素的状态。
    VoxelState state(VoxelCorner coarseCorner, VoxelCorner fineCorner) const
    {
        return (materialMask & voxelBit(coarseCorner, fineCorner)) != 0 ? VoxelState::Material : VoxelState::Empty;
    }

    // 设置两级角点路径对应最高层体素的状态。
    void setState(VoxelCorner coarseCorner, VoxelCorner fineCorner, VoxelState stateValue)
    {
        assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

        const std::uint64_t bit = voxelBit(coarseCorner, fineCorner);

        if (stateValue == VoxelState::Material)
        {
            materialMask |= bit;
        }
        else
        {
            materialMask &= ~bit;
        }
    }

    // 将两级角点路径对应最高层体素设置为材料。
    void setMaterial(VoxelCorner coarseCorner, VoxelCorner fineCorner)
    {
        materialMask |= voxelBit(coarseCorner, fineCorner);
    }

    // 将两级角点路径对应最高层体素设置为空。
    void setEmpty(VoxelCorner coarseCorner, VoxelCorner fineCorner)
    {
        materialMask &= ~voxelBit(coarseCorner, fineCorner);
    }

    /// 八分区状态

    // 返回第一级角点对应八个最高层体素的压缩状态。
    VoxelState octantState(VoxelCorner coarseCorner) const
    {
        const std::uint8_t bits = octantBits(coarseCorner);

        if (bits == 0)
        {
            return VoxelState::Empty;
        }

        if (bits == 0xFF)
        {
            return VoxelState::Material;
        }

        return VoxelState::Subdivided;
    }

    // 将第一级角点对应的八个最高层体素统一设置为指定叶状态。
    void setOctantState(VoxelCorner coarseCorner, VoxelState stateValue)
    {
        assert(stateValue == VoxelState::Empty || stateValue == VoxelState::Material);

        const std::uint64_t mask = octantMask(coarseCorner);

        if (stateValue == VoxelState::Material)
        {
            materialMask |= mask;
        }
        else
        {
            materialMask &= ~mask;
        }
    }

    // 返回第一级角点对应的八位材料掩码。
    std::uint8_t octantBits(VoxelCorner coarseCorner) const
    {
        const unsigned int cornerIndex = checkedCornerIndex(coarseCorner);
        return static_cast<std::uint8_t>((materialMask >> (cornerIndex * VoxelsPerOctant)) & 0xFFU);
    }

    // 返回全部为空的第一级八分区掩码。
    std::uint8_t emptyOctantMask() const
    {
        std::uint8_t result = 0;

        for (unsigned int cornerIndex = 0; cornerIndex < OctantCount; ++cornerIndex)
        {
            if (octantBits(static_cast<VoxelCorner>(cornerIndex)) == 0)
            {
                result = static_cast<std::uint8_t>(result | static_cast<std::uint8_t>(1U << cornerIndex));
            }
        }

        return result;
    }

    // 返回全部为材料的第一级八分区掩码。
    std::uint8_t materialOctantMask() const
    {
        std::uint8_t result = 0;

        for (unsigned int cornerIndex = 0; cornerIndex < OctantCount; ++cornerIndex)
        {
            if (octantBits(static_cast<VoxelCorner>(cornerIndex)) == 0xFF)
            {
                result = static_cast<std::uint8_t>(result | static_cast<std::uint8_t>(1U << cornerIndex));
            }
        }

        return result;
    }

    // 返回同时包含空体素和材料体素的第一级八分区掩码。
    std::uint8_t mixedOctantMask() const
    {
        return static_cast<std::uint8_t>(~static_cast<std::uint8_t>(emptyOctantMask() | materialOctantMask()));
    }

    /// 掩码布尔运算

    // 执行当前叶块减去工具叶块的材料掩码运算，返回结果是否改变。
    bool cut(const VoxelLeafBlock& tool)
    {
        const std::uint64_t previousMask = materialMask;
        materialMask &= ~tool.materialMask;
        return materialMask != previousMask;
    }

    // 执行当前叶块与另一叶块的材料并集，返回结果是否改变。
    bool fuse(const VoxelLeafBlock& other)
    {
        const std::uint64_t previousMask = materialMask;
        materialMask |= other.materialMask;
        return materialMask != previousMask;
    }

    // 执行当前叶块与另一叶块的材料交集，返回结果是否改变。
    bool common(const VoxelLeafBlock& other)
    {
        const std::uint64_t previousMask = materialMask;
        materialMask &= other.materialMask;
        return materialMask != previousMask;
    }

    // 执行当前叶块与另一叶块的材料异或，返回结果是否改变。
    bool exclusiveOr(const VoxelLeafBlock& other)
    {
        const std::uint64_t previousMask = materialMask;
        materialMask ^= other.materialMask;
        return materialMask != previousMask;
    }

    /// 状态与统计

    // 检查叶块是否不包含材料体素。
    bool isEmpty() const
    {
        return materialMask == 0;
    }

    // 检查叶块是否全部由材料体素组成。
    bool isFull() const
    {
        return materialMask == fullMask();
    }

    // 返回叶块中的材料体素数量。
    std::size_t materialVoxelCount() const
    {
        std::uint64_t remainingMask = materialMask;
        std::size_t count = 0;

        while (remainingMask != 0)
        {
            remainingMask &= remainingMask - 1;
            ++count;
        }

        return count;
    }

    /// 索引映射

    // 返回两级角点路径在64位材料掩码中的位索引。
    static unsigned int voxelBitIndex(VoxelCorner coarseCorner, VoxelCorner fineCorner)
    {
        return checkedCornerIndex(coarseCorner) * VoxelsPerOctant + checkedCornerIndex(fineCorner);
    }

    // 返回两级角点路径对应的单体素位掩码。
    static std::uint64_t voxelBit(VoxelCorner coarseCorner, VoxelCorner fineCorner)
    {
        return static_cast<std::uint64_t>(1) << voxelBitIndex(coarseCorner, fineCorner);
    }

    // 返回第一级角点对应八个连续最高层体素的位掩码。
    static std::uint64_t octantMask(VoxelCorner coarseCorner)
    {
        return static_cast<std::uint64_t>(0xFF) << (checkedCornerIndex(coarseCorner) * VoxelsPerOctant);
    }

    // 返回全部64个最高层体素对应的位掩码。
    static std::uint64_t fullMask()
    {
        return ~static_cast<std::uint64_t>(0);
    }

    // 检查并返回指定角点的无符号索引。
    static unsigned int checkedCornerIndex(VoxelCorner corner)
    {
        const unsigned int cornerIndex = static_cast<unsigned int>(corner);

        assert(cornerIndex < OctantCount);
        return cornerIndex;
    }

    std::uint64_t materialMask; // 两级角点按coarse×8+fine映射到连续64位材料状态。
};

static_assert(sizeof(VoxelLeafBlock) == 8, "VoxelLeafBlock must be exactly 8 bytes.");

}

#endif // MYVOXEL_VOXELLEAFBLOCK_H