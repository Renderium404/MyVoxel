#ifndef MYVOXEL_CORE_MASK_VOXELLEAFMASK_H
#define MYVOXEL_CORE_MASK_VOXELLEAFMASK_H

#include <cassert>
#include <cstdint>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "MyVoxel/Core/Storage/VoxelBlock.h"

namespace MyVoxel
{

static_assert(VoxelCornerCount == 8, "VoxelLeafMask requires exactly eight voxel corners.");

const unsigned int VoxelLeafAxisCellCount = 4; // 一个掩码叶块在每个坐标轴方向包含四个细层体素。
const unsigned int VoxelLeafGroupCount = 8; // 一个掩码叶块包含八个粗层体素组。
const unsigned int VoxelLeafCellCount = 64; // 一个掩码叶块包含4×4×4个细层体素。

const std::uint64_t EmptyVoxelLeafMask = static_cast<std::uint64_t>(0); // 完全为空的64位叶块材料掩码。
const std::uint64_t FullVoxelLeafMask = ~static_cast<std::uint64_t>(0); // 完全包含材料的64位叶块材料掩码。

const std::uint8_t EmptyVoxelLeafGroupMask = static_cast<std::uint8_t>(0); // 不包含任何粗层体素组的八位掩码。
const std::uint8_t FullVoxelLeafGroupMask = static_cast<std::uint8_t>(0xFFU); // 包含全部粗层体素组的八位掩码。

const std::uint64_t LeafByteLowSevenBitsMask = 0x7F7F7F7F7F7F7F7FULL; // 每个字节的低七位，用于无跨字节进位的零字节检测。
const std::uint64_t LeafByteHighBitMask = 0x8080808080808080ULL; // 每个粗层八位组的最高位。
const std::uint64_t LeafGroupCompressMultiplier = 0x0002040810204081ULL; // 将八个字节最高位压缩到低八位。
const std::uint64_t LeafGroupSpreadStage1Mask = 0x0000000F0000000FULL; // 八位组展开第一阶段使用的位分布掩码。
const std::uint64_t LeafGroupSpreadStage2Mask = 0x0003000300030003ULL; // 八位组展开第二阶段使用的位分布掩码。
const std::uint64_t LeafGroupSpreadStage3Mask = 0x0101010101010101ULL; // 八位组展开到每个字节最低位的分布掩码。

const std::uint64_t LeafBitCountPairMask = 0x5555555555555555ULL; // 64位并行置位计数使用的相邻位掩码。
const std::uint64_t LeafBitCountTwoBitMask = 0x3333333333333333ULL; // 64位并行置位计数使用的两位组掩码。
const std::uint64_t LeafBitCountNibbleMask = 0x0F0F0F0F0F0F0F0FULL; // 64位并行置位计数使用的半字节掩码。
const std::uint64_t LeafBitCountByteMultiplier = 0x0101010101010101ULL; // 将八个字节中的局部计数累加到最高字节。

// 保存掩码叶块中八个粗层逻辑体素的批量状态掩码。
struct VoxelLeafGroupStateMasks
{
    std::uint8_t empty = 0;         // 完全为空的粗层体素组。
    std::uint8_t material = 0;      // 完全包含材料的粗层体素组。
    std::uint8_t subdivided = 0;    // 同时包含空和材料细层体素的粗层体素组。
    std::uint8_t terminal = 0;      // 完全为空或完全包含材料的粗层体素组。

    // 检查三个逻辑状态是否互不重叠并完整覆盖八个粗层体素组。
    bool isValid() const
    {
        return (empty & material) == 0 &&
               (empty & subdivided) == 0 &&
               (material & subdivided) == 0 &&
               static_cast<std::uint8_t>(empty | material | subdivided) == FullVoxelLeafGroupMask &&
               terminal == static_cast<std::uint8_t>(empty | material);
    }
};

/// 叶块整体状态

// 判断64位叶块材料掩码是否完全为空。
inline bool isLeafMaskEmpty(std::uint64_t materialMask)
{
    return materialMask == EmptyVoxelLeafMask;
}

// 判断64位叶块材料掩码是否完全包含材料。
inline bool isLeafMaskMaterial(std::uint64_t materialMask)
{
    return materialMask == FullVoxelLeafMask;
}

// 判断叶块是否完全为空。
inline bool isLeafBlockEmpty(const VoxelLeafBlock& block)
{
    return isLeafMaskEmpty(block.materialMask);
}

// 判断叶块是否完全包含材料。
inline bool isLeafBlockMaterial(const VoxelLeafBlock& block)
{
    return isLeafMaskMaterial(block.materialMask);
}

// 判断叶块是否可以整体折叠为空或材料终止体素。
inline bool isLeafBlockCollapsible(const VoxelLeafBlock& block)
{
    return isLeafBlockEmpty(block) || isLeafBlockMaterial(block);
}

// 返回64位叶块材料掩码整体对应的逻辑状态。
inline VoxelState leafMaskState(std::uint64_t materialMask)
{
    if (isLeafMaskEmpty(materialMask))
    {
        return VoxelState::Empty;
    }

    if (isLeafMaskMaterial(materialMask))
    {
        return VoxelState::Material;
    }

    return VoxelState::Subdivided;
}

// 返回叶块整体对应的逻辑状态。
inline VoxelState leafBlockState(const VoxelLeafBlock& block)
{
    return leafMaskState(block.materialMask);
}

/// 细层体素位映射

// 返回叶块局部坐标对应的粗层角点，三个坐标范围均为[0, 3]。
inline VoxelCorner leafCellCoarseCorner(unsigned int x, unsigned int y, unsigned int z)
{
    assert(x < VoxelLeafAxisCellCount);
    assert(y < VoxelLeafAxisCellCount);
    assert(z < VoxelLeafAxisCellCount);

    const unsigned int cornerIndex = ((x >> 1U) & 1U) | (((y >> 1U) & 1U) << 1U) | (((z >> 1U) & 1U) << 2U);
    return static_cast<VoxelCorner>(cornerIndex);
}

// 返回叶块局部坐标对应的细层角点，三个坐标范围均为[0, 3]。
inline VoxelCorner leafCellFineCorner(unsigned int x, unsigned int y, unsigned int z)
{
    assert(x < VoxelLeafAxisCellCount);
    assert(y < VoxelLeafAxisCellCount);
    assert(z < VoxelLeafAxisCellCount);

    const unsigned int cornerIndex = (x & 1U) | ((y & 1U) << 1U) | ((z & 1U) << 2U);
    return static_cast<VoxelCorner>(cornerIndex);
}

// 返回叶块局部坐标对应的64位材料位索引。
inline unsigned int leafCellBitIndex(unsigned int x, unsigned int y, unsigned int z)
{
    const unsigned int coarseIndex = static_cast<unsigned int>(leafCellCoarseCorner(x, y, z));
    const unsigned int fineIndex = static_cast<unsigned int>(leafCellFineCorner(x, y, z));
    return coarseIndex * static_cast<unsigned int>(VoxelCornerCount) + fineIndex;
}

// 返回叶块局部坐标对应的64位材料位。
inline std::uint64_t leafCellBit(unsigned int x, unsigned int y, unsigned int z)
{
    return static_cast<std::uint64_t>(1ULL << leafCellBitIndex(x, y, z));
}

// 返回64位材料位索引对应的粗层角点。
inline VoxelCorner leafBitCoarseCorner(unsigned int bitIndex)
{
    assert(bitIndex < VoxelLeafCellCount);
    return static_cast<VoxelCorner>(bitIndex >> 3U);
}

// 返回64位材料位索引对应的细层角点。
inline VoxelCorner leafBitFineCorner(unsigned int bitIndex)
{
    assert(bitIndex < VoxelLeafCellCount);
    return static_cast<VoxelCorner>(bitIndex & 7U);
}

// 返回64位材料位索引对应的叶块局部X坐标。
inline unsigned int leafBitX(unsigned int bitIndex)
{
    assert(bitIndex < VoxelLeafCellCount);
    return ((bitIndex >> 0U) & 1U) | (((bitIndex >> 3U) & 1U) << 1U);
}

// 返回64位材料位索引对应的叶块局部Y坐标。
inline unsigned int leafBitY(unsigned int bitIndex)
{
    assert(bitIndex < VoxelLeafCellCount);
    return ((bitIndex >> 1U) & 1U) | (((bitIndex >> 4U) & 1U) << 1U);
}

// 返回64位材料位索引对应的叶块局部Z坐标。
inline unsigned int leafBitZ(unsigned int bitIndex)
{
    assert(bitIndex < VoxelLeafCellCount);
    return ((bitIndex >> 2U) & 1U) | (((bitIndex >> 5U) & 1U) << 1U);
}

// 判断指定叶块局部体素是否包含材料。
inline bool leafCellMaterial(std::uint64_t materialMask, unsigned int x, unsigned int y, unsigned int z)
{
    return (materialMask & leafCellBit(x, y, z)) != 0;
}

// 判断指定叶块局部体素是否包含材料。
inline bool leafCellMaterial(const VoxelLeafBlock& block, unsigned int x, unsigned int y, unsigned int z)
{
    return leafCellMaterial(block.materialMask, x, y, z);
}

/// 粗层体素组

// 返回指定粗层角点对应的连续八位材料掩码。
inline std::uint64_t leafGroupMask(VoxelCorner coarseCorner)
{
    return static_cast<std::uint64_t>(0xFFULL << leafMaskOffset(coarseCorner));
}

// 返回指定粗层角点对应的八位细层材料状态。
inline std::uint8_t leafGroupBits(std::uint64_t materialMask, VoxelCorner coarseCorner)
{
    return maskBits(materialMask, coarseCorner);
}

// 返回叶块中指定粗层角点对应的八位细层材料状态。
inline std::uint8_t leafGroupBits(const VoxelLeafBlock& block, VoxelCorner coarseCorner)
{
    return leafGroupBits(block.materialMask, coarseCorner);
}

// 返回指定粗层体素组对应的逻辑状态。
inline VoxelState leafGroupState(std::uint64_t materialMask, VoxelCorner coarseCorner)
{
    const std::uint8_t groupBits = leafGroupBits(materialMask, coarseCorner);

    if (groupBits == static_cast<std::uint8_t>(0))
    {
        return VoxelState::Empty;
    }

    if (groupBits == static_cast<std::uint8_t>(0xFFU))
    {
        return VoxelState::Material;
    }

    return VoxelState::Subdivided;
}

// 返回叶块中指定粗层体素组对应的逻辑状态。
inline VoxelState leafGroupState(const VoxelLeafBlock& block, VoxelCorner coarseCorner)
{
    return leafGroupState(block.materialMask, coarseCorner);
}

/// 粗层状态批量归约

// 返回每个值为零的字节对应的最高位。
inline std::uint64_t leafZeroByteHighBits(std::uint64_t value)
{
    const std::uint64_t lowSevenBitSum = (value & LeafByteLowSevenBitsMask) + LeafByteLowSevenBitsMask;
    return static_cast<std::uint64_t>(~(lowSevenBitSum | value | LeafByteLowSevenBitsMask)) & LeafByteHighBitMask;
}

// 将八个字节的最高位压缩为低八位组掩码。
inline std::uint8_t compressLeafGroupHighBits(std::uint64_t highBits)
{
    return static_cast<std::uint8_t>((highBits * LeafGroupCompressMultiplier) >> 56U);
}

// 返回64位叶块中完全为空的粗层体素组。
inline std::uint8_t leafEmptyGroupMask(std::uint64_t materialMask)
{
    return compressLeafGroupHighBits(leafZeroByteHighBits(materialMask));
}

// 返回64位叶块中完全包含材料的粗层体素组。
inline std::uint8_t leafMaterialGroupMask(std::uint64_t materialMask)
{
    return compressLeafGroupHighBits(leafZeroByteHighBits(~materialMask));
}

// 返回64位叶块中同时包含空和材料的粗层体素组。
inline std::uint8_t leafSubdividedGroupMask(std::uint64_t materialMask)
{
    const std::uint8_t terminalMask = static_cast<std::uint8_t>(leafEmptyGroupMask(materialMask) | leafMaterialGroupMask(materialMask));
    return static_cast<std::uint8_t>(~terminalMask);
}

// 返回64位叶块中为空或材料终止状态的粗层体素组。
inline std::uint8_t leafTerminalGroupMask(std::uint64_t materialMask)
{
    return static_cast<std::uint8_t>(leafEmptyGroupMask(materialMask) | leafMaterialGroupMask(materialMask));
}

// 一次返回64位叶块中八个粗层逻辑体素的全部状态掩码。
inline VoxelLeafGroupStateMasks leafGroupStateMasks(std::uint64_t materialMask)
{
    VoxelLeafGroupStateMasks result;

    result.empty = leafEmptyGroupMask(materialMask);
    result.material = leafMaterialGroupMask(materialMask);
    result.terminal = static_cast<std::uint8_t>(result.empty | result.material);
    result.subdivided = static_cast<std::uint8_t>(~result.terminal);

    assert(result.isValid());
    return result;
}

// 一次返回叶块中八个粗层逻辑体素的全部状态掩码。
inline VoxelLeafGroupStateMasks leafGroupStateMasks(const VoxelLeafBlock& block)
{
    return leafGroupStateMasks(block.materialMask);
}

/// 粗层组掩码展开

// 将八位粗层组位置展开为64位掩码，每个置位组展开为一个完整0xFF字节。
inline std::uint64_t expandLeafGroupMask(std::uint8_t groupMask)
{
    std::uint64_t expanded = static_cast<std::uint64_t>(groupMask);

    expanded = (expanded | (expanded << 28U)) & LeafGroupSpreadStage1Mask;
    expanded = (expanded | (expanded << 14U)) & LeafGroupSpreadStage2Mask;
    expanded = (expanded | (expanded << 7U)) & LeafGroupSpreadStage3Mask;

    return expanded * static_cast<std::uint64_t>(0xFFULL);
}

// 返回指定粗层组位置中当前包含的全部材料位。
inline std::uint64_t selectLeafGroups(std::uint64_t materialMask, std::uint8_t groupMask)
{
    return materialMask & expandLeafGroupMask(groupMask);
}

/// 置位统计与遍历

// 返回64位叶块掩码中的置位数量。
inline unsigned int leafMaskBitCount(std::uint64_t mask)
{
    mask = mask - ((mask >> 1U) & LeafBitCountPairMask);
    mask = (mask & LeafBitCountTwoBitMask) + ((mask >> 2U) & LeafBitCountTwoBitMask);
    mask = (mask + (mask >> 4U)) & LeafBitCountNibbleMask;

    return static_cast<unsigned int>((mask * LeafBitCountByteMultiplier) >> 56U);
}

// 返回叶块中的材料细层体素数量。
inline unsigned int leafMaterialCellCount(const VoxelLeafBlock& block)
{
    return leafMaskBitCount(block.materialMask);
}

// 返回非零64位掩码中最低置位的位索引。
inline unsigned int firstLeafBitIndex(std::uint64_t mask)
{
    assert(mask != 0);

#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long bitIndex = 0;
    _BitScanForward64(&bitIndex, mask);
    return static_cast<unsigned int>(bitIndex);
#elif defined(_MSC_VER)
    unsigned long bitIndex = 0;
    const unsigned long lowerMask = static_cast<unsigned long>(mask & 0xFFFFFFFFULL);

    if (_BitScanForward(&bitIndex, lowerMask))
    {
        return static_cast<unsigned int>(bitIndex);
    }

    const unsigned long upperMask = static_cast<unsigned long>(mask >> 32U);
    const unsigned char found = _BitScanForward(&bitIndex, upperMask);

    assert(found != 0);
    return static_cast<unsigned int>(bitIndex) + 32U;
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<unsigned int>(__builtin_ctzll(mask));
#else
    unsigned int bitIndex = 0;

    while ((mask & static_cast<std::uint64_t>(1ULL)) == 0)
    {
        mask >>= 1U;
        ++bitIndex;
    }

    return bitIndex;
#endif
}

// 返回最低置位的位索引，并从原掩码中删除该位。
inline unsigned int takeFirstLeafBitIndex(std::uint64_t& mask)
{
    const unsigned int bitIndex = firstLeafBitIndex(mask);
    mask &= mask - static_cast<std::uint64_t>(1ULL);
    return bitIndex;
}

/// 材料位修改

// 替换叶块中的全部64位材料状态，返回内容是否发生变化。
inline bool setLeafMaterialMask(VoxelLeafBlock& block, std::uint64_t materialMask)
{
    if (block.materialMask == materialMask)
    {
        return false;
    }

    block.materialMask = materialMask;
    return true;
}

// 设置指定材料位，返回内容是否发生变化。
inline bool setLeafMaterialBits(VoxelLeafBlock& block, std::uint64_t cellMask, bool material)
{
    const std::uint64_t currentMask = block.materialMask;
    const std::uint64_t targetMask = material ? currentMask | cellMask : currentMask & ~cellMask;

    return setLeafMaterialMask(block, targetMask);
}

// 设置指定叶块局部体素的材料状态，返回内容是否发生变化。
inline bool setLeafCellMaterial(VoxelLeafBlock& block, unsigned int x, unsigned int y, unsigned int z, bool material)
{
    return setLeafMaterialBits(block, leafCellBit(x, y, z), material);
}

// 替换指定粗层体素组中的八位细层材料状态，返回内容是否发生变化。
inline bool setLeafGroupBits(VoxelLeafBlock& block, VoxelCorner coarseCorner, std::uint8_t groupBits)
{
    const unsigned int offset = leafMaskOffset(coarseCorner);
    const std::uint64_t groupMask = static_cast<std::uint64_t>(0xFFULL << offset);
    const std::uint64_t shiftedBits = static_cast<std::uint64_t>(groupBits) << offset;
    const std::uint64_t targetMask = (block.materialMask & ~groupMask) | shiftedBits;

    return setLeafMaterialMask(block, targetMask);
}

// 将指定粗层体素组设置为空或材料状态，返回内容是否发生变化。
inline bool setLeafGroupState(VoxelLeafBlock& block, VoxelCorner coarseCorner, VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    const std::uint8_t groupBits = state == VoxelState::Material ? static_cast<std::uint8_t>(0xFFU) : static_cast<std::uint8_t>(0);
    return setLeafGroupBits(block, coarseCorner, groupBits);
}

// 将指定的多个粗层体素组统一设置为空或材料状态，返回内容是否发生变化。
inline bool setLeafGroupStates(VoxelLeafBlock& block, std::uint8_t groupMask, VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    return setLeafMaterialBits(block, expandLeafGroupMask(groupMask), state == VoxelState::Material);
}

// 反转指定材料位的空和材料状态，返回是否实际处理了任何位置。
inline bool invertLeafMaterialBits(VoxelLeafBlock& block, std::uint64_t cellMask)
{
    if (cellMask == 0)
    {
        return false;
    }

    block.materialMask ^= cellMask;
    return true;
}

// 反转指定粗层体素组中的全部材料位，返回是否实际处理了任何位置。
inline bool invertLeafGroupStates(VoxelLeafBlock& block, std::uint8_t groupMask)
{
    return invertLeafMaterialBits(block, expandLeafGroupMask(groupMask));
}

// 反转叶块中的全部64个细层体素状态。
inline void invertLeafMaterialMask(VoxelLeafBlock& block)
{
    block.materialMask = ~block.materialMask;
}

}

#endif // MYVOXEL_CORE_MASK_VOXELLEAFMASK_H