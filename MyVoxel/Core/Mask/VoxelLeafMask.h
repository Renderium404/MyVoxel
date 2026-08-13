#ifndef MYVOXEL_CORE_MASK_VOXELLEAFMASK_H
#define MYVOXEL_CORE_MASK_VOXELLEAFMASK_H

#include <cassert>
#include <cmath>
#include <cstdint>

#include "MyVoxel/Core/Mask/MaskUtils.h"
#include "MyVoxel/Core/Storage/LeafBlock.h"
#include "MyVoxel/Core/Storage/MaskBlock.h"

namespace MyVoxel
{

static_assert(VoxelCornerCount == 8, "VoxelLeafMask requires exactly eight voxel corners.");

const unsigned int VoxelLeafAxisCellCount = 4; // 一个MaskLeaf在每个坐标轴方向包含四个细层体素。
const unsigned int VoxelLeafGroupCount = 8; // 一个MaskLeaf包含八个粗层体素组。
const unsigned int VoxelLeafCellCount = 64; // 一个MaskLeaf包含4×4×4个细层体素。

const std::uint64_t EmptyVoxelMaterialMask = static_cast<std::uint64_t>(0); // 不包含任何材料体素的64位材料掩码。
const std::uint64_t FullVoxelMaterialMask = ~static_cast<std::uint64_t>(0); // 全部64个体素均包含材料的材料掩码。
const std::uint8_t EmptyVoxelLeafGroupMask = static_cast<std::uint8_t>(0); // 不包含任何细层材料位的八位组。
const std::uint8_t FullVoxelLeafGroupMask = static_cast<std::uint8_t>(0xFFU); // 八个细层体素全部包含材料的八位组。

static_assert(VoxelLeafCellCount == MaskBlockSampleCount, "VoxelLeafMask and MaskBlock sample counts must match.");
static_assert(VoxelLeafCellCount == LeafBlockSampleCount, "VoxelLeafMask and LeafBlock sample counts must match.");

const std::uint64_t LeafByteLowSevenBitsMask = 0x7F7F7F7F7F7F7F7FULL; // 每个字节低七位，用于无跨字节进位的零字节检测。
const std::uint64_t LeafByteHighBitMask = 0x8080808080808080ULL; // 每个粗层八位组的最高位。
const std::uint64_t LeafGroupCompressMultiplier = 0x0002040810204081ULL; // 将八个字节最高位压缩到低八位。
const std::uint64_t LeafGroupSpreadStage1Mask = 0x0000000F0000000FULL; // 八位组展开第一阶段分布掩码。
const std::uint64_t LeafGroupSpreadStage2Mask = 0x0003000300030003ULL; // 八位组展开第二阶段分布掩码。
const std::uint64_t LeafGroupSpreadStage3Mask = 0x0101010101010101ULL; // 八位组展开到每个字节最低位的分布掩码。

// 保存MaskLeaf中八个粗层逻辑体素的批量材料状态。
struct VoxelLeafGroupStateMasks
{
    std::uint8_t empty = 0; // 八个细层体素全部为空的粗层体素组。
    std::uint8_t material = 0; // 八个细层体素全部包含材料的粗层体素组。
    std::uint8_t subdivided = 0; // 同时包含Empty和Material细层体素的粗层体素组。
    std::uint8_t terminal = 0; // 材料状态完全一致的粗层体素组。

    // 检查三个逻辑状态是否互不重叠并完整覆盖八个粗层体素组。
    bool isValid() const
    {
        return (empty & material) == 0 &&
               (empty & subdivided) == 0 &&
               (material & subdivided) == 0 &&
               static_cast<std::uint8_t>(empty | material | subdivided) == static_cast<std::uint8_t>(0xFFU) &&
               terminal == static_cast<std::uint8_t>(empty | material);
    }
};

/// 整体材料状态

// 判断MaskBlock是否完全为空。
inline bool isMaskBlockEmpty(const MaskBlock& block)
{
    return block.materialMask == EmptyVoxelMaterialMask;
}

// 判断MaskBlock是否完全包含材料。
inline bool isMaskBlockMaterial(const MaskBlock& block)
{
    return block.materialMask == FullVoxelMaterialMask;
}

// 判断MaskBlock是否同时包含空和材料样本。
inline bool isMaskBlockMixed(const MaskBlock& block)
{
    return !isMaskBlockEmpty(block) && !isMaskBlockMaterial(block);
}

// 返回64位材料掩码整体对应的二值聚合状态，不表示MaskLeaf可以折叠。
inline VoxelState materialMaskState(std::uint64_t materialMask)
{
    if (materialMask == EmptyVoxelMaterialMask)
    {
        return VoxelState::Empty;
    }

    if (materialMask == FullVoxelMaterialMask)
    {
        return VoxelState::Material;
    }

    return VoxelState::Subdivided;
}

// 返回MaskBlock整体对应的二值材料聚合状态。
inline VoxelState maskBlockState(const MaskBlock& block)
{
    return materialMaskState(block.materialMask);
}

/// 材料位访问

// 返回指定样本索引是否包含材料。
inline bool leafMaterialBit(const MaskBlock& block, unsigned int sampleIndex)
{
    assert(sampleIndex < MaskBlockSampleCount);
    return maskBit(block.materialMask, sampleIndex);
}

// 返回粗层角点和细层角点对应的样本是否包含材料。
inline bool leafMaterialBit(const MaskBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return leafMaterialBit(block, leafSampleIndex(coarseCorner, fineCorner));
}

// 设置指定样本索引的材料状态。
inline void setLeafMaterialBit(MaskBlock& block, unsigned int sampleIndex, bool material)
{
    assert(sampleIndex < MaskBlockSampleCount);
    setMaskBit(block.materialMask, sampleIndex, material);
}

// 设置粗层角点和细层角点对应的材料状态。
inline void setLeafMaterialBit(MaskBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner, bool material)
{
    setLeafMaterialBit(block, leafSampleIndex(coarseCorner, fineCorner), material);
}

/// 细层体素坐标映射

// 返回粗层角点在64位叶掩码中的八位组偏移。
inline unsigned int leafGroupOffset(VoxelCorner coarseCorner)
{
    const unsigned int coarseIndex = static_cast<unsigned int>(coarseCorner);

    assert(coarseIndex < static_cast<unsigned int>(VoxelCornerCount));

    return coarseIndex * static_cast<unsigned int>(VoxelCornerCount);
}

// 返回叶块局部坐标对应的粗层角点，三个坐标范围均为[0,3]。
inline VoxelCorner leafCellCoarseCorner(unsigned int x, unsigned int y, unsigned int z)
{
    assert(x < VoxelLeafAxisCellCount);
    assert(y < VoxelLeafAxisCellCount);
    assert(z < VoxelLeafAxisCellCount);

    return static_cast<VoxelCorner>(((x >> 1U) & 1U) | (((y >> 1U) & 1U) << 1U) | (((z >> 1U) & 1U) << 2U));
}

// 返回叶块局部坐标对应的细层角点，三个坐标范围均为[0,3]。
inline VoxelCorner leafCellFineCorner(unsigned int x, unsigned int y, unsigned int z)
{
    assert(x < VoxelLeafAxisCellCount);
    assert(y < VoxelLeafAxisCellCount);
    assert(z < VoxelLeafAxisCellCount);

    return static_cast<VoxelCorner>((x & 1U) | ((y & 1U) << 1U) | ((z & 1U) << 2U));
}

// 返回叶块局部坐标对应的64位材料位索引。
inline unsigned int leafCellBitIndex(unsigned int x, unsigned int y, unsigned int z)
{
    return leafSampleIndex(leafCellCoarseCorner(x, y, z), leafCellFineCorner(x, y, z));
}

// 返回叶块局部坐标对应的64位材料位。
inline std::uint64_t leafCellBit(unsigned int x, unsigned int y, unsigned int z)
{
    return maskBitValue(leafCellBitIndex(x, y, z));
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

// 判断指定叶块局部位置是否包含材料。
inline bool leafCellMaterial(const MaskBlock& block, unsigned int x, unsigned int y, unsigned int z)
{
    return leafMaterialBit(block, leafCellBitIndex(x, y, z));
}

/// 粗层体素组

// 返回指定粗层角点对应的连续八位材料掩码。
inline std::uint64_t leafGroupMask(VoxelCorner coarseCorner)
{
    return static_cast<std::uint64_t>(0xFFULL << leafGroupOffset(coarseCorner));
}

// 返回指定粗层角点对应的八位细层材料状态。
inline std::uint8_t leafGroupBits(std::uint64_t materialMask, VoxelCorner coarseCorner)
{
    return static_cast<std::uint8_t>((materialMask >> leafGroupOffset(coarseCorner)) & 0xFFULL);
}

// 返回MaskBlock中指定粗层角点对应的八位细层材料状态。
inline std::uint8_t leafGroupBits(const MaskBlock& block, VoxelCorner coarseCorner)
{
    return leafGroupBits(block.materialMask, coarseCorner);
}

// 返回指定粗层体素组对应的二值材料聚合状态，不表示该组距离Value一致。
inline VoxelState leafGroupState(std::uint64_t materialMask, VoxelCorner coarseCorner)
{
    const std::uint8_t groupBits = leafGroupBits(materialMask, coarseCorner);

    if (groupBits == EmptyVoxelLeafGroupMask)
    {
        return VoxelState::Empty;
    }

    if (groupBits == FullVoxelLeafGroupMask)
    {
        return VoxelState::Material;
    }

    return VoxelState::Subdivided;
}

// 返回MaskBlock中指定粗层体素组对应的二值材料聚合状态。
inline VoxelState leafGroupState(const MaskBlock& block, VoxelCorner coarseCorner)
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

// 返回64位材料掩码中完全为空的粗层体素组。
inline std::uint8_t leafEmptyGroupMask(std::uint64_t materialMask)
{
    return compressLeafGroupHighBits(leafZeroByteHighBits(materialMask));
}

// 返回64位材料掩码中完全包含材料的粗层体素组。
inline std::uint8_t leafMaterialGroupMask(std::uint64_t materialMask)
{
    return compressLeafGroupHighBits(leafZeroByteHighBits(~materialMask));
}

// 返回64位材料掩码中同时包含空和材料的粗层体素组。
inline std::uint8_t leafSubdividedGroupMask(std::uint64_t materialMask)
{
    const std::uint8_t terminalMask = static_cast<std::uint8_t>(leafEmptyGroupMask(materialMask) | leafMaterialGroupMask(materialMask));
    return static_cast<std::uint8_t>(~terminalMask);
}

// 一次返回64位材料掩码中八个粗层逻辑体素的全部状态。
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

// 一次返回MaskBlock中八个粗层逻辑体素的全部状态。
inline VoxelLeafGroupStateMasks leafGroupStateMasks(const MaskBlock& block)
{
    return leafGroupStateMasks(block.materialMask);
}

/// 粗层组展开与选择

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

/// 材料掩码修改

// 替换MaskBlock中的全部64位材料状态，返回内容是否发生变化。
inline bool setLeafMaterialMask(MaskBlock& block, std::uint64_t materialMask)
{
    if (block.materialMask == materialMask)
    {
        return false;
    }

    block.materialMask = materialMask;
    return true;
}

// 设置指定材料位集合，返回内容是否发生变化。
inline bool setLeafMaterialBits(MaskBlock& block, std::uint64_t cellMask, bool material)
{
    const std::uint64_t targetMask = material ? block.materialMask | cellMask : block.materialMask & ~cellMask;
    return setLeafMaterialMask(block, targetMask);
}

// 设置指定叶块局部体素材料状态，返回内容是否发生变化。
inline bool setLeafCellMaterial(MaskBlock& block, unsigned int x, unsigned int y, unsigned int z, bool material)
{
    return setLeafMaterialBits(block, leafCellBit(x, y, z), material);
}

// 替换指定粗层体素组中的八位细层材料状态，返回内容是否发生变化。
inline bool setLeafGroupBits(MaskBlock& block, VoxelCorner coarseCorner, std::uint8_t groupBits)
{
    const unsigned int offset = leafGroupOffset(coarseCorner);
    const std::uint64_t groupMask = static_cast<std::uint64_t>(0xFFULL << offset);
    const std::uint64_t targetMask = (block.materialMask & ~groupMask) | (static_cast<std::uint64_t>(groupBits) << offset);
    return setLeafMaterialMask(block, targetMask);
}

// 将指定粗层体素组设置为空或材料状态，返回内容是否发生变化。
inline bool setLeafGroupState(MaskBlock& block, VoxelCorner coarseCorner, VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    return setLeafGroupBits(block, coarseCorner, state == VoxelState::Material ? FullVoxelLeafGroupMask : EmptyVoxelLeafGroupMask);
}

// 将指定多个粗层体素组统一设置为空或材料状态，返回内容是否发生变化。
inline bool setLeafGroupStates(MaskBlock& block, std::uint8_t groupMask, VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    return setLeafMaterialBits(block, expandLeafGroupMask(groupMask), state == VoxelState::Material);
}

// 反转指定材料位集合，返回是否实际处理了任何位置。
inline bool invertLeafMaterialBits(MaskBlock& block, std::uint64_t cellMask)
{
    if (cellMask == 0)
    {
        return false;
    }

    block.materialMask ^= cellMask;
    return true;
}

// 反转指定粗层体素组中的全部材料位，返回是否实际处理了任何位置。
inline bool invertLeafGroupStates(MaskBlock& block, std::uint8_t groupMask)
{
    return invertLeafMaterialBits(block, expandLeafGroupMask(groupMask));
}

// 反转全部64个材料状态。
inline void invertLeafMaterialMask(MaskBlock& block)
{
    block.materialMask = ~block.materialMask;
}

/// 距离与材料联合操作

// 同时初始化距离叶块和材料掩码，距离小于等于零视为材料。
inline void reset(MaskBlock& maskBlock, LeafBlock& leafBlock, float distance)
{
    assert(std::isfinite(static_cast<double>(distance)));

    reset(leafBlock, distance);
    reset(maskBlock, distance <= 0.0f ? VoxelState::Material : VoxelState::Empty);
}

// 同时设置指定样本的有符号距离和材料状态，距离小于等于零视为材料。
inline void setLeafDistance(MaskBlock& maskBlock, LeafBlock& leafBlock, unsigned int sampleIndex, float distance)
{
    assert(sampleIndex < LeafBlockSampleCount);
    assert(std::isfinite(static_cast<double>(distance)));

    leafBlock.distances[sampleIndex] = distance;
    setLeafMaterialBit(maskBlock, sampleIndex, distance <= 0.0f);
}

// 同时设置粗层角点和细层角点对应样本的有符号距离和材料状态。
inline void setLeafDistance(MaskBlock& maskBlock, LeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner, float distance)
{
    setLeafDistance(maskBlock, leafBlock, leafSampleIndex(coarseCorner, fineCorner), distance);
}

// 根据当前64个距离样本重新生成完整材料掩码。
inline void rebuildMask(const LeafBlock& leafBlock, MaskBlock& maskBlock)
{
    maskBlock.materialMask = 0;

    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        if (leafBlock.distances[sampleIndex] <= 0.0f)
        {
            maskBlock.materialMask |= maskBitValue(sampleIndex);
        }
    }
}

// 检查材料掩码是否与全部距离样本的材料判定一致。
inline bool isMaskConsistent(const MaskBlock& maskBlock, const LeafBlock& leafBlock)
{
    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        const float distance = leafBlock.distances[sampleIndex];

        if (!std::isfinite(static_cast<double>(distance)) || leafMaterialBit(maskBlock, sampleIndex) != (distance <= 0.0f))
        {
            return false;
        }
    }

    return true;
}

// 返回MaskBlock中的材料细层体素数量。
inline unsigned int leafMaterialCellCount(const MaskBlock& block)
{
    return maskBitCount(block.materialMask);
}

}

#endif // MYVOXEL_CORE_MASK_VOXELLEAFMASK_H