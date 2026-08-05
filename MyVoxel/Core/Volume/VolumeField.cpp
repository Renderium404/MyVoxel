#include "VolumeField.h"

#include <cmath>

#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断距离值是否为有限数值。
bool isFiniteDistance(float value)
{
    return std::isfinite(static_cast<double>(value));
}

}

namespace MyVoxel
{

VolumeField::VolumeField(VoxelLevel sampleLevel)
    : m_sampleLevel(sampleLevel)
    , m_blockLevel(BaseVoxelLevel)
    , m_completelyDirty(true)
{
    MYVOXEL_ASSERT_MESSAGE(sampleLevel >= static_cast<VoxelLevel>(2), "VolumeField sample level must be at least level 2.");
    m_blockLevel = static_cast<VoxelLevel>(sampleLevel - static_cast<VoxelLevel>(2));

    MYVOXEL_ASSERT_MESSAGE(isValid(), "VolumeField construction produced invalid state.");
}

/// 状态判断

bool VolumeField::isValid() const
{
    if (m_sampleLevel < static_cast<VoxelLevel>(2) || static_cast<unsigned int>(m_blockLevel) + 2U != static_cast<unsigned int>(m_sampleLevel))
    {
        return false;
    }

    if (m_completelyDirty && !m_dirtyBlocks.empty())
    {
        return false;
    }

    if (m_blockIndices.size() != m_blockPool.allocatedBlockCount())
    {
        return false;
    }

    for (BlockIndexMap::const_iterator iterator = m_blockIndices.begin(); iterator != m_blockIndices.end(); ++iterator)
    {
        if (!m_blockPool.containsBlock(iterator->second))
        {
            return false;
        }

        const VolumeBlock& block = m_blockPool.block(iterator->second);

        for (unsigned int sample = 0; sample < VolumeBlockSampleCount; ++sample)
        {
            if (!isFiniteDistance(volumeDistance(block, sample)))
            {
                return false;
            }
        }
    }

    return true;
}

bool VolumeField::isEmpty() const
{
    return m_blockIndices.empty();
}

bool VolumeField::isCurrent() const
{
    return !m_completelyDirty && m_dirtyBlocks.empty();
}

bool VolumeField::isCompletelyDirty() const
{
    return m_completelyDirty;
}

bool VolumeField::isBlockDirty(const VoxelCellAddress& blockAddress) const
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField dirty query requires a block-level address.");
    return m_completelyDirty || m_dirtyBlocks.find(blockAddress.index) != m_dirtyBlocks.end();
}

/// 距离场层级

VoxelLevel VolumeField::sampleLevel() const
{
    return m_sampleLevel;
}

VoxelLevel VolumeField::blockLevel() const
{
    return m_blockLevel;
}

bool VolumeField::supportsBlockAddress(const VoxelCellAddress& blockAddress) const
{
    return blockAddress.level == m_blockLevel;
}

bool VolumeField::supportsSampleAddress(const VoxelCellAddress& sampleAddress) const
{
    return sampleAddress.level == m_sampleLevel;
}

/// 地址转换

VoxelCellAddress VolumeField::blockAddress(const VoxelCellAddress& sampleAddress) const
{
    MYVOXEL_ASSERT_MESSAGE(supportsSampleAddress(sampleAddress), "VolumeField sample address must use the configured sample level.");
    return ancestorCellAddress(sampleAddress, m_blockLevel);
}

unsigned int VolumeField::sampleIndex(const VoxelCellAddress& sampleAddress) const
{
    MYVOXEL_ASSERT_MESSAGE(supportsSampleAddress(sampleAddress), "VolumeField sample address must use the configured sample level.");

    const VoxelCorner fineCorner = childCornerInParent(sampleAddress);
    const VoxelCellAddress coarseAddress = parentCellAddress(sampleAddress);
    const VoxelCorner coarseCorner = childCornerInParent(coarseAddress);
    const unsigned int result = volumeSampleIndex(coarseCorner, fineCorner);

    MYVOXEL_ASSERT_MESSAGE(result < VolumeBlockSampleCount, "VolumeField sample index exceeds VolumeBlock range.");
    return result;
}

VoxelCellAddress VolumeField::sampleAddress(const VoxelCellAddress& blockAddress, unsigned int sampleIndexValue) const
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField sample construction requires a block-level address.");
    MYVOXEL_ASSERT_MESSAGE(sampleIndexValue < VolumeBlockSampleCount, "VolumeField sample index exceeds VolumeBlock range.");

    const VoxelCorner coarseCorner = leafBitCoarseCorner(sampleIndexValue);
    const VoxelCorner fineCorner = leafBitFineCorner(sampleIndexValue);
    const VoxelCellAddress coarseAddress = childCellAddress(blockAddress, coarseCorner);
    const VoxelCellAddress result = childCellAddress(coarseAddress, fineCorner);

    MYVOXEL_ASSERT_MESSAGE(result.level == m_sampleLevel, "VolumeField sample construction produced an unexpected level.");
    return result;
}

VoxelCellAddress VolumeField::sampleAddress(const VoxelCellAddress& blockAddress, unsigned int x, unsigned int y, unsigned int z) const
{
    MYVOXEL_ASSERT_MESSAGE(x < VoxelLeafAxisCellCount && y < VoxelLeafAxisCellCount && z < VoxelLeafAxisCellCount,
                           "VolumeField local sample coordinates must be in range [0, 3].");
    return sampleAddress(blockAddress, leafCellBitIndex(x, y, z));
}

/// 距离块访问

bool VolumeField::containsBlock(const VoxelCellAddress& blockAddress) const
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField block query requires a block-level address.");
    return findBlockIterator(blockAddress) != m_blockIndices.end();
}

const VolumeBlock* VolumeField::findBlock(const VoxelCellAddress& blockAddress) const
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField block query requires a block-level address.");

    const BlockIndexMap::const_iterator iterator = findBlockIterator(blockAddress);
    return iterator == m_blockIndices.end() ? nullptr : &m_blockPool.block(iterator->second);
}

VolumeBlock* VolumeField::editBlock(const VoxelCellAddress& blockAddress)
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField block edit requires a block-level address.");

    const BlockIndexMap::iterator iterator = findBlockIterator(blockAddress);

    if (iterator == m_blockIndices.end())
    {
        return nullptr;
    }

    markBlockDirty(blockAddress);
    return &m_blockPool.block(iterator->second);
}

VolumeBlock& VolumeField::ensureBlock(const VoxelCellAddress& blockAddress, float initialDistance)
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField block creation requires a block-level address.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteDistance(initialDistance), "VolumeField initial distance must be finite.");

    BlockIndexMap::iterator existing = findBlockIterator(blockAddress);

    if (existing != m_blockIndices.end())
    {
        markBlockDirty(blockAddress);
        return m_blockPool.block(existing->second);
    }

    const VoxelIndex blockIndex = m_blockPool.allocateBlock();
    m_blockPool.initializeBlock(blockIndex, initialDistance);

    try
    {
        const std::pair<BlockIndexMap::iterator, bool> inserted = m_blockIndices.insert(std::make_pair(blockAddress.index, blockIndex));

        if (!inserted.second)
        {
            m_blockPool.releaseBlock(blockIndex);
            markBlockDirty(blockAddress);
            return m_blockPool.block(inserted.first->second);
        }

        if (!m_completelyDirty)
        {
            try
            {
                m_dirtyBlocks.insert(blockAddress.index);
            }
            catch (...)
            {
                m_blockIndices.erase(inserted.first);
                m_blockPool.releaseBlock(blockIndex);
                throw;
            }
        }

        return m_blockPool.block(blockIndex);
    }
    catch (...)
    {
        if (m_blockPool.containsBlock(blockIndex))
        {
            m_blockPool.releaseBlock(blockIndex);
        }

        throw;
    }
}

bool VolumeField::eraseBlock(const VoxelCellAddress& blockAddress)
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField block removal requires a block-level address.");

    const BlockIndexMap::iterator iterator = findBlockIterator(blockAddress);
    m_dirtyBlocks.erase(blockAddress.index);

    if (iterator == m_blockIndices.end())
    {
        return false;
    }

    m_blockPool.releaseBlock(iterator->second);
    m_blockIndices.erase(iterator);
    return true;
}

/// 有效状态管理

void VolumeField::markBlockDirty(const VoxelCellAddress& blockAddress)
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField dirty marking requires a block-level address.");

    if (!m_completelyDirty)
    {
        m_dirtyBlocks.insert(blockAddress.index);
    }
}

void VolumeField::markBlockValid(const VoxelCellAddress& blockAddress)
{
    MYVOXEL_ASSERT_MESSAGE(supportsBlockAddress(blockAddress), "VolumeField valid marking requires a block-level address.");
    MYVOXEL_ASSERT_MESSAGE(!m_completelyDirty, "VolumeField cannot confirm one block while the entire field is dirty.");
    m_dirtyBlocks.erase(blockAddress.index);
}

void VolumeField::markAllDirty()
{
    m_completelyDirty = true;
    m_dirtyBlocks.clear();
}

void VolumeField::markAllValid()
{
    m_completelyDirty = false;
    m_dirtyBlocks.clear();
}

/// 资源管理

void VolumeField::resetPreservingStorage()
{
    m_blockIndices.clear();
    m_dirtyBlocks.clear();
    m_blockPool.rewind();
    m_completelyDirty = true;
}

void VolumeField::releaseStorage()
{
    m_blockIndices.clear();
    m_dirtyBlocks.clear();
    m_blockPool.clear();
    m_completelyDirty = true;
}

/// 存储统计与遍历

std::size_t VolumeField::blockCount() const
{
    return m_blockIndices.size();
}

std::size_t VolumeField::dirtyBlockCount() const
{
    return m_dirtyBlocks.size();
}

std::size_t VolumeField::allocatedSampleBytes() const
{
    return m_blockIndices.size() * sizeof(VolumeBlock);
}

std::size_t VolumeField::storageCapacityBytes() const
{
    return m_blockPool.storageCapacityBytes();
}

const VolumeField::BlockIndexMap& VolumeField::blockIndices() const
{
    return m_blockIndices;
}

const VolumeField::DirtyBlockSet& VolumeField::dirtyBlocks() const
{
    return m_dirtyBlocks;
}

/// 内部辅助

VolumeField::BlockIndexMap::iterator VolumeField::findBlockIterator(const VoxelCellAddress& blockAddress)
{
    return m_blockIndices.find(blockAddress.index);
}

VolumeField::BlockIndexMap::const_iterator VolumeField::findBlockIterator(const VoxelCellAddress& blockAddress) const
{
    return m_blockIndices.find(blockAddress.index);
}

}