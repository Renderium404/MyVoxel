#include "VoxelChangeSet.h"

namespace MyVoxel
{

/// 状态判断

bool VoxelChangeSet::hasChanges() const
{
    return !m_modifiedRootIndices.empty();
}

bool VoxelChangeSet::containsModifiedRoot(const VoxelCellIndex& rootIndex) const
{
    return m_modifiedRootIndices.find(rootIndex) != m_modifiedRootIndices.end();
}

/// 修改记录

void VoxelChangeSet::addModifiedRoot(const VoxelCellIndex& rootIndex)
{
    m_modifiedRootIndices.insert(rootIndex);
}

void VoxelChangeSet::accumulate(const VoxelChangeSet& other)
{
    m_modifiedRootIndices.insert(other.m_modifiedRootIndices.begin(), other.m_modifiedRootIndices.end());
}

void VoxelChangeSet::clear()
{
    m_modifiedRootIndices.clear();
}

/// 记录访问

std::size_t VoxelChangeSet::modifiedRootCount() const
{
    return m_modifiedRootIndices.size();
}

const VoxelChangeSet::RootIndexSet& VoxelChangeSet::modifiedRootIndices() const
{
    return m_modifiedRootIndices;
}

}