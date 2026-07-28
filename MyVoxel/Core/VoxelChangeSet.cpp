#include "VoxelChangeSet.h"

namespace MyVoxel
{

void VoxelChangeSet::clear()
{
    m_modifiedRootIndices.clear();
}

bool VoxelChangeSet::hasChanges() const
{
    return !m_modifiedRootIndices.empty();
}

std::size_t VoxelChangeSet::modifiedRootCount() const
{
    return m_modifiedRootIndices.size();
}

bool VoxelChangeSet::containsModifiedRoot(const VoxelCellIndex& rootIndex) const
{
    return m_modifiedRootIndices.find(rootIndex) != m_modifiedRootIndices.end();
}

void VoxelChangeSet::addModifiedRoot(const VoxelCellIndex& rootIndex)
{
    m_modifiedRootIndices.insert(rootIndex);
}

void VoxelChangeSet::accumulate(const VoxelChangeSet& other)
{
    m_modifiedRootIndices.insert(other.m_modifiedRootIndices.begin(), other.m_modifiedRootIndices.end());
}

const VoxelChangeSet::RootIndexSet& VoxelChangeSet::modifiedRootIndices() const
{
    return m_modifiedRootIndices;
}

}