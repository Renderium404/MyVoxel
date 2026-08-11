#include "VoxelTreeAccessor.h"

#include <algorithm>

#include "../../Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelTreeAccessor::PathEntry::PathEntry(const VoxelCellAddress& addressValue, const VoxelTreeCursor& cursorValue)
    : address(addressValue)
    , cursor(cursorValue)
{
}

VoxelTreeAccessor::VoxelTreeAccessor(const VoxelTree& tree, const VoxelCellAddress& rootAddress, VoxelLevel maximumLevel)
    : m_tree(&tree)
    , m_rootAddress(rootAddress)
    , m_targetAddress(rootAddress)
    , m_maximumLevel(maximumLevel)
    , m_state(VoxelState::Empty)
    , m_reusedLevel(BaseVoxelLevel)
    , m_visitedNodeCount(0)
{
    MYVOXEL_REQUIRE_MESSAGE(rootAddress.level == BaseVoxelLevel, "VoxelTreeAccessor root address must be at BaseVoxelLevel.");
    MYVOXEL_REQUIRE_MESSAGE(maximumLevel >= BaseVoxelLevel, "VoxelTreeAccessor maximum level is invalid.");
    MYVOXEL_REQUIRE_MESSAGE(tree.isValid(), "Cannot create VoxelTreeAccessor for an invalid VoxelTree.");

    reset();
}

/// 地址定位

VoxelState VoxelTreeAccessor::seek(const VoxelCellAddress& address)
{
    MYVOXEL_REQUIRE_MESSAGE(address.level <= m_maximumLevel, "Target voxel address exceeds the configured maximum level.");
    MYVOXEL_REQUIRE_MESSAGE(MyVoxel::rootCellAddress(address) == m_rootAddress, "Target voxel address does not belong to this VoxelTree root.");

    MyVoxel::buildCornerPath(m_rootAddress, address, m_targetCorners);

    std::size_t commonCornerCount = 0;
    const std::size_t comparableCount = std::min(m_cachedCorners.size(), m_targetCorners.size());

    while (commonCornerCount < comparableCount && m_cachedCorners[commonCornerCount] == m_targetCorners[commonCornerCount])
    {
        ++commonCornerCount;
    }

    m_cachedCorners.resize(commonCornerCount);
    m_path.erase(
        m_path.begin() + static_cast<std::ptrdiff_t>(commonCornerCount + 1),
        m_path.end());

    m_targetAddress = address;
    m_reusedLevel = m_path.back().address.level;
    m_visitedNodeCount = 0;

    while (m_cachedCorners.size() < m_targetCorners.size())
    {
        const PathEntry& currentEntry = m_path.back();

        if (!currentEntry.cursor.isSubdivided())
        {
            break;
        }

        const VoxelCorner corner = m_targetCorners[m_cachedCorners.size()];
        const VoxelCellAddress childAddress = childCellAddress(currentEntry.address, corner);
        const VoxelTreeCursor childCursor = currentEntry.cursor.child(corner);

        m_cachedCorners.push_back(corner);
        m_path.push_back(PathEntry(childAddress, childCursor));
        ++m_visitedNodeCount;
    }

    m_state = m_path.back().cursor.state();
    return m_state;
}

void VoxelTreeAccessor::reset()
{
    MYVOXEL_REQUIRE_MESSAGE(m_tree, "VoxelTreeAccessor is not bound to a VoxelTree.");
    MYVOXEL_REQUIRE_MESSAGE(m_tree->isValid(), "Cannot reset VoxelTreeAccessor for an invalid VoxelTree.");

    m_path.clear();
    m_cachedCorners.clear();
    m_targetCorners.clear();

    const VoxelTreeCursor rootCursor(*m_tree);
    m_path.push_back(PathEntry(m_rootAddress, rootCursor));

    m_targetAddress = m_rootAddress;
    m_state = rootCursor.state();
    m_reusedLevel = BaseVoxelLevel;
    m_visitedNodeCount = 0;
}

/// 定位结果

const VoxelCellAddress& VoxelTreeAccessor::targetAddress() const
{
    return m_targetAddress;
}

const VoxelCellAddress& VoxelTreeAccessor::resolvedAddress() const
{
    MYVOXEL_ASSERT(!m_path.empty());
    return m_path.back().address;
}

VoxelState VoxelTreeAccessor::state() const
{
    return m_state;
}

bool VoxelTreeAccessor::isExact() const
{
    return resolvedAddress() == m_targetAddress;
}

bool VoxelTreeAccessor::isImplicit() const
{
    return !isExact();
}

const VoxelTreeCursor& VoxelTreeAccessor::cursor() const
{
    MYVOXEL_ASSERT(!m_path.empty());
    return m_path.back().cursor;
}

/// 缓存统计

VoxelLevel VoxelTreeAccessor::reusedLevel() const
{
    return m_reusedLevel;
}

std::size_t VoxelTreeAccessor::visitedNodeCount() const
{
    return m_visitedNodeCount;
}

std::size_t VoxelTreeAccessor::cachedNodeCount() const
{
    return m_path.size();
}

}