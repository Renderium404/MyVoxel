#include "VoxelTreeConstCursor.h"

#include <cassert>
#include <vector>

namespace MyVoxel
{

VoxelTreeConstCursor::VoxelTreeConstCursor(const VoxelForest& forest, const VoxelCellAddress& address)
    : m_cursor(createCursor(forest, address))
{
}

VoxelTreeConstCursor::VoxelTreeConstCursor(const VoxelPackedTreeConstCursor& cursor)
    : m_cursor(cursor)
{
}

VoxelState VoxelTreeConstCursor::state() const
{
    return m_cursor.state();
}

bool VoxelTreeConstCursor::isMaskLeaf() const
{
    return m_cursor.isMaskLeaf();
}

bool VoxelTreeConstCursor::canAccessChildren() const
{
    return m_cursor.canAccessChildren();
}

VoxelTreeConstCursor VoxelTreeConstCursor::child(VoxelCorner corner) const
{
    return VoxelTreeConstCursor(m_cursor.child(corner));
}

const VoxelLeafBlock& VoxelTreeConstCursor::maskLeaf() const
{
    return m_cursor.maskLeaf();
}

VoxelPackedTreeConstCursor VoxelTreeConstCursor::createCursor(const VoxelForest& forest, const VoxelCellAddress& address)
{
    const VoxelCellAddress rootAddress = VoxelPackedForest::rootCellAddress(address);
    const VoxelPackedRootTree* tree = forest.findRootTree(rootAddress.index);

    assert(tree);

    VoxelPackedTreeConstCursor cursor(*tree);
    std::vector<VoxelCorner> path;

    VoxelPackedForest::buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        assert(cursor.canAccessChildren());
        cursor = cursor.child(path[pathIndex]);
    }

    return cursor;
}

}