#include "VoxelTree.h"

namespace MyVoxel
{

VoxelTree::VoxelTree(VoxelState state)
    : root(state)
{
}

VoxelTree::VoxelTree(const VoxelTree& other)
    : root(other.root)
    , nodePool(other.nodePool)
{
}

}