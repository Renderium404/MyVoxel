#include "VoxelFaceAddress.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelFaceAddress::VoxelFaceAddress()
    : direction(VoxelFaceDirection::NegativeX)
{
}

VoxelFaceAddress::VoxelFaceAddress(const VoxelCellIndex& cellIndexValue, VoxelFaceDirection directionValue)
    : cellIndex(cellIndexValue)
    , direction(directionValue)
{
    MYVOXEL_ASSERT_MESSAGE(isValidVoxelFaceDirection(direction), "Voxel face direction must be in range [0, 5].");
}

bool VoxelFaceAddress::operator==(const VoxelFaceAddress& other) const
{
    return cellIndex == other.cellIndex && direction == other.direction;
}

bool VoxelFaceAddress::operator!=(const VoxelFaceAddress& other) const
{
    return !(*this == other);
}

bool VoxelFaceAddress::operator<(const VoxelFaceAddress& other) const
{
    if (cellIndex != other.cellIndex)
    {
        return cellIndex < other.cellIndex;
    }

    return static_cast<unsigned int>(direction) < static_cast<unsigned int>(other.direction);
}

/// 面方向查询

bool isValidVoxelFaceDirection(VoxelFaceDirection direction)
{
    return static_cast<unsigned int>(direction) < VoxelFaceDirectionCount;
}

VoxelFaceDirection oppositeVoxelFaceDirection(VoxelFaceDirection direction)
{
    MYVOXEL_ASSERT_MESSAGE(isValidVoxelFaceDirection(direction), "Cannot query the opposite of an invalid voxel face direction.");

    switch (direction)
    {
    case VoxelFaceDirection::NegativeX:
        return VoxelFaceDirection::PositiveX;

    case VoxelFaceDirection::PositiveX:
        return VoxelFaceDirection::NegativeX;

    case VoxelFaceDirection::NegativeY:
        return VoxelFaceDirection::PositiveY;

    case VoxelFaceDirection::PositiveY:
        return VoxelFaceDirection::NegativeY;

    case VoxelFaceDirection::NegativeZ:
        return VoxelFaceDirection::PositiveZ;

    case VoxelFaceDirection::PositiveZ:
        return VoxelFaceDirection::NegativeZ;
    }

    MYVOXEL_ASSERT_MESSAGE(false, "Cannot query the opposite of an unknown voxel face direction.");
    return VoxelFaceDirection::NegativeX;
}

}