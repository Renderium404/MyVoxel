#include "VoxelAddress.h"

#include <cassert>
#include <cstdint>
#include <limits>

namespace
{

// 对有符号整数执行向负无穷取整的二分。
MyVoxel::VoxelIndex floorDivideByTwo(MyVoxel::VoxelIndex value)
{
    MyVoxel::VoxelIndex quotient = value / 2;
    const MyVoxel::VoxelIndex remainder = value % 2;

    if (remainder < 0)
    {
        --quotient;
    }

    return quotient;
}

// 返回始终位于0或1范围内的二进制余数。
unsigned int positiveModuloTwo(MyVoxel::VoxelIndex value)
{
    const MyVoxel::VoxelIndex remainder = value % 2;
    return static_cast<unsigned int>(remainder < 0 ? remainder + 2 : remainder);
}

// 返回父索引分量对应的指定侧子索引分量。
MyVoxel::VoxelIndex childIndex(MyVoxel::VoxelIndex parent, bool maximumSide)
{
    const std::int64_t child = static_cast<std::int64_t>(parent) * 2 + (maximumSide ? 1 : 0);
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(child >= minimum);
    assert(child <= maximum);

    return static_cast<MyVoxel::VoxelIndex>(child);
}

// 判断角点是否位于X轴最大侧。
bool usesMaximumX(MyVoxel::VoxelCorner corner)
{
    const unsigned int value = static_cast<unsigned int>(corner);
    assert(value < static_cast<unsigned int>(MyVoxel::VoxelCornerCount));
    return (value & 1U) != 0U;
}

// 判断角点是否位于Y轴最大侧。
bool usesMaximumY(MyVoxel::VoxelCorner corner)
{
    const unsigned int value = static_cast<unsigned int>(corner);
    assert(value < static_cast<unsigned int>(MyVoxel::VoxelCornerCount));
    return (value & 2U) != 0U;
}

// 判断角点是否位于Z轴最大侧。
bool usesMaximumZ(MyVoxel::VoxelCorner corner)
{
    const unsigned int value = static_cast<unsigned int>(corner);
    assert(value < static_cast<unsigned int>(MyVoxel::VoxelCornerCount));
    return (value & 4U) != 0U;
}

}

namespace MyVoxel
{

VoxelCellIndex::VoxelCellIndex(VoxelIndex xValue, VoxelIndex yValue, VoxelIndex zValue)
    : x(xValue)
    , y(yValue)
    , z(zValue)
{
}

bool VoxelCellIndex::operator==(const VoxelCellIndex& other) const
{
    return x == other.x && y == other.y && z == other.z;
}

bool VoxelCellIndex::operator!=(const VoxelCellIndex& other) const
{
    return !(*this == other);
}

bool VoxelCellIndex::operator<(const VoxelCellIndex& other) const
{
    if (x != other.x)
    {
        return x < other.x;
    }

    if (y != other.y)
    {
        return y < other.y;
    }

    return z < other.z;
}

VoxelCellAddress::VoxelCellAddress(const VoxelCellIndex& indexValue, VoxelLevel levelValue)
    : index(indexValue)
    , level(levelValue)
{
}

bool VoxelCellAddress::operator==(const VoxelCellAddress& other) const
{
    return index == other.index && level == other.level;
}

bool VoxelCellAddress::operator!=(const VoxelCellAddress& other) const
{
    return !(*this == other);
}

bool VoxelCellAddress::operator<(const VoxelCellAddress& other) const
{
    if (level != other.level)
    {
        return level < other.level;
    }

    return index < other.index;
}

bool hasParentCell(const VoxelCellAddress& address)
{
    return address.level > BaseVoxelLevel;
}

VoxelCellAddress parentCellAddress(const VoxelCellAddress& address)
{
    assert(hasParentCell(address));

    return VoxelCellAddress(
        VoxelCellIndex(
            floorDivideByTwo(address.index.x),
            floorDivideByTwo(address.index.y),
            floorDivideByTwo(address.index.z)),
        static_cast<VoxelLevel>(address.level - 1));
}

VoxelCellAddress childCellAddress(const VoxelCellAddress& address, VoxelCorner childCorner)
{
    assert(address.level < (std::numeric_limits<VoxelLevel>::max)());

    return VoxelCellAddress(
        VoxelCellIndex(
            childIndex(address.index.x, usesMaximumX(childCorner)),
            childIndex(address.index.y, usesMaximumY(childCorner)),
            childIndex(address.index.z, usesMaximumZ(childCorner))),
        static_cast<VoxelLevel>(address.level + 1));
}

VoxelCorner childCornerInParent(const VoxelCellAddress& address)
{
    assert(hasParentCell(address));

    const unsigned int xBit = positiveModuloTwo(address.index.x);
    const unsigned int yBit = positiveModuloTwo(address.index.y);
    const unsigned int zBit = positiveModuloTwo(address.index.z);

    return static_cast<VoxelCorner>(xBit | (yBit << 1) | (zBit << 2));
}

}