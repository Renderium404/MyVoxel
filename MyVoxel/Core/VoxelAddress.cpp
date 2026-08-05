#include "VoxelAddress.h"

#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const unsigned int XCornerMask = 1; // VoxelCorner第0位表示X方向子体素位置。
const unsigned int YCornerMask = 2; // VoxelCorner第1位表示Y方向子体素位置。
const unsigned int ZCornerMask = 4; // VoxelCorner第2位表示Z方向子体素位置。

// 返回整数索引按二等分层级规则对应的父级索引。
MyVoxel::VoxelIndex parentIndex(MyVoxel::VoxelIndex index)
{
    const MyVoxel::VoxelIndex quotient = index / 2;
    const MyVoxel::VoxelIndex remainder = index % 2;
    return remainder < 0 ? quotient - 1 : quotient;
}

// 返回当前索引在父级索引中的二进制方向，结果固定为0或1。
unsigned int childBit(MyVoxel::VoxelIndex index)
{
    const MyVoxel::VoxelIndex parent = parentIndex(index);
    return static_cast<unsigned int>(index - parent * 2);
}

// 返回指定父级索引和二进制方向对应的子级索引。
MyVoxel::VoxelIndex childIndex(MyVoxel::VoxelIndex parent, unsigned int bit)
{
    MYVOXEL_ASSERT_MESSAGE(bit <= 1, "Voxel child bit must be zero or one.");

    const std::int64_t result = static_cast<std::int64_t>(parent) * 2 + static_cast<std::int64_t>(bit);
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(result >= minimum && result <= maximum, "Voxel child index exceeds VoxelIndex range.");
    return static_cast<MyVoxel::VoxelIndex>(result);
}

// 返回一个闭区间中包含的整数数量。
std::uint64_t indexCount(MyVoxel::VoxelIndex minimum, MyVoxel::VoxelIndex maximum)
{
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(maximum) - static_cast<std::int64_t>(minimum) + 1);
}

}

namespace MyVoxel
{

VoxelCellIndex::VoxelCellIndex()
    : x(0)
    , y(0)
    , z(0)
{
}

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

VoxelCellAddress::VoxelCellAddress()
    : level(BaseVoxelLevel)
{
}

VoxelCellAddress::VoxelCellAddress(const VoxelCellIndex& indexValue, VoxelLevel levelValue)
    : index(indexValue)
    , level(levelValue)
{
}

bool VoxelCellAddress::operator==(const VoxelCellAddress& other) const
{
    return level == other.level && index == other.index;
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

VoxelCellRange::VoxelCellRange()
    : minimum(0, 0, 0)
    , maximum(-1, -1, -1)
    , level(BaseVoxelLevel)
{
}

VoxelCellRange::VoxelCellRange(const VoxelCellIndex& minimumValue, const VoxelCellIndex& maximumValue, VoxelLevel levelValue)
    : minimum(minimumValue)
    , maximum(maximumValue)
    , level(levelValue)
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelCellRange minimum index must not exceed maximum index.");
}

/// 状态判断

bool VoxelCellRange::isValid() const
{
    return minimum.x <= maximum.x && minimum.y <= maximum.y && minimum.z <= maximum.z;
}

bool VoxelCellRange::contains(const VoxelCellIndex& cellIndex) const
{
    if (!isValid())
    {
        return false;
    }

    return cellIndex.x >= minimum.x && cellIndex.x <= maximum.x &&
           cellIndex.y >= minimum.y && cellIndex.y <= maximum.y &&
           cellIndex.z >= minimum.z && cellIndex.z <= maximum.z;
}

bool VoxelCellRange::contains(const VoxelCellAddress& address) const
{
    return isValid() && address.level == level && contains(address.index);
}

/// 范围属性

std::uint64_t VoxelCellRange::countX() const
{
    return isValid() ? indexCount(minimum.x, maximum.x) : 0;
}

std::uint64_t VoxelCellRange::countY() const
{
    return isValid() ? indexCount(minimum.y, maximum.y) : 0;
}

std::uint64_t VoxelCellRange::countZ() const
{
    return isValid() ? indexCount(minimum.z, maximum.z) : 0;
}

/// 体素层级关系

bool hasParentCell(const VoxelCellAddress& address)
{
    return address.level > BaseVoxelLevel;
}

VoxelCellAddress parentCellAddress(const VoxelCellAddress& address)
{
    MYVOXEL_ASSERT_MESSAGE(hasParentCell(address), "Base level voxel cell does not have a parent.");

    const VoxelCellIndex parent(parentIndex(address.index.x), parentIndex(address.index.y), parentIndex(address.index.z));
    return VoxelCellAddress(parent, static_cast<VoxelLevel>(address.level - 1));
}

VoxelCellAddress childCellAddress(const VoxelCellAddress& address, VoxelCorner childCorner)
{
    MYVOXEL_ASSERT_MESSAGE(address.level < (std::numeric_limits<VoxelLevel>::max)(), "Voxel level exceeds VoxelLevel range.");

    const unsigned int cornerValue = static_cast<unsigned int>(childCorner);

    MYVOXEL_ASSERT_MESSAGE(cornerValue < VoxelCornerCount, "Voxel child corner must be in range [0, 7].");

    const unsigned int xBit = cornerValue & XCornerMask;
    const unsigned int yBit = (cornerValue & YCornerMask) >> 1;
    const unsigned int zBit = (cornerValue & ZCornerMask) >> 2;

    const VoxelCellIndex child(childIndex(address.index.x, xBit), childIndex(address.index.y, yBit), childIndex(address.index.z, zBit));
    return VoxelCellAddress(child, static_cast<VoxelLevel>(address.level + 1));
}

VoxelCorner childCornerInParent(const VoxelCellAddress& address)
{
    MYVOXEL_ASSERT_MESSAGE(hasParentCell(address), "Base level voxel cell does not have a parent corner.");

    const unsigned int xBit = childBit(address.index.x);
    const unsigned int yBit = childBit(address.index.y);
    const unsigned int zBit = childBit(address.index.z);
    return static_cast<VoxelCorner>(xBit | (yBit << 1) | (zBit << 2));
}

VoxelCellAddress ancestorCellAddress(const VoxelCellAddress& address, VoxelLevel targetLevel)
{
    MYVOXEL_ASSERT_MESSAGE(targetLevel <= address.level, "Voxel ancestor level must not exceed the current level.");

    VoxelCellAddress ancestor = address;

    while (ancestor.level > targetLevel)
    {
        ancestor = parentCellAddress(ancestor);
    }

    return ancestor;
}

VoxelCellAddress rootCellAddress(const VoxelCellAddress& address)
{
    return ancestorCellAddress(address, BaseVoxelLevel);
}

void buildCornerPath(const VoxelCellAddress& ancestorAddress,
                     const VoxelCellAddress& targetAddress,
                     std::vector<VoxelCorner>& path)
{
    MYVOXEL_ASSERT_MESSAGE(
        ancestorAddress.level <= targetAddress.level,
        "Voxel path ancestor level must not exceed the target level.");

    const std::size_t cornerCount =
        static_cast<std::size_t>(
            targetAddress.level - ancestorAddress.level);

    path.resize(cornerCount);

    VoxelCellAddress currentAddress = targetAddress;

    for (std::size_t pathIndex = cornerCount; pathIndex > 0; --pathIndex)
    {
        path[pathIndex - 1] = childCornerInParent(currentAddress);
        currentAddress = parentCellAddress(currentAddress);
    }

    MYVOXEL_ASSERT_MESSAGE(
        currentAddress == ancestorAddress,
        "Voxel path start address must be an ancestor of the target address.");
}

}