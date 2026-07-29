#include "BooleanAlgorithmCommon.h"

#include <cassert>
#include <limits>

namespace
{

const double CellCenterScale = 0.5; // 包围盒中心位于最小点和最大点的中间位置。

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

CutCellResult::CutCellResult(VoxelState stateValue, bool changedValue)
    : state(stateValue)
    , changed(changedValue)
{
}

VoxelCellIndex rootCellIndex(VoxelCellAddress address)
{
    while (hasParentCell(address))
    {
        address = parentCellAddress(address);
    }

    return address.index;
}

MyMath::Vector3 boundsCenter(const Bounds3& bounds)
{
    assert(bounds.isValid());

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    return MyMath::Vector3(
        (minimum.x() + maximum.x()) * CellCenterScale,
        (minimum.y() + maximum.y()) * CellCenterScale,
        (minimum.z() + maximum.z()) * CellCenterScale);
}

MyMath::Vector3 cellCenter(const VoxelShape& shape, const VoxelCellAddress& address)
{
    assert(shape.grid().isValid());
    assert(address.level <= shape.grid().maximumLevel());
    return boundsCenter(shape.grid().cellBounds(address));
}

double cellEdgeLength(const VoxelGrid& grid, VoxelLevel level)
{
    assert(grid.isValid());
    assert(level <= grid.maximumLevel());

    const VoxelCellAddress zeroAddress(VoxelCellIndex(0, 0, 0), level);
    const Bounds3 bounds = grid.cellBounds(zeroAddress);
    return bounds.maximum().x() - bounds.minimum().x();
}

std::vector<VoxelCellAddress> collectMaterialCells(const VoxelShape& shape)
{
    std::vector<VoxelCellAddress> addresses;

    shape.forest().forEachMaterialCell(
        [&addresses](const VoxelCellAddress& address)
        {
            addresses.push_back(address);
        });

    return addresses;
}

std::vector<VoxelCellAddress> collectRootCellsInRange(const VoxelShape& shape, const VoxelCellIndex& minimumRootIndex, const VoxelCellIndex& maximumRootIndex)
{
    std::vector<VoxelCellAddress> addresses;

    shape.forest().forEachRootCellInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&addresses](const VoxelCellAddress& address)
        {
            addresses.push_back(address);
        });

    return addresses;
}

std::uint64_t saturatedMultiply(std::uint64_t first, std::uint64_t second)
{
    if (first == 0 || second == 0)
    {
        return 0;
    }

    const std::uint64_t maximumValue = (std::numeric_limits<std::uint64_t>::max)();

    if (first > maximumValue / second)
    {
        return maximumValue;
    }

    return first * second;
}

}
}
}