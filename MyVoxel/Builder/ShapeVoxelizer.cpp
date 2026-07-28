#include "ShapeVoxelizer.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelNodeForest.h"
#include "MyVoxel/Shape/ShapeBounds.h"
#include "MyVoxel/Shape/ShapeRegionRelation.h"

namespace
{

// 返回指定层级的体素边长。
double voxelEdgeLength(double baseVoxelEdgeLength, MyVoxel::VoxelLevel level)
{
    return std::ldexp(baseVoxelEdgeLength, -static_cast<int>(level));
}

// 返回包含指定最小坐标的第0层体素索引。
MyVoxel::VoxelIndex minimumRootIndex(double coordinate, double edgeLength)
{
    const double value = std::floor(coordinate / edgeLength);
    const double minimum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(value >= minimum && value <= maximum);
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 返回与指定最大坐标之前区域相交的最后一个第0层体素索引。
MyVoxel::VoxelIndex maximumRootIndex(double coordinate, double edgeLength)
{
    const double value = std::ceil(coordinate / edgeLength) - 1.0;
    const double minimum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(value >= minimum && value <= maximum);
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 返回指定体素在Shape局部坐标系中的轴对齐包围盒。
MyVoxel::ShapeBounds cellBounds(const MyVoxel::VoxelCellAddress& address, double baseVoxelEdgeLength)
{
    const double edgeLength = voxelEdgeLength(baseVoxelEdgeLength, address.level);
    const double minimumX = static_cast<double>(address.index.x) * edgeLength;
    const double minimumY = static_cast<double>(address.index.y) * edgeLength;
    const double minimumZ = static_cast<double>(address.index.z) * edgeLength;
    return MyVoxel::ShapeBounds(minimumX, minimumY, minimumZ, minimumX + edgeLength, minimumY + edgeLength, minimumZ + edgeLength);
}

// 返回指定体素在Shape局部坐标系中的中心点。
MyMath::Vector3 cellCenter(const MyVoxel::VoxelCellAddress& address, double baseVoxelEdgeLength)
{
    const double edgeLength = voxelEdgeLength(baseVoxelEdgeLength, address.level);
    const double centerOffset = edgeLength * 0.5; // 体素中心距离体素最小角为半个体素边长。
    return MyMath::Vector3(static_cast<double>(address.index.x) * edgeLength + centerOffset, static_cast<double>(address.index.y) * edgeLength + centerOffset, static_cast<double>(address.index.z) * edgeLength + centerOffset);
}

// 递归构建当前体素范围内的连续Shape材料。
void voxelizeCell(MyVoxel::VoxelNodeForest& forest, const MyVoxel::Shape& shape, const MyVoxel::VoxelCellAddress& address, double baseVoxelEdgeLength, MyVoxel::VoxelLevel maximumLevel)
{
    const MyVoxel::ShapeBounds bounds = cellBounds(address, baseVoxelEdgeLength);
    const MyVoxel::ShapeRegionRelation relation = shape.classifyLocalBounds(bounds);

    if (relation == MyVoxel::ShapeRegionRelation::Outside)
    {
        return;
    }

    if (relation == MyVoxel::ShapeRegionRelation::Inside)
    {
        forest.setState(address, MyVoxel::VoxelState::Material);
        return;
    }

    if (address.level == maximumLevel)
    {
        if (shape.containsLocalPoint(cellCenter(address, baseVoxelEdgeLength)))
        {
            forest.setState(address, MyVoxel::VoxelState::Material);
        }

        return;
    }

    assert(address.level < maximumLevel);

    for (int i = 0; i < MyVoxel::VoxelCornerCount; ++i)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(i);
        voxelizeCell(forest, shape, MyVoxel::childCellAddress(address, corner), baseVoxelEdgeLength, maximumLevel);
    }
}

}

namespace MyVoxel
{

VoxelShape voxelize(const Shape& shape, const VoxelizationParameters& parameters)
{
    assert(std::isfinite(parameters.baseVoxelEdgeLength) && parameters.baseVoxelEdgeLength > 0.0);

    const ShapeBounds bounds = shape.localBounds();

    assert(bounds.isValid());
    assert(bounds.hasVolume());

    VoxelShape result;
    result.setBaseVoxelEdgeLength(parameters.baseVoxelEdgeLength);
    result.setMaximumLevel(parameters.maximumLevel);
    result.setTransform(shape.transform());

    VoxelNodeForest& forest = result.forest();

    const VoxelIndex minimumX = minimumRootIndex(bounds.minimumX, parameters.baseVoxelEdgeLength);
    const VoxelIndex minimumY = minimumRootIndex(bounds.minimumY, parameters.baseVoxelEdgeLength);
    const VoxelIndex minimumZ = minimumRootIndex(bounds.minimumZ, parameters.baseVoxelEdgeLength);
    const VoxelIndex maximumX = maximumRootIndex(bounds.maximumX, parameters.baseVoxelEdgeLength);
    const VoxelIndex maximumY = maximumRootIndex(bounds.maximumY, parameters.baseVoxelEdgeLength);
    const VoxelIndex maximumZ = maximumRootIndex(bounds.maximumZ, parameters.baseVoxelEdgeLength);

    for (std::int64_t z = minimumZ; z <= maximumZ; ++z)
    {
        for (std::int64_t y = minimumY; y <= maximumY; ++y)
        {
            for (std::int64_t x = minimumX; x <= maximumX; ++x)
            {
                const VoxelCellIndex index(static_cast<VoxelIndex>(x), static_cast<VoxelIndex>(y), static_cast<VoxelIndex>(z));
                voxelizeCell(forest, shape, VoxelCellAddress(index, BaseVoxelLevel), parameters.baseVoxelEdgeLength, parameters.maximumLevel);
            }
        }
    }

    return result;
}

VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel)
{
    VoxelizationParameters parameters;
    parameters.baseVoxelEdgeLength = baseVoxelEdgeLength;
    parameters.maximumLevel = maximumLevel;
    return voxelize(shape, parameters);
}

}