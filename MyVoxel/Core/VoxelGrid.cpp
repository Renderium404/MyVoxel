#include "VoxelGrid.h"

#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double HalfScale = 0.5; // 体素最小角点与最大角点中点计算使用的固定比例。

// 判断数值是否为有限数值。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

// 判断数值是否为有限正数。
bool isFinitePositive(double value)
{
    return isFiniteValue(value) && value > 0.0;
}

// 判断数值是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 将已经取整的浮点索引转换为VoxelIndex。
MyVoxel::VoxelIndex toVoxelIndex(double value)
{
    const double minimum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(value), "Voxel grid index must be finite.");
    MYVOXEL_ASSERT_MESSAGE(value >= minimum && value <= maximum, "Voxel grid index exceeds VoxelIndex range.");
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 返回指定坐标所属的体素索引，网格边界归入正方向体素。
MyVoxel::VoxelIndex pointCellIndex(double coordinate, double origin, double edgeLength)
{
    return toVoxelIndex(std::floor((coordinate - origin) / edgeLength));
}

// 返回有体积区间最小坐标覆盖的第一个体素索引。
MyVoxel::VoxelIndex intervalMinimumCellIndex(double minimum, double origin, double edgeLength)
{
    return toVoxelIndex(std::floor((minimum - origin) / edgeLength));
}

// 返回有体积区间最大坐标覆盖的最后一个体素索引，区间最大边界按开区间处理。
MyVoxel::VoxelIndex intervalMaximumCellIndex(double maximum, double origin, double edgeLength)
{
    return toVoxelIndex(std::ceil((maximum - origin) / edgeLength) - 1.0);
}

}

namespace MyVoxel
{

VoxelGrid::VoxelGrid()
    : m_origin(0.0, 0.0, 0.0)
    , m_baseCellEdgeLength(1.0)
    , m_maximumLevel(BaseVoxelLevel)
{
}

VoxelGrid::VoxelGrid(double baseCellEdgeLength, VoxelLevel maximumLevel)
    : m_origin(0.0, 0.0, 0.0)
    , m_baseCellEdgeLength(baseCellEdgeLength)
    , m_maximumLevel(maximumLevel)
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelGrid parameters must define a valid spatial grid.");
}

VoxelGrid::VoxelGrid(const MyMath::Vector3& origin, double baseCellEdgeLength, VoxelLevel maximumLevel)
    : m_origin(origin)
    , m_baseCellEdgeLength(baseCellEdgeLength)
    , m_maximumLevel(maximumLevel)
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "VoxelGrid parameters must define a valid spatial grid.");
}

/// 状态判断

bool VoxelGrid::isValid() const
{
    const double minimumEdgeLength = std::ldexp(m_baseCellEdgeLength, -static_cast<int>(m_maximumLevel));
    return m_origin.isFinite() && isFinitePositive(m_baseCellEdgeLength) && isFinitePositive(minimumEdgeLength);
}

bool VoxelGrid::isEqualTo(const VoxelGrid& other, double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegative(epsilon), "VoxelGrid comparison epsilon must be finite and non-negative.");

    return m_origin.isEqualTo(other.m_origin, epsilon) &&
           std::fabs(m_baseCellEdgeLength - other.m_baseCellEdgeLength) <= epsilon &&
           m_maximumLevel == other.m_maximumLevel;
}

/// 网格参数

const MyMath::Vector3& VoxelGrid::origin() const
{
    return m_origin;
}

double VoxelGrid::baseCellEdgeLength() const
{
    return m_baseCellEdgeLength;
}

VoxelLevel VoxelGrid::maximumLevel() const
{
    return m_maximumLevel;
}

double VoxelGrid::cellEdgeLength(VoxelLevel level) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid VoxelGrid.");
    MYVOXEL_ASSERT_MESSAGE(level <= m_maximumLevel, "Voxel level exceeds the grid maximum level.");

    const double edgeLength = std::ldexp(m_baseCellEdgeLength, -static_cast<int>(level));

    MYVOXEL_ASSERT_MESSAGE(isFinitePositive(edgeLength), "Voxel cell edge length is not representable.");
    return edgeLength;
}

double VoxelGrid::minimumCellEdgeLength() const
{
    return cellEdgeLength(m_maximumLevel);
}

/// 坐标与索引转换

VoxelCellIndex VoxelGrid::cellIndex(const MyMath::Vector3& point, VoxelLevel level) const
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Voxel grid query point must be finite.");

    const double edgeLength = cellEdgeLength(level);

    return VoxelCellIndex(
        pointCellIndex(point.x(), m_origin.x(), edgeLength),
        pointCellIndex(point.y(), m_origin.y(), edgeLength),
        pointCellIndex(point.z(), m_origin.z(), edgeLength));
}

VoxelCellAddress VoxelGrid::cellAddress(const MyMath::Vector3& point, VoxelLevel level) const
{
    return VoxelCellAddress(cellIndex(point, level), level);
}

VoxelCellRange VoxelGrid::cellRange(const Bounds3& bounds, VoxelLevel level) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Voxel grid query bounds must be valid.");
    MYVOXEL_ASSERT_MESSAGE(bounds.hasVolume(), "Voxel grid query bounds must have volume.");

    const double edgeLength = cellEdgeLength(level);
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    const VoxelCellIndex minimumIndex(
        intervalMinimumCellIndex(minimum.x(), m_origin.x(), edgeLength),
        intervalMinimumCellIndex(minimum.y(), m_origin.y(), edgeLength),
        intervalMinimumCellIndex(minimum.z(), m_origin.z(), edgeLength));

    const VoxelCellIndex maximumIndex(
        intervalMaximumCellIndex(maximum.x(), m_origin.x(), edgeLength),
        intervalMaximumCellIndex(maximum.y(), m_origin.y(), edgeLength),
        intervalMaximumCellIndex(maximum.z(), m_origin.z(), edgeLength));

    return VoxelCellRange(minimumIndex, maximumIndex, level);
}

/// 体素空间属性

Bounds3 VoxelGrid::cellBounds(const VoxelCellAddress& address) const
{
    const double edgeLength = cellEdgeLength(address.level);
    
    const double minimumX = m_origin.x() + static_cast<double>(address.index.x) * edgeLength;
    const double minimumY = m_origin.y() + static_cast<double>(address.index.y) * edgeLength;
    const double minimumZ = m_origin.z() + static_cast<double>(address.index.z) * edgeLength;
    const double maximumX = minimumX + edgeLength;
    const double maximumY = minimumY + edgeLength;
    const double maximumZ = minimumZ + edgeLength;

    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(minimumX) && isFiniteValue(minimumY) && isFiniteValue(minimumZ), "Voxel cell minimum coordinate must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(maximumX) && isFiniteValue(maximumY) && isFiniteValue(maximumZ), "Voxel cell maximum coordinate must be finite.");
    MYVOXEL_ASSERT_MESSAGE(maximumX > minimumX && maximumY > minimumY && maximumZ > minimumZ, "Voxel cell size is not representable at the requested coordinate.");

    return Bounds3(MyMath::Vector3(minimumX, minimumY, minimumZ), MyMath::Vector3(maximumX, maximumY, maximumZ));
}

MyMath::Vector3 VoxelGrid::cellCenter(const VoxelCellAddress& address) const
{
    const Bounds3 bounds = cellBounds(address);
    return bounds.minimum() + bounds.size() * HalfScale;
}

MyMath::Vector3 VoxelGrid::cellCorner(const VoxelCellAddress& address, VoxelCorner corner) const
{
    const std::size_t cornerIndex = static_cast<std::size_t>(corner);

    MYVOXEL_ASSERT_MESSAGE(cornerIndex < VoxelCornerCount, "Voxel corner must be in range [0, 7].");
    return cellBounds(address).corner(cornerIndex);
}

}