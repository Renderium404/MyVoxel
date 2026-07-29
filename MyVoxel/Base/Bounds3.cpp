#include "Bounds3.h"

#include <algorithm>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const std::size_t XCornerMask = 1;
const std::size_t YCornerMask = 2;
const std::size_t ZCornerMask = 4;

// 判断标量是否为有限数值。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

// 判断标量是否为有限非负数值。
bool isFiniteNonNegativeValue(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 判断两个角点能否构成有效包围盒。
bool isValidRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum)
{
    return minimum.isFinite() &&
           maximum.isFinite() &&
           minimum.x() <= maximum.x() &&
           minimum.y() <= maximum.y() &&
           minimum.z() <= maximum.z();
}

}

namespace MyVoxel
{

Bounds3::Bounds3()
    : m_minimum()
    , m_maximum()
    , m_valid(false)
{
}

Bounds3::Bounds3(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum)
    : m_minimum(minimum)
    , m_maximum(maximum)
    , m_valid(isValidRange(minimum, maximum))
{
    MYVOXEL_ASSERT_MESSAGE(m_valid, "Bounds3 minimum must not be greater than maximum.");
}

/// 包围盒创建

Bounds3 Bounds3::fromCenterAndSize(const MyMath::Vector3& center, const MyMath::Vector3& size)
{
    const bool valid = center.isFinite() &&
                       size.isFinite() &&
                       size.x() >= 0.0 &&
                       size.y() >= 0.0 &&
                       size.z() >= 0.0;

    MYVOXEL_ASSERT_MESSAGE(valid, "Bounds3 center and size must be finite, and size must be non-negative.");

    if (!valid)
    {
        return Bounds3();
    }

    const MyMath::Vector3 halfSize = size * 0.5;
    return Bounds3(center - halfSize, center + halfSize);
}

/// 状态判断

bool Bounds3::isValid() const
{
    return m_valid && isValidRange(m_minimum, m_maximum);
}

bool Bounds3::hasVolume(double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(epsilon), "Bounds3 volume epsilon must be finite and non-negative.");

    if (!isValid())
    {
        return false;
    }

    const MyMath::Vector3 currentSize = size();
    return currentSize.x() > epsilon && currentSize.y() > epsilon && currentSize.z() > epsilon;
}

bool Bounds3::isEqualTo(const Bounds3& other, double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(epsilon), "Bounds3 comparison epsilon must be finite and non-negative.");

    if (isValid() != other.isValid())
    {
        return false;
    }

    if (!isValid())
    {
        return true;
    }

    return m_minimum.isEqualTo(other.m_minimum, epsilon) && m_maximum.isEqualTo(other.m_maximum, epsilon);
}

/// 范围访问

const MyMath::Vector3& Bounds3::minimum() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the minimum point of an invalid Bounds3.");
    return m_minimum;
}

const MyMath::Vector3& Bounds3::maximum() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the maximum point of an invalid Bounds3.");
    return m_maximum;
}

MyMath::Vector3 Bounds3::center() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot calculate the center of an invalid Bounds3.");
    return (m_minimum + m_maximum) * 0.5;
}

MyMath::Vector3 Bounds3::size() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot calculate the size of an invalid Bounds3.");
    return m_maximum - m_minimum;
}

MyMath::Vector3 Bounds3::extent() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot calculate the extent of an invalid Bounds3.");
    return size() * 0.5;
}

double Bounds3::volume() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot calculate the volume of an invalid Bounds3.");

    const MyMath::Vector3 currentSize = size();
    return currentSize.x() * currentSize.y() * currentSize.z();
}

MyMath::Vector3 Bounds3::corner(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access a corner of an invalid Bounds3.");
    MYVOXEL_ASSERT_MESSAGE(index < CornerCount, "Bounds3 corner index is out of range.");

    const double x = (index & XCornerMask) != 0 ? m_maximum.x() : m_minimum.x();
    const double y = (index & YCornerMask) != 0 ? m_maximum.y() : m_minimum.y();
    const double z = (index & ZCornerMask) != 0 ? m_maximum.z() : m_minimum.z();

    return MyMath::Vector3(x, y, z);
}

/// 空间关系

bool Bounds3::contains(const MyMath::Vector3& point, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Bounds3.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Bounds3 query point must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(tolerance), "Bounds3 query tolerance must be finite and non-negative.");

    return point.x() >= m_minimum.x() - tolerance &&
           point.x() <= m_maximum.x() + tolerance &&
           point.y() >= m_minimum.y() - tolerance &&
           point.y() <= m_maximum.y() + tolerance &&
           point.z() >= m_minimum.z() - tolerance &&
           point.z() <= m_maximum.z() + tolerance;
}

bool Bounds3::contains(const Bounds3& bounds, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Bounds3.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Bounds3 query bounds must be valid.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(tolerance), "Bounds3 query tolerance must be finite and non-negative.");

    return bounds.m_minimum.x() >= m_minimum.x() - tolerance &&
           bounds.m_maximum.x() <= m_maximum.x() + tolerance &&
           bounds.m_minimum.y() >= m_minimum.y() - tolerance &&
           bounds.m_maximum.y() <= m_maximum.y() + tolerance &&
           bounds.m_minimum.z() >= m_minimum.z() - tolerance &&
           bounds.m_maximum.z() <= m_maximum.z() + tolerance;
}

bool Bounds3::intersects(const Bounds3& bounds, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Bounds3.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Bounds3 query bounds must be valid.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(tolerance), "Bounds3 query tolerance must be finite and non-negative.");

    return m_maximum.x() + tolerance >= bounds.m_minimum.x() &&
           bounds.m_maximum.x() + tolerance >= m_minimum.x() &&
           m_maximum.y() + tolerance >= bounds.m_minimum.y() &&
           bounds.m_maximum.y() + tolerance >= m_minimum.y() &&
           m_maximum.z() + tolerance >= bounds.m_minimum.z() &&
           bounds.m_maximum.z() + tolerance >= m_minimum.z();
}

/// 范围修改

void Bounds3::clear()
{
    m_minimum = MyMath::Vector3::zero();
    m_maximum = MyMath::Vector3::zero();
    m_valid = false;
}

void Bounds3::include(const MyMath::Vector3& point)
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Bounds3 included point must be finite.");

    if (!point.isFinite())
    {
        return;
    }

    if (!isValid())
    {
        m_minimum = point;
        m_maximum = point;
        m_valid = true;
        return;
    }

    m_minimum.set(
        std::min(m_minimum.x(), point.x()),
        std::min(m_minimum.y(), point.y()),
        std::min(m_minimum.z(), point.z()));

    m_maximum.set(
        std::max(m_maximum.x(), point.x()),
        std::max(m_maximum.y(), point.y()),
        std::max(m_maximum.z(), point.z()));
}

void Bounds3::include(const Bounds3& bounds)
{
    if (!bounds.isValid())
    {
        return;
    }

    if (!isValid())
    {
        *this = bounds;
        return;
    }

    include(bounds.m_minimum);
    include(bounds.m_maximum);
}

/// 范围变换

Bounds3 Bounds3::expanded(double distance) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegativeValue(distance), "Bounds3 expansion distance must be finite and non-negative.");

    if (!isValid() || !isFiniteNonNegativeValue(distance))
    {
        return Bounds3();
    }

    const MyMath::Vector3 offset(distance, distance, distance);
    return Bounds3(m_minimum - offset, m_maximum + offset);
}

Bounds3 Bounds3::translated(const MyMath::Vector3& offset) const
{
    MYVOXEL_ASSERT_MESSAGE(offset.isFinite(), "Bounds3 translation offset must be finite.");

    if (!isValid() || !offset.isFinite())
    {
        return Bounds3();
    }

    return Bounds3(m_minimum + offset, m_maximum + offset);
}

Bounds3 Bounds3::transformed(const MyMath::Matrix4& transform) const
{
    MYVOXEL_ASSERT_MESSAGE(transform.isAffine(), "Bounds3 transformation matrix must be affine.");

    if (!isValid() || !transform.isAffine())
    {
        return Bounds3();
    }

    Bounds3 result;

    for (int cornerIndex = 0; cornerIndex < CornerCount; ++cornerIndex)
    {
        result.include(transform.transformPoint(corner(cornerIndex)));
    }

    return result;
}

}