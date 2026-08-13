#include "Geometry_Arc.h"

#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，角度计算统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整圆对应的弧度值。
const double BoundsAngleToleranceScale = 64.0; // 三维圆弧包围盒极值角判断覆盖三角函数与角度规范化舍入误差。

// 判断数值是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 创建以指定有限点为原点且方向与世界坐标系一致的圆弧坐标系。
MyMath::CoordinateSystem worldXYAt(const MyMath::Vector3& center)
{
    MyMath::CoordinateSystem coordinateSystem;

    if (center.isFinite())
    {
        coordinateSystem.setOrigin(center);
    }

    return coordinateSystem;
}

// 将角度规范化到[0,2π)范围。
double normalizedAngle(double angle)
{
    double result = std::fmod(angle, TwoPi);

    if (result < 0.0)
    {
        result += TwoPi;
    }

    return result;
}

// 返回从起始角沿逆时针方向到目标角的非负角距离。
double positiveAngleDistance(double startAngle, double targetAngle)
{
    return normalizedAngle(targetAngle - startAngle);
}

// 返回三维向量指定坐标分量。
double component(const MyMath::Vector3& value, int axis)
{
    if (axis == 0)
    {
        return value.x();
    }

    if (axis == 1)
    {
        return value.y();
    }

    return value.z();
}

}

namespace MyVoxel
{

Geometry_Arc::Geometry_Arc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle)
    : m_coordinateSystem(worldXYAt(center))
    , m_center(center)
    , m_radius(radius)
    , m_startAngle(startAngle)
    , m_sweepAngle(sweepAngle)
    , m_length(radius * std::fabs(sweepAngle))
{
    const bool valid = center.isFinite() && isFiniteValue(radius) && radius > 0.0 && isFiniteValue(startAngle) &&
                       isFiniteValue(sweepAngle) && sweepAngle != 0.0 && std::fabs(sweepAngle) <= TwoPi;
    MYVOXEL_ASSERT_MESSAGE(valid, "Geometry_Arc parameters must define a finite non-degenerate circular arc with |sweepAngle| <= 2*pi.");

    if (valid)
    {
        updateCachedData();
    }
}

Geometry_Arc::Geometry_Arc(const MyMath::CoordinateSystem& coordinateSystem, double radius, double startAngle, double sweepAngle)
    : m_coordinateSystem(coordinateSystem)
    , m_center(coordinateSystem.origin())
    , m_radius(radius)
    , m_startAngle(startAngle)
    , m_sweepAngle(sweepAngle)
    , m_length(radius * std::fabs(sweepAngle))
{
    const bool valid = coordinateSystem.isValid() && isFiniteValue(radius) && radius > 0.0 && isFiniteValue(startAngle) &&
                       isFiniteValue(sweepAngle) && sweepAngle != 0.0 && std::fabs(sweepAngle) <= TwoPi;
    MYVOXEL_ASSERT_MESSAGE(valid, "Geometry_Arc requires a valid orthogonal coordinate system, positive radius and non-zero sweep within 2*pi.");

    if (valid)
    {
        updateCachedData();
    }
}

/// 圆弧坐标系

const MyMath::CoordinateSystem& Geometry_Arc::coordinateSystem() const
{
    return m_coordinateSystem;
}

const MyMath::Vector3& Geometry_Arc::center() const
{
    return m_center;
}

MyMath::Vector3 Geometry_Arc::xAxis() const
{
    return m_coordinateSystem.xAxis();
}

MyMath::Vector3 Geometry_Arc::yAxis() const
{
    return m_coordinateSystem.yAxis();
}

MyMath::Vector3 Geometry_Arc::normal() const
{
    return m_coordinateSystem.zAxis();
}

/// 圆弧参数

double Geometry_Arc::radius() const
{
    return m_radius;
}

double Geometry_Arc::startAngle() const
{
    return m_startAngle;
}

double Geometry_Arc::sweepAngle() const
{
    return m_sweepAngle;
}

bool Geometry_Arc::containsAngle(double angle, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(angle), "Geometry_Arc query angle must be finite.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(tolerance) && tolerance >= 0.0, "Geometry_Arc angle tolerance must be finite and non-negative.");

    const double absoluteSweep = std::fabs(m_sweepAngle);

    if (absoluteSweep >= TwoPi - tolerance)
    {
        return true;
    }

    const double distance = m_sweepAngle > 0.0 ? positiveAngleDistance(m_startAngle, angle) : positiveAngleDistance(angle, m_startAngle);
    return distance <= absoluteSweep + tolerance;
}

/// 曲线属性

CurveKind Geometry_Arc::kind() const
{
    return CurveKind::Arc;
}

const MyMath::Vector3& Geometry_Arc::startPoint() const
{
    return m_startPoint;
}

const MyMath::Vector3& Geometry_Arc::endPoint() const
{
    return m_endPoint;
}

double Geometry_Arc::length() const
{
    return m_length;
}

const Bounds3& Geometry_Arc::bounds() const
{
    return m_bounds;
}

/// 参数查询

MyMath::Vector3 Geometry_Arc::pointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Geometry_Arc parameter must be in [0,1].");
    return pointAtAngle(m_startAngle + m_sweepAngle * t);
}

MyMath::Vector3 Geometry_Arc::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Geometry_Arc parameter must be in [0,1].");

    const double angle = m_startAngle + m_sweepAngle * t;
    const double direction = m_sweepAngle > 0.0 ? 1.0 : -1.0;
    const MyMath::Vector3 localTangent(-std::sin(angle) * direction, std::cos(angle) * direction, 0.0);
    return m_coordinateSystem.mapVector(localTangent);
}

/// 曲线创建

Foundation::RefPtr<const Geometry_Curve> Geometry_Arc::reversed() const
{
    return Foundation::RefPtr<const Geometry_Curve>(new Geometry_Arc(m_coordinateSystem, m_radius, m_startAngle + m_sweepAngle, -m_sweepAngle));
}

/// 内部辅助

MyMath::Vector3 Geometry_Arc::pointAtAngle(double angle) const
{
    return m_coordinateSystem.toGlobal(MyMath::Vector3(m_radius * std::cos(angle), m_radius * std::sin(angle), 0.0));
}

void Geometry_Arc::updateCachedData()
{
    m_startPoint = pointAtAngle(m_startAngle);
    m_endPoint = pointAtAngle(m_startAngle + m_sweepAngle);
    m_bounds.clear();
    m_bounds.include(m_startPoint);
    m_bounds.include(m_endPoint);

    const MyMath::Vector3 localXAxis = m_coordinateSystem.xAxis();
    const MyMath::Vector3 localYAxis = m_coordinateSystem.yAxis();
    const double angleTolerance = (std::numeric_limits<double>::epsilon)() * BoundsAngleToleranceScale; // 仅用于判断理论极值角是否属于圆弧。

    for (int axis = 0; axis < 3; ++axis)
    {
        const double cosineCoefficient = component(localXAxis, axis);
        const double sineCoefficient = component(localYAxis, axis);

        if (cosineCoefficient == 0.0 && sineCoefficient == 0.0)
        {
            continue;
        }

        const double maximumAngle = std::atan2(sineCoefficient, cosineCoefficient);
        const double minimumAngle = maximumAngle + Pi;

        if (containsAngle(maximumAngle, angleTolerance))
        {
            m_bounds.include(pointAtAngle(maximumAngle));
        }

        if (containsAngle(minimumAngle, angleTolerance))
        {
            m_bounds.include(pointAtAngle(minimumAngle));
        }
    }
}

}
