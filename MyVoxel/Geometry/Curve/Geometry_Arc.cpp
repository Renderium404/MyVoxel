#include "Geometry_Arc.h"

#include <cmath>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，角度计算统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整圆对应的弧度值。
const double CardinalAngles[4] = {0.0, Pi * 0.5, Pi, Pi * 1.5}; // XY平面圆在X、Y方向取得极值的四个标准角。

// 判断数值是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

// 判断指定点是否为局部XY平面中的有限点。
bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
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

}

namespace MyVoxel
{

Geometry_Arc::Geometry_Arc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle)
    : m_center(center)
    , m_radius(radius)
    , m_startAngle(startAngle)
    , m_sweepAngle(sweepAngle)
    , m_length(radius * std::fabs(sweepAngle))
{
    const bool valid = isFinitePlanarPoint(center) && isFiniteValue(radius) && radius > 0.0 && isFiniteValue(startAngle) && isFiniteValue(sweepAngle) && sweepAngle != 0.0 && std::fabs(sweepAngle) <= TwoPi;
    MYVOXEL_ASSERT_MESSAGE(valid, "Geometry_Arc parameters must define a finite non-degenerate planar circular arc with |sweepAngle| <= 2*pi.");
    updateCachedData();
}

/// 圆弧参数

const MyMath::Vector3& Geometry_Arc::center() const
{
    return m_center;
}

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

    const double angle = m_startAngle + m_sweepAngle * t;
    return MyMath::Vector3(m_center.x() + m_radius * std::cos(angle), m_center.y() + m_radius * std::sin(angle), 0.0);
}

MyMath::Vector3 Geometry_Arc::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Geometry_Arc parameter must be in [0,1].");

    const double angle = m_startAngle + m_sweepAngle * t;
    const double direction = m_sweepAngle > 0.0 ? 1.0 : -1.0;
    return MyMath::Vector3(-std::sin(angle) * direction, std::cos(angle) * direction, 0.0);
}

/// 曲线创建

Foundation::RefPtr<const Geometry_Curve> Geometry_Arc::reversed() const
{
    return Foundation::RefPtr<const Geometry_Curve>(new Geometry_Arc(m_center, m_radius, m_startAngle + m_sweepAngle, -m_sweepAngle));
}

void Geometry_Arc::updateCachedData()
{
    m_startPoint = pointAt(0.0);
    m_endPoint = pointAt(1.0);
    m_bounds.clear();
    m_bounds.include(m_startPoint);
    m_bounds.include(m_endPoint);

    const double angleTolerance = std::numeric_limits<double>::epsilon() * 32.0; // 覆盖标准角三角函数和角度规范化产生的舍入误差。

    for (int index = 0; index < 4; ++index)
    {
        if (containsAngle(CardinalAngles[index], angleTolerance))
        {
            m_bounds.include(MyMath::Vector3(m_center.x() + m_radius * std::cos(CardinalAngles[index]), m_center.y() + m_radius * std::sin(CardinalAngles[index]), 0.0));
        }
    }
}


}
