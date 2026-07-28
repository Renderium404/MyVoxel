#include "Vector3.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace
{

// 判断浮点数是否为有限数值，不包含NaN和正负无穷。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();

    return value == value && value != infinity && value != -infinity;
}

}

namespace MyMath
{

const double Vector3::DefaultEpsilon = 1.0e-12; // 默认浮点比较误差。

Vector3::Vector3()
    : m_x(0.0)
    , m_y(0.0)
    , m_z(0.0)
{
}

Vector3::Vector3(double x, double y, double z)
    : m_x(x)
    , m_y(y)
    , m_z(z)
{
}

/// 常用数据

Vector3 Vector3::zero()
{
    return Vector3(0.0, 0.0, 0.0);
}

Vector3 Vector3::unitX()
{
    return Vector3(1.0, 0.0, 0.0);
}

Vector3 Vector3::unitY()
{
    return Vector3(0.0, 1.0, 0.0);
}

Vector3 Vector3::unitZ()
{
    return Vector3(0.0, 0.0, 1.0);
}

/// 分量访问

double Vector3::x() const
{
    return m_x;
}

double Vector3::y() const
{
    return m_y;
}

double Vector3::z() const
{
    return m_z;
}

void Vector3::setX(double x)
{
    m_x = x;
}

void Vector3::setY(double y)
{
    m_y = y;
}

void Vector3::setZ(double z)
{
    m_z = z;
}

void Vector3::set(double x, double y, double z)
{
    m_x = x;
    m_y = y;
    m_z = z;
}

/// 状态判断

bool Vector3::isFinite() const
{
    return isFiniteValue(m_x) && isFiniteValue(m_y) && isFiniteValue(m_z);
}

bool Vector3::isVector(double epsilon) const
{
    return isFinite() && lengthSquared() > epsilon * epsilon;
}

bool Vector3::isZero(double epsilon) const
{
    return isFinite() && lengthSquared() <= epsilon * epsilon;
}

bool Vector3::isUnit(double epsilon) const
{
    return isFinite() && std::fabs(length() - 1.0) <= epsilon;
}

bool Vector3::isEqualTo(const Vector3& other, double epsilon) const
{
    return isFinite() && other.isFinite() && distanceSquaredTo(other) <= epsilon * epsilon;
}

/// 长度与距离

double Vector3::lengthSquared() const
{
    return m_x * m_x + m_y * m_y + m_z * m_z;
}

double Vector3::length() const
{
    return std::sqrt(lengthSquared());
}

double Vector3::distanceSquaredTo(const Vector3& other) const
{
    const double deltaX = m_x - other.m_x;
    const double deltaY = m_y - other.m_y;
    const double deltaZ = m_z - other.m_z;

    return deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
}

double Vector3::distanceTo(const Vector3& other) const
{
    return std::sqrt(distanceSquaredTo(other));
}

/// 向量计算

Vector3 Vector3::normalized(double epsilon) const
{
    if (!isVector(epsilon))
    {
        return Vector3::zero();
    }

    return *this / length();
}

bool Vector3::normalize(double epsilon)
{
    if (!isVector(epsilon))
    {
        return false;
    }

    const double vectorLength = length();

    m_x /= vectorLength;
    m_y /= vectorLength;
    m_z /= vectorLength;

    return true;
}

double Vector3::dot(const Vector3& first, const Vector3& second)
{
    return first.m_x * second.m_x + first.m_y * second.m_y + first.m_z * second.m_z;
}

Vector3 Vector3::cross(const Vector3& first, const Vector3& second)
{
    return Vector3(first.m_y * second.m_z - first.m_z * second.m_y,
                   first.m_z * second.m_x - first.m_x * second.m_z,
                   first.m_x * second.m_y - first.m_y * second.m_x);
}

/// 算术运算

Vector3 Vector3::operator+(const Vector3& other) const
{
    return Vector3(m_x + other.m_x, m_y + other.m_y, m_z + other.m_z);
}

Vector3 Vector3::operator-(const Vector3& other) const
{
    return Vector3(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z);
}

Vector3 Vector3::operator-() const
{
    return Vector3(-m_x, -m_y, -m_z);
}

Vector3 Vector3::operator*(double scalar) const
{
    return Vector3(m_x * scalar, m_y * scalar, m_z * scalar);
}

Vector3 Vector3::operator/(double scalar) const
{
    assert(scalar != 0.0);

    return Vector3(m_x / scalar, m_y / scalar, m_z / scalar);
}

Vector3& Vector3::operator+=(const Vector3& other)
{
    m_x += other.m_x;
    m_y += other.m_y;
    m_z += other.m_z;

    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other)
{
    m_x -= other.m_x;
    m_y -= other.m_y;
    m_z -= other.m_z;

    return *this;
}

Vector3& Vector3::operator*=(double scalar)
{
    m_x *= scalar;
    m_y *= scalar;
    m_z *= scalar;

    return *this;
}

Vector3& Vector3::operator/=(double scalar)
{
    assert(scalar != 0.0);

    m_x /= scalar;
    m_y /= scalar;
    m_z /= scalar;

    return *this;
}

Vector3 operator*(double scalar, const Vector3& vector)
{
    return vector * scalar;
}

}