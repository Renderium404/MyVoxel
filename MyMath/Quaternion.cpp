#include "Quaternion.h"

#include <algorithm>
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

const double Quaternion::DefaultEpsilon = 1.0e-10; // 旋转单位性及插值计算默认误差。

Quaternion::Quaternion()
    : m_w(1.0)
    , m_x(0.0)
    , m_y(0.0)
    , m_z(0.0)
{
}

Quaternion::Quaternion(double w, double x, double y, double z)
    : m_w(w)
    , m_x(x)
    , m_y(y)
    , m_z(z)
{
}

/// 四元数创建

Quaternion Quaternion::zero()
{
    return Quaternion(0.0, 0.0, 0.0, 0.0);
}

Quaternion Quaternion::identity()
{
    return Quaternion();
}

Quaternion Quaternion::fromAxisAngle(const Vector3& axis, double angle, double epsilon)
{
    if (!axis.isVector(epsilon) || !isFiniteValue(angle))
    {
        return Quaternion::zero();
    }

    const Vector3 normalizedAxis = axis.normalized(epsilon);
    const double halfAngle = angle * 0.5; // 四元数使用旋转角的一半计算各分量。
    const double sinHalfAngle = std::sin(halfAngle);

    return Quaternion(std::cos(halfAngle),
                      normalizedAxis.x() * sinHalfAngle,
                      normalizedAxis.y() * sinHalfAngle,
                      normalizedAxis.z() * sinHalfAngle);
}

Quaternion Quaternion::fromRotationMatrix(const Matrix4& matrix, double epsilon)
{
    if (!matrix.isRotationMatrix(epsilon))
    {
        return Quaternion::zero();
    }

    const double m00 = matrix(0, 0);
    const double m11 = matrix(1, 1);
    const double m22 = matrix(2, 2);
    const double trace = m00 + m11 + m22;

    Quaternion quaternion;

    if (trace > 0.0)
    {
        const double divisor = 2.0 * std::sqrt(std::max(0.0, trace + 1.0));

        quaternion.m_w = 0.25 * divisor;
        quaternion.m_x = (matrix(2, 1) - matrix(1, 2)) / divisor;
        quaternion.m_y = (matrix(0, 2) - matrix(2, 0)) / divisor;
        quaternion.m_z = (matrix(1, 0) - matrix(0, 1)) / divisor;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double divisor = 2.0 * std::sqrt(std::max(0.0, 1.0 + m00 - m11 - m22));

        quaternion.m_w = (matrix(2, 1) - matrix(1, 2)) / divisor;
        quaternion.m_x = 0.25 * divisor;
        quaternion.m_y = (matrix(0, 1) + matrix(1, 0)) / divisor;
        quaternion.m_z = (matrix(0, 2) + matrix(2, 0)) / divisor;
    }
    else if (m11 > m22)
    {
        const double divisor = 2.0 * std::sqrt(std::max(0.0, 1.0 + m11 - m00 - m22));

        quaternion.m_w = (matrix(0, 2) - matrix(2, 0)) / divisor;
        quaternion.m_x = (matrix(0, 1) + matrix(1, 0)) / divisor;
        quaternion.m_y = 0.25 * divisor;
        quaternion.m_z = (matrix(1, 2) + matrix(2, 1)) / divisor;
    }
    else
    {
        const double divisor = 2.0 * std::sqrt(std::max(0.0, 1.0 + m22 - m00 - m11));

        quaternion.m_w = (matrix(1, 0) - matrix(0, 1)) / divisor;
        quaternion.m_x = (matrix(0, 2) + matrix(2, 0)) / divisor;
        quaternion.m_y = (matrix(1, 2) + matrix(2, 1)) / divisor;
        quaternion.m_z = 0.25 * divisor;
    }

    return quaternion.normalized(epsilon);
}

/// 分量访问

double Quaternion::w() const
{
    return m_w;
}

double Quaternion::x() const
{
    return m_x;
}

double Quaternion::y() const
{
    return m_y;
}

double Quaternion::z() const
{
    return m_z;
}

void Quaternion::setW(double w)
{
    m_w = w;
}

void Quaternion::setX(double x)
{
    m_x = x;
}

void Quaternion::setY(double y)
{
    m_y = y;
}

void Quaternion::setZ(double z)
{
    m_z = z;
}

void Quaternion::set(double w, double x, double y, double z)
{
    m_w = w;
    m_x = x;
    m_y = y;
    m_z = z;
}

/// 状态判断

bool Quaternion::isFinite() const
{
    return isFiniteValue(m_w) && isFiniteValue(m_x) &&
           isFiniteValue(m_y) && isFiniteValue(m_z);
}

bool Quaternion::isZero(double epsilon) const
{
    return isFinite() && lengthSquared() <= epsilon * epsilon;
}

bool Quaternion::isIdentity(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    const Quaternion positiveIdentity = Quaternion::identity();
    const Quaternion negativeIdentity(-1.0, 0.0, 0.0, 0.0);

    return isEqualTo(positiveIdentity, epsilon) || isEqualTo(negativeIdentity, epsilon);
}

bool Quaternion::isUnit(double epsilon) const
{
    return isFinite() && std::fabs(length() - 1.0) <= epsilon;
}

bool Quaternion::isEqualTo(const Quaternion& other, double epsilon) const
{
    if (!isFinite() || !other.isFinite())
    {
        return false;
    }

    const double deltaW = m_w - other.m_w;
    const double deltaX = m_x - other.m_x;
    const double deltaY = m_y - other.m_y;
    const double deltaZ = m_z - other.m_z;

    return deltaW * deltaW + deltaX * deltaX +
           deltaY * deltaY + deltaZ * deltaZ <= epsilon * epsilon;
}

bool Quaternion::isSameRotation(const Quaternion& other, double epsilon) const
{
    if (!isUnit(epsilon) || !other.isUnit(epsilon))
    {
        return false;
    }

    return isEqualTo(other, epsilon) || isEqualTo(-other, epsilon);
}

/// 长度与归一化

double Quaternion::lengthSquared() const
{
    return m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z;
}

double Quaternion::length() const
{
    return std::sqrt(lengthSquared());
}

Quaternion Quaternion::normalized(double epsilon) const
{
    if (!isFinite() || isZero(epsilon))
    {
        return Quaternion::zero();
    }

    return *this / length();
}

bool Quaternion::normalize(double epsilon)
{
    if (!isFinite() || isZero(epsilon))
    {
        return false;
    }

    const double quaternionLength = length();

    m_w /= quaternionLength;
    m_x /= quaternionLength;
    m_y /= quaternionLength;
    m_z /= quaternionLength;

    return true;
}

/// 四元数变换

Quaternion Quaternion::conjugated() const
{
    return Quaternion(m_w, -m_x, -m_y, -m_z);
}

Quaternion Quaternion::inverted(double epsilon) const
{
    if (!isFinite() || isZero(epsilon))
    {
        return Quaternion::zero();
    }

    return conjugated() / lengthSquared();
}

void Quaternion::toAxisAngle(Vector3& axis, double& angle, double epsilon) const
{
    assert(isUnit(epsilon));

    const Quaternion canonical = m_w < 0.0 ? -*this : *this;
    const double boundedW = std::max(-1.0, std::min(1.0, canonical.m_w));
    const double calculatedAngle = 2.0 * std::acos(boundedW);
    const double sinHalfAngle = std::sqrt(std::max(0.0, 1.0 - boundedW * boundedW));

    if (sinHalfAngle <= epsilon)
    {
        axis = Vector3::unitX(); // 零旋转的旋转轴不唯一，统一返回X正方向。
        angle = 0.0;
        return;
    }

    axis = Vector3(canonical.m_x / sinHalfAngle,
                   canonical.m_y / sinHalfAngle,
                   canonical.m_z / sinHalfAngle).normalized(epsilon);
    angle = calculatedAngle;
}

Matrix4 Quaternion::toRotationMatrix(double epsilon) const
{
    if (!isUnit(epsilon))
    {
        return Matrix4::zero();
    }

    const double xx = m_x * m_x;
    const double yy = m_y * m_y;
    const double zz = m_z * m_z;
    const double xy = m_x * m_y;
    const double xz = m_x * m_z;
    const double yz = m_y * m_z;
    const double wx = m_w * m_x;
    const double wy = m_w * m_y;
    const double wz = m_w * m_z;

    Matrix4 matrix = Matrix4::identity();

    matrix(0, 0) = 1.0 - 2.0 * (yy + zz);
    matrix(0, 1) = 2.0 * (xy - wz);
    matrix(0, 2) = 2.0 * (xz + wy);
    matrix(1, 0) = 2.0 * (xy + wz);
    matrix(1, 1) = 1.0 - 2.0 * (xx + zz);
    matrix(1, 2) = 2.0 * (yz - wx);
    matrix(2, 0) = 2.0 * (xz - wy);
    matrix(2, 1) = 2.0 * (yz + wx);
    matrix(2, 2) = 1.0 - 2.0 * (xx + yy);

    return matrix;
}

Vector3 Quaternion::rotateVector(const Vector3& vector, double epsilon) const
{
    assert(isUnit(epsilon));
    assert(vector.isFinite());

    const Vector3 quaternionVector(m_x, m_y, m_z);
    const Vector3 intermediate = 2.0 * Vector3::cross(quaternionVector, vector);

    return vector + m_w * intermediate + Vector3::cross(quaternionVector, intermediate);
}

/// 插值计算

Quaternion Quaternion::slerp(const Quaternion& from, const Quaternion& to, double factor,
                             double epsilon)
{
    if (!from.isUnit(epsilon) || !to.isUnit(epsilon) ||
        !isFiniteValue(factor) || factor < 0.0 || factor > 1.0)
    {
        return Quaternion::zero();
    }

    Quaternion target = to;
    double cosine = dot(from, target);

    if (cosine < 0.0)
    {
        target = -target;
        cosine = -cosine;
    }

    cosine = std::max(-1.0, std::min(1.0, cosine));

    if (1.0 - cosine <= epsilon)
    {
        return (from * (1.0 - factor) + target * factor).normalized(epsilon);
    }

    const double interpolationAngle = std::acos(cosine);
    const double sinAngle = std::sin(interpolationAngle);

    if (std::fabs(sinAngle) <= epsilon)
    {
        return Quaternion::zero();
    }

    const double fromWeight = std::sin((1.0 - factor) * interpolationAngle) / sinAngle;
    const double toWeight = std::sin(factor * interpolationAngle) / sinAngle;

    return (from * fromWeight + target * toWeight).normalized(epsilon);
}

/// 四元数运算

double Quaternion::dot(const Quaternion& first, const Quaternion& second)
{
    return first.m_w * second.m_w + first.m_x * second.m_x +
           first.m_y * second.m_y + first.m_z * second.m_z;
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
    return Quaternion(m_w + other.m_w, m_x + other.m_x,
                      m_y + other.m_y, m_z + other.m_z);
}

Quaternion Quaternion::operator-(const Quaternion& other) const
{
    return Quaternion(m_w - other.m_w, m_x - other.m_x,
                      m_y - other.m_y, m_z - other.m_z);
}

Quaternion Quaternion::operator-() const
{
    return Quaternion(-m_w, -m_x, -m_y, -m_z);
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
    return Quaternion(m_w * other.m_w - m_x * other.m_x - m_y * other.m_y - m_z * other.m_z,
                      m_w * other.m_x + m_x * other.m_w + m_y * other.m_z - m_z * other.m_y,
                      m_w * other.m_y - m_x * other.m_z + m_y * other.m_w + m_z * other.m_x,
                      m_w * other.m_z + m_x * other.m_y - m_y * other.m_x + m_z * other.m_w);
}

Quaternion Quaternion::operator*(double scalar) const
{
    return Quaternion(m_w * scalar, m_x * scalar, m_y * scalar, m_z * scalar);
}

Quaternion Quaternion::operator/(double scalar) const
{
    assert(scalar != 0.0);

    return Quaternion(m_w / scalar, m_x / scalar, m_y / scalar, m_z / scalar);
}

Quaternion& Quaternion::operator+=(const Quaternion& other)
{
    m_w += other.m_w;
    m_x += other.m_x;
    m_y += other.m_y;
    m_z += other.m_z;

    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other)
{
    m_w -= other.m_w;
    m_x -= other.m_x;
    m_y -= other.m_y;
    m_z -= other.m_z;

    return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& other)
{
    *this = *this * other;
    return *this;
}

Quaternion& Quaternion::operator*=(double scalar)
{
    m_w *= scalar;
    m_x *= scalar;
    m_y *= scalar;
    m_z *= scalar;

    return *this;
}

Quaternion& Quaternion::operator/=(double scalar)
{
    assert(scalar != 0.0);

    m_w /= scalar;
    m_x /= scalar;
    m_y /= scalar;
    m_z /= scalar;

    return *this;
}

Quaternion operator*(double scalar, const Quaternion& quaternion)
{
    return quaternion * scalar;
}

}