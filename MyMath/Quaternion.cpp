#include "Quaternion.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "MathUtils.h"

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
    if (!axis.isVector(epsilon) || !MyMath::isFinite(angle))
    {
        return Quaternion::zero();
    }

    const Vector3 normalizedAxis = axis.normalized(epsilon);
    const double halfAngle = angle * 0.5; // 四元数使用旋转角的一半计算各分量。
    const double sinHalfAngle = std::sin(halfAngle);

    return Quaternion(std::cos(halfAngle),
                      normalizedAxis.m_x * sinHalfAngle,
                      normalizedAxis.m_y * sinHalfAngle,
                      normalizedAxis.m_z * sinHalfAngle);
}

Quaternion Quaternion::fromRotationMatrix(const Matrix3& matrix, double epsilon)
{
    if (!matrix.isRotationMatrix(epsilon))
    {
        return Quaternion::zero();
    }

    const double m00 = matrix.m_values[0];
    const double m01 = matrix.m_values[1];
    const double m02 = matrix.m_values[2];
    const double m10 = matrix.m_values[3];
    const double m11 = matrix.m_values[4];
    const double m12 = matrix.m_values[5];
    const double m20 = matrix.m_values[6];
    const double m21 = matrix.m_values[7];
    const double m22 = matrix.m_values[8];
    const double matrixTrace = m00 + m11 + m22;

    Quaternion quaternion;

    if (matrixTrace > 0.0)
    {
        const double divisor = 2.0 * std::sqrt((std::max)(0.0, matrixTrace + 1.0));

        if (divisor <= epsilon)
        {
            return Quaternion::zero();
        }

        quaternion.m_w = 0.25 * divisor;
        quaternion.m_x = (m21 - m12) / divisor;
        quaternion.m_y = (m02 - m20) / divisor;
        quaternion.m_z = (m10 - m01) / divisor;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double divisor = 2.0 * std::sqrt((std::max)(0.0, 1.0 + m00 - m11 - m22));

        if (divisor <= epsilon)
        {
            return Quaternion::zero();
        }

        quaternion.m_w = (m21 - m12) / divisor;
        quaternion.m_x = 0.25 * divisor;
        quaternion.m_y = (m01 + m10) / divisor;
        quaternion.m_z = (m02 + m20) / divisor;
    }
    else if (m11 > m22)
    {
        const double divisor = 2.0 * std::sqrt((std::max)(0.0, 1.0 + m11 - m00 - m22));

        if (divisor <= epsilon)
        {
            return Quaternion::zero();
        }

        quaternion.m_w = (m02 - m20) / divisor;
        quaternion.m_x = (m01 + m10) / divisor;
        quaternion.m_y = 0.25 * divisor;
        quaternion.m_z = (m12 + m21) / divisor;
    }
    else
    {
        const double divisor = 2.0 * std::sqrt((std::max)(0.0, 1.0 + m22 - m00 - m11));

        if (divisor <= epsilon)
        {
            return Quaternion::zero();
        }

        quaternion.m_w = (m10 - m01) / divisor;
        quaternion.m_x = (m02 + m20) / divisor;
        quaternion.m_y = (m12 + m21) / divisor;
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
    return MyMath::isFinite(m_w) && MyMath::isFinite(m_x) &&
           MyMath::isFinite(m_y) && MyMath::isFinite(m_z);
}

bool Quaternion::isZero(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    const double scale = maximumAbsolute(m_w, m_x, m_y, m_z);

    if (scale == 0.0)
    {
        return true;
    }

    const double normalizedLength = scaledNorm(m_w, m_x, m_y, m_z, scale);

    return scale <= epsilon / normalizedLength;
}

bool Quaternion::isIdentity(double epsilon) const
{
    if (!isFinite())
    {
        return false;
    }

    return isEqualTo(Quaternion::identity(), epsilon) ||
           isEqualTo(Quaternion(-1.0, 0.0, 0.0, 0.0), epsilon);
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

    return norm(m_w - other.m_w,
                m_x - other.m_x,
                m_y - other.m_y,
                m_z - other.m_z) <= epsilon;
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
    return norm(m_w, m_x, m_y, m_z);
}

Quaternion Quaternion::normalized(double epsilon) const
{
    if (!isFinite())
    {
        return Quaternion::zero();
    }

    const double scale = maximumAbsolute(m_w, m_x, m_y, m_z);

    if (scale == 0.0)
    {
        return Quaternion::zero();
    }

    const double normalizedLength = scaledNorm(m_w, m_x, m_y, m_z, scale);

    if (scale <= epsilon / normalizedLength)
    {
        return Quaternion::zero();
    }

    return Quaternion(m_w / scale / normalizedLength,
                      m_x / scale / normalizedLength,
                      m_y / scale / normalizedLength,
                      m_z / scale / normalizedLength);
}

bool Quaternion::normalize(double epsilon)
{
    if (!isFinite())
    {
        return false;
    }

    const double scale = maximumAbsolute(m_w, m_x, m_y, m_z);

    if (scale == 0.0)
    {
        return false;
    }

    const double normalizedLength = scaledNorm(m_w, m_x, m_y, m_z, scale);

    if (scale <= epsilon / normalizedLength)
    {
        return false;
    }

    m_w = m_w / scale / normalizedLength;
    m_x = m_x / scale / normalizedLength;
    m_y = m_y / scale / normalizedLength;
    m_z = m_z / scale / normalizedLength;

    return true;
}

/// 四元数变换

Quaternion Quaternion::conjugated() const
{
    return Quaternion(m_w, -m_x, -m_y, -m_z);
}

Quaternion Quaternion::inverted(double epsilon) const
{
    if (!isFinite())
    {
        return Quaternion::zero();
    }

    const double scale = maximumAbsolute(m_w, m_x, m_y, m_z);

    if (scale == 0.0)
    {
        return Quaternion::zero();
    }

    const double scaledW = m_w / scale;
    const double scaledX = m_x / scale;
    const double scaledY = m_y / scale;
    const double scaledZ = m_z / scale;
    const double scaledLengthSquared =
        scaledW * scaledW +
        scaledX * scaledX +
        scaledY * scaledY +
        scaledZ * scaledZ;

    const double normalizedLength = std::sqrt(scaledLengthSquared);

    if (scale <= epsilon / normalizedLength)
    {
        return Quaternion::zero();
    }

    const double factor = 1.0 / scale / scaledLengthSquared;

    return Quaternion(scaledW * factor,
                      -scaledX * factor,
                      -scaledY * factor,
                      -scaledZ * factor);
}

void Quaternion::toAxisAngle(Vector3& axis, double& angle, double epsilon) const
{
    assert(isUnit(epsilon));

    const Quaternion canonical = m_w < 0.0 ? -*this : *this;
    const double boundedW = clamp(canonical.m_w, -1.0, 1.0);
    const double calculatedAngle = 2.0 * std::acos(boundedW);
    const double sinHalfAngle = std::sqrt((std::max)(0.0, 1.0 - boundedW * boundedW));

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

Matrix3 Quaternion::toRotationMatrix(double epsilon) const
{
    if (!isUnit(epsilon))
    {
        return Matrix3::zero();
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

    return Matrix3(1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
                   2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
                   2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy));
}

Vector3 Quaternion::rotateVector(const Vector3& vector, double epsilon) const
{
    assert(isUnit(epsilon));
    assert(vector.isFinite());

    const double intermediateX = 2.0 * (m_y * vector.m_z - m_z * vector.m_y);
    const double intermediateY = 2.0 * (m_z * vector.m_x - m_x * vector.m_z);
    const double intermediateZ = 2.0 * (m_x * vector.m_y - m_y * vector.m_x);

    return Vector3(vector.m_x + m_w * intermediateX + m_y * intermediateZ - m_z * intermediateY,
                   vector.m_y + m_w * intermediateY + m_z * intermediateX - m_x * intermediateZ,
                   vector.m_z + m_w * intermediateZ + m_x * intermediateY - m_y * intermediateX);
}

/// 插值计算

Quaternion Quaternion::slerp(const Quaternion& from, const Quaternion& to, double factor, double epsilon)
{
    if (!from.isUnit(epsilon) ||
        !to.isUnit(epsilon) ||
        !MyMath::isFinite(factor) ||
        factor < 0.0 ||
        factor > 1.0)
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

    cosine = clamp(cosine, -1.0, 1.0);

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

/// 算术运算

double Quaternion::dot(const Quaternion& first, const Quaternion& second)
{
    return first.m_w * second.m_w +
           first.m_x * second.m_x +
           first.m_y * second.m_y +
           first.m_z * second.m_z;
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
    return Quaternion(m_w + other.m_w,
                      m_x + other.m_x,
                      m_y + other.m_y,
                      m_z + other.m_z);
}

Quaternion Quaternion::operator-(const Quaternion& other) const
{
    return Quaternion(m_w - other.m_w,
                      m_x - other.m_x,
                      m_y - other.m_y,
                      m_z - other.m_z);
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