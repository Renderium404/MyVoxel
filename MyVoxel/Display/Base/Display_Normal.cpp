#include "Display_Normal.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Display_Normal::Display_Normal()
    : m_x(0.0f)
    , m_y(0.0f)
    , m_z(0.0f)
{
}

Display_Normal::Display_Normal(double x, double y, double z)
    : m_x(static_cast<float>(x))
    , m_y(static_cast<float>(y))
    , m_z(static_cast<float>(z))
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(x) && std::isfinite(y) && std::isfinite(z),
                           "Display normal components must be finite.");
}

Display_Normal::Display_Normal(const MyMath::Vector3& vector)
    : m_x(static_cast<float>(vector.x()))
    , m_y(static_cast<float>(vector.y()))
    , m_z(static_cast<float>(vector.z()))
{
    MYVOXEL_ASSERT_MESSAGE(vector.isFinite(), "Display normal vector must be finite.");
}

/// 状态判断

bool Display_Normal::isFinite() const
{
    return std::isfinite(static_cast<double>(m_x)) && std::isfinite(static_cast<double>(m_y)) && std::isfinite(static_cast<double>(m_z));
}

bool Display_Normal::isZero() const
{
    return m_x == 0.0f && m_y == 0.0f && m_z == 0.0f;
}

bool Display_Normal::isDirection(double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(epsilon) && epsilon >= 0.0,
                           "Display normal direction epsilon must be finite and non-negative.");
    return vector().isVector(epsilon);
}

bool Display_Normal::isUnit(double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(epsilon) && epsilon >= 0.0,
                           "Display normal unit epsilon must be finite and non-negative.");
    return vector().isUnit(epsilon);
}

/// 法线分量

float Display_Normal::x() const
{
    return m_x;
}

float Display_Normal::y() const
{
    return m_y;
}

float Display_Normal::z() const
{
    return m_z;
}

MyMath::Vector3 Display_Normal::vector() const
{
    return MyMath::Vector3(static_cast<double>(m_x), static_cast<double>(m_y), static_cast<double>(m_z));
}

/// 法线创建

Display_Normal Display_Normal::normalized(double epsilon) const
{
    MYVOXEL_ASSERT_MESSAGE(isDirection(epsilon), "Cannot normalize an invalid display normal.");
    if (!isDirection(epsilon))
    {
        return Display_Normal();
    }
    return Display_Normal(vector().normalized(epsilon));
}

Display_Normal Display_Normal::reversed() const
{
    MYVOXEL_ASSERT_MESSAGE(isFinite(), "Cannot reverse a non-finite display normal.");
    return Display_Normal(-m_x, -m_y, -m_z);
}

/// 比较

bool Display_Normal::operator==(const Display_Normal& other) const
{
    return m_x == other.m_x && m_y == other.m_y && m_z == other.m_z;
}

bool Display_Normal::operator!=(const Display_Normal& other) const
{
    return !(*this == other);
}

}
