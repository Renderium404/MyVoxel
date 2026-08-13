#include "Display_Color.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断颜色分量是否为有限的[0,1]数据。
bool isValidColorComponent(double value)
{
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

}

namespace MyVoxel
{

Display_Color::Display_Color()
    : m_red(1.0f)
    , m_green(1.0f)
    , m_blue(1.0f)
    , m_alpha(1.0f)
{
}

Display_Color::Display_Color(double red, double green, double blue, double alpha)
    : m_red(static_cast<float>(red))
    , m_green(static_cast<float>(green))
    , m_blue(static_cast<float>(blue))
    , m_alpha(static_cast<float>(alpha))
{
    MYVOXEL_ASSERT_MESSAGE(isValidColorComponent(red) && isValidColorComponent(green) && isValidColorComponent(blue) && isValidColorComponent(alpha),
                           "Display color components must be finite and lie in [0, 1].");
}

/// 状态判断

bool Display_Color::isValid() const
{
    return isValidColorComponent(m_red) && isValidColorComponent(m_green) && isValidColorComponent(m_blue) && isValidColorComponent(m_alpha);
}

/// 颜色分量

float Display_Color::red() const
{
    return m_red;
}

float Display_Color::green() const
{
    return m_green;
}

float Display_Color::blue() const
{
    return m_blue;
}

float Display_Color::alpha() const
{
    return m_alpha;
}

/// 颜色创建

Display_Color Display_Color::withAlpha(double alpha) const
{
    return Display_Color(m_red, m_green, m_blue, alpha);
}

Display_Color Display_Color::interpolated(const Display_Color& target, double factor) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid() && target.isValid(), "Display color interpolation requires valid colors.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(factor) && factor >= 0.0 && factor <= 1.0,
                           "Display color interpolation factor must be finite and lie in [0, 1].");
    return Display_Color(m_red + (target.m_red - m_red) * factor,
                         m_green + (target.m_green - m_green) * factor,
                         m_blue + (target.m_blue - m_blue) * factor,
                         m_alpha + (target.m_alpha - m_alpha) * factor);
}

Display_Color Display_Color::inverted() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot invert an invalid display color.");
    return Display_Color(1.0 - m_red, 1.0 - m_green, 1.0 - m_blue, m_alpha);
}

/// 比较

bool Display_Color::operator==(const Display_Color& other) const
{
    return m_red == other.m_red && m_green == other.m_green && m_blue == other.m_blue && m_alpha == other.m_alpha;
}

bool Display_Color::operator!=(const Display_Color& other) const
{
    return !(*this == other);
}

/// 标准颜色

Display_Color Display_Color::black()
{
    return Display_Color(0.0, 0.0, 0.0);
}

Display_Color Display_Color::white()
{
    return Display_Color(1.0, 1.0, 1.0);
}

Display_Color Display_Color::gray()
{
    return Display_Color(0.5, 0.5, 0.5);
}

Display_Color Display_Color::redColor()
{
    return Display_Color(1.0, 0.0, 0.0);
}

Display_Color Display_Color::greenColor()
{
    return Display_Color(0.0, 1.0, 0.0);
}

Display_Color Display_Color::blueColor()
{
    return Display_Color(0.0, 0.0, 1.0);
}

Display_Color Display_Color::cyan()
{
    return Display_Color(0.0, 1.0, 1.0);
}

Display_Color Display_Color::magenta()
{
    return Display_Color(1.0, 0.0, 1.0);
}

Display_Color Display_Color::yellow()
{
    return Display_Color(1.0, 1.0, 0.0);
}

Display_Color Display_Color::transparent()
{
    return Display_Color(0.0, 0.0, 0.0, 0.0);
}

}
