#include "Geometry_Polyline.h"

#include <algorithm>
#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断标量是否为有限正数。
bool isFinitePositive(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity && value > 0.0;
}

}

namespace MyVoxel
{

Geometry_Polyline::Geometry_Polyline(const std::vector<MyMath::Vector3>& points)
    : m_points(points)
    , m_length(0.0)
    , m_closed(false)
{
    MYVOXEL_ASSERT_MESSAGE(m_points.size() >= 2, "Geometry_Polyline requires at least two points.");
    rebuild();
}

/// 折线数据

std::size_t Geometry_Polyline::pointCount() const
{
    return m_points.size();
}

std::size_t Geometry_Polyline::segmentCount() const
{
    return m_points.size() > 1 ? m_points.size() - 1 : 0;
}

const MyMath::Vector3& Geometry_Polyline::point(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(index < m_points.size(), "Geometry_Polyline point index is out of range.");
    return m_points[index];
}

const std::vector<MyMath::Vector3>& Geometry_Polyline::points() const
{
    return m_points;
}

double Geometry_Polyline::segmentLength(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(index < segmentCount(), "Geometry_Polyline segment index is out of range.");
    return m_cumulativeLengths[index + 1] - m_cumulativeLengths[index];
}

bool Geometry_Polyline::isClosed() const
{
    return m_closed;
}

/// 曲线属性

CurveKind Geometry_Polyline::kind() const
{
    return CurveKind::Polyline;
}

const MyMath::Vector3& Geometry_Polyline::startPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(!m_points.empty(), "Cannot access the start point of an invalid Geometry_Polyline.");
    return m_points.front();
}

const MyMath::Vector3& Geometry_Polyline::endPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(!m_points.empty(), "Cannot access the end point of an invalid Geometry_Polyline.");
    return m_points.back();
}

double Geometry_Polyline::length() const
{
    return m_length;
}

const Bounds3& Geometry_Polyline::bounds() const
{
    MYVOXEL_ASSERT_MESSAGE(m_bounds.isValid(), "Cannot access the bounds of an invalid Geometry_Polyline.");
    return m_bounds;
}

/// 参数查询

MyMath::Vector3 Geometry_Polyline::pointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Geometry_Polyline parameter must be in [0,1].");
    MYVOXEL_ASSERT_MESSAGE(m_length > 0.0 && m_segmentDirections.size() == segmentCount(), "Cannot query an invalid Geometry_Polyline.");

    if (t <= 0.0)
    {
        return m_points.front();
    }

    if (t >= 1.0)
    {
        return m_points.back();
    }

    const double distance = t * m_length;
    const std::size_t segmentIndex = segmentIndexAtDistance(distance);
    const double segmentStartDistance = m_cumulativeLengths[segmentIndex];
    const double currentSegmentLength = m_cumulativeLengths[segmentIndex + 1] - segmentStartDistance;
    const double localParameter = (distance - segmentStartDistance) / currentSegmentLength;
    return m_points[segmentIndex] + (m_points[segmentIndex + 1] - m_points[segmentIndex]) * localParameter;
}

MyMath::Vector3 Geometry_Polyline::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Geometry_Polyline parameter must be in [0,1].");
    MYVOXEL_ASSERT_MESSAGE(m_length > 0.0 && !m_segmentDirections.empty(), "Cannot query an invalid Geometry_Polyline.");

    if (t >= 1.0)
    {
        return m_segmentDirections.back();
    }

    return m_segmentDirections[segmentIndexAtDistance(t * m_length)];
}

/// 曲线创建

Foundation::RefPtr<const Geometry_Curve> Geometry_Polyline::reversed() const
{
    std::vector<MyMath::Vector3> reversedPoints(m_points.rbegin(), m_points.rend());
    return Foundation::RefPtr<const Geometry_Curve>(new Geometry_Polyline(reversedPoints));
}

/// 内部辅助

std::size_t Geometry_Polyline::segmentIndexAtDistance(double distance) const
{
    MYVOXEL_ASSERT_MESSAGE(distance >= 0.0 && distance <= m_length, "Geometry_Polyline arc-length query is outside the valid range.");
    MYVOXEL_ASSERT_MESSAGE(m_cumulativeLengths.size() == m_points.size() && m_cumulativeLengths.size() >= 2, "Geometry_Polyline cumulative-length cache is invalid.");

    const std::vector<double>::const_iterator upper = std::upper_bound(m_cumulativeLengths.begin(), m_cumulativeLengths.end(), distance);
    std::size_t index = upper == m_cumulativeLengths.begin() ? 0 : static_cast<std::size_t>((upper - m_cumulativeLengths.begin()) - 1);

    if (index >= segmentCount())
    {
        index = segmentCount() - 1;
    }

    return index;
}

void Geometry_Polyline::rebuild()
{
    m_cumulativeLengths.clear();
    m_segmentDirections.clear();
    m_bounds.clear();
    m_length = 0.0;
    m_closed = false;

    if (m_points.size() < 2)
    {
        return;
    }

    m_cumulativeLengths.reserve(m_points.size());
    m_segmentDirections.reserve(m_points.size() - 1);
    m_cumulativeLengths.push_back(0.0);

    for (std::size_t index = 0; index < m_points.size(); ++index)
    {
        MYVOXEL_ASSERT_MESSAGE(m_points[index].isFinite(), "Geometry_Polyline points must be finite.");

        if (!m_points[index].isFinite())
        {
            m_cumulativeLengths.clear();
            m_segmentDirections.clear();
            m_bounds.clear();
            m_length = 0.0;
            return;
        }

        m_bounds.include(m_points[index]);

        if (index == 0)
        {
            continue;
        }

        const MyMath::Vector3 segment = m_points[index] - m_points[index - 1];
        const double segmentLengthValue = m_points[index - 1].distanceTo(m_points[index]);
        MYVOXEL_ASSERT_MESSAGE(isFinitePositive(segmentLengthValue), "Geometry_Polyline adjacent points must define a finite non-degenerate segment.");

        if (!isFinitePositive(segmentLengthValue) || !isFinitePositive(m_length + segmentLengthValue))
        {
            m_cumulativeLengths.clear();
            m_segmentDirections.clear();
            m_bounds.clear();
            m_length = 0.0;
            return;
        }

        m_length += segmentLengthValue;
        m_cumulativeLengths.push_back(m_length);
        m_segmentDirections.push_back(segment / segmentLengthValue);
    }

    MYVOXEL_ASSERT_MESSAGE(m_bounds.isValid() && m_cumulativeLengths.size() == m_points.size() && m_segmentDirections.size() == m_points.size() - 1, "Geometry_Polyline cache construction failed.");
    m_closed = m_points.front().isEqualTo(m_points.back(), 0.0);
}

}