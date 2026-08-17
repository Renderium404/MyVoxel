#include "FeatureTrace.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{
namespace Modeling
{

FeatureTrace::FeatureTrace()
{
}

/// 状态判断

bool FeatureTrace::isEmpty() const
{
    return m_points.empty();
}

bool FeatureTrace::hasSegment() const
{
    return m_points.size() >= 2;
}

bool FeatureTrace::isClosed() const
{
    return m_points.size() >= 3 && m_points.front().isEqualTo(m_points.back(), 0.0);
}

/// 特征点访问

std::size_t FeatureTrace::pointCount() const
{
    return m_points.size();
}

const MyMath::Vector3& FeatureTrace::point(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(index < m_points.size(), "FeatureTrace point index is out of range.");
    return m_points[index];
}

const std::vector<MyMath::Vector3>& FeatureTrace::points() const
{
    return m_points;
}

const MyMath::Vector3& FeatureTrace::startPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(!m_points.empty(), "Cannot access the start point of an empty FeatureTrace.");
    return m_points.front();
}

const MyMath::Vector3& FeatureTrace::endPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(!m_points.empty(), "Cannot access the end point of an empty FeatureTrace.");
    return m_points.back();
}

/// 特征点编辑

bool FeatureTrace::addPoint(const MyMath::Vector3& point)
{
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "FeatureTrace requires finite feature points.");

    if (!point.isFinite())
    {
        return false;
    }

    if (!m_points.empty() && m_points.back().isEqualTo(point, 0.0))
    {
        return false;
    }

    m_points.push_back(point);
    return true;
}

bool FeatureTrace::removeLastPoint()
{
    if (m_points.empty())
    {
        return false;
    }

    m_points.pop_back();
    return true;
}

void FeatureTrace::clear()
{
    m_points.clear();
}

}
}