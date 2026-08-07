#include "CurveDiscretizer.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 默认圆弧显示精度按完整圆64段确定。
const double DefaultMaximumArcAngle = Pi / 32.0; // 默认每段最大5.625度，完整圆最多64段。
const std::size_t DefaultMaximumArcSegmentCount = 4096; // 防止异常小角度参数导致单曲线显示点数量无界增长。

}

namespace MyVoxel
{

CurveDiscretizationOptions::CurveDiscretizationOptions()
    : maximumArcAngle(DefaultMaximumArcAngle)
    , maximumArcSegmentCount(DefaultMaximumArcSegmentCount)
{
}

bool CurveDiscretizationOptions::isValid() const
{
    return std::isfinite(maximumArcAngle) && maximumArcAngle > 0.0 && maximumArcSegmentCount > 0;
}

std::size_t CurveDiscretizer::segmentCount(const Topology_Curve& curve, const CurveDiscretizationOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(curve.isValid(), "Curve discretization requires a valid Topology_Curve.");
    MYVOXEL_ASSERT_MESSAGE(options.isValid(), "Curve discretization requires valid options.");

    if (!curve.isValid() || !options.isValid())
    {
        return 0;
    }

    if (curve.kind() == CurveKind::Line)
    {
        return 1;
    }

    const Geometry_Arc& arc = static_cast<const Geometry_Arc&>(curve.geometry());
    const double rawCount = std::ceil(std::fabs(arc.sweepAngle()) / options.maximumArcAngle);
    std::size_t count = rawCount > 1.0 ? static_cast<std::size_t>(rawCount) : 1;

    if (count > options.maximumArcSegmentCount)
    {
        count = options.maximumArcSegmentCount;
    }

    return count;
}

void CurveDiscretizer::appendSegments(const Topology_Curve& curve, std::vector<MyMath::Vector3>& points,
                                       const CurveDiscretizationOptions& options)
{
    const std::size_t count = segmentCount(curve, options);

    if (count == 0)
    {
        return;
    }

    points.reserve(points.size() + count * 2);

    if (curve.kind() == CurveKind::Line)
    {
        points.push_back(curve.startPoint());
        points.push_back(curve.endPoint());
        return;
    }

    MyMath::Vector3 previous = curve.pointAt(0.0);

    for (std::size_t index = 1; index <= count; ++index)
    {
        const double parameter = static_cast<double>(index) / static_cast<double>(count);
        const MyMath::Vector3 current = curve.pointAt(parameter);
        points.push_back(previous);
        points.push_back(current);
        previous = current;
    }
}

std::vector<MyMath::Vector3> CurveDiscretizer::buildSegments(const Topology_Curve& curve, const CurveDiscretizationOptions& options)
{
    std::vector<MyMath::Vector3> result;
    appendSegments(curve, result, options);
    return result;
}

}
