#include "Display_CurveBuilder.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Instance/Curve.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"

namespace
{

const double TwoPi = 6.283185307179586476925286766559; // 完整圆对应的弧度值。

// 根据两点弦长返回半径为radius的较小圆心角，输入弧段不得超过π。
double chordAngle(const MyMath::Vector3& first, const MyMath::Vector3& second, double radius)
{
    const double chordLength = (second - first).length();
    double ratio = chordLength / (2.0 * radius);
    ratio = (std::max)(0.0, (std::min)(1.0, ratio));
    return 2.0 * std::asin(ratio);
}

// 根据Edge规范化pointAt接口恢复当前实际使用圆弧的绝对扫掠角。
double edgeArcSweepAngle(const MyVoxel::Topology_Edge& edge, const MyVoxel::Geometry_Arc& arc)
{
    const MyMath::Vector3 start = edge.pointAt(0.0);
    const MyMath::Vector3 middle = edge.pointAt(0.5);
    const MyMath::Vector3 end = edge.pointAt(1.0);

    // 完整Arc扫掠不超过2π，将Edge分为两个半区间后每段扫掠不超过π，可通过弦长唯一恢复实际角度。
    return chordAngle(start, middle, arc.radius()) + chordAngle(middle, end, arc.radius());
}

// 返回指定Edge显示离散使用的线段数量。
unsigned int segmentCount(const MyVoxel::Topology_Edge& edge, const MyVoxel::Display_CurveBuildOptions& options)
{
    if (edge.geometry().kind() == MyVoxel::CurveKind::Line)
    {
        return 1;
    }

    const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(edge.geometry());
    const double sweepAngle = edgeArcSweepAngle(edge, arc);
    const double scaledCount = sweepAngle * static_cast<double>(options.fullCircleSegmentCount) / TwoPi;
    return (std::max)(1U, static_cast<unsigned int>(std::ceil(scaledCount)));
}

// 将Edge按GL_LINES语义展开为成对端点。
std::vector<MyMath::Vector3> buildLinePoints(const MyVoxel::Topology_Edge& edge, unsigned int count)
{
    std::vector<MyMath::Vector3> points;
    points.reserve(static_cast<std::size_t>(count) * 2);

    for (unsigned int index = 0; index < count; ++index)
    {
        const double firstParameter = static_cast<double>(index) / static_cast<double>(count);
        const double secondParameter = static_cast<double>(index + 1) / static_cast<double>(count);
        points.push_back(edge.pointAt(firstParameter));
        points.push_back(edge.pointAt(secondParameter));
    }

    return points;
}

}

namespace MyVoxel
{

/// 支持判断

bool Display_CurveBuilder::supports(const Topology_Edge& edge)
{
    if (!edge.isValid())
    {
        return false;
    }

    return edge.geometry().kind() == CurveKind::Line || edge.geometry().kind() == CurveKind::Arc;
}

/// 资源创建

Display_ResourceId Display_CurveBuilder::createResource(const Topology_Edge& edge, Display_ResourceManager& resourceManager,
                                                        const Display_Color& color, const Display_CurveBuildOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(supports(edge), "Display_CurveBuilder requires a supported valid Topology_Edge.");
    MYVOXEL_ASSERT_MESSAGE(color.isValid(), "Display_CurveBuilder requires a valid display color.");
    MYVOXEL_ASSERT_MESSAGE(options.isValid(), "Display_CurveBuilder requires valid curve build options.");

    if (!supports(edge) || !color.isValid() || !options.isValid())
    {
        return 0;
    }

    const unsigned int count = segmentCount(edge, options);
    return resourceManager.createLineResource(buildLinePoints(edge, count), color);
}

/// 对象创建

Display_ObjectId Display_CurveBuilder::createObject(const Curve& curve, Display_ResourceId resourceId,
                                                    Display_ObjectManager& objectManager, Display_ObjectUsage usage,
                                                    double lineWidth)
{
    MYVOXEL_ASSERT_MESSAGE(curve.isValid(), "Display_CurveBuilder requires a valid Curve instance.");

    if (!curve.isValid() || resourceId == 0)
    {
        return 0;
    }

    const Display_ObjectId objectId = objectManager.createObject(resourceId, usage, 0);

    if (objectId == 0)
    {
        return 0;
    }

    const Display_Object objectValue = objectManager.object(objectId);

    if (!objectValue.isValid() || objectValue.resourceKind() != Display_ResourceKind::Line ||
        !objectManager.setLocalToWorld(objectId, curve.localToWorld()) ||
        !objectManager.setLineWidth(objectId, lineWidth))
    {
        objectManager.remove(objectId);
        return 0;
    }

    return objectId;
}

}