#include "ModelingBuilder.h"

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Curve/ArcCurve.h"
#include "MyVoxel/Geometry/Curve/LineCurve.h"
#include "MyVoxel/Geometry/Revolved/RevolvedGeometry.h"
#include "MyVoxel/Geometry/ShapeGeometry.h"

namespace MyVoxel
{
namespace Modeling
{

/// 平面曲线创建

Foundation::RefPtr<const Geometry::Curve> ModelingBuilder::line(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint)
{
    return Foundation::RefPtr<const Geometry::Curve>(new Geometry::LineCurve(startPoint, endPoint));
}

Foundation::RefPtr<const Geometry::Curve> ModelingBuilder::arc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle)
{
    return Foundation::RefPtr<const Geometry::Curve>(new Geometry::ArcCurve(center, radius, startAngle, sweepAngle));
}

/// 闭合轮廓创建

Geometry::CurveLoop ModelingBuilder::curveLoop(const std::vector<Foundation::RefPtr<const Geometry::Curve>>& curves, double connectionTolerance)
{
    return Geometry::CurveLoop(curves, connectionTolerance);
}

/// 连续实体创建

Geometry::Shape ModelingBuilder::revolve(const Geometry::CurveLoop& profile)
{
    MYVOXEL_ASSERT_MESSAGE(profile.isValid(), "ModelingBuilder revolve requires a valid CurveLoop.");

    const Foundation::RefPtr<const Geometry::ShapeGeometry> geometry = Foundation::makeRef<Geometry::RevolvedGeometry>(profile);
    return Geometry::Shape(geometry);
}

}
}