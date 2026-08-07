#include "RevolvedModeling.h"

#include <vector>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Construction/Geometry_Revolved.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"

namespace MyVoxel
{
namespace Modeling
{

/// 局部Topology_Shape创建

Topology_Shape createRevolved(const Topology_Wire& profile)
{
    MYVOXEL_ASSERT_MESSAGE(profile.isValid(), "Revolved modeling requires a valid Topology_Wire.");
    MYVOXEL_ASSERT_MESSAGE(profile.isClosed(), "Revolved modeling requires a closed Topology_Wire.");
    MYVOXEL_ASSERT_MESSAGE(profile.hasArea(), "Revolved modeling requires a non-zero-area Topology_Wire.");

    if (!profile.isValid() || !profile.isClosed() || !profile.hasArea())
    {
        return Topology_Shape();
    }

    std::vector<Foundation::RefPtr<const Geometry_Curve> > profileCurves;
    profileCurves.reserve(profile.curveCount());

    for (std::size_t index = 0; index < profile.curveCount(); ++index)
    {
        profileCurves.push_back(profile.curve(index).geometryResource());
    }

    const Foundation::RefPtr<Geometry_Revolved> geometry =
        Foundation::makeRef<Geometry_Revolved>(profileCurves, profile.connectionTolerance());
    const Foundation::RefPtr<const Geometry_Shape> shapeGeometry = geometry;
    return Topology_Shape(shapeGeometry);
}

/// 空间Shape实例创建

Shape makeRevolved(const Topology_Wire& profile)
{
    return Shape(createRevolved(profile));
}

Shape makeRevolved(const Topology_Wire& profile, const MyMath::Matrix4& localToWorld)
{
    return Shape(createRevolved(profile), localToWorld);
}

}
}
