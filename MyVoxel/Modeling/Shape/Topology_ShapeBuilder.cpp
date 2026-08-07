#include "Topology_ShapeBuilder.h"

#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Surface/Geometry_Plane.h"
#include "MyVoxel/Topology/Topology_Curve.h"
#include "MyVoxel/Topology/Topology_Face.h"
#include "MyVoxel/Topology/Topology_Wire.h"

namespace
{

const double HalfScale = 0.5; // 标准Box完整尺寸转换为Face参数半尺寸使用的固定比例。
const double BoxWireConnectionTolerance = 0.0; // Box参数边界端点由相同数值直接构造，因此要求精确连接。

MyVoxel::Topology_Curve makeParameterLine(double startU, double startV, double endU, double endV)
{
    return MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(startU, startV, 0.0), MyMath::Vector3(endU, endV, 0.0))));
}

MyVoxel::Topology_Wire makeRectangleWire(double halfU, double halfV)
{
    std::vector<MyVoxel::Topology_Curve> curves;
    curves.reserve(4); // 一个矩形参数外环固定由四条有向直线段组成。
    curves.push_back(makeParameterLine(-halfU, -halfV, halfU, -halfV));
    curves.push_back(makeParameterLine(halfU, -halfV, halfU, halfV));
    curves.push_back(makeParameterLine(halfU, halfV, -halfU, halfV));
    curves.push_back(makeParameterLine(-halfU, halfV, -halfU, -halfV));
    return MyVoxel::Topology_Wire(curves, BoxWireConnectionTolerance);
}

MyVoxel::Topology_Face makePlaneFace(const MyMath::Vector3& origin, const MyMath::Vector3& axisU,
                                     const MyMath::Vector3& axisV, double halfU, double halfV)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Surface> surface(
        new MyVoxel::Geometry_Plane(origin, axisU, axisV));
    return MyVoxel::Topology_Face(surface, makeRectangleWire(halfU, halfV));
}

MyVoxel::Topology_Shape buildBox(const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Shape>& geometry)
{
    const MyVoxel::Geometry_Box& box = static_cast<const MyVoxel::Geometry_Box&>(*geometry);
    const double halfX = box.sizeX() * HalfScale;
    const double halfY = box.sizeY() * HalfScale;
    const double halfZ = box.sizeZ() * HalfScale;
    std::vector<MyVoxel::Topology_Face> faces;
    faces.reserve(6); // 标准封闭Box固定由六个矩形平面Face组成。

    // 每个Plane的axisU×axisV直接指向Box外侧，因此Topology_Face正向法线即实体外法线。
    faces.push_back(makePlaneFace(MyMath::Vector3(-halfX, 0.0, 0.0), MyMath::Vector3(0.0, -1.0, 0.0),
                                  MyMath::Vector3(0.0, 0.0, 1.0), halfY, halfZ));
    faces.push_back(makePlaneFace(MyMath::Vector3(halfX, 0.0, 0.0), MyMath::Vector3(0.0, 1.0, 0.0),
                                  MyMath::Vector3(0.0, 0.0, 1.0), halfY, halfZ));
    faces.push_back(makePlaneFace(MyMath::Vector3(0.0, -halfY, 0.0), MyMath::Vector3(1.0, 0.0, 0.0),
                                  MyMath::Vector3(0.0, 0.0, 1.0), halfX, halfZ));
    faces.push_back(makePlaneFace(MyMath::Vector3(0.0, halfY, 0.0), MyMath::Vector3(-1.0, 0.0, 0.0),
                                  MyMath::Vector3(0.0, 0.0, 1.0), halfX, halfZ));
    faces.push_back(makePlaneFace(MyMath::Vector3(0.0, 0.0, -halfZ), MyMath::Vector3(-1.0, 0.0, 0.0),
                                  MyMath::Vector3(0.0, 1.0, 0.0), halfX, halfY));
    faces.push_back(makePlaneFace(MyMath::Vector3(0.0, 0.0, halfZ), MyMath::Vector3(1.0, 0.0, 0.0),
                                  MyMath::Vector3(0.0, 1.0, 0.0), halfX, halfY));

    return MyVoxel::Topology_Shape(geometry, faces);
}

}

namespace MyVoxel
{
namespace Modeling
{

/// 支持判断

bool Topology_ShapeBuilder::supports(const Geometry_Shape& geometry)
{
    return geometry.kind() == ShapeKind::Box;
}

/// 拓扑构建

Topology_Shape Topology_ShapeBuilder::build(const Foundation::RefPtr<const Geometry_Shape>& geometry)
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_ShapeBuilder geometry must not be null.");
    MYVOXEL_ASSERT_MESSAGE(geometry && supports(*geometry), "Topology_ShapeBuilder does not support the specified Geometry_Shape.");

    if (!geometry || !supports(*geometry))
    {
        return Topology_Shape();
    }

    switch (geometry->kind())
    {
    case ShapeKind::Box:
        return buildBox(geometry);

    case ShapeKind::Sphere:
    case ShapeKind::Cylinder:
    case ShapeKind::ConeFrustum:
    case ShapeKind::Revolved:
    case ShapeKind::Mesh:
    case ShapeKind::Custom:
        break;
    }

    return Topology_Shape();
}

}
}