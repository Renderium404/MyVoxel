#ifndef MYVOXEL_MODELING_SHAPE_TOPOLOGY_SHAPEBUILDER_H
#define MYVOXEL_MODELING_SHAPE_TOPOLOGY_SHAPEBUILDER_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Topology/Topology_Shape.h"

namespace MyVoxel
{
namespace Modeling
{

// 将标准实体几何资源转换为包含边界Face的完整局部拓扑Shape。
//
// 当前只实现Geometry_Box；后续Cylinder、ConeFrustum、Sphere和Revolved均在此扩展几何到边界拓扑的构造，
// FaceMesher和ShapeMesher不直接识别ShapeKind。
class Topology_ShapeBuilder
{
public:
    /// 支持判断

    // 判断指定实体几何是否具有当前标准边界拓扑构造实现。
    static bool supports(const Geometry_Shape& geometry);

    /// 拓扑构建

    // 根据不可变实体几何资源建立完整局部拓扑Shape。
    static Topology_Shape build(const Foundation::RefPtr<const Geometry_Shape>& geometry);

private:
    Topology_ShapeBuilder() = delete;
};

}
}

#endif // MYVOXEL_MODELING_SHAPE_TOPOLOGY_SHAPEBUILDER_H