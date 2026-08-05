#ifndef MYVOXEL_MESHING_SHAPEMESHER_H
#define MYVOXEL_MESHING_SHAPEMESHER_H

#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Geometry/Shape.h"

namespace MyVoxel
{
namespace Meshing
{

// 保存标准连续几何体的三角网格离散参数。
struct ShapeMeshingOptions
{
    ShapeMeshingOptions();

    unsigned int circularSegmentCount; // 圆柱、圆锥、圆锥台和球体经向离散段数。
    unsigned int sphereStackCount; // 球体从北极到南极的纬向离散段数。
};

// 将标准连续Geometry::Shape转换为局部空间三角网格。
//
// 当前支持Box、Sphere、Cylinder和ConeFrustum。
// ShapeKind::Custom不具有统一解析参数，不在当前类中执行通用网格化。
class ShapeMesher
{
public:
    // 判断指定Shape是否具有当前支持的标准网格化实现。
    static bool supports(const Geometry::Shape& shape);

    // 使用统一颜色将指定Shape转换为局部空间三角网格。
    static Geometry::Mesh build(const Geometry::Shape& shape,
                                const Geometry::MeshColor& color,
                                const ShapeMeshingOptions& options = ShapeMeshingOptions());

private:
    ShapeMesher() = delete;
};

}
}

#endif // MYVOXEL_MESHING_SHAPEMESHER_H