#ifndef MYVOXEL_MESH_SHAPEMESHER_H
#define MYVOXEL_MESH_SHAPEMESHER_H

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace MyVoxel
{

// 保存连续实体几何转三角网格使用的离散参数。
struct ShapeMeshingOptions
{
    ShapeMeshingOptions();

    unsigned int circularSegmentCount; // 完整旋转圆周使用的离散段数。
    unsigned int sphereStackCount; // 球体从北极到南极使用的纬向离散段数。
    unsigned int profileArcSegmentCount; // 回转轮廓中的完整2π圆弧使用的离散段数，局部圆弧按扫掠角同比例分配。
};

// 将数学定义的连续Shape离散为可直接建立Display_MeshResource的三角网格。
//
// 本类服务Geometry_Shape / Topology_Shape / Shape数学实体，不负责B-Rep Face或Solid离散。
// build(Shape)始终返回Shape局部坐标系中的网格，不应用Shape::localToWorld()；buildWorld(Shape)才显式烘焙实例变换。
// 当前支持Box、Sphere、Cylinder、ConeFrustum、Revolved和Mesh，Custom没有统一离散规则。
class ShapeMesher
{
public:
    /// 支持判断

    // 判断指定连续几何资源是否具有标准网格化实现。
    static bool supports(const Geometry_Shape& geometry);
    // 判断指定局部拓扑Shape是否具有标准网格化实现。
    static bool supports(const Topology_Shape& topology);
    // 判断指定空间Shape是否具有标准网格化实现。
    static bool supports(const Shape& shape);

    /// 局部网格构建

    // 使用统一颜色将指定连续几何资源转换为局部空间三角网格。
    static Mesh build(const Geometry_Shape& geometry, const Display_Color& color, const ShapeMeshingOptions& options = ShapeMeshingOptions());
    // 使用统一颜色将指定局部拓扑Shape转换为局部空间三角网格。
    static Mesh build(const Topology_Shape& topology, const Display_Color& color, const ShapeMeshingOptions& options = ShapeMeshingOptions());
    // 使用统一颜色将指定空间Shape的局部几何转换为局部空间三角网格，不应用Shape实例变换。
    static Mesh build(const Shape& shape, const Display_Color& color, const ShapeMeshingOptions& options = ShapeMeshingOptions());

    /// 世界网格构建

    // 将指定空间Shape转换为已经应用localToWorld变换的世界空间三角网格。
    static Mesh buildWorld(const Shape& shape, const Display_Color& color, const ShapeMeshingOptions& options = ShapeMeshingOptions());

private:
    ShapeMesher() = delete;
};

}

#endif // MYVOXEL_MESH_SHAPEMESHER_H