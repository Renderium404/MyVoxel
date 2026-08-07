#ifndef MYVOXEL_MESH_FACEMESHER_H
#define MYVOXEL_MESH_FACEMESHER_H

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Topology/Face.h"
#include "MyVoxel/Topology/Topology_Face.h"

namespace MyVoxel
{

// 保存Topology_Face转三角网格使用的离散和二维三角化参数。
struct FaceMeshingOptions
{
    FaceMeshingOptions();

    unsigned int curveSegmentCount; // 非直线参数边界曲线默认离散段数，当前主要用于圆弧。
    double triangulationTolerance; // UV平面重复点、共线点和三角形判定使用的绝对误差。
};

// 将裁剪参数曲面Face转换为局部三角网格。
//
// 当前稳定版本只支持无孔Plane Face；外边界支持由Line和Arc组成的Topology_Wire。
// 有内环Face明确返回不支持，后续孔洞三角化在该接口上继续扩展。
// build(Face)返回Face局部网格，不应用实例变换；buildWorld(Face)显式烘焙Face::localToWorld()。
class FaceMesher
{
public:
    /// 支持判断

    // 判断指定局部拓扑Face是否具有当前标准网格化实现。
    static bool supports(const Topology_Face& face);
    // 判断指定空间Face是否具有当前标准网格化实现。
    static bool supports(const Face& face);

    /// 局部网格构建
    // 使用统一显示颜色将指定局部拓扑Face转换为局部空间三角网格。
    static Mesh build(const Topology_Face& face, const Display_Color& color, const FaceMeshingOptions& options = FaceMeshingOptions());
    // 使用统一显示颜色将指定空间Face的局部拓扑转换为局部空间三角网格，不应用Face实例变换。
    static Mesh build(const Face& face, const Display_Color& color, const FaceMeshingOptions& options = FaceMeshingOptions());

    /// 世界网格构建
    // 将指定空间Face转换为已经应用localToWorld变换的世界空间三角网格。
    static Mesh buildWorld(const Face& face, const Display_Color& color, const FaceMeshingOptions& options = FaceMeshingOptions());

private:
    FaceMesher() = delete;
};

}

#endif // MYVOXEL_MESH_FACEMESHER_H