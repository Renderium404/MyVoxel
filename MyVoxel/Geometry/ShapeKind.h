#ifndef MYVOXEL_GEOMETRY_SHAPEKIND_H
#define MYVOXEL_GEOMETRY_SHAPEKIND_H

namespace MyVoxel
{
namespace Geometry
{

// 标识连续几何体的标准类型。
enum class ShapeKind
{
    Custom,        // 自定义连续几何体。
    Box,           // 长方体。
    Sphere,        // 球体。
    Cylinder,      // 圆柱体。
    ConeFrustum,   // 圆台体。
    Mesh           // 封闭三角网格几何体。
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPEKIND_H
