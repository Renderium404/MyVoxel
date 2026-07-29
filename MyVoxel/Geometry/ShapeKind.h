#ifndef MYVOXEL_GEOMETRY_SHAPEKIND_H
#define MYVOXEL_GEOMETRY_SHAPEKIND_H

namespace MyVoxel
{
namespace Geometry
{

// 标识连续几何体的标准类型。
enum class ShapeKind
{
    Custom,             //
    Box,                //长方体
    Sphere,             //球
    Cylinder,           //圆柱
    ConeFrustum         //
};

}
}

#endif // MYVOXEL_GEOMETRY_SHAPEKIND_H