#ifndef MYVOXEL_GEOMETRY_SURFACE_SURFACE_KIND_H
#define MYVOXEL_GEOMETRY_SURFACE_SURFACE_KIND_H

namespace MyVoxel
{

// 表示连续曲面的标准几何类型。
enum class SurfaceKind
{
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Revolved,
    Custom
};

}

#endif // MYVOXEL_GEOMETRY_SURFACE_SURFACE_KIND_H