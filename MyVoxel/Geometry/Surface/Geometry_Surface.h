#ifndef MYVOXEL_GEOMETRY_SURFACE_GEOMETRY_SURFACE_H
#define MYVOXEL_GEOMETRY_SURFACE_GEOMETRY_SURFACE_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
#include "SurfaceKind.h"

namespace MyVoxel
{

// 定义不可变连续参数曲面，参数点使用局部XY平面坐标(x,y,0)表示(u,v)。
class Geometry_Surface : public Foundation::ReferenceCounted
{
public:
    /// 曲面属性

    // 返回当前曲面的标准类型。
    virtual SurfaceKind kind() const = 0;

    /// 参数查询

    // 返回参数(u,v)对应的局部三维曲面点。
    virtual MyMath::Vector3 pointAt(double u, double v) const = 0;
    // 返回参数(u,v)对应且与曲面正向一致的单位法线。
    virtual MyMath::Vector3 normalAt(double u, double v) const = 0;
    // 返回参数(u,v)处关于u的一阶偏导向量。
    virtual MyMath::Vector3 derivativeUAt(double u, double v) const = 0;
    // 返回参数(u,v)处关于v的一阶偏导向量。
    virtual MyMath::Vector3 derivativeVAt(double u, double v) const = 0;

    /// 范围查询

    // 返回指定局部UV轴对齐参数范围映射到曲面后的保守局部三维包围盒。
    virtual Bounds3 localBounds(const Bounds3& parameterBounds) const = 0;

protected:
    // 通过侵入式引用计数管理曲面几何生命周期。
    ~Geometry_Surface() override = default;
};

}

#endif // MYVOXEL_GEOMETRY_SURFACE_GEOMETRY_SURFACE_H