#ifndef MYVOXEL_GEOMETRY_CURVE_GEOMETRY_CURVE_H
#define MYVOXEL_GEOMETRY_CURVE_GEOMETRY_CURVE_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Geometry_Object.h"
#include "CurveKind.h"

namespace MyVoxel
{

// 定义三维几何空间中有限、有向且不可变的曲线几何，统一使用[0,1]规范化参数。
class Geometry_Curve : public Geometry_Object
{
public:
    /// 曲线属性

    // 返回当前曲线的标准类型。
    virtual CurveKind kind() const = 0;
    // 返回曲线起点。
    virtual const MyMath::Vector3& startPoint() const = 0;
    // 返回曲线终点。
    virtual const MyMath::Vector3& endPoint() const = 0;
    // 返回曲线完整长度。
    virtual double length() const = 0;
    // 返回曲线在自身局部几何空间中的三维轴对齐包围盒。
    virtual const Bounds3& bounds() const = 0;

    /// 参数查询

    // 返回规范化参数t对应的曲线点，t必须位于[0,1]。
    virtual MyMath::Vector3 pointAt(double t) const = 0;
    // 返回规范化参数t对应且沿曲线前进方向的单位切向量，t必须位于[0,1]。
    virtual MyMath::Vector3 tangentAt(double t) const = 0;

    /// 曲线创建

    // 返回几何轨迹相同且方向相反的新曲线几何。
    virtual Foundation::RefPtr<const Geometry_Curve> reversed() const = 0;

protected:
    // 通过Geometry_Object侵入式引用计数管理具体曲线几何生命周期。
    ~Geometry_Curve() override = default;
};

}

#endif // MYVOXEL_GEOMETRY_CURVE_GEOMETRY_CURVE_H
