#ifndef MYVOXEL_GEOMETRY_CURVE_GEOMETRY_CURVE_H
#define MYVOXEL_GEOMETRY_CURVE_GEOMETRY_CURVE_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "CurveKind.h"

namespace MyVoxel
{

// 定义局部XY平面中有限、有向且不可变的曲线几何，所有曲线点的Z分量均为零。
class Geometry_Curve : public Foundation::ReferenceCounted
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

    // 返回曲线在局部XY平面中的轴对齐包围盒。
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
    // 通过侵入式引用计数管理曲线几何生命周期。
    ~Geometry_Curve() override = default;
};


}

#endif // MYVOXEL_GEOMETRY_CURVE_GEOMETRY_CURVE_H
