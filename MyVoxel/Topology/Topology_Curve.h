#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_CURVE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_CURVE_H

#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"

namespace MyVoxel
{

// 表示持有不可变曲线几何资源的局部拓扑曲线值对象，不包含任何空间放置变换。
class Topology_Curve
{
public:
    // 构造不持有几何资源的空拓扑曲线。
    Topology_Curve();
    // 使用不可变曲线几何资源构造局部拓扑曲线。
    explicit Topology_Curve(const Foundation::RefPtr<const Geometry_Curve>& geometry);
    Topology_Curve(const Topology_Curve&) = default;
    Topology_Curve& operator=(const Topology_Curve&) = default;

    /// 状态判断

    // 判断当前拓扑曲线是否持有有效几何资源。
    bool isValid() const;
    // 判断当前拓扑曲线是否未持有几何资源。
    bool isNull() const;
    // 判断当前拓扑曲线是否持有有效几何资源。
    explicit operator bool() const;
    // 判断当前拓扑曲线是否与另一条拓扑曲线共享同一份几何资源。
    bool sharesGeometryWith(const Topology_Curve& other) const;

    /// 几何资源

    // 返回当前拓扑曲线持有的不可变几何资源，空对象调用属于调用错误。
    const Geometry_Curve& geometry() const;
    // 返回当前拓扑曲线持有的不可变几何资源普通指针，空对象返回空指针。
    const Geometry_Curve* geometryPointer() const;
    // 返回当前拓扑曲线持有的不可变几何资源引用计数指针。
    const Foundation::RefPtr<const Geometry_Curve>& geometryResource() const;
    // 返回当前拓扑曲线的标准类型，空对象调用属于调用错误。
    CurveKind kind() const;

    /// 局部空间数据与查询

    // 返回曲线起点。
    const MyMath::Vector3& startPoint() const;
    // 返回曲线终点。
    const MyMath::Vector3& endPoint() const;
    // 返回曲线完整长度。
    double length() const;
    // 返回曲线在局部XY平面中的轴对齐包围盒。
    const Bounds3& bounds() const;
    // 返回规范化参数t对应的曲线点，t必须位于[0,1]。
    MyMath::Vector3 pointAt(double t) const;
    // 返回规范化参数t对应且沿曲线前进方向的单位切向量，t必须位于[0,1]。
    MyMath::Vector3 tangentAt(double t) const;

    /// 拓扑创建

    // 返回几何轨迹相同且方向相反的新拓扑曲线，空对象返回空拓扑曲线。
    Topology_Curve reversed() const;

private:
    Foundation::RefPtr<const Geometry_Curve> m_geometry; // 当前拓扑曲线共享的不可变几何资源。
};

// 表示局部拓扑曲线的有序列表。
typedef std::vector<Topology_Curve> Topology_CurveList;

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_CURVE_H