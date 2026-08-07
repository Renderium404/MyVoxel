#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_WIRE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_WIRE_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Geometry/Shape/ShapeRelation.h"
#include "Topology_Curve.h"

namespace MyVoxel
{

// 表示由局部拓扑曲线按顺序连接形成的不可变Wire，支持开放曲线链和闭合轮廓。
// 当前版本验证曲线存在和相邻端点连续，不执行自相交检查或端点自动修复。
class Topology_Wire
{
public:
    // 构造不包含曲线的空拓扑Wire。
    Topology_Wire();
    // 使用有序局部拓扑曲线构造Wire，相邻端点距离不得大于connectionTolerance。
    explicit Topology_Wire(const std::vector<Topology_Curve>& curves, double connectionTolerance = MyMath::Vector3::DefaultEpsilon);
    Topology_Wire(const Topology_Wire&) = default;
    Topology_Wire& operator=(const Topology_Wire&) = default;

    /// 状态判断

    // 判断当前Wire是否包含非空、有效且连续的局部拓扑曲线序列。
    bool isValid() const;
    // 判断当前Wire是否未包含任何局部拓扑曲线。
    bool isNull() const;
    // 判断当前Wire是否包含非空、有效且连续的局部拓扑曲线序列。
    explicit operator bool() const;
    // 判断当前Wire首尾端点是否在连接误差内闭合。
    bool isClosed() const;
    // 判断当前闭合Wire是否具有非零有符号面积。
    bool hasArea() const;
    // 判断当前闭合非零面积Wire是否为局部XY平面中的逆时针方向。
    bool isCounterClockwise() const;

    /// 拓扑数据

    // 返回Wire包含的局部拓扑曲线数量。
    std::size_t curveCount() const;
    // 返回指定编号的局部拓扑曲线。
    const Topology_Curve& curve(std::size_t index) const;
    // 返回Wire包含的全部局部拓扑曲线。
    const std::vector<Topology_Curve>& curves() const;
    // 返回相邻端点允许的最大连接距离。
    double connectionTolerance() const;

    /// 局部空间数据

    // 返回Wire起点。
    const MyMath::Vector3& startPoint() const;
    // 返回Wire终点。
    const MyMath::Vector3& endPoint() const;
    // 返回全部曲线局部长度之和。
    double length() const;
    // 返回Wire在局部XY平面中的轴对齐包围盒。
    const Bounds3& bounds() const;
    // 返回闭合Wire的精确有符号面积，逆时针为正，顺时针为负。
    double signedArea() const;

    /// 局部区域查询

    // 判断指定局部XY平面点是否位于闭合Wire围成的区域内部或边界上。
    bool containsPoint(const MyMath::Vector3& point, double tolerance = 0.0) const;
    // 返回指定局部XY平面轴对齐矩形与闭合Wire区域之间的保守空间关系。
    ShapeRelation classifyBounds(const Bounds3& bounds, double tolerance = 0.0) const;

    /// 拓扑创建

    // 返回曲线顺序和每条曲线方向同时反转的新局部拓扑Wire。
    Topology_Wire reversed() const;

private:
    // 验证当前曲线序列并建立Wire缓存。
    void rebuild();

private:
    std::vector<Topology_Curve> m_curves; // 按Wire方向排列的局部拓扑曲线。
    double m_connectionTolerance; // 相邻端点允许的最大连接距离。
    Bounds3 m_bounds; // Wire局部XY平面轴对齐包围盒。
    double m_length; // 全部曲线局部长度之和。
    double m_signedArea; // 当前闭合Wire的精确有符号面积。
    bool m_closed; // 当前Wire首尾端点是否闭合。
    bool m_valid; // 当前Wire是否包含完整有效数据。
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_WIRE_H