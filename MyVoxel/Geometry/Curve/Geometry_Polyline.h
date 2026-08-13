#ifndef MYVOXEL_GEOMETRY_CURVE_GEOMETRY_POLYLINE_H
#define MYVOXEL_GEOMETRY_CURVE_GEOMETRY_POLYLINE_H

#include <cstddef>
#include <vector>

#include "Geometry_Curve.h"

namespace MyVoxel
{

// 表示由连续非退化直线段组成的有限有向三维折线，规范参数按累计弧长分配。
class Geometry_Polyline : public Geometry_Curve
{
public:
    // 使用至少两个有限三维点创建折线，相邻点必须不同；允许首尾点相同以表达闭合折线。
    explicit Geometry_Polyline(const std::vector<MyMath::Vector3>& points);

    /// 折线数据

    // 返回折线控制点数量。
    std::size_t pointCount() const;
    // 返回折线线段数量。
    std::size_t segmentCount() const;
    // 返回指定控制点。
    const MyMath::Vector3& point(std::size_t index) const;
    // 返回完整控制点序列。
    const std::vector<MyMath::Vector3>& points() const;
    // 返回指定线段长度。
    double segmentLength(std::size_t index) const;
    // 判断折线首尾控制点是否精确重合。
    bool isClosed() const;

    /// 曲线属性

    // 返回折线类型。
    CurveKind kind() const override;
    // 返回第一个控制点。
    const MyMath::Vector3& startPoint() const override;
    // 返回最后一个控制点。
    const MyMath::Vector3& endPoint() const override;
    // 返回所有线段长度之和。
    double length() const override;
    // 返回所有控制点形成的三维轴对齐包围盒。
    const Bounds3& bounds() const override;

    /// 参数查询

    // 按累计弧长返回规范化参数t对应的折线点，t必须位于[0,1]。
    MyMath::Vector3 pointAt(double t) const override;
    // 返回规范化参数t所在有向线段的单位切向量；内部折点精确命中时统一取后继线段方向，t=1取末线段方向。
    MyMath::Vector3 tangentAt(double t) const override;

    /// 曲线创建

    // 返回控制点顺序完全反转后的新折线几何。
    Foundation::RefPtr<const Geometry_Curve> reversed() const override;

protected:
    // 通过Geometry_Object侵入式引用计数管理折线几何生命周期。
    ~Geometry_Polyline() override = default;

private:
    // 返回累计弧长distance所在的标准方向线段编号，内部折点归属于后继线段。
    std::size_t segmentIndexAtDistance(double distance) const;
    // 校验控制点并建立累计弧长、线段方向和包围盒缓存。
    void rebuild();

private:
    std::vector<MyMath::Vector3> m_points; // 按曲线前进方向保存的控制点。
    std::vector<double> m_cumulativeLengths; // 各控制点对应的累计弧长，第一个固定为零。
    std::vector<MyMath::Vector3> m_segmentDirections; // 各非退化线段的单位前进方向。
    double m_length; // 折线所有线段的完整累计长度。
    Bounds3 m_bounds; // 折线全部控制点形成的三维轴对齐包围盒。
    bool m_closed; // 首尾控制点是否精确重合。
};

}

#endif // MYVOXEL_GEOMETRY_CURVE_GEOMETRY_POLYLINE_H