#ifndef MYVOXEL_GEOMETRY_CURVE_CURVELOOP_H
#define MYVOXEL_GEOMETRY_CURVE_CURVELOOP_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/ShapeRelation.h"
#include "Curve.h"

namespace MyVoxel
{
namespace Geometry
{

// 表示由局部XY平面有限曲线首尾连接形成的不可变闭合轮廓。
// 当前版本验证曲线有效、顺序连续、首尾闭合和非零面积，不执行自相交检查或端点自动修复。
class CurveLoop
{
public:
    // 构造不包含曲线的无效闭合轮廓。
    CurveLoop();

    // 使用有序曲线创建闭合轮廓，相邻端点距离不得大于connectionTolerance。
    explicit CurveLoop(const std::vector<Foundation::RefPtr<const Geo_Curve>>& curves, double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

    /// 状态判断

    // 判断当前轮廓是否包含连续、闭合且具有非零面积的曲线序列。
    bool isValid() const;

    // 判断当前轮廓方向是否为局部XY平面中的逆时针方向。
    bool isCounterClockwise() const;

    /// 轮廓数据

    // 返回轮廓包含的曲线数量。
    std::size_t curveCount() const;

    // 返回指定编号的不可变曲线。
    const Curve& curve(std::size_t index) const;

    // 返回轮廓全部曲线引用。
    const std::vector<Foundation::RefPtr<const Curve>>& curves() const;

    // 返回轮廓端点连接误差。
    double connectionTolerance() const;

    // 返回轮廓局部XY平面轴对齐包围盒。
    const Bounds3& bounds() const;

    // 返回轮廓精确有符号面积，逆时针为正，顺时针为负。
    double signedArea() const;

    /// 空间查询

    // 判断指定局部XY平面点是否位于轮廓内部或边界上。
    bool containsPoint(const MyMath::Vector3& point, double tolerance = 0.0) const;

    // 返回指定局部XY平面轴对齐矩形与轮廓区域之间的保守空间关系。
    ShapeRelation classifyBounds(const Bounds3& bounds, double tolerance = 0.0) const;

    /// 轮廓创建

    // 返回曲线顺序和每条曲线方向同时反转的新轮廓。
    CurveLoop reversed() const;

private:
    // 根据当前曲线序列验证并建立轮廓缓存。
    void rebuild();

private:
    std::vector<Foundation::RefPtr<const Curve>> m_curves; // 按轮廓方向排列的不可变曲线。
    double m_connectionTolerance; // 相邻端点允许的最大连接距离。
    Bounds3 m_bounds; // 轮廓局部XY平面轴对齐包围盒。
    double m_signedArea; // 轮廓精确有符号面积。
    bool m_valid; // 当前轮廓是否包含完整有效数据。
};

}
}

#endif // MYVOXEL_GEOMETRY_CURVE_CURVELOOP_H
