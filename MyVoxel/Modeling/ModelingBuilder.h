#ifndef MYVOXEL_MODELING_MODELINGBUILDER_H
#define MYVOXEL_MODELING_MODELINGBUILDER_H

#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Curve.h"
#include "MyVoxel/Geometry/Curve/CurveLoop.h"
#include "MyVoxel/Geometry/Shape.h"

namespace MyVoxel
{
namespace Modeling
{

// 提供平面曲线、闭合轮廓和连续实体的统一创建入口。
class ModelingBuilder
{
public:
    /// 平面曲线创建

    // 创建局部XY平面中的有限有向直线段。
    static Foundation::RefPtr<const Geometry::Curve> line(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint);

    // 使用圆心、半径、起始角和带符号扫掠角创建局部XY平面圆弧，角度单位为弧度。
    static Foundation::RefPtr<const Geometry::Curve> arc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle);

    /// 闭合轮廓创建

    // 使用有序曲线创建闭合轮廓，相邻端点距离不得大于connectionTolerance。
    static Geometry::CurveLoop curveLoop(const std::vector<Foundation::RefPtr<const Geometry::Curve>>& curves, double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

    /// 连续实体创建

    // 将局部XY平面闭合轮廓映射到XOZ母线平面并绕局部Z轴完整旋转为连续Shape。
    static Geometry::Shape revolve(const Geometry::CurveLoop& profile);

private:
    ModelingBuilder() = delete;
};

}
}

#endif // MYVOXEL_MODELING_MODELINGBUILDER_H