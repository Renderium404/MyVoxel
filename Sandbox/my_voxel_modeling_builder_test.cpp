#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "MyVoxel/Geometry/Curve/ArcCurve.h"
#include "MyVoxel/Geometry/Curve/CurveKind.h"
#include "MyVoxel/Geometry/ShapeKind.h"
#include "MyVoxel/Modeling/ModelingBuilder.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，测试角度统一使用弧度制。
const double TestTolerance = 1.0e-10; // 建模构建器测试使用的统一数值误差。
int g_passed = 0;
int g_failed = 0;

void check(bool condition, const std::string& name)
{
    if (condition)
    {
        ++g_passed;
        std::cout << "[通过] " << name << std::endl;
    }
    else
    {
        ++g_failed;
        std::cout << "[失败] " << name << std::endl;
    }
}

bool nearlyEqual(double first, double second)
{
    return std::fabs(first - second) <= TestTolerance;
}

}

int main()
{
    using MyVoxel::Foundation::RefPtr;
    using MyVoxel::Geometry::Curve;
    using MyVoxel::Geometry::CurveKind;
    using MyVoxel::Geometry::CurveLoop;
    using MyVoxel::Geometry::Shape;
    using MyVoxel::Geometry::ShapeKind;
    using MyVoxel::Modeling::ModelingBuilder;

    std::cout << "开始通用建模构建器测试" << std::endl << std::endl;

    const RefPtr<const Curve> line = ModelingBuilder::line(MyMath::Vector3(0.0, -2.0, 0.0), MyMath::Vector3(3.0, -2.0, 0.0));
    check(line && line->kind() == CurveKind::Line, "构建直线");
    check(nearlyEqual(line->length(), 3.0), "直线参数保持");

    const RefPtr<const Curve> arc = ModelingBuilder::arc(MyMath::Vector3(3.0, 0.0, 0.0), 2.0, -Pi * 0.5, Pi);
    check(arc && arc->kind() == CurveKind::Arc, "构建圆弧");
    check(nearlyEqual(arc->length(), Pi * 2.0), "圆弧参数保持");

    std::vector<RefPtr<const Curve>> curves;
    curves.push_back(ModelingBuilder::line(MyMath::Vector3(0.0, -2.0, 0.0), MyMath::Vector3(3.0, -2.0, 0.0)));
    curves.push_back(ModelingBuilder::line(MyMath::Vector3(3.0, -2.0, 0.0), MyMath::Vector3(3.0, 2.0, 0.0)));
    curves.push_back(ModelingBuilder::line(MyMath::Vector3(3.0, 2.0, 0.0), MyMath::Vector3(0.0, 2.0, 0.0)));
    curves.push_back(ModelingBuilder::line(MyMath::Vector3(0.0, 2.0, 0.0), MyMath::Vector3(0.0, -2.0, 0.0)));

    const CurveLoop profile = ModelingBuilder::curveLoop(curves, TestTolerance);
    check(profile.isValid(), "构建闭合轮廓");
    check(profile.curveCount() == 4, "闭合轮廓曲线数量");
    check(nearlyEqual(profile.signedArea(), 12.0), "闭合轮廓面积");

    const Shape shape = ModelingBuilder::revolve(profile);
    check(shape.isValid(), "构建完整回转Shape");
    check(shape.kind() == ShapeKind::Revolved, "回转Shape类型");
    check(shape.containsLocalPoint(MyMath::Vector3(2.0, 0.0, 0.0)), "回转Shape内部点");
    check(shape.containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)), "回转Shape边界点");
    check(!shape.containsLocalPoint(MyMath::Vector3(3.1, 0.0, 0.0)), "回转Shape外部点");

    std::cout << std::endl << "Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}