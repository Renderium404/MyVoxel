#include <cmath>
#include <cstddef>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Geometry/BaseShape/BoxGeometry.h"
#include "MyVoxel/Geometry/BaseShape/ConeFrustumGeometry.h"
#include "MyVoxel/Geometry/BaseShape/CylinderGeometry.h"
#include "MyVoxel/Geometry/BaseShape/SphereGeometry.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Geometry/ShapeKind.h"
#include "MyVoxel/Modeling/PrimitiveModeling.h"

namespace
{

const double TestEpsilon = 1.0e-12; // 标准建模测试使用的浮点比较误差。

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 判断两个标量是否近似相等。
bool equalValue(double first, double second)
{
    return std::fabs(first - second) <= TestEpsilon;
}

// 判断两个三维数据是否近似相等。
bool equalVector(const MyMath::Vector3& first, const MyMath::Vector3& second)
{
    return first.isEqualTo(second, TestEpsilon);
}

// 测试标准长方体创建。
bool testMakeBox()
{
    const MyVoxel::Geometry::Shape shape = MyVoxel::Modeling::makeBox(4.0, 6.0, 8.0);
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyVoxel::Geometry::BoxGeometry* geometry = dynamic_cast<const MyVoxel::Geometry::BoxGeometry*>(shape.geometryPointer());

    const bool passed =
        shape.isValid() &&
        shape.kind() == MyVoxel::Geometry::ShapeKind::Box &&
        geometry != nullptr &&
        equalValue(geometry->sizeX(), 4.0) &&
        equalValue(geometry->sizeY(), 6.0) &&
        equalValue(geometry->sizeZ(), 8.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-2.0, -3.0, -4.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(2.0, 3.0, 4.0)) &&
        shape.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)) &&
        !shape.containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0));

    return check(passed, "PrimitiveModeling makeBox");
}

// 测试标准球体创建。
bool testMakeSphere()
{
    const MyVoxel::Geometry::Shape shape = MyVoxel::Modeling::makeSphere(5.0);
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyVoxel::Geometry::SphereGeometry* geometry = dynamic_cast<const MyVoxel::Geometry::SphereGeometry*>(shape.geometryPointer());

    const bool passed =
        shape.isValid() &&
        shape.kind() == MyVoxel::Geometry::ShapeKind::Sphere &&
        geometry != nullptr &&
        equalValue(geometry->radius(), 5.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-5.0, -5.0, -5.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(5.0, 5.0, 5.0)) &&
        shape.containsLocalPoint(MyMath::Vector3(3.0, 4.0, 0.0)) &&
        !shape.containsLocalPoint(MyMath::Vector3(5.1, 0.0, 0.0));

    return check(passed, "PrimitiveModeling makeSphere");
}

// 测试标准圆柱体创建。
bool testMakeCylinder()
{
    const MyVoxel::Geometry::Shape shape = MyVoxel::Modeling::makeCylinder(3.0, 10.0);
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyVoxel::Geometry::CylinderGeometry* geometry = dynamic_cast<const MyVoxel::Geometry::CylinderGeometry*>(shape.geometryPointer());

    const bool passed =
        shape.isValid() &&
        shape.kind() == MyVoxel::Geometry::ShapeKind::Cylinder &&
        geometry != nullptr &&
        equalValue(geometry->radius(), 3.0) &&
        equalValue(geometry->height(), 10.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-3.0, -3.0, -5.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(3.0, 3.0, 5.0)) &&
        shape.containsLocalPoint(MyMath::Vector3(3.0, 0.0, 5.0)) &&
        !shape.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 5.1));

    return check(passed, "PrimitiveModeling makeCylinder");
}

// 测试标准圆锥体创建。
bool testMakeCone()
{
    const MyVoxel::Geometry::Shape shape = MyVoxel::Modeling::makeCone(4.0, 10.0);
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyVoxel::Geometry::ConeFrustumGeometry* geometry = dynamic_cast<const MyVoxel::Geometry::ConeFrustumGeometry*>(shape.geometryPointer());

    const bool passed =
        shape.isValid() &&
        shape.kind() == MyVoxel::Geometry::ShapeKind::ConeFrustum &&
        geometry != nullptr &&
        equalValue(geometry->bottomRadius(), 4.0) &&
        equalValue(geometry->topRadius(), 0.0) &&
        equalValue(geometry->height(), 10.0) &&
        equalValue(geometry->radiusAt(-5.0), 4.0) &&
        equalValue(geometry->radiusAt(0.0), 2.0) &&
        equalValue(geometry->radiusAt(5.0), 0.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-4.0, -4.0, -5.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(4.0, 4.0, 5.0)) &&
        shape.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 5.0)) &&
        !shape.containsLocalPoint(MyMath::Vector3(0.1, 0.0, 5.0));

    return check(passed, "PrimitiveModeling makeCone");
}

// 测试标准圆锥台创建。
bool testMakeConeFrustum()
{
    const MyVoxel::Geometry::Shape shape = MyVoxel::Modeling::makeConeFrustum(5.0, 2.0, 12.0);
    const MyVoxel::Bounds3 bounds = shape.localBounds();
    const MyVoxel::Geometry::ConeFrustumGeometry* geometry = dynamic_cast<const MyVoxel::Geometry::ConeFrustumGeometry*>(shape.geometryPointer());

    const bool passed =
        shape.isValid() &&
        shape.kind() == MyVoxel::Geometry::ShapeKind::ConeFrustum &&
        geometry != nullptr &&
        equalValue(geometry->bottomRadius(), 5.0) &&
        equalValue(geometry->topRadius(), 2.0) &&
        equalValue(geometry->height(), 12.0) &&
        equalValue(geometry->radiusAt(-6.0), 5.0) &&
        equalValue(geometry->radiusAt(0.0), 3.5) &&
        equalValue(geometry->radiusAt(6.0), 2.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-5.0, -5.0, -6.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(5.0, 5.0, 6.0));

    return check(passed, "PrimitiveModeling makeConeFrustum");
}

// 测试建模结果复制时是否共享几何数据。
bool testShapeSharing()
{
    const MyVoxel::Geometry::Shape first = MyVoxel::Modeling::makeSphere(6.0);
    const MyVoxel::Geometry::Shape second = first;

    const bool passed =
        first.isValid() &&
        second.isValid() &&
        first.sharesGeometryWith(second) &&
        first.geometryPointer() == second.geometryPointer() &&
        first.geometry().referenceCount() == 2;

    return check(passed, "PrimitiveModeling Shape sharing");
}

// 测试多个建模调用是否创建独立几何数据。
bool testIndependentShapes()
{
    const MyVoxel::Geometry::Shape first = MyVoxel::Modeling::makeBox(2.0, 4.0, 6.0);
    const MyVoxel::Geometry::Shape second = MyVoxel::Modeling::makeBox(2.0, 4.0, 6.0);

    const bool passed =
        first.isValid() &&
        second.isValid() &&
        !first.sharesGeometryWith(second) &&
        first.geometryPointer() != second.geometryPointer() &&
        first.localBounds().isEqualTo(second.localBounds(), TestEpsilon);

    return check(passed, "PrimitiveModeling independent Shapes");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testMakeBox(),
        testMakeSphere(),
        testMakeCylinder(),
        testMakeCone(),
        testMakeConeFrustum(),
        testShapeSharing(),
        testIndependentShapes()
    };

    const std::size_t testCount = sizeof(results) / sizeof(results[0]);

    for (std::size_t testIndex = 0; testIndex < testCount; ++testIndex)
    {
        if (results[testIndex])
        {
            ++passedCount;
        }
        else
        {
            ++failedCount;
        }
    }

    std::cout << std::endl;
    std::cout << "Primitive modeling tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}