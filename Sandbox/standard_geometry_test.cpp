#include <cmath>
#include <cstddef>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/BaseShape/BoxGeometry.h"
#include "MyVoxel/Geometry/BaseShape/ConeFrustumGeometry.h"
#include "MyVoxel/Geometry/BaseShape/CylinderGeometry.h"
#include "MyVoxel/Geometry/BaseShape/SphereGeometry.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Geometry/ShapeKind.h"
#include "MyVoxel/Geometry/ShapeRelation.h"


namespace
{

const double TestEpsilon = 1.0e-12; // 标准几何体测试使用的浮点比较误差。

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

// 测试标准长方体参数和包围盒。
bool testBoxProperties()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::BoxGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::BoxGeometry>(4.0, 6.0, 8.0);

    const MyVoxel::Bounds3 bounds = geometry->localBounds();

    const bool passed =
        geometry->kind() == MyVoxel::Geometry::ShapeKind::Box &&
        equalValue(geometry->sizeX(), 4.0) &&
        equalValue(geometry->sizeY(), 6.0) &&
        equalValue(geometry->sizeZ(), 8.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-2.0, -3.0, -4.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(2.0, 3.0, 4.0));

    return check(passed, "BoxGeometry properties");
}

// 测试标准长方体点包含。
bool testBoxContainment()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::BoxGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::BoxGeometry>(4.0, 6.0, 8.0);

    const bool passed =
        geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)) &&
        geometry->containsLocalPoint(MyMath::Vector3(2.0, 3.0, 4.0)) &&
        !geometry->containsLocalPoint(MyMath::Vector3(2.1, 0.0, 0.0));

    return check(passed, "BoxGeometry point containment");
}

// 测试标准长方体区域分类。
bool testBoxClassification()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::BoxGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::BoxGeometry>(4.0, 6.0, 8.0);

    const MyVoxel::Bounds3 inside(MyMath::Vector3(-1.0, -2.0, -3.0), MyMath::Vector3(1.0, 2.0, 3.0));
    const MyVoxel::Bounds3 outside(MyMath::Vector3(3.0, 0.0, 0.0), MyMath::Vector3(4.0, 1.0, 1.0));
    const MyVoxel::Bounds3 intersecting(MyMath::Vector3(1.0, 2.0, 3.0), MyMath::Vector3(3.0, 4.0, 5.0));
    const MyVoxel::Bounds3 touching(MyMath::Vector3(2.0, -1.0, -1.0), MyMath::Vector3(3.0, 1.0, 1.0));

    const bool passed =
        geometry->classifyLocalBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside &&
        geometry->classifyLocalBounds(outside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        geometry->classifyLocalBounds(intersecting) == MyVoxel::Geometry::ShapeRelation::Intersecting &&
        geometry->classifyLocalBounds(touching) == MyVoxel::Geometry::ShapeRelation::Intersecting;

    return check(passed, "BoxGeometry classification");
}

// 测试标准球体参数和包围盒。
bool testSphereProperties()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::SphereGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::SphereGeometry>(3.0);

    const MyVoxel::Bounds3 bounds = geometry->localBounds();

    const bool passed =
        geometry->kind() == MyVoxel::Geometry::ShapeKind::Sphere &&
        equalValue(geometry->radius(), 3.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-3.0, -3.0, -3.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(3.0, 3.0, 3.0));

    return check(passed, "SphereGeometry properties");
}

// 测试标准球体点包含。
bool testSphereContainment()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::SphereGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::SphereGeometry>(3.0);

    const bool passed =
        geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)) &&
        geometry->containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)) &&
        geometry->containsLocalPoint(MyMath::Vector3(1.0, 2.0, 2.0)) &&
        !geometry->containsLocalPoint(MyMath::Vector3(3.1, 0.0, 0.0));

    return check(passed, "SphereGeometry point containment");
}

// 测试标准球体区域分类。
bool testSphereClassification()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::SphereGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::SphereGeometry>(3.0);

    const MyVoxel::Bounds3 inside(MyMath::Vector3(-1.0, -1.0, -1.0), MyMath::Vector3(1.0, 1.0, 1.0));
    const MyVoxel::Bounds3 outside(MyMath::Vector3(4.0, -0.5, -0.5), MyMath::Vector3(5.0, 0.5, 0.5));
    const MyVoxel::Bounds3 intersecting(MyMath::Vector3(2.5, -0.5, -0.5), MyMath::Vector3(3.5, 0.5, 0.5));
    const MyVoxel::Bounds3 touching(MyMath::Vector3(3.0, 0.0, 0.0), MyMath::Vector3(4.0, 1.0, 1.0));

    const bool passed =
        geometry->classifyLocalBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside &&
        geometry->classifyLocalBounds(outside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        geometry->classifyLocalBounds(intersecting) == MyVoxel::Geometry::ShapeRelation::Intersecting &&
        geometry->classifyLocalBounds(touching) == MyVoxel::Geometry::ShapeRelation::Intersecting;

    return check(passed, "SphereGeometry classification");
}

// 测试标准圆柱体参数和包围盒。
bool testCylinderProperties()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::CylinderGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::CylinderGeometry>(3.0, 8.0);

    const MyVoxel::Bounds3 bounds = geometry->localBounds();

    const bool passed =
        geometry->kind() == MyVoxel::Geometry::ShapeKind::Cylinder &&
        equalValue(geometry->radius(), 3.0) &&
        equalValue(geometry->height(), 8.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-3.0, -3.0, -4.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(3.0, 3.0, 4.0));

    return check(passed, "CylinderGeometry properties");
}

// 测试标准圆柱体点包含。
bool testCylinderContainment()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::CylinderGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::CylinderGeometry>(3.0, 8.0);

    const bool passed =
        geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)) &&
        geometry->containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)) &&
        geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 4.0)) &&
        !geometry->containsLocalPoint(MyMath::Vector3(3.1, 0.0, 0.0)) &&
        !geometry->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 4.1));

    return check(passed, "CylinderGeometry point containment");
}

// 测试标准圆柱体区域分类。
bool testCylinderClassification()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::CylinderGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::CylinderGeometry>(3.0, 8.0);

    const MyVoxel::Bounds3 inside(MyMath::Vector3(-1.0, -1.0, -2.0), MyMath::Vector3(1.0, 1.0, 2.0));
    const MyVoxel::Bounds3 radialOutside(MyMath::Vector3(4.0, -0.5, -1.0), MyMath::Vector3(5.0, 0.5, 1.0));
    const MyVoxel::Bounds3 heightOutside(MyMath::Vector3(-1.0, -1.0, 5.0), MyMath::Vector3(1.0, 1.0, 6.0));
    const MyVoxel::Bounds3 intersecting(MyMath::Vector3(2.5, -0.5, -1.0), MyMath::Vector3(3.5, 0.5, 1.0));
    const MyVoxel::Bounds3 touching(MyMath::Vector3(-1.0, -1.0, 4.0), MyMath::Vector3(1.0, 1.0, 5.0));

    const bool passed =
        geometry->classifyLocalBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside &&
        geometry->classifyLocalBounds(radialOutside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        geometry->classifyLocalBounds(heightOutside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        geometry->classifyLocalBounds(intersecting) == MyVoxel::Geometry::ShapeRelation::Intersecting &&
        geometry->classifyLocalBounds(touching) == MyVoxel::Geometry::ShapeRelation::Intersecting;

    return check(passed, "CylinderGeometry classification");
}

// 测试标准圆锥台参数、截面半径和包围盒。
bool testConeFrustumProperties()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::ConeFrustumGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::ConeFrustumGeometry>(4.0, 2.0, 10.0);

    const MyVoxel::Bounds3 bounds = geometry->localBounds();

    const bool passed =
        geometry->kind() == MyVoxel::Geometry::ShapeKind::ConeFrustum &&
        equalValue(geometry->bottomRadius(), 4.0) &&
        equalValue(geometry->topRadius(), 2.0) &&
        equalValue(geometry->height(), 10.0) &&
        equalValue(geometry->radiusAt(-5.0), 4.0) &&
        equalValue(geometry->radiusAt(0.0), 3.0) &&
        equalValue(geometry->radiusAt(5.0), 2.0) &&
        equalVector(bounds.minimum(), MyMath::Vector3(-4.0, -4.0, -5.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(4.0, 4.0, 5.0));

    return check(passed, "ConeFrustumGeometry properties");
}

// 测试标准圆锥台和圆锥点包含。
bool testConeFrustumContainment()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::ConeFrustumGeometry> frustum =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::ConeFrustumGeometry>(4.0, 2.0, 10.0);

    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::ConeFrustumGeometry> cone =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::ConeFrustumGeometry>(4.0, 0.0, 10.0);

    const bool passed =
        frustum->containsLocalPoint(MyMath::Vector3(4.0, 0.0, -5.0)) &&
        frustum->containsLocalPoint(MyMath::Vector3(3.0, 0.0, 0.0)) &&
        frustum->containsLocalPoint(MyMath::Vector3(2.0, 0.0, 5.0)) &&
        !frustum->containsLocalPoint(MyMath::Vector3(3.1, 0.0, 0.0)) &&
        cone->containsLocalPoint(MyMath::Vector3(0.0, 0.0, 5.0)) &&
        !cone->containsLocalPoint(MyMath::Vector3(0.1, 0.0, 5.0)) &&
        equalValue(cone->radiusAt(0.0), 2.0);

    return check(passed, "ConeFrustumGeometry point containment");
}

// 测试标准圆锥台区域分类。
bool testConeFrustumClassification()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::ConeFrustumGeometry> geometry =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::ConeFrustumGeometry>(4.0, 2.0, 10.0);

    const MyVoxel::Bounds3 inside(MyMath::Vector3(-0.5, -0.5, -2.0), MyMath::Vector3(0.5, 0.5, 2.0));
    const MyVoxel::Bounds3 radialOutside(MyMath::Vector3(5.0, -0.5, -5.0), MyMath::Vector3(6.0, 0.5, 5.0));
    const MyVoxel::Bounds3 heightOutside(MyMath::Vector3(-1.0, -1.0, 6.0), MyMath::Vector3(1.0, 1.0, 7.0));
    const MyVoxel::Bounds3 intersecting(MyMath::Vector3(2.8, -0.2, -1.0), MyMath::Vector3(3.2, 0.2, 1.0));
    const MyVoxel::Bounds3 touching(MyMath::Vector3(-1.0, -1.0, -6.0), MyMath::Vector3(1.0, 1.0, -5.0));

    const bool passed =
        geometry->classifyLocalBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside &&
        geometry->classifyLocalBounds(radialOutside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        geometry->classifyLocalBounds(heightOutside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        geometry->classifyLocalBounds(intersecting) == MyVoxel::Geometry::ShapeRelation::Intersecting &&
        geometry->classifyLocalBounds(touching) == MyVoxel::Geometry::ShapeRelation::Intersecting;

    return check(passed, "ConeFrustumGeometry classification");
}

// 测试标准几何体与Shape值对象集成。
bool testShapeIntegration()
{
    const MyVoxel::Foundation::RefPtr<MyVoxel::Geometry::CylinderGeometry> cylinder =
        MyVoxel::Foundation::makeRef<MyVoxel::Geometry::CylinderGeometry>(3.0, 8.0);

    const std::size_t initialReferenceCount = cylinder->referenceCount();

    MyVoxel::Geometry::Shape shape;
    MyVoxel::Geometry::Shape copy;

    {
        const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::ShapeGeometry> geometry = cylinder;
        shape = MyVoxel::Geometry::Shape(geometry);
        copy = shape;

        const bool sharedPassed =
            shape.isValid() &&
            copy.isValid() &&
            shape.kind() == MyVoxel::Geometry::ShapeKind::Cylinder &&
            shape.sharesGeometryWith(copy) &&
            shape.geometryPointer() == cylinder.get() &&
            shape.containsLocalPoint(MyMath::Vector3(1.0, 1.0, 0.0)) &&
            cylinder->referenceCount() == initialReferenceCount + 3;

        if (!sharedPassed)
        {
            return check(false, "Standard geometry Shape integration");
        }
    }

    const bool passed =
        shape.isValid() &&
        copy.isValid() &&
        shape.sharesGeometryWith(copy) &&
        cylinder->referenceCount() == initialReferenceCount + 2;

    return check(passed, "Standard geometry Shape integration");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testBoxProperties(),
        testBoxContainment(),
        testBoxClassification(),
        testSphereProperties(),
        testSphereContainment(),
        testSphereClassification(),
        testCylinderProperties(),
        testCylinderContainment(),
        testCylinderClassification(),
        testConeFrustumProperties(),
        testConeFrustumContainment(),
        testConeFrustumClassification(),
        testShapeIntegration()
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
    std::cout << "Standard geometry tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}