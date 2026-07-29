#include <cstddef>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"

namespace
{

const double TestEpsilon = 1.0e-12; // 基础层测试使用的浮点比较误差。
const double QueryTolerance = 1.0e-5; // 容差查询测试允许的边界偏差。

// 输出测试结果并返回是否通过。
bool check(bool condition, const char* name)
{
    std::cout << (condition ? "[Passed] " : "[Failed] ") << name << std::endl;
    return condition;
}

// 判断两个三维数据是否近似相等。
bool equalVector(const MyMath::Vector3& first, const MyMath::Vector3& second)
{
    return first.isEqualTo(second, TestEpsilon);
}

// 测试默认包围盒状态。
bool testDefaultState()
{
    const MyVoxel::Bounds3 bounds;

    return check(!bounds.isValid() && !bounds.hasVolume(), "Bounds3 default state");
}

// 测试包围盒基本属性。
bool testProperties()
{
    const MyVoxel::Bounds3 bounds(MyMath::Vector3(-2.0, -4.0, -6.0), MyMath::Vector3(4.0, 8.0, 10.0));

    const bool passed =
        bounds.isValid() &&
        bounds.hasVolume() &&
        equalVector(bounds.minimum(), MyMath::Vector3(-2.0, -4.0, -6.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(4.0, 8.0, 10.0)) &&
        equalVector(bounds.center(), MyMath::Vector3(1.0, 2.0, 2.0)) &&
        equalVector(bounds.size(), MyMath::Vector3(6.0, 12.0, 16.0)) &&
        equalVector(bounds.extent(), MyMath::Vector3(3.0, 6.0, 8.0)) &&
        std::fabs(bounds.volume() - 1152.0) <= TestEpsilon;

    return check(passed, "Bounds3 properties");
}

// 测试通过中心和尺寸创建包围盒。
bool testCenterAndSizeCreation()
{
    const MyVoxel::Bounds3 bounds = MyVoxel::Bounds3::fromCenterAndSize(
        MyMath::Vector3(10.0, 20.0, 30.0),
        MyMath::Vector3(4.0, 6.0, 8.0));

    const bool passed =
        equalVector(bounds.minimum(), MyMath::Vector3(8.0, 17.0, 26.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(12.0, 23.0, 34.0));

    return check(passed, "Bounds3 center and size creation");
}

// 测试角点编号覆盖全部八个角点。
bool testCorners()
{
    const MyVoxel::Bounds3 bounds(MyMath::Vector3(1.0, 2.0, 3.0), MyMath::Vector3(4.0, 5.0, 6.0));

    const MyMath::Vector3 expectedCorners[MyVoxel::Bounds3::CornerCount] =
    {
        MyMath::Vector3(1.0, 2.0, 3.0),
        MyMath::Vector3(4.0, 2.0, 3.0),
        MyMath::Vector3(1.0, 5.0, 3.0),
        MyMath::Vector3(4.0, 5.0, 3.0),
        MyMath::Vector3(1.0, 2.0, 6.0),
        MyMath::Vector3(4.0, 2.0, 6.0),
        MyMath::Vector3(1.0, 5.0, 6.0),
        MyMath::Vector3(4.0, 5.0, 6.0)
    };

    for (int cornerIndex = 0; cornerIndex < MyVoxel::Bounds3::CornerCount; ++cornerIndex)
    {
        if (!equalVector(bounds.corner(cornerIndex), expectedCorners[cornerIndex]))
        {
            return check(false, "Bounds3 corners");
        }
    }

    return check(true, "Bounds3 corners");
}

// 测试点包含关系和闭区间边界语义。
bool testPointContainment()
{
    const MyVoxel::Bounds3 bounds(MyMath::Vector3(-1.0, -2.0, -3.0), MyMath::Vector3(4.0, 5.0, 6.0));

    const bool passed =
        bounds.contains(MyMath::Vector3(0.0, 0.0, 0.0)) &&
        bounds.contains(MyMath::Vector3(-1.0, -2.0, -3.0)) &&
        bounds.contains(MyMath::Vector3(4.0, 5.0, 6.0)) &&
        !bounds.contains(MyMath::Vector3(4.1, 5.0, 6.0)) &&
        bounds.contains(MyMath::Vector3(4.0 + QueryTolerance * 0.5, 5.0, 6.0), QueryTolerance);

    return check(passed, "Bounds3 point containment");
}

// 测试包围盒包含关系。
bool testBoundsContainment()
{
    const MyVoxel::Bounds3 outer(MyMath::Vector3(-10.0, -10.0, -10.0), MyMath::Vector3(10.0, 10.0, 10.0));
    const MyVoxel::Bounds3 inner(MyMath::Vector3(-2.0, -3.0, -4.0), MyMath::Vector3(5.0, 6.0, 7.0));
    const MyVoxel::Bounds3 outside(MyMath::Vector3(9.0, 9.0, 9.0), MyMath::Vector3(11.0, 11.0, 11.0));

    return check(outer.contains(inner) && !outer.contains(outside), "Bounds3 bounds containment");
}

// 测试相交、接触和容差相交关系。
bool testIntersection()
{
    const MyVoxel::Bounds3 first(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(10.0, 10.0, 10.0));
    const MyVoxel::Bounds3 overlapping(MyMath::Vector3(5.0, 5.0, 5.0), MyMath::Vector3(15.0, 15.0, 15.0));
    const MyVoxel::Bounds3 touching(MyMath::Vector3(10.0, 2.0, 2.0), MyMath::Vector3(12.0, 8.0, 8.0));
    const MyVoxel::Bounds3 separated(MyMath::Vector3(10.1, 2.0, 2.0), MyMath::Vector3(12.0, 8.0, 8.0));
    const MyVoxel::Bounds3 toleranceSeparated(MyMath::Vector3(10.0 + QueryTolerance * 0.5, 2.0, 2.0), MyMath::Vector3(12.0, 8.0, 8.0));

    const bool passed =
        first.intersects(overlapping) &&
        first.intersects(touching) &&
        !first.intersects(separated) &&
        first.intersects(toleranceSeparated, QueryTolerance);

    return check(passed, "Bounds3 intersection");
}

// 测试从无效状态累计点和包围盒。
bool testInclude()
{
    MyVoxel::Bounds3 bounds;

    bounds.include(MyMath::Vector3(2.0, 3.0, 4.0));
    bounds.include(MyMath::Vector3(-1.0, 8.0, 0.0));
    bounds.include(MyVoxel::Bounds3(MyMath::Vector3(-5.0, 1.0, -2.0), MyMath::Vector3(3.0, 10.0, 6.0)));

    const bool passed =
        equalVector(bounds.minimum(), MyMath::Vector3(-5.0, 1.0, -2.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(3.0, 10.0, 6.0));

    return check(passed, "Bounds3 include");
}

// 测试扩展和平移。
bool testExpandAndTranslate()
{
    const MyVoxel::Bounds3 bounds(MyMath::Vector3(-1.0, -2.0, -3.0), MyMath::Vector3(4.0, 5.0, 6.0));
    const MyVoxel::Bounds3 expanded = bounds.expanded(2.0);
    const MyVoxel::Bounds3 translated = bounds.translated(MyMath::Vector3(10.0, 20.0, 30.0));

    const bool passed =
        equalVector(expanded.minimum(), MyMath::Vector3(-3.0, -4.0, -5.0)) &&
        equalVector(expanded.maximum(), MyMath::Vector3(6.0, 7.0, 8.0)) &&
        equalVector(translated.minimum(), MyMath::Vector3(9.0, 18.0, 27.0)) &&
        equalVector(translated.maximum(), MyMath::Vector3(14.0, 25.0, 36.0));

    return check(passed, "Bounds3 expand and translate");
}

// 测试仿射旋转后的轴对齐包围盒。
bool testAffineTransform()
{
    const MyVoxel::Bounds3 bounds(MyMath::Vector3(-1.0, -2.0, -3.0), MyMath::Vector3(4.0, 5.0, 6.0));

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();

    transform.setValue(0, 0, 0.0);
    transform.setValue(0, 1, -1.0);
    transform.setValue(1, 0, 1.0);
    transform.setValue(1, 1, 0.0);
    transform.setTranslation(MyMath::Vector3(10.0, 20.0, 30.0));

    const MyVoxel::Bounds3 transformed = bounds.transformed(transform);

    const bool passed =
        equalVector(transformed.minimum(), MyMath::Vector3(5.0, 19.0, 27.0)) &&
        equalVector(transformed.maximum(), MyMath::Vector3(12.0, 24.0, 36.0));

    return check(passed, "Bounds3 affine transform");
}

// 测试无效包围盒相等和清理行为。
bool testInvalidState()
{
    MyVoxel::Bounds3 first;
    const MyVoxel::Bounds3 second;

    first.include(MyMath::Vector3(1.0, 2.0, 3.0));
    first.clear();

    const bool passed =
        !first.isValid() &&
        first.isEqualTo(second) &&
        !first.expanded(1.0).isValid() &&
        !first.translated(MyMath::Vector3(1.0, 2.0, 3.0)).isValid();

    return check(passed, "Bounds3 invalid state");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testDefaultState(),
        testProperties(),
        testCenterAndSizeCreation(),
        testCorners(),
        testPointContainment(),
        testBoundsContainment(),
        testIntersection(),
        testInclude(),
        testExpandAndTranslate(),
        testAffineTransform(),
        testInvalidState()
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
    std::cout << "Base tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}