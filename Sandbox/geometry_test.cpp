#include <atomic>
#include <cstddef>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/ParallelExecutor.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Geometry/ShapeGeometry.h"
#include "MyVoxel/Geometry/ShapeInstance.h"
#include "MyVoxel/Geometry/ShapeKind.h"
#include "MyVoxel/Geometry/ShapeRelation.h"

namespace
{

const double TestEpsilon = 1.0e-12; // 几何层测试使用的浮点比较误差。

// 提供几何层基础接口测试使用的不可变盒体。
class TestBoxGeometry : public MyVoxel::Geometry::ShapeGeometry
{
public:
    TestBoxGeometry()
        : m_bounds(MyMath::Vector3(-1.0, -2.0, -3.0), MyMath::Vector3(1.0, 2.0, 3.0))
    {
        ++s_aliveCount;
    }

    /// 几何属性

    // 返回测试几何体类型。
    MyVoxel::Geometry::ShapeKind kind() const override
    {
        return MyVoxel::Geometry::ShapeKind::Custom;
    }

    // 返回测试盒体局部包围盒。
    MyVoxel::Bounds3 localBounds() const override
    {
        return m_bounds;
    }

    /// 空间查询

    // 判断局部点是否位于测试盒体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override
    {
        return m_bounds.contains(point);
    }

    // 返回局部包围盒与测试盒体之间的保守空间关系。
    MyVoxel::Geometry::ShapeRelation classifyLocalBounds(const MyVoxel::Bounds3& bounds) const override
    {
        if (!m_bounds.intersects(bounds))
        {
            return MyVoxel::Geometry::ShapeRelation::Outside;
        }

        if (strictlyContains(bounds))
        {
            return MyVoxel::Geometry::ShapeRelation::Inside;
        }

        return MyVoxel::Geometry::ShapeRelation::Intersecting;
    }

    /// 生命周期统计

    // 清除测试几何体生命周期统计。
    static void resetStatistics()
    {
        s_aliveCount.store(0);
        s_destroyedCount.store(0);
    }

    // 返回当前存活的测试几何体数量。
    static int aliveCount()
    {
        return s_aliveCount.load();
    }

    // 返回已经销毁的测试几何体数量。
    static int destroyedCount()
    {
        return s_destroyedCount.load();
    }

protected:
    // 记录测试几何体销毁。
    ~TestBoxGeometry() override
    {
        --s_aliveCount;
        ++s_destroyedCount;
    }

private:
    // 判断指定包围盒是否严格位于测试盒体内部并且不接触边界。
    bool strictlyContains(const MyVoxel::Bounds3& bounds) const
    {
        return bounds.minimum().x() > m_bounds.minimum().x() &&
               bounds.maximum().x() < m_bounds.maximum().x() &&
               bounds.minimum().y() > m_bounds.minimum().y() &&
               bounds.maximum().y() < m_bounds.maximum().y() &&
               bounds.minimum().z() > m_bounds.minimum().z() &&
               bounds.maximum().z() < m_bounds.maximum().z();
    }

private:
    MyVoxel::Bounds3 m_bounds; // 测试盒体局部包围盒。

    static std::atomic<int> s_aliveCount; // 当前存活的测试几何体数量。
    static std::atomic<int> s_destroyedCount; // 已经销毁的测试几何体数量。
};

std::atomic<int> TestBoxGeometry::s_aliveCount(0);
std::atomic<int> TestBoxGeometry::s_destroyedCount(0);

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

// 创建测试使用的Shape。
MyVoxel::Geometry::Shape createTestShape()
{
    const MyVoxel::Foundation::RefPtr<TestBoxGeometry> geometry = MyVoxel::Foundation::makeRef<TestBoxGeometry>();
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry::ShapeGeometry> immutableGeometry = geometry;
    return MyVoxel::Geometry::Shape(immutableGeometry);
}

// 测试Shape和ShapeInstance默认状态。
bool testDefaultState()
{
    const MyVoxel::Geometry::Shape shape;
    const MyVoxel::Geometry::ShapeInstance instance;

    const bool passed =
        !shape.isValid() &&
        shape.geometryPointer() == nullptr &&
        !instance.isValid();

    return check(passed, "Geometry default state");
}

// 测试Shape使用自定义引用指针共享不可变几何数据。
bool testShapeSharing()
{
    const MyVoxel::Geometry::Shape first = createTestShape();
    const MyVoxel::Geometry::Shape second = first;
    const MyVoxel::Geometry::Shape independent = createTestShape();

    const bool passed =
        first.isValid() &&
        second.isValid() &&
        first.geometryPointer() != nullptr &&
        first.geometryPointer() == second.geometryPointer() &&
        first.geometryPointer() != independent.geometryPointer() &&
        first.sharesGeometryWith(second) &&
        second.sharesGeometryWith(first) &&
        !first.sharesGeometryWith(independent) &&
        first.geometry().referenceCount() == 2 &&
        independent.geometry().referenceCount() == 1 &&
        first.kind() == MyVoxel::Geometry::ShapeKind::Custom;

    return check(passed, "Shape geometry sharing");
}

// 测试Shape释放后是否正确销毁连续几何数据。
bool testShapeLifetime()
{
    TestBoxGeometry::resetStatistics();

    bool sharedPassed = false;

    {
        const MyVoxel::Geometry::Shape first = createTestShape();

        {
            const MyVoxel::Geometry::Shape second = first;

            sharedPassed =
                TestBoxGeometry::aliveCount() == 1 &&
                TestBoxGeometry::destroyedCount() == 0 &&
                first.geometry().referenceCount() == 2 &&
                second.geometry().referenceCount() == 2;
        }

        sharedPassed =
            sharedPassed &&
            TestBoxGeometry::aliveCount() == 1 &&
            TestBoxGeometry::destroyedCount() == 0 &&
            first.geometry().referenceCount() == 1;
    }

    const bool passed =
        sharedPassed &&
        TestBoxGeometry::aliveCount() == 0 &&
        TestBoxGeometry::destroyedCount() == 1;

    return check(passed, "Shape geometry lifetime");
}

// 测试Shape局部属性和点查询。
bool testShapeLocalQueries()
{
    const MyVoxel::Geometry::Shape shape = createTestShape();
    const MyVoxel::Bounds3 bounds = shape.localBounds();

    const bool passed =
        equalVector(bounds.minimum(), MyMath::Vector3(-1.0, -2.0, -3.0)) &&
        equalVector(bounds.maximum(), MyMath::Vector3(1.0, 2.0, 3.0)) &&
        shape.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)) &&
        shape.containsLocalPoint(MyMath::Vector3(1.0, 2.0, 3.0)) &&
        !shape.containsLocalPoint(MyMath::Vector3(1.1, 0.0, 0.0));

    return check(passed, "Shape local queries");
}

// 测试Shape局部包围盒分类。
bool testShapeLocalClassification()
{
    const MyVoxel::Geometry::Shape shape = createTestShape();

    const MyVoxel::Bounds3 inside(MyMath::Vector3(-0.5, -1.0, -1.5), MyMath::Vector3(0.5, 1.0, 1.5));
    const MyVoxel::Bounds3 outside(MyMath::Vector3(2.0, 2.0, 2.0), MyMath::Vector3(3.0, 3.0, 3.0));
    const MyVoxel::Bounds3 intersecting(MyMath::Vector3(0.5, 1.0, 2.0), MyMath::Vector3(1.5, 3.0, 4.0));
    const MyVoxel::Bounds3 touching(MyMath::Vector3(1.0, -1.0, -1.0), MyMath::Vector3(2.0, 1.0, 1.0));

    const bool passed =
        shape.classifyLocalBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside &&
        shape.classifyLocalBounds(outside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        shape.classifyLocalBounds(intersecting) == MyVoxel::Geometry::ShapeRelation::Intersecting &&
        shape.classifyLocalBounds(touching) == MyVoxel::Geometry::ShapeRelation::Intersecting;

    return check(passed, "Shape local classification");
}

// 测试单位变换Shape实例。
bool testIdentityInstance()
{
    const MyVoxel::Geometry::Shape shape = createTestShape();
    const MyVoxel::Geometry::ShapeInstance instance(shape);

    const bool passed =
        instance.isValid() &&
        instance.shape().sharesGeometryWith(shape) &&
        instance.localToWorld().isIdentity() &&
        instance.worldToLocal().isIdentity() &&
        instance.worldBounds().isEqualTo(shape.localBounds(), TestEpsilon) &&
        instance.containsWorldPoint(MyMath::Vector3(0.5, 1.0, 2.0));

    return check(passed, "ShapeInstance identity transform");
}

// 测试包含非均匀缩放和平移的Shape实例。
bool testAffineInstance()
{
    const MyVoxel::Geometry::Shape shape = createTestShape();

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 0) = 2.0;
    transform(1, 1) = 3.0;
    transform(2, 2) = 4.0;
    transform.setTranslation(MyMath::Vector3(10.0, 20.0, 30.0));

    const MyVoxel::Geometry::ShapeInstance instance(shape, transform);

    const MyMath::Vector3 localPoint(0.5, 1.0, 1.5);
    const MyMath::Vector3 worldPoint = transform.transformPoint(localPoint);
    const MyMath::Vector3 restoredPoint = instance.worldToLocal().transformPoint(worldPoint);

    const bool passed =
        instance.isValid() &&
        equalVector(worldPoint, MyMath::Vector3(11.0, 23.0, 36.0)) &&
        equalVector(restoredPoint, localPoint) &&
        instance.containsWorldPoint(worldPoint) &&
        !instance.containsWorldPoint(MyMath::Vector3(14.5, 20.0, 30.0)) &&
        equalVector(instance.worldBounds().minimum(), MyMath::Vector3(8.0, 14.0, 18.0)) &&
        equalVector(instance.worldBounds().maximum(), MyMath::Vector3(12.0, 26.0, 42.0));

    return check(passed, "ShapeInstance affine transform");
}

// 测试旋转实例的世界轴对齐包围盒。
bool testRotatedWorldBounds()
{
    const MyVoxel::Geometry::Shape shape = createTestShape();

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 0) = 0.0;
    transform(0, 1) = -1.0;
    transform(1, 0) = 1.0;
    transform(1, 1) = 0.0;
    transform.setTranslation(MyMath::Vector3(10.0, 20.0, 30.0));

    const MyVoxel::Geometry::ShapeInstance instance(shape, transform);

    const bool passed =
        equalVector(instance.worldBounds().minimum(), MyMath::Vector3(8.0, 19.0, 27.0)) &&
        equalVector(instance.worldBounds().maximum(), MyMath::Vector3(12.0, 21.0, 33.0));

    return check(passed, "ShapeInstance rotated world bounds");
}

// 测试Shape实例的世界包围盒分类。
bool testWorldClassification()
{
    const MyVoxel::Geometry::Shape shape = createTestShape();

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 0) = 2.0;
    transform(1, 1) = 3.0;
    transform(2, 2) = 4.0;
    transform.setTranslation(MyMath::Vector3(10.0, 20.0, 30.0));

    const MyVoxel::Geometry::ShapeInstance instance(shape, transform);

    const MyVoxel::Bounds3 inside(MyMath::Vector3(9.0, 17.0, 24.0), MyMath::Vector3(11.0, 23.0, 36.0));
    const MyVoxel::Bounds3 outside(MyMath::Vector3(20.0, 20.0, 30.0), MyMath::Vector3(22.0, 22.0, 32.0));
    const MyVoxel::Bounds3 touching(MyMath::Vector3(12.0, 17.0, 24.0), MyMath::Vector3(14.0, 23.0, 36.0));

    const bool passed =
        instance.classifyWorldBounds(inside) == MyVoxel::Geometry::ShapeRelation::Inside &&
        instance.classifyWorldBounds(outside) == MyVoxel::Geometry::ShapeRelation::Outside &&
        instance.classifyWorldBounds(touching) == MyVoxel::Geometry::ShapeRelation::Intersecting;

    return check(passed, "ShapeInstance world classification");
}

// 测试Shape查询能否通过统一执行器并发调用。
bool testConcurrentQueries()
{
    const MyVoxel::Geometry::Shape shape = createTestShape();
    const MyVoxel::Geometry::ShapeInstance instance(shape);
    MyVoxel::Foundation::ParallelExecutor executor(4);

    std::atomic<std::size_t> insideCount(0);

    executor.forEach(1000, [&](std::size_t taskIndex)
    {
        const double x = taskIndex % 2 == 0 ? 0.5 : 2.0;

        if (instance.containsWorldPoint(MyMath::Vector3(x, 0.0, 0.0)))
        {
            insideCount.fetch_add(1);
        }
    });

    const bool passed =
        insideCount.load() == 500 &&
        shape.geometry().referenceCount() == 2;

    return check(passed, "Geometry concurrent queries");
}

}

int main()
{
    int passedCount = 0;
    int failedCount = 0;

    const bool results[] =
    {
        testDefaultState(),
        testShapeSharing(),
        testShapeLifetime(),
        testShapeLocalQueries(),
        testShapeLocalClassification(),
        testIdentityInstance(),
        testAffineInstance(),
        testRotatedWorldBounds(),
        testWorldClassification(),
        testConcurrentQueries()
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
    std::cout << "Geometry tests" << std::endl;
    std::cout << "Passed: " << passedCount << std::endl;
    std::cout << "Failed: " << failedCount << std::endl;

    return failedCount == 0 ? 0 : 1;
}