#include <cstdlib>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Geometry_Object.h"
#include "MyVoxel/Instance/Instance_Object.h"
#include "MyVoxel/Topology/Topology_Object.h"
#include "MyVoxel/Topology/Topology_Orientation.h"
#include "MyVoxel/Topology/Topology_TObject.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++g_failedCount;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

// 仅用于验证Geometry_Object能够作为侵入式共享几何资源根类。
class TestGeometry : public MyVoxel::Geometry_Object
{
public:
    TestGeometry()
    {
    }

protected:
    ~TestGeometry() override
    {
    }
};

// 仅用于验证Topology_TObject能够作为共享拓扑身份实体根类。
class TestTObject : public MyVoxel::Topology_TObject
{
public:
    TestTObject()
    {
    }

protected:
    ~TestTObject() override
    {
    }
};

// 暴露Topology_Object受保护构造和反向辅助接口以验证轻量句柄基类。
class TestTopologyObject : public MyVoxel::Topology_Object
{
public:
    TestTopologyObject()
    {
    }

    TestTopologyObject(const MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject>& object, MyVoxel::Topology_Orientation orientation)
        : MyVoxel::Topology_Object(object, orientation)
    {
    }

    TestTopologyObject reversed() const
    {
        return isValid() ? TestTopologyObject(tObject(), reversedOrientation()) : TestTopologyObject();
    }
};

// 暴露Instance_Object受保护构造以验证公共空间放置逻辑。
class TestInstanceObject : public MyVoxel::Instance_Object
{
public:
    TestInstanceObject()
    {
    }

    explicit TestInstanceObject(const MyMath::Matrix4& localToWorld)
        : MyVoxel::Instance_Object(localToWorld)
    {
    }
};

}

int main()
{
    MyVoxel::Foundation::RefPtr<const TestGeometry> geometry(new TestGeometry());
    check(static_cast<bool>(geometry), "Geometry object managed by RefPtr");
    check(geometry->referenceCount() == 1, "Geometry object initial shared reference count");

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Object> geometryBase = geometry;
    check(static_cast<bool>(geometryBase), "Geometry object converts to base RefPtr");
    check(geometry->referenceCount() == 2, "Geometry base reference shares lifetime");

    const MyVoxel::Foundation::RefPtr<const TestTObject> tObject(new TestTObject());
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Topology_TObject> baseTObject = tObject;
    const TestTopologyObject forward(baseTObject, MyVoxel::Topology_Orientation::Forward);

    check(forward.isValid(), "Topology object with shared TObject valid");
    check(!forward.isNull(), "Valid topology object not null");
    check(static_cast<bool>(forward), "Valid topology object bool true");
    check(forward.isForward(), "Topology object forward orientation");
    check(!forward.isReversed(), "Forward topology object not reversed");
    check(forward.orientation() == MyVoxel::Topology_Orientation::Forward, "Topology orientation preserved");

    const TestTopologyObject copied = forward;
    check(copied.isSame(forward), "Copied topology handle shares topology identity");
    check(copied.isForward(), "Copied topology handle preserves orientation");

    const TestTopologyObject reversed = forward.reversed();
    check(reversed.isValid(), "Reversed topology handle valid");
    check(reversed.isSame(forward), "Reversed topology handle preserves topology identity");
    check(reversed.isReversed(), "Reversed topology handle flips orientation");
    check(reversed.reversed().isForward(), "Double topology reversal restores orientation");

    const MyVoxel::Foundation::RefPtr<const TestTObject> anotherTObject(new TestTObject());
    const TestTopologyObject independent(anotherTObject, MyVoxel::Topology_Orientation::Forward);
    check(!independent.isSame(forward), "Different TObject means different topology identity");

    const TestTopologyObject emptyTopology;
    check(!emptyTopology.isValid(), "Default topology object invalid");
    check(emptyTopology.isNull(), "Default topology object null");
    check(!static_cast<bool>(emptyTopology), "Default topology object bool false");
    check(!emptyTopology.isSame(emptyTopology), "Null topology object has no topology identity");

    check(MyVoxel::oppositeOrientation(MyVoxel::Topology_Orientation::Forward) == MyVoxel::Topology_Orientation::Reversed,
          "Forward opposite orientation");
    check(MyVoxel::oppositeOrientation(MyVoxel::Topology_Orientation::Reversed) == MyVoxel::Topology_Orientation::Forward,
          "Reversed opposite orientation");

    const TestInstanceObject identityInstance;
    check(identityInstance.isPlacementValid(), "Default instance placement valid");
    check(identityInstance.localToWorld().isIdentity(0.0), "Default instance uses identity local-to-world");
    check(identityInstance.worldToLocal().isIdentity(0.0), "Default instance uses identity world-to-local");

    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(4.0, -3.0, 2.0));
    const TestInstanceObject placedInstance(transform);
    check(placedInstance.isPlacementValid(), "Affine invertible instance placement valid");
    check(placedInstance.localToWorld().isEqualTo(transform, 0.0), "Instance preserves local-to-world transform");

    const MyMath::Vector3 localPoint(1.0, 2.0, 3.0);
    const MyMath::Vector3 worldPoint = placedInstance.localToWorld().transformPoint(localPoint);
    const MyMath::Vector3 restoredPoint = placedInstance.worldToLocal().transformPoint(worldPoint);
    check(worldPoint.isEqualTo(MyMath::Vector3(5.0, -1.0, 5.0), 0.0), "Instance local-to-world transforms points");
    check(restoredPoint.isEqualTo(localPoint, 0.0), "Instance world-to-local is exact inverse for translation");

    std::cout << "Base_Object Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}