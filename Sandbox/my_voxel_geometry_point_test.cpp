#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Geometry_Object.h"
#include "MyVoxel/Geometry/Point/Geometry_Point.h"

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

}

int main()
{
    /// 基础几何语义

    const MyMath::Vector3 position(1.25, -2.5, 3.75);
    const MyVoxel::Geometry_Point point(position);
    check(point.position().isEqualTo(position, 0.0), "Geometry point preserves exact position");
    check(point.position().isFinite(), "Geometry point position finite");

    /// 三维空间语义

    const MyVoxel::Geometry_Point origin(MyMath::Vector3(0.0, 0.0, 0.0));
    const MyVoxel::Geometry_Point spatial(MyMath::Vector3(-4.0, 5.0, 6.0));
    check(origin.position().isEqualTo(MyMath::Vector3(0.0, 0.0, 0.0), 0.0), "Geometry point supports origin");
    check(spatial.position().z() == 6.0, "Geometry point is not restricted to XY plane");

    /// 几何资源共享

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point> resource(new MyVoxel::Geometry_Point(position));
    check(static_cast<bool>(resource), "Geometry point managed by RefPtr");
    check(resource->referenceCount() == 1, "Geometry point initial shared reference count");

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Object> baseResource = resource;
    check(static_cast<bool>(baseResource), "Geometry point converts to Geometry_Object RefPtr");
    check(resource->referenceCount() == 2, "Geometry point base reference shares lifetime");

    /// 几何资源身份不等于拓扑身份

    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point> samePositionA(new MyVoxel::Geometry_Point(position));
    MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point> samePositionB(new MyVoxel::Geometry_Point(position));
    check(samePositionA->position().isEqualTo(samePositionB->position(), 0.0), "Independent geometry points may share coordinates");
    check(samePositionA.get() != samePositionB.get(), "Independent geometry resources remain separate resources");

    std::cout << "Geometry_Point Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}