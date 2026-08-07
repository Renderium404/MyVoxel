#include <iostream>

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyVoxel/Geometry/Shape/Geometry_Cylinder.h"
#include "MyVoxel/Geometry/Shape/Geometry_Sphere.h"

namespace
{

int passedCount = 0;
int failedCount = 0;

void check(bool condition, const char* name)
{
    if (condition)
    {
        ++passedCount;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++failedCount;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

template <typename GeometryType>
void checkStableBounds(const MyVoxel::Foundation::RefPtr<GeometryType>& geometry, const char* name)
{
    const MyVoxel::Bounds3* first = &geometry->localBounds();
    const MyVoxel::Bounds3* second = &geometry->localBounds();
    check(first == second && first->isValid(), name);
}

}

int main()
{
    checkStableBounds(MyVoxel::Foundation::makeRef<MyVoxel::Geometry_Box>(2.0, 4.0, 6.0), "Box base bounds");
    checkStableBounds(MyVoxel::Foundation::makeRef<MyVoxel::Geometry_Sphere>(3.0), "Sphere base bounds");
    checkStableBounds(MyVoxel::Foundation::makeRef<MyVoxel::Geometry_Cylinder>(2.0, 8.0), "Cylinder base bounds");
    checkStableBounds(MyVoxel::Foundation::makeRef<MyVoxel::Geometry_ConeFrustum>(3.0, 1.0, 8.0), "ConeFrustum base bounds");

    std::cout << "Passed " << passedCount << " / Failed " << failedCount << std::endl;
    return failedCount == 0 ? 0 : 1;
}
