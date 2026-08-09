#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Point/Geometry_Point.h"
#include "MyVoxel/Topology/Topology_Orientation.h"
#include "MyVoxel/Topology/Topology_Vertex.h"

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
    /// 空句柄

    const MyVoxel::Topology_Vertex empty;
    check(!empty.isValid(), "Default vertex invalid");
    check(empty.isNull(), "Default vertex null");
    check(!static_cast<bool>(empty), "Default vertex bool false");
    check(!empty.isSame(empty), "Null vertex has no topology identity");
    check(!empty.isForward(), "Invalid vertex not forward");
    check(!empty.isReversed(), "Invalid vertex not reversed");

    /// Geometry_Point支撑

    const MyMath::Vector3 position(1.0, -2.0, 3.5);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point> geometry(new MyVoxel::Geometry_Point(position));
    const MyVoxel::Topology_Vertex vertex(geometry);

    check(vertex.isValid(), "Vertex with geometry valid");
    check(!vertex.isNull(), "Valid vertex not null");
    check(static_cast<bool>(vertex), "Valid vertex bool true");
    check(vertex.isForward(), "New vertex defaults forward");
    check(vertex.orientation() == MyVoxel::Topology_Orientation::Forward, "New vertex orientation preserved");
    check(&vertex.geometry() == geometry.get(), "Vertex preserves exact geometry resource");
    check(vertex.geometry().position().isEqualTo(position, 0.0), "Vertex geometry preserves exact position");

    /// 句柄复制共享拓扑身份

    const MyVoxel::Topology_Vertex copied(vertex);
    check(copied.isValid(), "Copied vertex valid");
    check(copied.isSame(vertex), "Copied vertex shares topology identity");
    check(&copied.geometry() == &vertex.geometry(), "Copied vertex shares geometry resource");
    check(copied.orientation() == vertex.orientation(), "Copied vertex preserves orientation");

    MyVoxel::Topology_Vertex assigned;
    assigned = vertex;
    check(assigned.isValid(), "Assigned vertex valid");
    check(assigned.isSame(vertex), "Assigned vertex shares topology identity");
    check(&assigned.geometry() == &vertex.geometry(), "Assigned vertex shares geometry resource");

    /// 同一Geometry_Point可以支撑不同Topology_Vertex

    const MyVoxel::Topology_Vertex independentSameGeometry(geometry);
    check(independentSameGeometry.isValid(), "Independent same-geometry vertex valid");
    check(!independentSameGeometry.isSame(vertex), "Same geometry does not imply same topology identity");
    check(&independentSameGeometry.geometry() == &vertex.geometry(), "Independent vertices may share exact geometry resource");

    /// 相同位置的不同Geometry_Point同样不建立拓扑身份

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point> secondGeometry(new MyVoxel::Geometry_Point(position));
    const MyVoxel::Topology_Vertex secondVertex(secondGeometry);
    check(secondVertex.geometry().position().isEqualTo(vertex.geometry().position(), 0.0), "Different geometry resources may share coordinates");
    check(&secondVertex.geometry() != &vertex.geometry(), "Different geometry resources remain distinct");
    check(!secondVertex.isSame(vertex), "Equal coordinates do not imply same topology identity");

    /// 方向与拓扑身份分离

    const MyVoxel::Topology_Vertex reversed = vertex.reversed();
    check(reversed.isValid(), "Reversed vertex valid");
    check(reversed.isSame(vertex), "Reversed vertex preserves topology identity");
    check(reversed.isReversed(), "Reversed vertex flips orientation");
    check(&reversed.geometry() == &vertex.geometry(), "Reversed vertex preserves geometry resource");

    const MyVoxel::Topology_Vertex restored = reversed.reversed();
    check(restored.isSame(vertex), "Double reversed vertex preserves topology identity");
    check(restored.isForward(), "Double reversed vertex restores forward orientation");
    check(&restored.geometry() == &vertex.geometry(), "Double reversed vertex preserves geometry resource");

#ifdef NDEBUG
    /// Release下空Geometry_Point引用安全产生空Vertex；Debug断言直接暴露调用错误。

    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Point> nullGeometry;
    const MyVoxel::Topology_Vertex invalid(nullGeometry);
    check(invalid.isNull(), "Null geometry rejected");
    check(invalid.reversed().isNull(), "Reversing invalid vertex returns null");
#endif

    std::cout << "Topology_VertexV2 Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
