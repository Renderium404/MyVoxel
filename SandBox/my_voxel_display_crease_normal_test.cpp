#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Base/Display_Normal.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition){++g_passedCount; std::cout << "[PASS] " << name << std::endl;}
    else{++g_failedCount; std::cout << "[FAIL] " << name << std::endl;}
}

// 返回立方体角点使用的连续平滑源法线。
MyVoxel::Display_Normal cornerNormal(double x, double y, double z)
{
    MyMath::Vector3 normal(x, y, z);
    normal.normalize();
    return MyVoxel::Display_Normal(normal);
}

// 建立8个共享顶点、12个三角形的立方体，并故意给每个角点写入连续对角法线。
MyVoxel::Mesh buildSmoothNormalCube()
{
    MyVoxel::Mesh mesh;
    const MyVoxel::Display_Color color(0.55, 0.68, 0.88, 1.0);
    const std::uint32_t v0 = mesh.appendVertex(MyVoxel::MeshVertex(-1.0, -1.0, -1.0, cornerNormal(-1.0, -1.0, -1.0)));
    const std::uint32_t v1 = mesh.appendVertex(MyVoxel::MeshVertex( 1.0, -1.0, -1.0, cornerNormal( 1.0, -1.0, -1.0)));
    const std::uint32_t v2 = mesh.appendVertex(MyVoxel::MeshVertex( 1.0,  1.0, -1.0, cornerNormal( 1.0,  1.0, -1.0)));
    const std::uint32_t v3 = mesh.appendVertex(MyVoxel::MeshVertex(-1.0,  1.0, -1.0, cornerNormal(-1.0,  1.0, -1.0)));
    const std::uint32_t v4 = mesh.appendVertex(MyVoxel::MeshVertex(-1.0, -1.0,  1.0, cornerNormal(-1.0, -1.0,  1.0)));
    const std::uint32_t v5 = mesh.appendVertex(MyVoxel::MeshVertex( 1.0, -1.0,  1.0, cornerNormal( 1.0, -1.0,  1.0)));
    const std::uint32_t v6 = mesh.appendVertex(MyVoxel::MeshVertex( 1.0,  1.0,  1.0, cornerNormal( 1.0,  1.0,  1.0)));
    const std::uint32_t v7 = mesh.appendVertex(MyVoxel::MeshVertex(-1.0,  1.0,  1.0, cornerNormal(-1.0,  1.0,  1.0)));

    mesh.appendTriangle(v0, v2, v1, color); mesh.appendTriangle(v0, v3, v2, color); // -Z。
    mesh.appendTriangle(v4, v5, v6, color); mesh.appendTriangle(v4, v6, v7, color); // +Z。
    mesh.appendTriangle(v0, v1, v5, color); mesh.appendTriangle(v0, v5, v4, color); // -Y。
    mesh.appendTriangle(v3, v7, v6, color); mesh.appendTriangle(v3, v6, v2, color); // +Y。
    mesh.appendTriangle(v0, v4, v7, color); mesh.appendTriangle(v0, v7, v3, color); // -X。
    mesh.appendTriangle(v1, v2, v6, color); mesh.appendTriangle(v1, v6, v5, color); // +X。
    return mesh;
}

// 判断Display顶点法线是否严格接近某个坐标轴方向。
bool isAxisNormal(const MyVoxel::Display_MeshVertex& vertex)
{
    const double ax = std::fabs(static_cast<double>(vertex.normalX));
    const double ay = std::fabs(static_cast<double>(vertex.normalY));
    const double az = std::fabs(static_cast<double>(vertex.normalZ));
    const double maximum = (std::max)(ax, (std::max)(ay, az));
    const double sum = ax + ay + az;
    return maximum > 0.99999 && std::fabs(sum - 1.0) < 1.0e-5;
}

}

int main()
{
    std::cout << "MyVoxel crease-aware Display_MeshResource test" << std::endl << std::endl;

    const MyVoxel::Mesh cube = buildSmoothNormalCube();
    check(cube.isRenderable(), "Shared-vertex smooth-normal cube is renderable");

    const MyVoxel::Display_MeshResourceOptions sourceOptions;
    MyVoxel::Display_MeshResourceOptions creaseOptions;
    creaseOptions.normalMode = MyVoxel::Display_MeshNormalMode::CreaseAware;
    creaseOptions.creaseAngleDegrees = 30.0;
    MyVoxel::Display_MeshResourceOptions wideOptions;
    wideOptions.normalMode = MyVoxel::Display_MeshNormalMode::CreaseAware;
    wideOptions.creaseAngleDegrees = 120.0;

    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> source =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(cube, sourceOptions);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> crease =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(cube, creaseOptions);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> wide =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(cube, wideOptions);

    check(source->isValid() && crease->isValid() && wide->isValid(), "All display resources are valid");
    check(source->vertexCount() == 36 && crease->vertexCount() == 36, "Display resources keep triangle-corner expansion count");
    check(source->triangleCount() == cube.triangleCount() && crease->triangleCount() == cube.triangleCount(),
          "Crease processing preserves triangle count");
    check(source->localBounds().minimum().isEqualTo(crease->localBounds().minimum(), 0.0) &&
          source->localBounds().maximum().isEqualTo(crease->localBounds().maximum(), 0.0),
          "Crease processing preserves geometry bounds");

    bool allCreaseNormalsAreAxis = true;

    for (std::size_t index = 0; index < crease->vertices().size(); ++index)
    {
        allCreaseNormalsAreAxis = allCreaseNormalsAreAxis && isAxisNormal(crease->vertices()[index]);
    }

    check(allCreaseNormalsAreAxis, "30-degree crease mode splits cube display normals into six hard face directions");

    bool widePreservesSource = true;

    for (std::size_t index = 0; index < source->vertices().size(); ++index)
    {
        const MyVoxel::Display_MeshVertex& a = source->vertices()[index];
        const MyVoxel::Display_MeshVertex& b = wide->vertices()[index];
        widePreservesSource = widePreservesSource &&
            a.normalX == b.normalX && a.normalY == b.normalY && a.normalZ == b.normalZ;
    }

    check(widePreservesSource, "120-degree crease threshold keeps 90-degree cube faces in one smooth group");

    std::cout << std::endl << "Passed: " << g_passedCount << std::endl << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}