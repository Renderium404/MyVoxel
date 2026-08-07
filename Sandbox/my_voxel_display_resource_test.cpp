#include <iostream>

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Base/Display_Normal.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Display/Mesh/Display_MeshUpdate.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Mesh/Mesh.h"

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

MyVoxel::Mesh buildTriangleMesh()
{
    MyVoxel::Mesh mesh;
    const MyVoxel::Display_Color color(0.2, 0.4, 0.8, 1.0);
    const MyVoxel::Display_Normal normal(0.0, 0.0, 1.0);
    const std::uint32_t index0 = mesh.appendVertex(MyVoxel::MeshVertex(0.0, 0.0, 0.0, normal));
    const std::uint32_t index1 = mesh.appendVertex(MyVoxel::MeshVertex(1.0, 0.0, 0.0, normal));
    const std::uint32_t index2 = mesh.appendVertex(MyVoxel::MeshVertex(0.0, 1.0, 0.0, normal));
    mesh.appendTriangle(index0, index1, index2, color);
    return mesh;
}

}

int main()
{
    const MyVoxel::Display_Color color = MyVoxel::Display_Color::redColor().withAlpha(0.5);
    check(color.isValid() && color.red() == 1.0f && color.alpha() == 0.5f, "Display color");

    const MyVoxel::Display_Normal normal(0.0, 0.0, 1.0);
    check(normal.isUnit(), "Display normal");

    const MyVoxel::Mesh mesh = buildTriangleMesh();
    check(mesh.isValid() && mesh.isRenderable(), "Mesh display contract");
    check(mesh.vertices()[0].normal == normal, "Mesh uses Display_Normal");
    check(mesh.triangleColor(0) == MyVoxel::Display_Color(0.2, 0.4, 0.8, 1.0), "Mesh uses Display_Color");

    MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> mutableResource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> resource = mutableResource;

    check(resource->isValid() && resource->vertexCount() == 3 && resource->triangleCount() == 1,
          "Display mesh resource");

    MyVoxel::Display_MeshObjectSnapshot snapshot;
    snapshot.objectId = 1;
    snapshot.parts.push_back(MyVoxel::Display_MeshPartSnapshot(0, 1, resource));
    check(snapshot.isValid() && snapshot.triangleCount() == 1 && snapshot.localBounds().isValid(),
          "Display full snapshot");

    MyVoxel::Display_MeshUpdate replacement;
    replacement.objectId = 1;
    replacement.parts.push_back(MyVoxel::Display_MeshPartUpdate::replacement(0, 2, resource));
    check(replacement.isValid(), "Display incremental replacement");

    MyVoxel::Display_MeshUpdate removal;
    removal.objectId = 1;
    removal.parts.push_back(MyVoxel::Display_MeshPartUpdate::removal(0, 3));
    check(removal.isValid(), "Display incremental removal");

    MyVoxel::Display_MeshStateUpdate state;
    state.objectId = 1;
    state.hasVisible = true;
    state.visible = false;
    check(state.isValid(), "Display state update");

    std::cout << "Passed " << passedCount << " / Failed " << failedCount << std::endl;
    return failedCount == 0 ? 0 : 1;
}
