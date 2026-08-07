#include <cstdlib>
#include <iostream>

#include <QApplication>

#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Base/Display_Normal.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Display/Mesh/Display_MeshUpdate.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"

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

MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> buildResource()
{
    MyVoxel::Mesh mesh;
    const MyVoxel::Display_Normal normal(0.0, 0.0, 1.0);
    const MyVoxel::Display_Color color(0.3, 0.6, 0.9, 1.0);
    const std::uint32_t index0 = mesh.appendVertex(MyVoxel::MeshVertex(0.0, 0.0, 0.0, normal));
    const std::uint32_t index1 = mesh.appendVertex(MyVoxel::MeshVertex(1.0, 0.0, 0.0, normal));
    const std::uint32_t index2 = mesh.appendVertex(MyVoxel::MeshVertex(0.0, 1.0, 0.0, normal));
    mesh.appendTriangle(index0, index1, index2, color);

    MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> mutableResource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    return MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource>(mutableResource);
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    MyVoxelViewer::VoxelOpenGLWidget viewer;
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> resource = buildResource();

    MyVoxel::Display_MeshObjectSnapshot snapshot;
    snapshot.objectId = 1;
    snapshot.usage = MyVoxel::Display_MeshUsage::Dynamic;
    snapshot.parts.push_back(MyVoxel::Display_MeshPartSnapshot(10, 1, resource));

    check(viewer.submitDisplaySnapshot(snapshot), "submit full display snapshot");
    check(viewer.containsDisplayObject(1), "viewer contains display object");
    check(viewer.displayObjectCount() == 1, "display object count");
    check(viewer.displayObjectPartCount(1) == 1, "display part count");
    check(viewer.displayObjectTriangleCount(1) == 1, "display triangle count");

    MyVoxel::Display_MeshUpdate replacement;
    replacement.objectId = 1;
    replacement.parts.push_back(MyVoxel::Display_MeshPartUpdate::replacement(10, 2, resource));
    check(viewer.submitDisplayUpdate(replacement), "submit display replacement");
    check(viewer.lastUpdatedMeshPartCount() == 1, "replacement accepted");

    MyVoxel::Display_MeshUpdate stale;
    stale.objectId = 1;
    stale.parts.push_back(MyVoxel::Display_MeshPartUpdate::replacement(10, 1, resource));
    check(viewer.submitDisplayUpdate(stale), "stale update handled");
    check(viewer.lastUpdatedMeshPartCount() == 0, "stale version ignored");

    MyVoxel::Display_MeshStateUpdate state;
    state.objectId = 1;
    state.hasVisible = true;
    state.visible = false;
    check(viewer.submitDisplayStateUpdate(state), "submit display state");
    check(!viewer.isDisplayObjectVisible(1), "visibility updated");

    MyVoxel::Display_MeshUpdate removal;
    removal.objectId = 1;
    removal.parts.push_back(MyVoxel::Display_MeshPartUpdate::removal(10, 3));
    check(viewer.submitDisplayUpdate(removal), "submit display removal");
    check(viewer.displayObjectPartCount(1) == 0, "display part removed");
    check(viewer.displayObjectTriangleCount(1) == 0, "removed object triangle count");

    check(viewer.removeDisplayObject(1), "remove display object");
    check(viewer.displayObjectCount() == 0, "viewer scene empty");

    std::cout << "Passed " << g_passedCount << " / Failed " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
