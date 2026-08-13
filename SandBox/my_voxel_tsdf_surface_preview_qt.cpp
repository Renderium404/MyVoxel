#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

#include <QApplication>
#include <QStatusBar>
#include <QString>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Object/Display_ObjectSnapshot.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"

#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

const double BaseVoxelEdgeLength = 4.0; // 第0层体素边长固定为4，用于直观比较Level 3~6最高层分辨率。
const double TsdfBandCellCount = 4.0; // 截断距离固定为四个最高层采样间距，保证Surface Nets附近具有完整TSDF窄带。
const double SphereRadius = 1.5; // 所有对比对象使用同一个解析球半径，确保只观察体素分辨率差异。
const double DisplaySpacing = 4.0; // 四个球沿X方向间隔4个世界单位摆放，避免观察时互相遮挡。

typedef std::chrono::steady_clock TestClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const TestClock::time_point& begin, const TestClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 保存一个Level对应的完整TSDF和Surface Nets统计。
struct SurfaceCaseStatistics
{
    SurfaceCaseStatistics()
        : level(0)
        , spacing(0.0)
        , backgroundDistance(0.0)
        , rootCount(0)
        , vertexCount(0)
        , triangleCount(0)
        , voxelizationMilliseconds(0.0)
        , meshingMilliseconds(0.0)
    {
    }

    int level; // 当前VoxelGrid最高层级。
    double spacing; // 当前最高层Cell-Centered TSDF采样间距。
    double backgroundDistance; // 当前TSDF截断背景距离B。
    std::size_t rootCount; // 当前稀疏TSDF显式Root数量。
    std::size_t vertexCount; // 当前Surface Nets共享顶点数量。
    std::size_t triangleCount; // 当前最终显示三角形数量。
    double voxelizationMilliseconds; // 当前解析球体素化耗时。
    double meshingMilliseconds; // 当前Surface Nets重建耗时。
};

// 根据已经完成的Surface Nets Mesh建立Viewer正式Display对象快照。
MyVoxel::Display_ObjectSnapshot makeMeshSnapshot(MyVoxel::Display_ObjectId objectId, MyVoxel::Display_ResourceId resourceId,
                                                 const MyVoxel::Mesh& mesh, const MyMath::Vector3& displayTranslation)
{
    MYVOXEL_ASSERT_MESSAGE(mesh.isRenderable() && !mesh.isEmpty(),
                           "TSDF Surface Viewer snapshot requires a non-empty renderable Mesh.");

    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> mutableResource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_Resource> resource = mutableResource;

    MyVoxel::Display_ObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.resourceKind = MyVoxel::Display_ResourceKind::Mesh;
    snapshot.usage = MyVoxel::Display_ObjectUsage::Static;
    snapshot.stateVersion = 1;
    snapshot.localToWorld = MyMath::Matrix4::fromTranslation(displayTranslation);
    snapshot.visible = true;
    snapshot.parts.push_back(MyVoxel::Display_ObjectPartSnapshot(0, 1, resourceId, resource));

    MYVOXEL_ASSERT_MESSAGE(snapshot.isValid(), "TSDF Surface Viewer produced an invalid Display_ObjectSnapshot.");
    return snapshot;
}

// 为一个指定Level建立解析球TSDF、Uniform Surface Nets和Viewer快照。
MyVoxel::Display_ObjectSnapshot buildSphereCase(int level, MyVoxel::Display_ObjectId objectId,
                                                MyVoxel::Display_ResourceId resourceId,
                                                const MyMath::Vector3& displayTranslation,
                                                const MyVoxel::Display_Color& color,
                                                SurfaceCaseStatistics& statistics)
{
    const MyVoxel::VoxelLevel maximumLevel = static_cast<MyVoxel::VoxelLevel>(level);
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), BaseVoxelEdgeLength, maximumLevel);
    const double spacing = grid.minimumCellEdgeLength();
    const float backgroundDistance = static_cast<float>(spacing * TsdfBandCellCount);
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    const MyVoxel::Shape sphere = MyVoxel::Modeling::makeSphere(SphereRadius);

    const TestClock::time_point voxelizationBegin = TestClock::now();
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(sphere, reference);
    const TestClock::time_point voxelizationEnd = TestClock::now();

    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = color;

    const TestClock::time_point meshingBegin = TestClock::now();
    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(voxels, options);
    const TestClock::time_point meshingEnd = TestClock::now();

    MYVOXEL_ASSERT_MESSAGE(voxels.isValid(), "TSDF Surface Viewer voxelization must produce a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(mesh.isRenderable() && !mesh.isEmpty(),
                           "TSDF Surface Viewer meshing must produce a non-empty renderable Mesh.");

    statistics.level = level;
    statistics.spacing = spacing;
    statistics.backgroundDistance = backgroundDistance;
    statistics.rootCount = voxels.rootCount();
    statistics.vertexCount = mesh.vertexCount();
    statistics.triangleCount = mesh.triangleCount();
    statistics.voxelizationMilliseconds = elapsedMilliseconds(voxelizationBegin, voxelizationEnd);
    statistics.meshingMilliseconds = elapsedMilliseconds(meshingBegin, meshingEnd);

    return makeMeshSnapshot(objectId, resourceId, mesh, displayTranslation);
}

// 输出单个Level的TSDF和Surface Nets规模。
void printStatistics(const SurfaceCaseStatistics& statistics)
{
    std::cout
        << "Level " << statistics.level
        << " | h=" << statistics.spacing
        << " | B=" << statistics.backgroundDistance
        << " | roots=" << statistics.rootCount
        << " | vertices=" << statistics.vertexCount
        << " | triangles=" << statistics.triangleCount
        << " | voxelize=" << statistics.voxelizationMilliseconds << " ms"
        << " | mesh=" << statistics.meshingMilliseconds << " ms"
        << std::endl;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel TSDF Surface Nets Viewer - Sphere Level 3 / 4 / 5 / 6"));
    window.resize(1600, 900);
    window.show();

    // 必须先让QOpenGLWidget建立GUI上下文和后台共享RenderThread，再提交正式Display快照。
    application.processEvents();

    const MyVoxel::Display_Color colors[4] =
    {
        MyVoxel::Display_Color(0.88, 0.48, 0.38, 1.0),
        MyVoxel::Display_Color(0.92, 0.70, 0.34, 1.0),
        MyVoxel::Display_Color(0.38, 0.72, 0.92, 1.0),
        MyVoxel::Display_Color(0.62, 0.52, 0.92, 1.0)
    };

    const double firstX = -DisplaySpacing * 1.5;
    std::vector<SurfaceCaseStatistics> statistics(4);

    for (int caseIndex = 0; caseIndex < 4; ++caseIndex)
    {
        const int level = caseIndex + 3;
        const MyVoxel::Display_ObjectId objectId = static_cast<MyVoxel::Display_ObjectId>(caseIndex + 1);
        const MyVoxel::Display_ResourceId resourceId = static_cast<MyVoxel::Display_ResourceId>(caseIndex + 1);
        const MyMath::Vector3 translation(firstX + DisplaySpacing * static_cast<double>(caseIndex), 0.0, 0.0);
        const MyVoxel::Display_ObjectSnapshot snapshot =
            buildSphereCase(level, objectId, resourceId, translation, colors[caseIndex], statistics[caseIndex]);

        if (!snapshot.isValid() || !window.submitDisplaySnapshot(snapshot))
        {
            std::cerr << "[FAIL] Viewer submission failed for Level " << level << std::endl;
            return EXIT_FAILURE;
        }

        printStatistics(statistics[caseIndex]);
    }

    // 当前Viewer正式路径负责场景包围盒、相机和后台GPU资源；这里只在全部对象提交后执行一次Fit。
    application.processEvents();
    window.viewer()->setIsometricView();
    window.viewer()->fitAll();


    return application.exec();
}