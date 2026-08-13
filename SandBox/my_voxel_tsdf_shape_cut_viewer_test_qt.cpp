#include <chrono>
#include <cstdlib>
#include <iostream>
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
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Operation/BooleanOperation.h"

#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

const double BaseVoxelEdgeLength = 4.0; // 第0层体素边长固定为4，当前窗口测试只比较切削结果，不改变网格基准。
const MyVoxel::VoxelLevel MaximumLevel = static_cast<MyVoxel::VoxelLevel>(5); // 观察切削曲面时固定使用Level 5，最高层采样间距为0.125。
const float BackgroundDistance = 0.75f; // TSDF截断距离固定为0.75，对应Level 5下六个最高层采样间距。
const double WorkpieceSize = 3.0; // 四组测试统一使用3×3×3长方体工件。
const double DisplaySpacing = 4.2; // 四个结果沿X方向间隔4.2世界单位，避免相邻工件互相遮挡。

typedef std::chrono::steady_clock TestClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const TestClock::time_point& begin, const TestClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 保存一个窗口案例的切削和网格重建统计。
struct CutCaseStatistics
{
    CutCaseStatistics()
        : cutMilliseconds(0.0)
        , meshingMilliseconds(0.0)
        , rootCount(0)
        , vertexCount(0)
        , triangleCount(0)
    {
    }

    QString name; // 当前窗口案例名称。
    double cutMilliseconds; // 当前案例全部连续Shape切削耗时。
    double meshingMilliseconds; // 当前案例Surface Nets完整重建耗时。
    std::size_t rootCount; // 当前切削后VoxelShape显式Root数量。
    std::size_t vertexCount; // 当前Surface Nets共享顶点数量。
    std::size_t triangleCount; // 当前Surface Nets三角形数量。
};

// 将一个已经可渲染的Mesh转换为Viewer正式Display对象快照。
MyVoxel::Display_ObjectSnapshot makeMeshSnapshot(MyVoxel::Display_ObjectId objectId, MyVoxel::Display_ResourceId resourceId,
                                                 const MyVoxel::Mesh& mesh, const MyMath::Vector3& displayTranslation)
{
    MYVOXEL_ASSERT_MESSAGE(mesh.isRenderable() && !mesh.isEmpty(),
                           "TSDF Shape cut Viewer requires a non-empty renderable Mesh.");

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

    MYVOXEL_ASSERT_MESSAGE(snapshot.isValid(), "TSDF Shape cut Viewer produced an invalid Display_ObjectSnapshot.");
    return snapshot;
}

// 根据指定颜色重建Surface Nets并建立Viewer快照。
MyVoxel::Display_ObjectSnapshot buildDisplaySnapshot(const MyVoxel::VoxelShape& voxels, const QString& caseName,
                                                     const MyVoxel::Display_Color& color,
                                                     MyVoxel::Display_ObjectId objectId,
                                                     MyVoxel::Display_ResourceId resourceId,
                                                     const MyMath::Vector3& displayTranslation,
                                                     CutCaseStatistics& statistics)
{
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = color;

    const TestClock::time_point meshingBegin = TestClock::now();
    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(voxels, options);
    const TestClock::time_point meshingEnd = TestClock::now();

    MYVOXEL_ASSERT_MESSAGE(mesh.isRenderable() && !mesh.isEmpty(),
                           "TSDF Shape cut Viewer meshing must produce a renderable mesh.");

    statistics.name = caseName;
    statistics.meshingMilliseconds = elapsedMilliseconds(meshingBegin, meshingEnd);
    statistics.rootCount = voxels.rootCount();
    statistics.vertexCount = mesh.vertexCount();
    statistics.triangleCount = mesh.triangleCount();

    return makeMeshSnapshot(objectId, resourceId, mesh, displayTranslation);
}

// 在工件中使用平移球体切出一个明显的侧向球形凹坑。
bool applySphereCut(MyVoxel::VoxelShape& voxels)
{
    const MyMath::Matrix4 placement =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(1.15, 0.0, 0.15)); // 球心向+X移动，使球体穿过右侧面形成可直接观察的球形凹坑。
    const MyVoxel::Shape tool = MyVoxel::Modeling::makeSphere(1.0, placement);
    return MyVoxel::Operation::ShapeCutOperation::subtractInPlace(voxels, tool);
}

// 沿Y方向放置圆柱并从工件前后贯穿，形成圆形通孔。
bool applyCylinderCut(MyVoxel::VoxelShape& voxels)
{
    const MyMath::Matrix4 placement(
        1.0, 0.0, 0.0, -0.55,
        0.0, 0.0, -1.0, 0.0,
        0.0, 1.0, 0.0, -0.25,
        0.0, 0.0, 0.0, 1.0); // 标准圆柱轴沿局部Z轴；该右手矩阵绕X旋转90度并平移到(-0.55,0,-0.25)，形成沿Y方向贯穿工件的通孔。
    const MyVoxel::Shape tool = MyVoxel::Modeling::makeCylinder(0.52, 4.5, placement);
    return MyVoxel::Operation::ShapeCutOperation::subtractInPlace(voxels, tool);
}

// 输出一个窗口案例的切削和显示网格统计。
void printStatistics(const CutCaseStatistics& statistics)
{
    std::cout
        << statistics.name.toStdString()
        << " | roots=" << statistics.rootCount
        << " | vertices=" << statistics.vertexCount
        << " | triangles=" << statistics.triangleCount
        << " | cut=" << statistics.cutMilliseconds << " ms"
        << " | mesh=" << statistics.meshingMilliseconds << " ms"
        << std::endl;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), BaseVoxelEdgeLength, MaximumLevel);
    const MyVoxel::VoxelShape reference(grid, BackgroundDistance);
    const MyVoxel::Shape workpieceShape = MyVoxel::Modeling::makeBox(WorkpieceSize, WorkpieceSize, WorkpieceSize);

    const TestClock::time_point voxelizationBegin = TestClock::now();
    const MyVoxel::VoxelShape original = MyVoxel::Modeling::voxelizeAligned(workpieceShape, reference);
    const double voxelizationMilliseconds = elapsedMilliseconds(voxelizationBegin, TestClock::now());

    MYVOXEL_ASSERT_MESSAGE(original.isValid() && !original.isEmpty(),
                           "TSDF Shape cut Viewer requires a valid non-empty voxelized workpiece.");

    MyVoxel::VoxelShape sphereCut = original;
    MyVoxel::VoxelShape cylinderCut = original;
    MyVoxel::VoxelShape combinedCut = original;

    CutCaseStatistics statistics[4];
    statistics[0].name = QStringLiteral("Original");

    TestClock::time_point cutBegin = TestClock::now();
    const bool sphereChanged = applySphereCut(sphereCut);
    statistics[1].cutMilliseconds = elapsedMilliseconds(cutBegin, TestClock::now());

    cutBegin = TestClock::now();
    const bool cylinderChanged = applyCylinderCut(cylinderCut);
    statistics[2].cutMilliseconds = elapsedMilliseconds(cutBegin, TestClock::now());

    cutBegin = TestClock::now();
    const bool combinedSphereChanged = applySphereCut(combinedCut);
    const bool combinedCylinderChanged = applyCylinderCut(combinedCut);
    statistics[3].cutMilliseconds = elapsedMilliseconds(cutBegin, TestClock::now());

    if (!sphereChanged || !cylinderChanged || !combinedSphereChanged || !combinedCylinderChanged)
    {
        std::cerr << "[FAIL] TSDF Shape cut Viewer expected every cut operation to modify its workpiece."
                  << " sphere=" << sphereChanged
                  << " cylinder=" << cylinderChanged
                  << " combinedSphere=" << combinedSphereChanged
                  << " combinedCylinder=" << combinedCylinderChanged
                  << std::endl;
        return EXIT_FAILURE;
    }

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel TSDF Continuous Shape Cut Viewer"));
    window.resize(1680, 920);
    window.show();

    // 先建立GUI OpenGL上下文和后台共享RenderThread，再提交正式Display快照。
    application.processEvents();

    const MyVoxel::Display_Color colors[4] =
    {
        MyVoxel::Display_Color(0.62, 0.68, 0.78, 1.0),
        MyVoxel::Display_Color(0.90, 0.58, 0.38, 1.0),
        MyVoxel::Display_Color(0.36, 0.72, 0.92, 1.0),
        MyVoxel::Display_Color(0.66, 0.50, 0.92, 1.0)
    };

    const double firstX = -DisplaySpacing * 1.5;
    const MyVoxel::VoxelShape* cases[4] = {&original, &sphereCut, &cylinderCut, &combinedCut};
    const QString names[4] =
    {
        QStringLiteral("Original"),
        QStringLiteral("Sphere Cut"),
        QStringLiteral("Cylinder Cut"),
        QStringLiteral("Combined Cut")
    };

    for (int caseIndex = 0; caseIndex < 4; ++caseIndex)
    {
        const MyVoxel::Display_ObjectId objectId = static_cast<MyVoxel::Display_ObjectId>(caseIndex + 1);
        const MyVoxel::Display_ResourceId resourceId = static_cast<MyVoxel::Display_ResourceId>(caseIndex + 1);
        const MyMath::Vector3 translation(firstX + DisplaySpacing * static_cast<double>(caseIndex), 0.0, 0.0);

        MyVoxel::Display_ObjectSnapshot snapshot =
            buildDisplaySnapshot(*cases[caseIndex], names[caseIndex], colors[caseIndex],
                                 objectId, resourceId, translation, statistics[caseIndex]);

        if (!window.submitDisplaySnapshot(snapshot))
        {
            std::cerr << "[FAIL] Viewer submission failed for "
                      << names[caseIndex].toStdString() << std::endl;
            return EXIT_FAILURE;
        }

        printStatistics(statistics[caseIndex]);
    }

    std::cout
        << "Voxelization"
        << " | level=" << static_cast<int>(MaximumLevel)
        << " | h=" << grid.minimumCellEdgeLength()
        << " | B=" << BackgroundDistance
        << " | time=" << voxelizationMilliseconds << " ms"
        << std::endl;

    application.processEvents();
    window.viewer()->setIsometricView();
    window.viewer()->fitAll();


    return application.exec();
}