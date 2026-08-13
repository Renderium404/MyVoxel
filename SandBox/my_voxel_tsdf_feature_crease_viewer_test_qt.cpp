#include <chrono>
#include <cstdlib>
#include <iostream>

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
#include "MyVoxel/Operation/BooleanOperation.h"

#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

const double BaseVoxelEdgeLength = 4.0; // 当前效果A/B测试沿用既有体素基准。
const MyVoxel::VoxelLevel MaximumLevel = static_cast<MyVoxel::VoxelLevel>(5); // 固定Level 5，只比较几何和显示法线效果。
const float BackgroundDistance = 0.75f; // 固定TSDF截断距离。
const double WorkpieceSize = 3.0; // 原始工件尺寸。
const double DisplaySpacing = 4.2; // 四个A/B对象之间的显示间距。
const double FeatureAngleDegrees = 30.0; // QEF特征检测和显示硬边统一先使用30度阈值。

typedef std::chrono::steady_clock TestClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const TestClock::time_point& begin, const TestClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 将Mesh转换为Viewer正式Display快照，可选择是否启用30度硬边显示法线。
MyVoxel::Display_ObjectSnapshot makeSnapshot(MyVoxel::Display_ObjectId objectId, MyVoxel::Display_ResourceId resourceId,
                                             const MyVoxel::Mesh& mesh, const MyMath::Vector3& translation, bool creaseAware)
{
    MyVoxel::Display_MeshResourceOptions resourceOptions;

    if (creaseAware)
    {
        resourceOptions.normalMode = MyVoxel::Display_MeshNormalMode::CreaseAware;
        resourceOptions.creaseAngleDegrees = FeatureAngleDegrees;
    }

    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> mutableResource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh, resourceOptions);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_Resource> resource = mutableResource;

    MyVoxel::Display_ObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.resourceKind = MyVoxel::Display_ResourceKind::Mesh;
    snapshot.usage = MyVoxel::Display_ObjectUsage::Static;
    snapshot.stateVersion = 1;
    snapshot.localToWorld = MyMath::Matrix4::fromTranslation(translation);
    snapshot.visible = true;
    snapshot.parts.push_back(MyVoxel::Display_ObjectPartSnapshot(0, 1, resourceId, resource));
    return snapshot;
}

// 执行球窝切削。
bool applySphereCut(MyVoxel::VoxelShape& voxels)
{
    const MyVoxel::Shape tool = MyVoxel::Modeling::makeSphere(
        1.0, MyMath::Matrix4::fromTranslation(MyMath::Vector3(1.15, 0.0, 0.15)));
    return MyVoxel::Operation::ShapeCutOperation::subtractInPlace(voxels, tool);
}

// 执行沿Y方向贯穿的圆柱切削。
bool applyCylinderCut(MyVoxel::VoxelShape& voxels)
{
    const MyMath::Matrix4 placement(
        1.0, 0.0, 0.0, -0.55,
        0.0, 0.0, -1.0, 0.0,
        0.0, 1.0, 0.0, -0.25,
        0.0, 0.0, 0.0, 1.0); // 标准圆柱轴沿Z，该矩阵绕X旋转90度并平移。
    const MyVoxel::Shape tool = MyVoxel::Modeling::makeCylinder(0.52, 4.5, placement);
    return MyVoxel::Operation::ShapeCutOperation::subtractInPlace(voxels, tool);
}

// 使用指定Surface Nets模式构建网格并输出统计。
MyVoxel::Mesh buildMesh(const MyVoxel::VoxelShape& voxels, MyVoxel::Meshing::VolumeMeshingMode mode, const char* name)
{
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = MyVoxel::Display_Color(0.70, 0.74, 0.80, 1.0);
    options.mode = mode;
    options.featureAngleDegrees = FeatureAngleDegrees;

    const TestClock::time_point begin = TestClock::now();
    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(voxels, options);
    const double milliseconds = elapsedMilliseconds(begin, TestClock::now());

    std::cout << name << " | vertices=" << mesh.vertexCount() << " | triangles=" << mesh.triangleCount()
              << " | mesh=" << milliseconds << " ms" << std::endl;
    return mesh;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), BaseVoxelEdgeLength, MaximumLevel);
    const MyVoxel::VoxelShape reference(grid, BackgroundDistance);
    const MyVoxel::VoxelShape original =
        MyVoxel::Modeling::voxelizeAligned(MyVoxel::Modeling::makeBox(WorkpieceSize, WorkpieceSize, WorkpieceSize), reference);
    MyVoxel::VoxelShape combined = original;
    const bool sphereChanged = applySphereCut(combined);
    const bool cylinderChanged = applyCylinderCut(combined);

    if (!sphereChanged || !cylinderChanged)
    {
        std::cerr << "[FAIL] Combined cut did not modify the workpiece." << std::endl;
        return EXIT_FAILURE;
    }

    const MyVoxel::Mesh originalStandard =
        buildMesh(original, MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets, "Original Standard");
    const MyVoxel::Mesh originalFeature =
        buildMesh(original, MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets, "Original Feature");
    const MyVoxel::Mesh cutStandard =
        buildMesh(combined, MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets, "Combined Standard");
    const MyVoxel::Mesh cutFeature =
        buildMesh(combined, MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets, "Combined Feature");

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel Feature QEF + Crease-Aware Normals A/B"));
    window.resize(1680, 920);
    window.show();
    application.processEvents();

    const MyVoxel::Mesh* meshes[4] = {&originalStandard, &originalFeature, &cutStandard, &cutFeature};
    const bool creaseAware[4] = {false, true, false, true};
    const double firstX = -DisplaySpacing * 1.5;

    for (int index = 0; index < 4; ++index)
    {
        const MyVoxel::Display_ObjectSnapshot snapshot =
            makeSnapshot(static_cast<MyVoxel::Display_ObjectId>(index + 1),
                         static_cast<MyVoxel::Display_ResourceId>(index + 1),
                         *meshes[index],
                         MyMath::Vector3(firstX + DisplaySpacing * static_cast<double>(index), 0.0, 0.0),
                         creaseAware[index]);

        if (!window.submitDisplaySnapshot(snapshot))
        {
            std::cerr << "[FAIL] Viewer rejected A/B snapshot " << index << std::endl;
            return EXIT_FAILURE;
        }
    }

    application.processEvents();
    window.viewer()->setIsometricView();
    window.viewer()->fitAll();

    return application.exec();
}