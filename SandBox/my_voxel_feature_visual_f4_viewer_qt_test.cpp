#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <QAction>
#include <QApplication>
#include <QKeySequence>
#include <QStatusBar>
#include <QString>
#include <QToolBar>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Object/Display_ObjectSnapshot.h"
#include "MyVoxel/Display/Object/Display_ObjectUpdate.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

const double BaseVoxelEdgeLength = 4.0; // F4视觉验证继续使用已经通过测试的第0层体素边长4.0。
const MyVoxel::VoxelLevel MaximumLevel = static_cast<MyVoxel::VoxelLevel>(5); // 固定Level 5，对应最高层TSDF采样间距0.125。
const float BackgroundDistance = 0.75f; // TSDF截断距离固定为0.75，对应六个最高层采样间距。
const double FeatureAngleDegrees = 30.0; // QEF特征判断与CreaseAware显示法线统一使用30度。
const double FeatureSnapDistanceScale = 0.75; // 显式Feature约束距离沿用F4自动测试已经验证的0.75h。
const double ColumnSpacing = 4.8; // 三种Meshing模式沿X方向间隔4.8。
const double RowSpacing = 4.8; // 三种解析Shape沿Y方向间隔4.8。
const double FeatureCrossHalfSize = 0.09; // Box真实尖角使用0.18总宽度的小十字标记。
const unsigned int FullCircleDisplaySegmentCount = 128; // Cylinder完整圆Feature仅为显示离散成128段。
const double TwoPi = 6.283185307179586476925286766559; // 完整圆对应弧度。

typedef std::chrono::steady_clock TestClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const TestClock::time_point& begin, const TestClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 当前F4原始解析体只需要区分Box真实角点和Cylinder完整圆seam。
bool isDisplayedFeatureVertex(const MyVoxel::VoxelFeatureSet& features, const MyVoxel::Topology_Vertex& vertex)
{
    std::vector<MyVoxel::Topology_Edge> incidentEdges;
    features.incidentEdges(vertex, incidentEdges);

    if (incidentEdges.empty() || incidentEdges.size() >= 2)
    {
        return true;
    }

    const MyVoxel::Topology_Edge& edge = incidentEdges[0];
    return !(edge.startVertex().isSame(vertex) && edge.endVertex().isSame(vertex));
}

// 将真正用于Mesher约束的VoxelShape.features()直接展开为GL_LINES显示数据。
std::vector<MyMath::Vector3> buildFeatureLinePoints(const MyVoxel::VoxelShape& voxels)
{
    std::vector<MyMath::Vector3> points;

    if (!voxels.hasCompleteFeatures() || voxels.features().isEmpty())
    {
        return points;
    }

    const MyVoxel::VoxelFeatureSet& features = voxels.features();

    for (std::size_t edgeIndex = 0; edgeIndex < features.edges().size(); ++edgeIndex)
    {
        const MyVoxel::Topology_Edge& edge = features.edges()[edgeIndex];
        unsigned int segmentCount = 1;

        if (edge.geometry().kind() == MyVoxel::CurveKind::Arc)
        {
            const MyVoxel::Geometry_Arc& arc = static_cast<const MyVoxel::Geometry_Arc&>(edge.geometry());
            segmentCount = (std::max)(1U, static_cast<unsigned int>(std::ceil(
                std::fabs(arc.sweepAngle()) * static_cast<double>(FullCircleDisplaySegmentCount) / TwoPi)));
        }

        for (unsigned int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
        {
            points.push_back(edge.pointAt(static_cast<double>(segmentIndex) / static_cast<double>(segmentCount)));
            points.push_back(edge.pointAt(static_cast<double>(segmentIndex + 1) / static_cast<double>(segmentCount)));
        }
    }

    // 只给真实几何尖角增加三轴十字；Cylinder自环seam Vertex不会显示成尖角。
    for (std::size_t vertexIndex = 0; vertexIndex < features.vertices().size(); ++vertexIndex)
    {
        const MyVoxel::Topology_Vertex& vertex = features.vertices()[vertexIndex];

        if (!isDisplayedFeatureVertex(features, vertex))
        {
            continue;
        }

        const MyMath::Vector3& point = vertex.point();
        points.push_back(point + MyMath::Vector3(-FeatureCrossHalfSize, 0.0, 0.0)); points.push_back(point + MyMath::Vector3(FeatureCrossHalfSize, 0.0, 0.0));
        points.push_back(point + MyMath::Vector3(0.0, -FeatureCrossHalfSize, 0.0)); points.push_back(point + MyMath::Vector3(0.0, FeatureCrossHalfSize, 0.0));
        points.push_back(point + MyMath::Vector3(0.0, 0.0, -FeatureCrossHalfSize)); points.push_back(point + MyMath::Vector3(0.0, 0.0, FeatureCrossHalfSize));
    }

    return points;
}

// 三列统一使用CreaseAware显示法线，因此窗口主要比较Meshing几何位置差异。
MyVoxel::Display_ObjectSnapshot makeMeshSnapshot(MyVoxel::Display_ObjectId objectId, MyVoxel::Display_ResourceId resourceId,
                                                 const MyVoxel::Mesh& mesh, const MyMath::Vector3& translation)
{
    MyVoxel::Display_MeshResourceOptions resourceOptions;
    resourceOptions.normalMode = MyVoxel::Display_MeshNormalMode::CreaseAware;
    resourceOptions.creaseAngleDegrees = FeatureAngleDegrees;

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
    MYVOXEL_ASSERT_MESSAGE(snapshot.isValid(), "F4 visual viewer produced an invalid Mesh snapshot.");
    return snapshot;
}

// 建立Constrained列使用的真实Feature Line显示对象。
MyVoxel::Display_ObjectSnapshot makeFeatureSnapshot(MyVoxel::Display_ObjectId objectId, MyVoxel::Display_ResourceId resourceId,
                                                    const MyVoxel::VoxelShape& voxels, const MyMath::Vector3& translation)
{
    const std::vector<MyMath::Vector3> points = buildFeatureLinePoints(voxels);
    MYVOXEL_ASSERT_MESSAGE(points.size() >= 2 && (points.size() % 2) == 0, "F4 Feature overlay requires valid GL_LINES points.");

    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_LineResource> mutableResource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_LineResource>(points, MyVoxel::Display_Color(1.0, 0.24, 0.04, 1.0));
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_Resource> resource = mutableResource;

    MyVoxel::Display_ObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.resourceKind = MyVoxel::Display_ResourceKind::Line;
    snapshot.usage = MyVoxel::Display_ObjectUsage::Static;
    snapshot.stateVersion = 1;
    snapshot.localToWorld = MyMath::Matrix4::fromTranslation(translation);
    snapshot.visible = true;
    snapshot.lineWidth = 3.0f; // 显式Feature固定使用3像素橙色线，便于与W三角线框区分。
    snapshot.parts.push_back(MyVoxel::Display_ObjectPartSnapshot(0, 1, resourceId, resource));
    MYVOXEL_ASSERT_MESSAGE(snapshot.isValid(), "F4 visual viewer produced an invalid Feature snapshot.");
    return snapshot;
}

// 使用指定模式建立网格并输出窗口对照统计。
MyVoxel::Mesh buildMesh(const MyVoxel::VoxelShape& voxels, MyVoxel::Meshing::VolumeMeshingMode mode,
                        const MyVoxel::Display_Color& color, const char* shapeName, const char* modeName)
{
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = color;
    options.mode = mode;
    options.featureAngleDegrees = FeatureAngleDegrees;
    options.featureSnapDistanceScale = FeatureSnapDistanceScale;

    const TestClock::time_point begin = TestClock::now();
    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(voxels, options);
    const double milliseconds = elapsedMilliseconds(begin, TestClock::now());
    std::cout << shapeName << " | " << modeName << " | vertices=" << mesh.vertexCount()
              << " | triangles=" << mesh.triangleCount() << " | mesh=" << milliseconds << " ms" << std::endl;
    return mesh;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), BaseVoxelEdgeLength, MaximumLevel);
    const MyVoxel::VoxelShape reference(grid, BackgroundDistance);
    const MyVoxel::Shape shapes[3] =
    {
        MyVoxel::Modeling::makeBox(3.0, 3.0, 3.0),
        MyVoxel::Modeling::makeCylinder(1.30, 3.0),
        MyVoxel::Modeling::makeSphere(1.55)
    };
    const char* shapeNames[3] = {"Box", "Cylinder", "Sphere"};
    const MyVoxel::Display_Color shapeColors[3] =
    {
        MyVoxel::Display_Color(0.62, 0.70, 0.82, 1.0),
        MyVoxel::Display_Color(0.78, 0.66, 0.50, 1.0),
        MyVoxel::Display_Color(0.56, 0.73, 0.66, 1.0)
    };
    const MyVoxel::Meshing::VolumeMeshingMode modes[3] =
    {
        MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets,
        MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets,
        MyVoxel::Meshing::VolumeMeshingMode::FeatureConstrainedSurfaceNets
    };
    const char* modeNames[3] = {"Standard", "FeatureSensitive QEF", "FeatureConstrained"};
    MyVoxel::VoxelShape voxels[3] =
    {
        MyVoxel::Modeling::voxelizeAligned(shapes[0], reference),
        MyVoxel::Modeling::voxelizeAligned(shapes[1], reference),
        MyVoxel::Modeling::voxelizeAligned(shapes[2], reference)
    };

    MYVOXEL_ASSERT_MESSAGE(voxels[0].hasCompleteFeatures() && voxels[0].features().vertexCount() == 8 && voxels[0].features().edgeCount() == 12,
                           "F4 visual Box requires complete 8-vertex 12-edge Features.");
    MYVOXEL_ASSERT_MESSAGE(voxels[1].hasCompleteFeatures() && voxels[1].features().vertexCount() == 2 && voxels[1].features().edgeCount() == 2,
                           "F4 visual Cylinder requires two complete circular Features.");
    MYVOXEL_ASSERT_MESSAGE(voxels[2].hasCompleteFeatures() && voxels[2].features().isEmpty(),
                           "F4 visual Sphere requires a complete intentionally empty FeatureSet.");

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel F4 Explicit Feature-Constrained Surface Nets Visual Validation"));
    window.resize(1720, 980);
    window.show();
    application.processEvents(); // 先建立GUI OpenGL上下文和后台共享RenderThread，再提交正式Display快照。

    MyVoxel::Display_ObjectId meshObjectId = 1;
    MyVoxel::Display_ResourceId meshResourceId = 1;

    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            const MyVoxel::Mesh mesh = buildMesh(voxels[row], modes[column], shapeColors[row], shapeNames[row], modeNames[column]);
            const MyMath::Vector3 translation((static_cast<double>(column) - 1.0) * ColumnSpacing,
                                              (1.0 - static_cast<double>(row)) * RowSpacing, 0.0);

            if (!window.submitDisplaySnapshot(makeMeshSnapshot(meshObjectId, meshResourceId, mesh, translation)))
            {
                std::cerr << "[FAIL] Viewer rejected " << shapeNames[row] << " / " << modeNames[column] << "." << std::endl;
                return EXIT_FAILURE;
            }

            ++meshObjectId;
            ++meshResourceId;
        }
    }

    // 显式Feature只叠加到右侧Constrained列；Sphere没有Feature，因此实际只提交Box和Cylinder两个Line对象。
    std::vector<MyVoxel::Display_ObjectId> overlayObjectIds;

    for (int row = 0; row < 2; ++row)
    {
        const MyVoxel::Display_ObjectId objectId = static_cast<MyVoxel::Display_ObjectId>(101 + row);
        const MyVoxel::Display_ResourceId resourceId = static_cast<MyVoxel::Display_ResourceId>(101 + row);
        const MyMath::Vector3 translation(ColumnSpacing, (1.0 - static_cast<double>(row)) * RowSpacing, 0.0);

        if (!window.submitDisplaySnapshot(makeFeatureSnapshot(objectId, resourceId, voxels[row], translation)))
        {
            std::cerr << "[FAIL] Viewer rejected " << shapeNames[row] << " Feature overlay." << std::endl;
            return EXIT_FAILURE;
        }

        overlayObjectIds.push_back(objectId);
    }

    QToolBar* featureToolBar = window.addToolBar(QStringLiteral("Feature Validation"));
    featureToolBar->setMovable(false);
    QAction* featureAction = featureToolBar->addAction(QStringLiteral("Explicit Features"));
    featureAction->setCheckable(true);
    featureAction->setChecked(true);
    featureAction->setShortcut(QKeySequence(Qt::Key_E));
    featureAction->setShortcutContext(Qt::ApplicationShortcut);
    std::uint64_t overlayStateVersion = 2;

    QObject::connect(featureAction, &QAction::toggled,
        [&window, &overlayObjectIds, &overlayStateVersion](bool visible)
        {
            for (std::size_t index = 0; index < overlayObjectIds.size(); ++index)
            {
                MyVoxel::Display_ObjectStateUpdate update;
                update.objectId = overlayObjectIds[index];
                update.stateVersion = overlayStateVersion++;
                update.hasVisible = true;
                update.visible = visible;
                window.submitDisplayStateUpdate(update);
            }
        });

    application.processEvents();
    window.viewer()->setIsometricView();
    window.viewer()->fitAll();
    window.viewer()->setFocus(Qt::OtherFocusReason);
    // window.statusBar()->showMessage(QStringLiteral(
    //     "Columns: Standard | FeatureSensitive QEF | FeatureConstrained    Rows: Box | Cylinder | Sphere    "
    //     "Orange: exact VoxelShape Features on Constrained column    E: Features  W: Wireframe  F: Fit    "
    //     "Middle: Rotate  Shift+Middle: Precision Rotate  Left: Pan  Wheel: Zoom"));

    std::cout << "Feature Overlay | Box: 8 vertices / 12 lines | Cylinder: 2 circular edges / seam vertices hidden | Sphere: empty FeatureSet" << std::endl;
    return application.exec();
}