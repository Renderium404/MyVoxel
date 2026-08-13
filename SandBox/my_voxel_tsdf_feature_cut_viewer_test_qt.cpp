#include <chrono>
#include <cstdlib>
#include <iostream>

#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QObject>
#include <QStatusBar>
#include <QString>
#include <QTimer>
#include <QWheelEvent>

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

const double BaseVoxelEdgeLength = 4.0; // 当前A/B效果测试沿用既有体素基准。
const MyVoxel::VoxelLevel MaximumLevel = static_cast<MyVoxel::VoxelLevel>(5); // 固定Level 5，只比较Standard与Feature顶点定位。
const float BackgroundDistance = 0.75f; // 固定TSDF截断距离。
const double WorkpieceSize = 3.0; // 原始工件尺寸。
const double DisplaySpacing = 4.2; // 四个A/B对象之间的显示间距。
const double FeatureAngleDegrees = 30.0; // Feature模式达到30度零交叉法线变化才启用QEF。

typedef std::chrono::steady_clock TestClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const TestClock::time_point& begin, const TestClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 只记录每类交互事件第一次到达，避免鼠标移动刷屏。
class ViewerEventProbe : public QObject
{
public:
    ViewerEventProbe() : m_mousePressSeen(false), m_mouseMoveSeen(false), m_wheelSeen(false), m_keySeen(false){}

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        (void)watched;

        if (event->type() == QEvent::MouseButtonPress && !m_mousePressSeen)
        {
            m_mousePressSeen = true;
            std::cout << "[EVENT] Viewer received MouseButtonPress." << std::endl;
        }
        else if (event->type() == QEvent::MouseMove && !m_mouseMoveSeen)
        {
            m_mouseMoveSeen = true;
            std::cout << "[EVENT] Viewer received MouseMove." << std::endl;
        }
        else if (event->type() == QEvent::Wheel && !m_wheelSeen)
        {
            m_wheelSeen = true;
            std::cout << "[EVENT] Viewer received Wheel." << std::endl;
        }
        else if (event->type() == QEvent::KeyPress && !m_keySeen)
        {
            m_keySeen = true;
            std::cout << "[EVENT] Viewer received KeyPress." << std::endl;
        }

        return false;
    }

private:
    bool m_mousePressSeen; // 是否已经记录鼠标按下事件。
    bool m_mouseMoveSeen; // 是否已经记录鼠标移动事件。
    bool m_wheelSeen; // 是否已经记录滚轮事件。
    bool m_keySeen; // 是否已经记录键盘事件。
};

// 将Mesh转换为Viewer正式Display快照。
MyVoxel::Display_ObjectSnapshot makeSnapshot(MyVoxel::Display_ObjectId objectId, MyVoxel::Display_ResourceId resourceId,
                                             const MyVoxel::Mesh& mesh, const MyMath::Vector3& translation)
{
    MYVOXEL_ASSERT_MESSAGE(mesh.isRenderable() && !mesh.isEmpty(), "Feature A/B Viewer requires a non-empty renderable Mesh.");

    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> mutableResource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_Resource> resource = mutableResource;

    MyVoxel::Display_ObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.resourceKind = MyVoxel::Display_ResourceKind::Mesh;
    snapshot.usage = MyVoxel::Display_ObjectUsage::Static;
    snapshot.stateVersion = 1;
    snapshot.localToWorld = MyMath::Matrix4::fromTranslation(translation);
    snapshot.visible = true;
    snapshot.parts.push_back(MyVoxel::Display_ObjectPartSnapshot(0, 1, resourceId, resource));

    MYVOXEL_ASSERT_MESSAGE(snapshot.isValid(), "Feature A/B Viewer produced an invalid Display_ObjectSnapshot.");
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

// 输出Mesh局部包围盒，用于排除Feature QEF异常顶点破坏Viewer相机范围。
void printBounds(const MyVoxel::Mesh& mesh)
{
    const MyVoxel::Bounds3 bounds = mesh.localBounds();

    if (!bounds.isValid())
    {
        std::cout << " | bounds=INVALID";
        return;
    }

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();
    std::cout << " | bounds=[(" << minimum.x() << "," << minimum.y() << "," << minimum.z()
              << ")-(" << maximum.x() << "," << maximum.y() << "," << maximum.z() << ")]";
}

// 使用指定模式构建Surface Nets网格并输出统计。
MyVoxel::Mesh buildMesh(const MyVoxel::VoxelShape& voxels, MyVoxel::Meshing::VolumeMeshingMode mode,
                        const MyVoxel::Display_Color& color, const char* name)
{
    MyVoxel::Meshing::VolumeMeshingOptions options;
    options.color = color;
    options.mode = mode;
    options.featureAngleDegrees = FeatureAngleDegrees;

    const TestClock::time_point begin = TestClock::now();
    const MyVoxel::Mesh mesh = MyVoxel::Meshing::VolumeMesher::build(voxels, options);
    const double milliseconds = elapsedMilliseconds(begin, TestClock::now());

    std::cout << name
              << " | vertices=" << mesh.vertexCount()
              << " | triangles=" << mesh.triangleCount()
              << " | mesh=" << milliseconds << " ms";
    printBounds(mesh);
    std::cout << std::endl;
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

    const MyVoxel::Display_Color standardColor(0.48, 0.68, 0.86, 1.0);
    const MyVoxel::Display_Color featureColor(0.90, 0.58, 0.34, 1.0);

    const MyVoxel::Mesh originalStandard =
        buildMesh(original, MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets, standardColor, "Original Standard");
    const MyVoxel::Mesh originalFeature =
        buildMesh(original, MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets, featureColor, "Original Feature");
    const MyVoxel::Mesh cutStandard =
        buildMesh(combined, MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets, standardColor, "Combined Standard");
    const MyVoxel::Mesh cutFeature =
        buildMesh(combined, MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets, featureColor, "Combined Feature");

    if (!originalStandard.isRenderable() || !originalFeature.isRenderable() ||
        !cutStandard.isRenderable() || !cutFeature.isRenderable())
    {
        std::cerr << "[FAIL] At least one A/B mesh is not renderable." << std::endl;
        return EXIT_FAILURE;
    }

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel Surface Nets Feature QEF A/B - Interaction Check"));
    window.resize(1680, 920);
    window.show();

    // 与已经验证可交互的TSDF切削Viewer保持相同初始化顺序。
    application.processEvents();

    ViewerEventProbe eventProbe;
    window.viewer()->installEventFilter(&eventProbe);

    const MyVoxel::Mesh* meshes[4] = {&originalStandard, &originalFeature, &cutStandard, &cutFeature};
    const double firstX = -DisplaySpacing * 1.5;

    for (int index = 0; index < 4; ++index)
    {
        const MyVoxel::Display_ObjectSnapshot snapshot =
            makeSnapshot(static_cast<MyVoxel::Display_ObjectId>(index + 1),
                         static_cast<MyVoxel::Display_ResourceId>(index + 1),
                         *meshes[index],
                         MyMath::Vector3(firstX + DisplaySpacing * static_cast<double>(index), 0.0, 0.0));

        if (!window.submitDisplaySnapshot(snapshot))
        {
            std::cerr << "[FAIL] Viewer rejected A/B snapshot " << index << std::endl;
            return EXIT_FAILURE;
        }
    }

    application.processEvents();
    window.viewer()->setIsometricView();
    window.viewer()->fitAll();
    window.activateWindow();
    window.raise();
    window.viewer()->setFocus(Qt::OtherFocusReason);


    // 进入正式事件循环后再次只做一次焦点恢复，并报告后台Renderer状态；不改变用户视角。
    QTimer::singleShot(200, [&window]()
    {
        window.activateWindow();
        window.viewer()->setFocus(Qt::OtherFocusReason);
        const QString error = window.viewer()->backgroundRenderError();
        std::cout << "[VIEWER] backgroundRenderError="
                  << (error.isEmpty() ? "<empty>" : error.toStdString())
                  << std::endl;
    });

    return application.exec();
}