#include <cstdlib>
#include <vector>

#include <QApplication>
#include <QStatusBar>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Construction/Geometry_Revolved.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Geometry/Shape/Geometry_Box.h"
#include "MyVoxel/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyVoxel/Geometry/Shape/Geometry_Cylinder.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Geometry/Shape/Geometry_Sphere.h"
#include "MyVoxel/Mesh/Geometry_Mesh.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Mesh/ShapeMesher.h"
#include "MyVoxel/Topology/Shape.h"
#include "MyVoxel/Topology/Topology_Shape.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

// 将连续实体几何资源包装为数学定义Shape。
MyVoxel::Shape makeShape(MyVoxel::Geometry_Shape* geometry)
{
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Shape> resource(geometry);
    return MyVoxel::Shape(MyVoxel::Topology_Shape(resource));
}

// 创建一个位于正X半平面的闭合矩形母线，用于生成带中心孔的回转实体。
std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > makeRevolvedProfile()
{
    std::vector<MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve> > curves;
    curves.reserve(4); // 矩形母线固定由四条直线构成。
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(1.5, -2.0, 0.0), MyMath::Vector3(3.0, -2.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, -2.0, 0.0), MyMath::Vector3(3.0, 2.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, 2.0, 0.0), MyMath::Vector3(1.5, 2.0, 0.0))));
    curves.push_back(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(new MyVoxel::Geometry_Line(MyMath::Vector3(1.5, 2.0, 0.0), MyMath::Vector3(1.5, -2.0, 0.0))));
    return curves;
}

// 根据Shape建立单分片静态Display快照，并使用给定平移将对象放入场景。
MyVoxel::Display_MeshObjectSnapshot makeSnapshot(MyVoxel::Display_MeshObjectId objectId, const MyVoxel::Shape& shape,
                                                 const MyVoxel::Display_Color& color, const MyMath::Vector3& translation,
                                                 const MyVoxel::ShapeMeshingOptions& options)
{
    const MyVoxel::Mesh mesh = MyVoxel::ShapeMesher::build(shape, color, options);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> resource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> constResource = resource;

    MyVoxel::Display_MeshObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.usage = MyVoxel::Display_MeshUsage::Static;
    snapshot.localToWorld = MyMath::Matrix4::fromTranslation(translation);
    snapshot.visible = true;
    snapshot.parts.push_back(MyVoxel::Display_MeshPartSnapshot(0, 1, constResource));
    return snapshot;
}

// 向Viewer提交一个Shape完整快照。
bool submitShape(MyVoxelViewer::VoxelViewerWindow& window, MyVoxel::Display_MeshObjectId objectId,
                 const MyVoxel::Shape& shape, const MyVoxel::Display_Color& color,
                 const MyMath::Vector3& translation, const MyVoxel::ShapeMeshingOptions& options)
{
    const MyVoxel::Display_MeshObjectSnapshot snapshot = makeSnapshot(objectId, shape, color, translation, options);
    return snapshot.isValid() && window.submitDisplaySnapshot(snapshot);
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyVoxel::ShapeMeshingOptions options;
    options.circularSegmentCount = 64;
    options.sphereStackCount = 32;
    options.profileArcSegmentCount = 64;

    // 第一排：Box、Sphere、Cylinder、ConeFrustum。
    const MyVoxel::Shape box = makeShape(new MyVoxel::Geometry_Box(4.0, 5.0, 6.0));
    const MyVoxel::Shape sphere = makeShape(new MyVoxel::Geometry_Sphere(3.0));
    const MyVoxel::Shape cylinder = makeShape(new MyVoxel::Geometry_Cylinder(2.5, 6.0));
    const MyVoxel::Shape frustum = makeShape(new MyVoxel::Geometry_ConeFrustum(3.0, 1.5, 6.0));

    // 第二排：Cone、InvertedCone、Revolved、Geometry_Mesh。
    const MyVoxel::Shape cone = makeShape(new MyVoxel::Geometry_ConeFrustum(3.0, 0.0, 6.0));
    const MyVoxel::Shape invertedCone = makeShape(new MyVoxel::Geometry_ConeFrustum(0.0, 3.0, 6.0));
    const MyVoxel::Shape revolved = makeShape(new MyVoxel::Geometry_Revolved(makeRevolvedProfile(), 1.0e-9));

    const MyVoxel::Mesh sourceMesh = MyVoxel::ShapeMesher::build(box, MyVoxel::Display_Color(0.75, 0.75, 0.78, 1.0), options);
    const MyVoxel::Shape meshShape = makeShape(new MyVoxel::Geometry_Mesh(sourceMesh));

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel Shape Viewer - Box / Sphere / Cylinder / Frustum / Cone / Inverted Cone / Revolved / Mesh"));
    window.resize(1500, 900);
    window.show();

    // 先完成QOpenGLWidget初始化，再向后台OpenGL线程提交对象资源。
    application.processEvents();

    bool success = true;
    success = submitShape(window, 1, box, MyVoxel::Display_Color(0.20, 0.50, 0.88, 1.0), MyMath::Vector3(-13.5, 5.0, 0.0), options) && success;
    success = submitShape(window, 2, sphere, MyVoxel::Display_Color(0.90, 0.35, 0.25, 1.0), MyMath::Vector3(-4.5, 5.0, 0.0), options) && success;
    success = submitShape(window, 3, cylinder, MyVoxel::Display_Color(0.25, 0.72, 0.38, 1.0), MyMath::Vector3(4.5, 5.0, 0.0), options) && success;
    success = submitShape(window, 4, frustum, MyVoxel::Display_Color(0.90, 0.68, 0.20, 1.0), MyMath::Vector3(13.5, 5.0, 0.0), options) && success;
    success = submitShape(window, 5, cone, MyVoxel::Display_Color(0.58, 0.36, 0.86, 1.0), MyMath::Vector3(-13.5, -5.0, 0.0), options) && success;
    success = submitShape(window, 6, invertedCone, MyVoxel::Display_Color(0.20, 0.72, 0.75, 1.0), MyMath::Vector3(-4.5, -5.0, 0.0), options) && success;
    success = submitShape(window, 7, revolved, MyVoxel::Display_Color(0.88, 0.35, 0.62, 1.0), MyMath::Vector3(4.5, -5.0, 0.0), options) && success;
    success = submitShape(window, 8, meshShape, MyVoxel::Display_Color(0.55, 0.58, 0.62, 1.0), MyMath::Vector3(13.5, -5.0, 0.0), options) && success;

    if (!success)
    {
        return EXIT_FAILURE;
    }

    window.statusBar()->showMessage(QStringLiteral(
        "Top: Box | Sphere | Cylinder | ConeFrustum    Bottom: Cone | InvertedCone | Revolved | Geometry_Mesh   \nLeft: Rotate  Right: Pan  Wheel: Zoom  W: Wireframe"));

    window.viewer()->setIsometricView();
    window.viewer()->fitAll();

    return application.exec();
}