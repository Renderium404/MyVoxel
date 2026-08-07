#include <cstdlib>
#include <iostream>
#include <vector>

#include <QApplication>
#include <QStatusBar>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Adapter/Display_VoxelSurfaceAdapter.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Mesh/ShapeMesher.h"
#include "MyVoxel/Modeling/Shape/MeshModeling.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/RevolvedModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"
#include "MyVoxel/Topology/Shape.h"
#include "MyVoxel/Topology/Topology_Curve.h"
#include "MyVoxel/Topology/Topology_Shape.h"
#include "MyVoxel/Topology/Topology_Wire.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

// 创建回转体测试使用的闭合矩形母线。
MyVoxel::Topology_Wire makeRevolvedProfile()
{
    std::vector<MyVoxel::Topology_Curve> curves;
    curves.reserve(4); // 矩形闭合母线固定由四条直线构成。
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(1.5, -2.5, 0.0), MyMath::Vector3(3.0, -2.5, 0.0)))));
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, -2.5, 0.0), MyMath::Vector3(3.0, 2.5, 0.0)))));
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(3.0, 2.5, 0.0), MyMath::Vector3(1.5, 2.5, 0.0)))));
    curves.push_back(MyVoxel::Topology_Curve(MyVoxel::Foundation::RefPtr<const MyVoxel::Geometry_Curve>(
        new MyVoxel::Geometry_Line(MyMath::Vector3(1.5, 2.5, 0.0), MyMath::Vector3(1.5, -2.5, 0.0)))));
    return MyVoxel::Topology_Wire(curves, 1.0e-9);
}

// 根据连续Shape建立单分片静态显示快照。
MyVoxel::Display_MeshObjectSnapshot makeShapeSnapshot(MyVoxel::Display_MeshObjectId objectId, const MyVoxel::Topology_Shape& topology,
                                                      const MyVoxel::Display_Color& color, const MyMath::Vector3& position)
{
    const MyVoxel::Shape shape(topology);
    const MyVoxel::Mesh mesh = MyVoxel::ShapeMesher::build(shape, color);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> resource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> constResource = resource;

    MyVoxel::Display_MeshObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.usage = MyVoxel::Display_MeshUsage::Static;
    snapshot.localToWorld = MyMath::Matrix4::fromTranslation(position);
    snapshot.visible = true;
    snapshot.parts.push_back(MyVoxel::Display_MeshPartSnapshot(1, 1, constResource));
    return snapshot;
}

// 对一个数学Shape执行一次体素化，并提交连续体和体素体两个显示对象。
bool submitPair(MyVoxelViewer::VoxelViewerWindow& window, const char* name, MyVoxel::Display_MeshObjectId shapeObjectId,
                MyVoxel::Display_MeshObjectId voxelObjectId, const MyVoxel::Topology_Shape& topology,
                const MyVoxel::VoxelGrid& grid, const MyVoxel::Display_Color& shapeColor,
                const MyVoxel::Display_Color& voxelColor, const MyMath::Vector3& shapePosition,
                const MyMath::Vector3& voxelPosition)
{
    const MyVoxel::Display_MeshObjectSnapshot shapeSnapshot =
        makeShapeSnapshot(shapeObjectId, topology, shapeColor, shapePosition);

    MyVoxel::Modeling::VoxelizationStatistics statistics;
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelize(topology, grid, statistics);

    MyVoxel::VoxelSurfaceCache cache;
    cache.rebuild(voxels, voxelColor);

    MyVoxel::Display_VoxelSurfaceAdapter adapter(voxelObjectId);
    const MyVoxel::Display_MeshObjectSnapshot voxelSnapshot =
        adapter.buildSnapshot(cache, MyMath::Matrix4::fromTranslation(voxelPosition), true);

    if (!shapeSnapshot.isValid() || !voxelSnapshot.isValid())
    {
        std::cout << "[FAIL] " << name << " snapshot invalid" << std::endl;
        return false;
    }

    const bool shapeSubmitted = window.submitDisplaySnapshot(shapeSnapshot);
    const bool voxelSubmitted = window.submitDisplaySnapshot(voxelSnapshot);

    std::cout << name
              << " | roots=" << voxels.rootCount()
              << " | visited=" << statistics.visitedCellCount
              << " | surfaceRoots=" << cache.rootCount()
              << " | parts=" << adapter.activePartCount()
              << " | shapeTriangles=" << shapeSnapshot.triangleCount()
              << " | voxelTriangles=" << voxelSnapshot.triangleCount()
              << std::endl;

    return shapeSubmitted && voxelSubmitted;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    // 第0层边长8.0、最高层级4，因此最高层体素边长为8/2^4=0.5。
    const MyVoxel::VoxelGrid grid(8.0, static_cast<MyVoxel::VoxelLevel>(6));

    const MyVoxel::Topology_Shape box = MyVoxel::Modeling::createBox(5.0, 5.0, 5.0);
    const MyVoxel::Topology_Shape sphere = MyVoxel::Modeling::createSphere(2.8);
    const MyVoxel::Topology_Shape cylinder = MyVoxel::Modeling::createCylinder(2.5, 5.5);
    const MyVoxel::Topology_Shape frustum = MyVoxel::Modeling::createConeFrustum(3.0, 1.5, 5.5);
    const MyVoxel::Topology_Shape cone = MyVoxel::Modeling::createCone(3.0, 5.5);
    const MyVoxel::Topology_Shape invertedCone = MyVoxel::Modeling::createConeFrustum(0.0, 3.0, 5.5);
    const MyVoxel::Topology_Shape revolved = MyVoxel::Modeling::createRevolved(makeRevolvedProfile());

    // Geometry_Mesh继续作为一种数学Shape参与同一体素化链路，这里使用Box显示网格构造封闭Mesh实体。
    const MyVoxel::Mesh sourceMesh = MyVoxel::ShapeMesher::build(box, MyVoxel::Display_Color::gray());
    const MyVoxel::Topology_Shape meshShape = MyVoxel::Modeling::createMesh(sourceMesh);

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel All Shape Voxelization - Shape / Voxel Pairs"));
    window.resize(1800, 1000);
    window.show();
    application.processEvents();

    const double shapeX0 = -18.0; // 每行第一组连续Shape的X位置。
    const double voxelX0 = -11.0; // 每行第一组体素体的X位置。
    const double shapeX1 = 3.0; // 每行第二组连续Shape的X位置。
    const double voxelX1 = 10.0; // 每行第二组体素体的X位置。
    const double rowY0 = 12.0; // 第一行Y位置。
    const double rowY1 = 4.0; // 第二行Y位置。
    const double rowY2 = -4.0; // 第三行Y位置。
    const double rowY3 = -12.0; // 第四行Y位置。

    bool success = true;

    success = submitPair(window, "Box", 1, 101, box, grid,
                         MyVoxel::Display_Color(0.20, 0.55, 0.90, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX0, rowY0, 0.0), MyMath::Vector3(voxelX0, rowY0, 0.0)) && success;

    success = submitPair(window, "Sphere", 2, 102, sphere, grid,
                         MyVoxel::Display_Color(0.30, 0.70, 0.40, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX1, rowY0, 0.0), MyMath::Vector3(voxelX1, rowY0, 0.0)) && success;

    success = submitPair(window, "Cylinder", 3, 103, cylinder, grid,
                         MyVoxel::Display_Color(0.60, 0.40, 0.85, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX0, rowY1, 0.0), MyMath::Vector3(voxelX0, rowY1, 0.0)) && success;

    success = submitPair(window, "ConeFrustum", 4, 104, frustum, grid,
                         MyVoxel::Display_Color(0.20, 0.75, 0.75, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX1, rowY1, 0.0), MyMath::Vector3(voxelX1, rowY1, 0.0)) && success;

    success = submitPair(window, "Cone", 5, 105, cone, grid,
                         MyVoxel::Display_Color(0.90, 0.35, 0.35, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX0, rowY2, 0.0), MyMath::Vector3(voxelX0, rowY2, 0.0)) && success;

    success = submitPair(window, "InvertedCone", 6, 106, invertedCone, grid,
                         MyVoxel::Display_Color(0.80, 0.65, 0.20, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX1, rowY2, 0.0), MyMath::Vector3(voxelX1, rowY2, 0.0)) && success;

    success = submitPair(window, "Revolved", 7, 107, revolved, grid,
                         MyVoxel::Display_Color(0.90, 0.35, 0.70, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX0, rowY3, 0.0), MyMath::Vector3(voxelX0, rowY3, 0.0)) && success;

    success = submitPair(window, "Geometry_Mesh", 8, 108, meshShape, grid,
                         MyVoxel::Display_Color(0.55, 0.58, 0.62, 1.0), MyVoxel::Display_Color(0.95, 0.55, 0.20, 1.0),
                         MyMath::Vector3(shapeX1, rowY3, 0.0), MyMath::Vector3(voxelX1, rowY3, 0.0)) && success;

    if (!success)
    {
        return EXIT_FAILURE;
    }

    std::cout << "Voxel base edge: " << grid.baseCellEdgeLength() << std::endl;
    std::cout << "Voxel finest edge: " << grid.minimumCellEdgeLength() << std::endl;

    window.statusBar()->showMessage(QStringLiteral(
        "Each pair: continuous Shape on the left, voxelized body on the right.Rows: Box/Sphere | Cylinder/ConeFrustum | Cone/InvertedCone | Revolved/Geometry_Mesh. Finest voxel edge: 0.5"));

    window.viewer()->setIsometricView();
    window.viewer()->fitAll();

    return application.exec();
}