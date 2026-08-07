#include <cstdlib>
#include <iostream>

#include <QApplication>
#include <QStatusBar>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Adapter/Display_VoxelSurfaceAdapter.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Mesh/ShapeMesher.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Operation/ShapeBooleanOperation.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"
#include "MyVoxel/Topology/Shape.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

// 将连续Shape转换为静态Display快照；Shape实例变换先烘焙进Mesh，displayPosition仅负责窗口排版。
MyVoxel::Display_MeshObjectSnapshot makeShapeSnapshot(MyVoxel::Display_MeshObjectId objectId, const MyVoxel::Shape& shape,
                                                      const MyVoxel::Display_Color& color, const MyMath::Vector3& displayPosition)
{
    const MyVoxel::Mesh mesh = MyVoxel::ShapeMesher::buildWorld(shape, color);
    const MyVoxel::Foundation::RefPtr<MyVoxel::Display_MeshResource> mutableResource =
        MyVoxel::Foundation::makeRef<MyVoxel::Display_MeshResource>(mesh);
    const MyVoxel::Foundation::RefPtr<const MyVoxel::Display_MeshResource> resource = mutableResource;

    MyVoxel::Display_MeshObjectSnapshot snapshot;
    snapshot.objectId = objectId;
    snapshot.usage = MyVoxel::Display_MeshUsage::Static;
    snapshot.localToWorld = MyMath::Matrix4::fromTranslation(displayPosition);
    snapshot.visible = true;
    snapshot.parts.push_back(MyVoxel::Display_MeshPartSnapshot(1, 1, resource));
    return snapshot;
}

// 将VoxelShape完整重建为Surface并转换为动态Display快照。
MyVoxel::Display_MeshObjectSnapshot makeVoxelSnapshot(MyVoxel::Display_MeshObjectId objectId, const MyVoxel::VoxelShape& voxels,
                                                      const MyVoxel::Display_Color& color, const MyMath::Vector3& displayPosition,
                                                      std::size_t& rootCount, std::size_t& partCount, std::size_t& triangleCount)
{
    MyVoxel::VoxelSurfaceCache cache;
    cache.rebuild(voxels, color);

    MyVoxel::Display_VoxelSurfaceAdapter adapter(objectId);
    const MyVoxel::Display_MeshObjectSnapshot snapshot =
        adapter.buildSnapshot(cache, MyMath::Matrix4::fromTranslation(displayPosition), true);

    rootCount = cache.rootCount();
    partCount = adapter.activePartCount();
    triangleCount = snapshot.triangleCount();
    return snapshot;
}

// 提交一个体素布尔结果并输出当前Surface规模。
bool submitVoxelResult(MyVoxelViewer::VoxelViewerWindow& window, const char* name, MyVoxel::Display_MeshObjectId objectId,
                       const MyVoxel::VoxelShape& voxels, const MyVoxel::Display_Color& color, const MyMath::Vector3& position)
{
    std::size_t rootCount = 0;
    std::size_t partCount = 0;
    std::size_t triangleCount = 0;
    const MyVoxel::Display_MeshObjectSnapshot snapshot =
        makeVoxelSnapshot(objectId, voxels, color, position, rootCount, partCount, triangleCount);

    std::cout << name
              << " | roots=" << rootCount
              << " | parts=" << partCount
              << " | triangles=" << triangleCount
              << std::endl;

    return snapshot.isValid() && window.submitDisplaySnapshot(snapshot);
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    // 第0层边长8.0、最高层级4，因此最高层体素边长固定为0.5。
    const MyVoxel::VoxelGrid grid(8.0, static_cast<MyVoxel::VoxelLevel>(7));

    // A为局部原点长方体；B为发生明显偏移且与A部分重叠的球体。
    const MyVoxel::Shape shapeA = MyVoxel::Modeling::makeBox(7.0, 7.0, 7.0);
    const MyVoxel::Shape shapeB = MyVoxel::Modeling::makeSphere(
        4.0,
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(2.2, 1.2, 0.8)));

    // 工件体素体以A建立，ShapeBooleanOperation将B自动转换到A的体素地址空间。
    const MyVoxel::VoxelShape voxelA = MyVoxel::Modeling::voxelize(shapeA, grid);

    const MyVoxel::VoxelShape resultUnion =
        MyVoxel::Operation::ShapeBooleanOperation::unite(voxelA, shapeB);
    const MyVoxel::VoxelShape resultIntersection =
        MyVoxel::Operation::ShapeBooleanOperation::intersect(voxelA, shapeB);
    const MyVoxel::VoxelShape resultDifference =
        MyVoxel::Operation::ShapeBooleanOperation::subtract(voxelA, shapeB);
    const MyVoxel::VoxelShape resultExclusiveOr =
        MyVoxel::Operation::ShapeBooleanOperation::exclusiveOr(voxelA, shapeB);

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel Boolean Compare - Inputs / Union / Intersection / Difference / XOR"));
    window.resize(1800, 1000);
    window.show();
    application.processEvents();

    // 左侧两行显示连续输入A/B；右侧2x2显示四种体素布尔结果。
    const MyMath::Vector3 inputAPosition(-18.0, 6.0, 0.0);
    const MyMath::Vector3 inputBPosition(-18.0, -6.0, 0.0);
    const MyMath::Vector3 unionPosition(-4.0, 6.0, 0.0);
    const MyMath::Vector3 intersectionPosition(9.0, 6.0, 0.0);
    const MyMath::Vector3 differencePosition(-4.0, -6.0, 0.0);
    const MyMath::Vector3 xorPosition(9.0, -6.0, 0.0);

    const MyVoxel::Display_MeshObjectSnapshot inputASnapshot =
        makeShapeSnapshot(1, shapeA, MyVoxel::Display_Color(0.20, 0.55, 0.90, 1.0), inputAPosition);
    const MyVoxel::Display_MeshObjectSnapshot inputBSnapshot =
        makeShapeSnapshot(2, shapeB, MyVoxel::Display_Color(0.25, 0.75, 0.40, 1.0), inputBPosition);

    bool success = inputASnapshot.isValid() && inputBSnapshot.isValid();
    success = window.submitDisplaySnapshot(inputASnapshot) && success;
    success = window.submitDisplaySnapshot(inputBSnapshot) && success;

    success = submitVoxelResult(window, "Union", 101, resultUnion,
                                MyVoxel::Display_Color(0.20, 0.75, 0.85, 1.0), unionPosition) && success;
    success = submitVoxelResult(window, "Intersection", 102, resultIntersection,
                                MyVoxel::Display_Color(0.95, 0.80, 0.20, 1.0), intersectionPosition) && success;
    success = submitVoxelResult(window, "Difference A-B", 103, resultDifference,
                                MyVoxel::Display_Color(0.95, 0.50, 0.20, 1.0), differencePosition) && success;
    success = submitVoxelResult(window, "ExclusiveOr", 104, resultExclusiveOr,
                                MyVoxel::Display_Color(0.80, 0.35, 0.85, 1.0), xorPosition) && success;

    if (!success)
    {
        return EXIT_FAILURE;
    }

    std::cout << "Input A: Box 7x7x7" << std::endl;
    std::cout << "Input B: Sphere r=4 translated by (2.2, 1.2, 0.8)" << std::endl;
    std::cout << "Voxel base edge: " << grid.baseCellEdgeLength() << std::endl;
    std::cout << "Voxel finest edge: " << grid.minimumCellEdgeLength() << std::endl;
    std::cout << "Layout:" << std::endl;
    std::cout << "  Left top    = Input A" << std::endl;
    std::cout << "  Left bottom = Input B" << std::endl;
    std::cout << "  Right top-left     = Union" << std::endl;
    std::cout << "  Right top-right    = Intersection" << std::endl;
    std::cout << "  Right bottom-left  = Difference A-B" << std::endl;
    std::cout << "  Right bottom-right = ExclusiveOr" << std::endl;

    window.statusBar()->showMessage(QStringLiteral(
        "Left: A(Box) / B(offset Sphere) | Right top: Union / Intersection | Right bottom: Difference(A-B) / XOR | Finest voxel edge: 0.5"));

    window.viewer()->setIsometricView();
    window.viewer()->fitAll();

    return application.exec();
}