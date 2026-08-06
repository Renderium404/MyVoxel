#include "MyVoxel/Modeling/PrimitiveModeling.h"
#include "MyVoxel/Modeling/ShapeVoxelization.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"
#include "MyMath/Vector3.h"
#include "MyMath/CoordinateSystem.h"

#include <QApplication>
#include <QTimer>

#include <iostream>

MyMath::Matrix4 move(const MyMath::Vector3& position)
{
    MyMath::CoordinateSystem coordinateSystem;
    coordinateSystem.setOrigin(position);
    return coordinateSystem.toMatrix();
}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    const MyVoxel::Geometry::MeshColor workpieceColor(0.55, 0.55, 0.55, 1.0);
    const MyVoxel::Geometry::MeshColor cutColor(0.90, 0.25, 0.15, 1.0);

    const MyVoxel::Geometry::Shape workpieceGeometry = MyVoxel::Modeling::makeCone(12.0, 20.0);
    const MyVoxel::Geometry::Shape toolGeometry = MyVoxel::Modeling::makeCone(5.0, 20.0);

    const MyVoxel::Geometry::ShapeInstance workpieceInstance(workpieceGeometry, move(MyMath::Vector3(0.0, 0.0, 0.0)));
    const MyVoxel::Geometry::ShapeInstance toolInstance(toolGeometry, move(MyMath::Vector3(7.0, 0.0, 0.0)));

    const MyVoxel::VoxelGrid grid(16.0, 8);
    MyVoxel::VoxelShape voxelShape = MyVoxel::Modeling::voxelize(workpieceInstance, grid);

    MyVoxel::VoxelSurfaceCache surfaceCache;
    surfaceCache.rebuild(voxelShape, workpieceColor);

    MyVoxelViewer::VoxelViewerWindow window;
    const MyVoxelViewer::MeshObjectId workpieceObjectId = window.addMeshCache(surfaceCache);
    window.show();
    window.viewer()->fitAll();

    QTimer timer;
    timer.setSingleShot(true);

    QObject::connect(&timer, &QTimer::timeout, [&]()
    {
        MyVoxel::VoxelChangeSet changes(static_cast<MyVoxel::VoxelLevel>(static_cast<unsigned int>(voxelShape.grid().maximumLevel()) - 2U));
        const bool changed = MyVoxel::Operation::BooleanOperation::subtractInPlace(voxelShape, toolInstance, &changes);

        if (!changed)
        {
            std::cout << "Boolean operation did not modify the workpiece." << std::endl;
            return;
        }

        const MyVoxel::VoxelSurfaceCacheUpdate cacheUpdate = surfaceCache.update(voxelShape, changes, cutColor);

        if (!cacheUpdate.changedRootIndices.empty())
        {
            window.updateRootMeshes(workpieceObjectId, surfaceCache, cacheUpdate.changedRootIndices);
        }

        std::cout << "Modified roots: " << changes.modifiedRootCount()
                  << " | Rebuilt roots: " << cacheUpdate.changedRootIndices.size()
                  << std::endl;
    });

    timer.start(1000); // 窗口显示1秒后执行一次布尔切削，便于观察更新前后的结果。
    return application.exec();
}