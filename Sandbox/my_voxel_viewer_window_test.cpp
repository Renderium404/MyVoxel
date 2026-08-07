#include <cstdlib>

#include <QApplication>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Display/Adapter/Display_VoxelSurfaceAdapter.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Surface/VoxelSurfaceCache.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"

namespace
{

// 返回Surface增量跟踪使用的掩码叶块层级。
MyVoxel::VoxelLevel surfaceTrackingLevel(const MyVoxel::VoxelGrid& grid)
{
    const unsigned int maximumLevel = static_cast<unsigned int>(grid.maximumLevel());
    return maximumLevel >= 2U ? static_cast<MyVoxel::VoxelLevel>(maximumLevel - 2U) : MyVoxel::BaseVoxelLevel;
}

// 将指定第0层Root设置为完整材料并返回变化记录。
MyVoxel::VoxelChangeSet setMaterialRoot(MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellIndex& rootIndex)
{
    MyVoxel::VoxelShapeSession session(shape, surfaceTrackingLevel(shape.grid()));
    session.setTree(rootIndex, MyVoxel::VoxelTree(MyVoxel::VoxelState::Material));
    return session.takeChanges();
}

// 向当前体素形体增加一个Root，并使用指定颜色更新表面缓存。
void appendColoredRoot(MyVoxel::VoxelShape& shape, MyVoxel::VoxelSurfaceCache& cache,
                       const MyVoxel::VoxelCellIndex& rootIndex, const MyVoxel::Display_Color& color)
{
    const MyVoxel::VoxelChangeSet changes = setMaterialRoot(shape, rootIndex);
    cache.update(shape, changes, color);
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    // 最小体素边长为1，最高层级为3，因此每个第0层Root单轴包含8个最高层体素。
    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(3));
    MyVoxel::VoxelShape shape(grid);

    const MyVoxel::VoxelCellIndex firstRoot(0, 0, 0);
    setMaterialRoot(shape, firstRoot);

    MyVoxel::VoxelSurfaceCache surfaceCache;
    surfaceCache.rebuild(shape, MyVoxel::Display_Color(0.18, 0.48, 0.82, 1.0));

    // 依次增加三个相邻Root，使新暴露表面保留不同颜色，形成便于观察的L形实体。
    appendColoredRoot(shape, surfaceCache, MyVoxel::VoxelCellIndex(1, 0, 0),
                      MyVoxel::Display_Color(0.88, 0.30, 0.22, 1.0));
    appendColoredRoot(shape, surfaceCache, MyVoxel::VoxelCellIndex(0, 1, 0),
                      MyVoxel::Display_Color(0.24, 0.72, 0.38, 1.0));
    appendColoredRoot(shape, surfaceCache, MyVoxel::VoxelCellIndex(0, 0, 1),
                      MyVoxel::Display_Color(0.90, 0.68, 0.18, 1.0));

    MyVoxel::Display_VoxelSurfaceAdapter surfaceAdapter(1);
    const MyVoxel::Display_MeshObjectSnapshot snapshot =surfaceAdapter.buildSnapshot(surfaceCache, MyMath::Matrix4::identity(), true);

    MyVoxelViewer::VoxelViewerWindow window;
    window.setWindowTitle(QStringLiteral("MyVoxel Viewer/OpenGL Display Test"));
    window.showMaximized();

    // 先让QOpenGLWidget完成初始化，再提交完整Display快照。
    application.processEvents();

    if (!window.submitDisplaySnapshot(snapshot))
    {
        return EXIT_FAILURE;
    }

    window.viewer()->setIsometricView();
    window.viewer()->fitAll();

    return application.exec();
}