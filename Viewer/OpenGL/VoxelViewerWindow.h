#ifndef MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H
#define MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H

#include <cstddef>

#include <QMainWindow>
#include <QMatrix4x4>

#include "VoxelOpenGLWidget.h"

namespace MyVoxelViewer
{

// 提供多网格OpenGL视口、基础视图工具栏和交互提示的独立显示窗口。
class VoxelViewerWindow : public QMainWindow
{
public:
    explicit VoxelViewerWindow(QWidget* parent = nullptr);

    /// 视口访问

    // 返回当前窗口持有的OpenGL网格视口。
    VoxelOpenGLWidget* viewer();

    // 返回当前窗口持有的只读OpenGL网格视口。
    const VoxelOpenGLWidget* viewer() const;

    /// 通用网格对象

    // 添加一个普通三角网格对象并返回对象标识。
    MeshObjectId addMesh(const MyVoxel::Geometry::Mesh& mesh, const QMatrix4x4& modelMatrix = QMatrix4x4());

    // 替换指定普通网格对象的数据。
    bool setMesh(MeshObjectId objectId, const MyVoxel::Geometry::Mesh& mesh);

    // 添加一个支持根级增量更新的体素网格缓存对象并返回对象标识。
    MeshObjectId addMeshCache(const MyVoxel::VoxelSurfaceCache& cache, const QMatrix4x4& modelMatrix = QMatrix4x4());

    // 使用完整缓存替换指定体素网格对象。
    bool setMeshCache(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache);

    // 根据缓存更新指定体素对象的根集合。
    bool updateRootMeshes(MeshObjectId objectId,
                          const MyVoxel::VoxelSurfaceCache& cache,
                          const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices);

    // 设置指定对象的模型矩阵。
    bool setMeshObjectMatrix(MeshObjectId objectId, const QMatrix4x4& matrix);

    // 设置指定对象是否可见。
    bool setMeshObjectVisible(MeshObjectId objectId, bool visible);

    // 删除指定网格对象。
    bool removeMeshObject(MeshObjectId objectId);

    // 清空全部网格对象。
    void clearMeshes();

    // 返回当前网格对象数量。
    std::size_t meshObjectCount() const;

    /// 单体素对象兼容入口

    // 使用完整根级缓存替换默认体素对象。
    void setMeshCache(const MyVoxel::VoxelSurfaceCache& cache);

    // 仅更新默认体素对象的指定根集合。
    void updateRootMeshes(const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices);

private:
    // 创建适应窗口、标准视图和线框模式工具栏。
    void createToolBar();

private:
    VoxelOpenGLWidget* m_viewer = nullptr; // 当前窗口中央OpenGL网格视口。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H