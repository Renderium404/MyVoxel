#ifndef MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H
#define MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H

#include <cstdint>

#include <QMainWindow>

#include "MyVoxel/Display/Line/Display_LineSnapshot.h"
#include "MyVoxel/Display/Mesh/Display_MeshSnapshot.h"
#include "MyVoxel/Display/Mesh/Display_MeshUpdate.h"

namespace MyVoxelViewer
{

class VoxelOpenGLWidget;

// 提供通用Mesh和Line Display OpenGL视口、基础视图工具栏和交互提示的独立显示窗口。
class VoxelViewerWindow : public QMainWindow
{
public:
    explicit VoxelViewerWindow(QWidget* parent = nullptr);

    /// 视口访问

    VoxelOpenGLWidget* viewer();
    const VoxelOpenGLWidget* viewer() const;

    /// Display对象提交

    bool submitDisplaySnapshot(const MyVoxel::Display_MeshObjectSnapshot& snapshot);
    bool submitDisplaySnapshot(const MyVoxel::Display_LineObjectSnapshot& snapshot);
    bool submitDisplayUpdate(const MyVoxel::Display_MeshUpdate& update);
    bool submitDisplayStateUpdate(const MyVoxel::Display_MeshStateUpdate& update);
    bool removeDisplayObject(std::uint64_t objectId);
    void clearDisplayObjects();

private:
    void createToolBar();

private:
    VoxelOpenGLWidget* m_viewer; // 当前窗口中央通用Display OpenGL视口。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H
