#ifndef MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H
#define MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H

#include <QMainWindow>

#include "MyVoxel/Display/Object/Display_ObjectSnapshot.h"
#include "MyVoxel/Display/Object/Display_ObjectUpdate.h"

namespace MyVoxelViewer
{

class VoxelOpenGLWidget;

// 提供通用Display_Object OpenGL视口、基础视图工具栏和交互提示的独立显示窗口。
class VoxelViewerWindow : public QMainWindow
{
public:
    explicit VoxelViewerWindow(QWidget* parent = nullptr);

    /// 视口访问

    VoxelOpenGLWidget* viewer();
    const VoxelOpenGLWidget* viewer() const;

    /// Display对象提交

    bool submitDisplaySnapshot(const MyVoxel::Display_ObjectSnapshot& snapshot);
    bool submitDisplayPartsUpdate(const MyVoxel::Display_ObjectPartsUpdate& update);
    bool submitDisplayStateUpdate(const MyVoxel::Display_ObjectStateUpdate& update);
    bool removeDisplayObject(MyVoxel::Display_ObjectId objectId);
    void clearDisplayObjects();

private:
    void createToolBar();

private:
    VoxelOpenGLWidget* m_viewer; // 当前窗口中央通用Display OpenGL视口。
};

}

#endif // MYVOXEL_VIEWER_OPENGL_VOXELVIEWERWINDOW_H