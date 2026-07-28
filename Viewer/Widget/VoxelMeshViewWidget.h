#ifndef MYVOXELVIEWER_VOXELMESHVIEWWIDGET_H
#define MYVOXELVIEWER_VOXELMESHVIEWWIDGET_H

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>

#include "Viewer/OpenGL/DisplayMesh.h"

class QKeyEvent;
class QMouseEvent;
class QOpenGLExtraFunctions;
class QWheelEvent;

namespace MyVoxelViewer
{

// 使用OpenGL显示体素表面三角网格。
class VoxelMeshViewWidget : public QOpenGLWidget
{
public:
    explicit VoxelMeshViewWidget(QWidget* parent = nullptr);

    /// 网格数据

    void setMesh(const DisplayMesh& mesh);
    void setMesh(DisplayMesh&& mesh);
    const DisplayMesh& mesh() const;

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    /// OpenGL资源

    // 创建渐变背景Shader。
    void createBackgroundShaderProgram();

    // 创建三角网格光照Shader。
    void createMeshShaderProgram();

    // 创建背景和网格顶点数组及缓冲对象。
    void createBuffers();

    // 更新网格顶点和索引缓冲，并尽量复用现有GPU容量。
    void updateMeshBuffer();

    /// 绘制流程

    // 绘制全屏工业风渐变背景。
    void drawBackground();

    // 使用指定相机和观察中心绘制三角网格。
    void drawMesh(const QVector3D& eye, const QVector3D& viewCenter);

    /// 网格状态

    // 更新网格包围球。
    void updateBounds();

    // 在网格变化后更新GPU缓冲和窗口。
    void applyMeshChange();

    QOpenGLExtraFunctions* m_functions = nullptr; // 当前OpenGL函数入口。

    QOpenGLShaderProgram m_backgroundProgram; // 渐变背景Shader。
    QOpenGLShaderProgram m_meshProgram; // 三角网格光照Shader。

    QOpenGLVertexArrayObject m_backgroundVao; // 全屏背景顶点数组对象。
    QOpenGLVertexArrayObject m_meshVao; // 网格顶点数组对象。
    QOpenGLBuffer m_vertexBuffer; // 网格顶点缓冲。
    QOpenGLBuffer m_indexBuffer; // 网格索引缓冲。

    DisplayMesh m_mesh;
    QVector3D m_center; // 当前网格包围盒中心。
    QVector3D m_viewOffset; // 当前观察中心偏移。
    QPoint m_lastMousePosition; // 上一次鼠标位置。

    int m_vertexBufferCapacity = 0; // 当前顶点缓冲已分配字节数。
    int m_indexBufferCapacity = 0; // 当前索引缓冲已分配字节数。

    float m_radius = 1.0f; // 当前网格包围球半径。
    float m_yaw = 35.0f; // 观察方位角。
    float m_pitch = 25.0f; // 观察俯仰角。
    float m_cameraScale = 2.4f; // 相机距离倍率。
    bool m_initialized = false; // OpenGL资源是否已经初始化。
};

}

#endif // MYVOXELVIEWER_VOXELMESHVIEWWIDGET_H