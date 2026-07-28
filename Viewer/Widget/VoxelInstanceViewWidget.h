#ifndef MYVOXELVIEWER_VOXELINSTANCEVIEWWIDGET_H
#define MYVOXELVIEWER_VOXELINSTANCEVIEWWIDGET_H

#include <vector>

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QVector3D>

#include "Viewer/OpenGL/VoxelDisplayInstance.h"

class QKeyEvent;
class QOpenGLExtraFunctions;

namespace MyVoxelViewer
{

// 使用OpenGL实例化方式显示体素实例。
class VoxelInstanceViewWidget : public QOpenGLWidget
{
public:
    // 创建体素实例显示窗口。
    explicit VoxelInstanceViewWidget(QWidget* parent = nullptr);

    // 设置需要显示的体素实例。
    void setInstances(const std::vector<VoxelDisplayInstance>& instances);

    // 返回当前显示实例。
    const std::vector<VoxelDisplayInstance>& instances() const;

    // 设置是否自动旋转。
    void setAutoRotate(bool enabled);

    // 设置每帧自动旋转角度。
    void setRotateStep(double degrees);

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void createShaderProgram();
    void createBuffers();
    void updateInstanceBuffer();
    void updateBounds();

    QOpenGLExtraFunctions* m_functions = nullptr; // 当前OpenGL函数入口。
    QOpenGLShaderProgram m_program; // 实例化显示Shader。
    QOpenGLVertexArrayObject m_vao; // 顶点数组对象。
    QOpenGLBuffer m_vertexBuffer; // 立方体顶点缓冲。
    QOpenGLBuffer m_indexBuffer; // 立方体索引缓冲。
    QOpenGLBuffer m_instanceBuffer; // 实例数据缓冲。

    std::vector<VoxelDisplayInstance> m_instances; // 当前显示实例集合。
    QVector3D m_center; // 当前实例包围盒中心。
    float m_radius = 1.0f; // 当前实例包围球半径。
    float m_angle = 30.0f; // 当前观察角度。
    float m_rotateStep = 0.35f; // 每帧自动旋转角度。
    bool m_initialized = false; // OpenGL资源是否已经初始化。
    bool m_autoRotate = true; // 是否自动旋转。
};

}

#endif // MYVOXELVIEWER_VOXELINSTANCEVIEWWIDGET_H