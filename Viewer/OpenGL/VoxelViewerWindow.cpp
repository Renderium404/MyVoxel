#include "VoxelViewerWindow.h"

#include <QAction>
#include <QStatusBar>
#include <QString>
#include <QToolBar>

namespace MyVoxelViewer
{

VoxelViewerWindow::VoxelViewerWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_viewer(new VoxelOpenGLWidget(this))
{
    setWindowTitle(QStringLiteral("MyVoxel Background OpenGL Viewer"));
    setCentralWidget(m_viewer);
    resize(1280, 800);
    createToolBar();
    statusBar()->showMessage(QStringLiteral("Left: Rotate  Right: Pan  Wheel: Zoom  Double Click/F: Fit  1: Isometric  2: Front  3: Top  4: Right  W: Wireframe"));
}

/// 视口访问

VoxelOpenGLWidget* VoxelViewerWindow::viewer()
{
    return m_viewer;
}

const VoxelOpenGLWidget* VoxelViewerWindow::viewer() const
{
    return m_viewer;
}

/// 通用网格对象

MeshObjectId VoxelViewerWindow::addMesh(const MyVoxel::Geometry::Mesh& mesh, const QMatrix4x4& modelMatrix)
{
    return m_viewer->addMesh(mesh, modelMatrix);
}

bool VoxelViewerWindow::setMesh(MeshObjectId objectId, const MyVoxel::Geometry::Mesh& mesh)
{
    return m_viewer->setMesh(objectId, mesh);
}

MeshObjectId VoxelViewerWindow::addMeshCache(const MyVoxel::VoxelSurfaceCache& cache, const QMatrix4x4& modelMatrix)
{
    return m_viewer->addMeshCache(cache, modelMatrix);
}

bool VoxelViewerWindow::setMeshCache(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache)
{
    return m_viewer->setMeshCache(objectId, cache);
}

bool VoxelViewerWindow::updateRootMeshes(MeshObjectId objectId, const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices)
{
    return m_viewer->updateRootMeshes(objectId, cache, changedRootIndices);
}

bool VoxelViewerWindow::setMeshObjectMatrix(MeshObjectId objectId, const QMatrix4x4& matrix)
{
    return m_viewer->setMeshObjectMatrix(objectId, matrix);
}

bool VoxelViewerWindow::setMeshObjectVisible(MeshObjectId objectId, bool visible)
{
    return m_viewer->setMeshObjectVisible(objectId, visible);
}

bool VoxelViewerWindow::removeMeshObject(MeshObjectId objectId)
{
    return m_viewer->removeMeshObject(objectId);
}

void VoxelViewerWindow::clearMeshes()
{
    m_viewer->clearMeshes();
}

std::size_t VoxelViewerWindow::meshObjectCount() const
{
    return m_viewer->meshObjectCount();
}

/// 单体素对象兼容入口

void VoxelViewerWindow::setMeshCache(const MyVoxel::VoxelSurfaceCache& cache)
{
    m_viewer->setMeshCache(cache);
}

void VoxelViewerWindow::updateRootMeshes(const MyVoxel::VoxelSurfaceCache& cache, const MyVoxel::VoxelSurfaceCache::RootIndexSet& changedRootIndices)
{
    m_viewer->updateRootMeshes(cache, changedRootIndices);
}

/// 界面构建

void VoxelViewerWindow::createToolBar()
{
    QToolBar* toolBar = addToolBar(QStringLiteral("View"));
    toolBar->setMovable(false);
    QAction* fitAction = toolBar->addAction(QStringLiteral("Fit"));
    QAction* isometricAction = toolBar->addAction(QStringLiteral("Isometric"));
    QAction* frontAction = toolBar->addAction(QStringLiteral("Front"));
    QAction* topAction = toolBar->addAction(QStringLiteral("Top"));
    QAction* rightAction = toolBar->addAction(QStringLiteral("Right"));
    toolBar->addSeparator();
    QAction* wireframeAction = toolBar->addAction(QStringLiteral("Wireframe"));
    wireframeAction->setCheckable(true);
    connect(fitAction, &QAction::triggered, m_viewer, &VoxelOpenGLWidget::fitAll);
    connect(isometricAction, &QAction::triggered, m_viewer, &VoxelOpenGLWidget::setIsometricView);
    connect(frontAction, &QAction::triggered, m_viewer, &VoxelOpenGLWidget::setFrontView);
    connect(topAction, &QAction::triggered, m_viewer, &VoxelOpenGLWidget::setTopView);
    connect(rightAction, &QAction::triggered, m_viewer, &VoxelOpenGLWidget::setRightView);
    connect(wireframeAction, &QAction::toggled, m_viewer, &VoxelOpenGLWidget::setWireframe);
}

}