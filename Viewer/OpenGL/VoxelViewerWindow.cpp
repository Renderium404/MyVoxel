#include "VoxelViewerWindow.h"

#include <QAction>
#include <QStatusBar>
#include <QString>
#include <QToolBar>

#include "VoxelOpenGLWidget.h"

namespace MyVoxelViewer
{

VoxelViewerWindow::VoxelViewerWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_viewer(new VoxelOpenGLWidget(this))
{
    setWindowTitle(QStringLiteral("MyVoxel Display OpenGL Viewer"));
    setCentralWidget(m_viewer);
    resize(1280, 800);
    createToolBar();
    statusBar()->showMessage(QStringLiteral("Middle: Rotate  Shift+Middle: Precision Rotate  Left: Pan  Wheel: Zoom  Double Click/F: Fit  1: Isometric  2: Front  3: Top  4: Right  W: Mesh Wireframe"));
}

VoxelOpenGLWidget* VoxelViewerWindow::viewer()
{
    return m_viewer;
}

const VoxelOpenGLWidget* VoxelViewerWindow::viewer() const
{
    return m_viewer;
}

bool VoxelViewerWindow::submitDisplaySnapshot(const MyVoxel::Display_ObjectSnapshot& snapshot)
{
    return m_viewer->submitDisplaySnapshot(snapshot);
}

bool VoxelViewerWindow::submitDisplayPartsUpdate(const MyVoxel::Display_ObjectPartsUpdate& update)
{
    return m_viewer->submitDisplayPartsUpdate(update);
}

bool VoxelViewerWindow::submitDisplayStateUpdate(const MyVoxel::Display_ObjectStateUpdate& update)
{
    return m_viewer->submitDisplayStateUpdate(update);
}

bool VoxelViewerWindow::removeDisplayObject(MyVoxel::Display_ObjectId objectId)
{
    return m_viewer->removeDisplayObject(objectId);
}

void VoxelViewerWindow::clearDisplayObjects()
{
    m_viewer->clearDisplayObjects();
}

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