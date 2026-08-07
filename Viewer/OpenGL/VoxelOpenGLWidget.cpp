#include "VoxelOpenGLWidget.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <utility>

#include <QCoreApplication>
#include <QDebug>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QMutexLocker>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>
#include <QSurfaceFormat>
#include <QThread>
#include <QWheelEvent>

namespace
{

const float Pi = 3.14159265358979323846f; // 角度与弧度转换使用的圆周率。
const float MinimumRadius = 0.001f; // 空场景和极小场景使用的最小包围球半径。
const float MinimumCameraScale = 0.35f; // 滚轮缩放允许的最小相机距离比例。
const float MaximumCameraScale = 50.0f; // 滚轮缩放允许的最大相机距离比例。

typedef std::chrono::steady_clock SnapshotClock;

double elapsedMilliseconds(const SnapshotClock::time_point& begin, const SnapshotClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

float clampFloat(float value, float minimum, float maximum)
{
    return (std::max)(minimum, (std::min)(value, maximum));
}

float degreeToRadian(float degree)
{
    return degree * Pi / 180.0f;
}

void expandWorldBounds(const MyVoxel::Bounds3& localBounds, const QMatrix4x4& modelMatrix,
                       bool& hasPoint, QVector3D& minimum, QVector3D& maximum)
{
    if (!localBounds.isValid())
    {
        return;
    }

    const MyMath::Vector3& localMinimum = localBounds.minimum();
    const MyMath::Vector3& localMaximum = localBounds.maximum();
    const float x[2] = {static_cast<float>(localMinimum.x()), static_cast<float>(localMaximum.x())};
    const float y[2] = {static_cast<float>(localMinimum.y()), static_cast<float>(localMaximum.y())};
    const float z[2] = {static_cast<float>(localMinimum.z()), static_cast<float>(localMaximum.z())};

    for (unsigned int xi = 0; xi < 2; ++xi)
    {
        for (unsigned int yi = 0; yi < 2; ++yi)
        {
            for (unsigned int zi = 0; zi < 2; ++zi)
            {
                const QVector3D point = modelMatrix.map(QVector3D(x[xi], y[yi], z[zi]));

                if (!hasPoint)
                {
                    minimum = point;
                    maximum = point;
                    hasPoint = true;
                    continue;
                }

                minimum.setX((std::min)(minimum.x(), point.x()));
                minimum.setY((std::min)(minimum.y(), point.y()));
                minimum.setZ((std::min)(minimum.z(), point.z()));
                maximum.setX((std::max)(maximum.x(), point.x()));
                maximum.setY((std::max)(maximum.y(), point.y()));
                maximum.setZ((std::max)(maximum.z(), point.z()));
            }
        }
    }
}

QEvent::Type sceneBoundsChangedEventType()
{
    static const int type = QEvent::registerEventType();
    return static_cast<QEvent::Type>(type);
}

}

namespace MyVoxelViewer
{

VoxelOpenGLWidget::PartInfo::PartInfo()
    : version(0)
    , active(false)
    , triangleCount(0)
    , segmentCount(0)
{
}

VoxelOpenGLWidget::SceneObjectInfo::SceneObjectInfo()
    : kind(SceneMeshObject)
    , visible(true)
{
    modelMatrix.setToIdentity();
}

VoxelOpenGLWidget::VoxelOpenGLWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_lastUpdatedMeshPartCount(0)
    , m_sceneBoundsEventPending(0)
    , m_renderThread(new VoxelRenderThread())
    , m_renderContext(nullptr)
    , m_offscreenSurface(nullptr)
    , m_glInitialized(false)
    , m_wireframe(false)
    , m_functions(nullptr)
    , m_presentTextureId(0)
    , m_presentFrameId(0)
    , m_presentFrameVersion(0)
    , m_center(0.0f, 0.0f, 0.0f)
    , m_viewOffset(0.0f, 0.0f, 0.0f)
    , m_radius(1.0f)
    , m_cameraScale(2.8f)
    , m_yaw(45.0f)
    , m_pitch(35.264f)
    , m_viewportSize(1, 1)
{
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(0); // 当前后台FBO使用普通共享颜色纹理，不启用MSAA。
    setFormat(format);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(false);
    setMinimumSize(480, 320);
}

VoxelOpenGLWidget::~VoxelOpenGLWidget()
{
    cleanupOpenGL();
    QMutexLocker locker(&m_sceneMutex);

    for (SceneObjectMap::iterator iterator = m_sceneObjects.begin(); iterator != m_sceneObjects.end(); ++iterator)
    {
        delete iterator->second;
    }

    m_sceneObjects.clear();
    locker.unlock();
    delete m_renderThread;
    m_renderThread = nullptr;
}

/// Display对象提交

bool VoxelOpenGLWidget::submitDisplaySnapshot(const MyVoxel::Display_MeshObjectSnapshot& snapshot)
{
    if (!snapshot.isValid())
    {
        return false;
    }

    const SnapshotClock::time_point begin = SnapshotClock::now();

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo*& slot = m_sceneObjects[snapshot.objectId];

        if (!slot) slot = new SceneObjectInfo();

        SceneObjectInfo& object = *slot;
        object.kind = SceneMeshObject;
        object.modelMatrix = toQMatrix4x4(snapshot.localToWorld);
        object.visible = snapshot.visible;
        object.parts.clear();

        for (std::size_t index = 0; index < snapshot.parts.size(); ++index)
        {
            const MyVoxel::Display_MeshPartSnapshot& source = snapshot.parts[index];
            PartInfo part;
            part.version = source.version;
            part.active = true;
            part.triangleCount = source.resource->triangleCount();
            part.localBounds = source.resource->localBounds();
            object.parts.insert(std::make_pair(source.partId, part));
        }

        rebuildObjectBounds(object);
        m_lastUpdatedMeshPartCount = snapshot.parts.size();
    }

    m_renderThread->enqueueReplaceObject(snapshot, elapsedMilliseconds(begin, SnapshotClock::now()), 0.0);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::submitDisplaySnapshot(const MyVoxel::Display_LineObjectSnapshot& snapshot)
{
    if (!snapshot.isValid())
    {
        return false;
    }

    const SnapshotClock::time_point begin = SnapshotClock::now();

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo*& slot = m_sceneObjects[snapshot.objectId];

        if (!slot) slot = new SceneObjectInfo();

        SceneObjectInfo& object = *slot;
        object.kind = SceneLineObject;
        object.modelMatrix = toQMatrix4x4(snapshot.localToWorld);
        object.visible = snapshot.visible;
        object.parts.clear();

        for (std::size_t index = 0; index < snapshot.parts.size(); ++index)
        {
            const MyVoxel::Display_LinePartSnapshot& source = snapshot.parts[index];
            PartInfo part;
            part.version = source.version;
            part.active = true;
            part.segmentCount = source.resource->segmentCount();
            part.localBounds = source.resource->localBounds();
            object.parts.insert(std::make_pair(source.partId, part));
        }

        rebuildObjectBounds(object);
    }

    m_renderThread->enqueueReplaceObject(snapshot, elapsedMilliseconds(begin, SnapshotClock::now()), 0.0);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::submitDisplayUpdate(const MyVoxel::Display_MeshUpdate& update)
{
    if (!update.isValid())
    {
        return false;
    }

    const SnapshotClock::time_point begin = SnapshotClock::now();
    MyVoxel::Display_MeshUpdate filtered;
    filtered.objectId = update.objectId;

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(update.objectId);

        if (!object || object->kind != SceneMeshObject)
        {
            m_lastUpdatedMeshPartCount = 0;
            return false;
        }

        filtered.parts.reserve(update.parts.size());

        for (std::size_t index = 0; index < update.parts.size(); ++index)
        {
            const MyVoxel::Display_MeshPartUpdate& source = update.parts[index];
            SceneObjectInfo::PartMap::iterator iterator = object->parts.find(source.partId);

            if (iterator != object->parts.end() && source.version <= iterator->second.version)
            {
                continue;
            }

            PartInfo& part = object->parts[source.partId];
            part.version = source.version;

            if (source.operation == MyVoxel::Display_MeshPartOperation::Replace)
            {
                part.active = true;
                part.triangleCount = source.resource->triangleCount();
                part.segmentCount = 0;
                part.localBounds = source.resource->localBounds();
            }
            else
            {
                part.active = false;
                part.triangleCount = 0;
                part.segmentCount = 0;
                part.localBounds.clear();
            }

            filtered.parts.push_back(source);
        }

        rebuildObjectBounds(*object);
        m_lastUpdatedMeshPartCount = filtered.parts.size();
    }

    if (filtered.parts.empty())
    {
        return true;
    }

    m_renderThread->enqueueUpdateParts(filtered, elapsedMilliseconds(begin, SnapshotClock::now()), 0.0);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::submitDisplayStateUpdate(const MyVoxel::Display_MeshStateUpdate& update)
{
    if (!update.isValid())
    {
        return false;
    }

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(update.objectId);

        if (!object)
        {
            return false;
        }

        if (update.hasLocalToWorld) object->modelMatrix = toQMatrix4x4(update.localToWorld);
        if (update.hasVisible) object->visible = update.visible;
    }

    m_renderThread->enqueueStateUpdate(update);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::removeDisplayObject(std::uint64_t objectId)
{
    SceneObjectInfo* removed = nullptr;

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectMap::iterator iterator = m_sceneObjects.find(objectId);

        if (iterator == m_sceneObjects.end())
        {
            return false;
        }

        removed = iterator->second;
        m_sceneObjects.erase(iterator);
    }

    delete removed;
    m_renderThread->enqueueRemoveObject(objectId);
    requestBoundsUpdate();
    return true;
}

void VoxelOpenGLWidget::clearDisplayObjects()
{
    SceneObjectMap oldObjects;

    {
        QMutexLocker locker(&m_sceneMutex);
        oldObjects.swap(m_sceneObjects);
        m_lastUpdatedMeshPartCount = 0;
    }

    for (SceneObjectMap::iterator iterator = oldObjects.begin(); iterator != oldObjects.end(); ++iterator)
    {
        delete iterator->second;
    }

    m_renderThread->enqueueClearObjects();
    requestBoundsUpdate();
}

/// Display对象查询

bool VoxelOpenGLWidget::containsDisplayObject(std::uint64_t objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    return findSceneObjectLocked(objectId) != nullptr;
}

std::size_t VoxelOpenGLWidget::displayObjectCount() const
{
    QMutexLocker locker(&m_sceneMutex);
    return m_sceneObjects.size();
}

std::size_t VoxelOpenGLWidget::displayObjectPartCount(std::uint64_t objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object) return 0;

    std::size_t count = 0;

    for (SceneObjectInfo::PartMap::const_iterator iterator = object->parts.begin(); iterator != object->parts.end(); ++iterator)
    {
        if (iterator->second.active) ++count;
    }

    return count;
}

std::size_t VoxelOpenGLWidget::displayObjectTriangleCount(std::uint64_t objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object || object->kind != SceneMeshObject) return 0;

    std::size_t count = 0;

    for (SceneObjectInfo::PartMap::const_iterator iterator = object->parts.begin(); iterator != object->parts.end(); ++iterator)
    {
        if (iterator->second.active) count += iterator->second.triangleCount;
    }

    return count;
}

std::size_t VoxelOpenGLWidget::displayObjectSegmentCount(std::uint64_t objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object || object->kind != SceneLineObject) return 0;

    std::size_t count = 0;

    for (SceneObjectInfo::PartMap::const_iterator iterator = object->parts.begin(); iterator != object->parts.end(); ++iterator)
    {
        if (iterator->second.active) count += iterator->second.segmentCount;
    }

    return count;
}

bool VoxelOpenGLWidget::isDisplayObjectVisible(std::uint64_t objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);
    return object && object->visible;
}

std::size_t VoxelOpenGLWidget::lastUpdatedMeshPartCount() const
{
    QMutexLocker locker(&m_sceneMutex);
    return m_lastUpdatedMeshPartCount;
}

const MeshUpdateStatistics& VoxelOpenGLWidget::lastMeshUpdateStatistics() const { return m_lastMeshUpdateStatistics; }
const QString& VoxelOpenGLWidget::backgroundRenderError() const { return m_backgroundRenderError; }

/// 显示模式

void VoxelOpenGLWidget::setWireframe(bool enabled)
{
    if (m_wireframe == enabled) return;
    m_wireframe = enabled;
    m_renderThread->enqueueWireframe(enabled);
}

bool VoxelOpenGLWidget::isWireframe() const { return m_wireframe; }

/// 相机控制

void VoxelOpenGLWidget::fitAll()
{
    assert(QThread::currentThread() == thread());
    updateBounds();
    m_viewOffset = QVector3D(0.0f, 0.0f, 0.0f);
    m_cameraScale = 2.8f;
    submitCameraState();
}

void VoxelOpenGLWidget::setIsometricView() { m_yaw = 45.0f; m_pitch = 35.264f; fitAll(); }
void VoxelOpenGLWidget::setFrontView() { m_yaw = 90.0f; m_pitch = 0.0f; fitAll(); }
void VoxelOpenGLWidget::setTopView() { m_yaw = 90.0f; m_pitch = 89.0f; fitAll(); }
void VoxelOpenGLWidget::setRightView() { m_yaw = 0.0f; m_pitch = 0.0f; fitAll(); }

/// Qt事件和OpenGL生命周期

bool VoxelOpenGLWidget::event(QEvent* event)
{
    if (event->type() == RenderFrameReadyEvent::eventType())
    {
        RenderFrameReadyEvent* frameEvent = static_cast<RenderFrameReadyEvent*>(event);
        m_presentTextureId = frameEvent->textureId();
        m_presentFrameId = frameEvent->frameId();
        m_presentFrameVersion = frameEvent->frameVersion();
        m_presentTextureSize = frameEvent->textureSize();

        {
            QMutexLocker locker(&m_sceneMutex);
            m_lastMeshUpdateStatistics = frameEvent->statistics();
            m_backgroundRenderError.clear();
        }

        m_renderThread->acceptPresentedFrame(m_presentFrameId, m_presentFrameVersion);
        update();
        return true;
    }

    if (event->type() == RenderFailureEvent::eventType())
    {
        RenderFailureEvent* failureEvent = static_cast<RenderFailureEvent*>(event);

        {
            QMutexLocker locker(&m_sceneMutex);
            m_backgroundRenderError = failureEvent->message();
        }

        qWarning() << "MyVoxel background OpenGL renderer:" << failureEvent->message();
        update();
        return true;
    }

    if (event->type() == sceneBoundsChangedEventType())
    {
        m_sceneBoundsEventPending.store(0);
        updateBounds();
        submitCameraState();
        return true;
    }

    return QOpenGLWidget::event(event);
}

void VoxelOpenGLWidget::initializeGL()
{
    m_functions = context()->versionFunctions<QOpenGLFunctions_3_3_Core>();

    if (!m_functions)
    {
        m_backgroundRenderError = QStringLiteral("OpenGL 3.3 Core function table is unavailable in the GUI context.");
        return;
    }

    m_functions->initializeOpenGLFunctions();
    m_functions->glDisable(GL_DEPTH_TEST);
    m_functions->glClearColor(0.045f, 0.055f, 0.075f, 1.0f);

    if (!createPresentResources() || !startBackgroundRenderer())
    {
        return;
    }

    m_glInitialized = true;
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, [this]() { cleanupOpenGL(); }, Qt::DirectConnection);
    submitCameraState();
    m_renderThread->enqueueWireframe(m_wireframe);
    m_renderThread->requestFrame();
}

void VoxelOpenGLWidget::resizeGL(int width, int height)
{
    if (m_functions) m_functions->glViewport(0, 0, width, height);
    m_viewportSize = QSize((std::max)(width, 1), (std::max)(height, 1));
    submitCameraState();
}

void VoxelOpenGLWidget::paintGL()
{
    if (!m_functions) return;
    m_functions->glClear(GL_COLOR_BUFFER_BIT);

    if (!m_glInitialized || m_presentTextureId == 0) return;

    m_functions->glDisable(GL_DEPTH_TEST);
    m_functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    m_presentProgram.bind();
    m_presentProgram.setUniformValue("u_texture", 0);
    m_presentVao.bind();
    m_functions->glActiveTexture(GL_TEXTURE0);
    m_functions->glBindTexture(GL_TEXTURE_2D, m_presentTextureId);
    m_functions->glDrawArrays(GL_TRIANGLES, 0, 3);
    m_functions->glBindTexture(GL_TEXTURE_2D, 0);
    m_presentVao.release();
    m_presentProgram.release();
}

void VoxelOpenGLWidget::keyPressEvent(QKeyEvent* event)
{
    switch (event->key())
    {
    case Qt::Key_F: fitAll(); event->accept(); return;
    case Qt::Key_1: setIsometricView(); event->accept(); return;
    case Qt::Key_2: setFrontView(); event->accept(); return;
    case Qt::Key_3: setTopView(); event->accept(); return;
    case Qt::Key_4: setRightView(); event->accept(); return;
    case Qt::Key_W: setWireframe(!m_wireframe); event->accept(); return;
    default: break;
    }

    QOpenGLWidget::keyPressEvent(event);
}

void VoxelOpenGLWidget::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePosition = event->pos();
    setFocus();
    event->accept();
}

void VoxelOpenGLWidget::mouseMoveEvent(QMouseEvent* event)
{
    const QPoint delta = event->pos() - m_lastMousePosition;
    m_lastMousePosition = event->pos();

    if (event->buttons() & Qt::LeftButton)
    {
        m_yaw -= static_cast<float>(delta.x()) * 0.35f;
        m_pitch += static_cast<float>(delta.y()) * 0.35f;
        m_pitch = clampFloat(m_pitch, -89.0f, 89.0f);
        submitCameraState();
    }
    else if (event->buttons() & Qt::RightButton)
    {
        const float cameraDistance = (std::max)(m_radius * m_cameraScale, 0.1f);
        const float moveScale = cameraDistance * 0.0015f;
        m_viewOffset -= cameraRight() * static_cast<float>(delta.x()) * moveScale;
        m_viewOffset += cameraUp() * static_cast<float>(delta.y()) * moveScale;
        submitCameraState();
    }

    event->accept();
}

void VoxelOpenGLWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        fitAll();
        event->accept();
        return;
    }

    QOpenGLWidget::mouseDoubleClickEvent(event);
}

void VoxelOpenGLWidget::wheelEvent(QWheelEvent* event)
{
    if (event->angleDelta().y() > 0) m_cameraScale *= 0.90f;
    else if (event->angleDelta().y() < 0) m_cameraScale *= 1.10f;
    m_cameraScale = clampFloat(m_cameraScale, MinimumCameraScale, MaximumCameraScale);
    submitCameraState();
    event->accept();
}

/// CPU场景元数据

VoxelOpenGLWidget::SceneObjectInfo* VoxelOpenGLWidget::findSceneObjectLocked(std::uint64_t objectId)
{
    SceneObjectMap::iterator iterator = m_sceneObjects.find(objectId);
    return iterator == m_sceneObjects.end() ? nullptr : iterator->second;
}

const VoxelOpenGLWidget::SceneObjectInfo* VoxelOpenGLWidget::findSceneObjectLocked(std::uint64_t objectId) const
{
    SceneObjectMap::const_iterator iterator = m_sceneObjects.find(objectId);
    return iterator == m_sceneObjects.end() ? nullptr : iterator->second;
}

void VoxelOpenGLWidget::rebuildObjectBounds(SceneObjectInfo& object)
{
    object.localBounds.clear();

    for (SceneObjectInfo::PartMap::const_iterator iterator = object.parts.begin(); iterator != object.parts.end(); ++iterator)
    {
        if (iterator->second.active) object.localBounds.include(iterator->second.localBounds);
    }
}

/// 后台OpenGL生命周期

bool VoxelOpenGLWidget::createPresentResources()
{
    const char* vertexShader =
        "#version 330 core\n"
        "out vec2 v_uv;\n"
        "void main()\n"
        "{\n"
        "    vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));\n"
        "    vec2 position = positions[gl_VertexID];\n"
        "    gl_Position = vec4(position, 0.0, 1.0);\n"
        "    v_uv = position * 0.5 + 0.5;\n"
        "}\n";

    const char* fragmentShader =
        "#version 330 core\n"
        "in vec2 v_uv;\n"
        "uniform sampler2D u_texture;\n"
        "out vec4 fragColor;\n"
        "void main()\n"
        "{\n"
        "    fragColor = texture(u_texture, v_uv);\n"
        "}\n";

    if (!m_presentProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader) ||
        !m_presentProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader) ||
        !m_presentProgram.link())
    {
        m_backgroundRenderError = QStringLiteral("Present shader initialization failed: ") + m_presentProgram.log();
        return false;
    }

    if (!m_presentVao.create())
    {
        m_backgroundRenderError = QStringLiteral("Failed to create the present OpenGL vertex array object.");
        return false;
    }

    return true;
}

bool VoxelOpenGLWidget::startBackgroundRenderer()
{
    if (!QOpenGLContext::supportsThreadedOpenGL())
    {
        m_backgroundRenderError = QStringLiteral("The current Qt platform or OpenGL driver does not support threaded OpenGL.");
        return false;
    }

    m_offscreenSurface = new QOffscreenSurface();
    m_offscreenSurface->setFormat(context()->format());
    m_offscreenSurface->create();

    if (!m_offscreenSurface->isValid())
    {
        m_backgroundRenderError = QStringLiteral("Failed to create the background OpenGL offscreen surface.");
        delete m_offscreenSurface;
        m_offscreenSurface = nullptr;
        return false;
    }

    m_renderContext = new QOpenGLContext();
    m_renderContext->setFormat(context()->format());
    m_renderContext->setShareContext(context());

    if (!m_renderContext->create() || !m_renderContext->isValid())
    {
        m_backgroundRenderError = QStringLiteral("Failed to create the background shared OpenGL context.");
        delete m_renderContext;
        m_renderContext = nullptr;
        delete m_offscreenSurface;
        m_offscreenSurface = nullptr;
        return false;
    }

    if (!m_renderThread->startRendering(m_renderContext, m_offscreenSurface, this, thread()))
    {
        m_backgroundRenderError = QStringLiteral("Failed to start the background OpenGL render thread.");
        delete m_renderContext;
        m_renderContext = nullptr;
        delete m_offscreenSurface;
        m_offscreenSurface = nullptr;
        return false;
    }

    return true;
}

void VoxelOpenGLWidget::stopBackgroundRenderer()
{
    if (m_renderThread) m_renderThread->stopRendering();
    delete m_renderContext;
    m_renderContext = nullptr;
    delete m_offscreenSurface;
    m_offscreenSurface = nullptr;
    m_presentTextureId = 0;
    m_presentFrameId = 0;
    m_presentFrameVersion = 0;
    m_presentTextureSize = QSize();
}

void VoxelOpenGLWidget::cleanupOpenGL()
{
    if (!m_glInitialized && !m_renderContext && !m_offscreenSurface && !m_presentVao.isCreated()) return;

    stopBackgroundRenderer();
    QCoreApplication::removePostedEvents(this, RenderFrameReadyEvent::eventType());
    QCoreApplication::removePostedEvents(this, RenderFailureEvent::eventType());

    if (context())
    {
        makeCurrent();
        if (m_presentVao.isCreated()) m_presentVao.destroy();
        m_presentProgram.removeAllShaders();
        doneCurrent();
    }

    m_glInitialized = false;
    m_functions = nullptr;
}

/// 相机和场景范围

void VoxelOpenGLWidget::updateBounds()
{
    bool hasPoint = false;
    QVector3D minimum;
    QVector3D maximum;

    {
        QMutexLocker locker(&m_sceneMutex);

        for (SceneObjectMap::const_iterator iterator = m_sceneObjects.begin(); iterator != m_sceneObjects.end(); ++iterator)
        {
            const SceneObjectInfo& object = *iterator->second;

            if (object.visible) expandWorldBounds(object.localBounds, object.modelMatrix, hasPoint, minimum, maximum);
        }
    }

    if (!hasPoint)
    {
        m_center = QVector3D(0.0f, 0.0f, 0.0f);
        m_radius = 1.0f;
        return;
    }

    m_center = (minimum + maximum) * 0.5f;
    m_radius = (std::max)((maximum - minimum).length() * 0.5f, MinimumRadius);
}

void VoxelOpenGLWidget::submitCameraState()
{
    RenderCameraState state;
    state.center = m_center;
    state.viewOffset = m_viewOffset;
    state.radius = m_radius;
    state.cameraScale = m_cameraScale;
    state.yaw = m_yaw;
    state.pitch = m_pitch;
    state.viewportSize = m_viewportSize;
    m_renderThread->enqueueCameraState(state);
}

void VoxelOpenGLWidget::requestBoundsUpdate()
{
    if (QThread::currentThread() == thread())
    {
        updateBounds();
        submitCameraState();
        return;
    }

    if (m_sceneBoundsEventPending.testAndSetOrdered(0, 1))
    {
        QCoreApplication::postEvent(this, new QEvent(sceneBoundsChangedEventType()));
    }
}

QVector3D VoxelOpenGLWidget::cameraDirection() const
{
    const float yaw = degreeToRadian(m_yaw);
    const float pitch = degreeToRadian(m_pitch);
    return QVector3D(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), std::sin(pitch)).normalized();
}

QVector3D VoxelOpenGLWidget::cameraRight() const
{
    const QVector3D forward = -cameraDirection();
    return QVector3D::crossProduct(forward, QVector3D(0.0f, 0.0f, 1.0f)).normalized();
}

QVector3D VoxelOpenGLWidget::cameraUp() const
{
    const QVector3D forward = -cameraDirection();
    return QVector3D::crossProduct(cameraRight(), forward).normalized();
}

}
