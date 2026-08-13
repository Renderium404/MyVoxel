#include "VoxelOpenGLWidget.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>

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

#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"

namespace
{

const float Pi = 3.14159265358979323846f; // 角度与弧度转换使用的圆周率。
const float MinimumRadius = 0.001f; // 空场景和极小场景使用的最小包围球半径。
const float CameraVerticalFieldOfView = 45.0f; // 后台透视投影固定使用45度垂直视场角，平移像素比例必须与其保持一致。
const float RotationReferencePixels = 720.0f; // 旋转灵敏度参考尺寸，小视口不因尺寸变小而突然变得过于敏感。
const float RotationDegreesPerReference = 180.0f; // 在参考尺寸上拖动720像素约旋转180度。
const float RotationMinimumGain = 0.90f; // 低速小幅拖动降低旋转增益，提升精细调整稳定性。
const float RotationMaximumGain = 1.30f; // 快速拖动允许提高旋转增益，避免大角度调整过慢。
const float RotationGainDistance = 24.0f; // 单次鼠标移动达到24像素时使用最大旋转增益。
const float PrecisionRotationMultiplier = 0.35f; // 按住Shift时使用35%旋转速度进行精细调整。
const float WheelZoomFactorPerStep = 0.88f; // 每120单位标准滚轮角度缩放12%，正反方向使用严格互逆比例。
const float CameraDirectionEpsilon = 1.0e-6f; // 构造相机姿态时判断方向平行或退化使用的单位向量阈值。
const float MinimumCameraDistanceRatio = 1.0e-6f; // 相机距离最低保留场景参考半径的一百万分之一，仅用于避免eye与viewCenter重合。
const float MinimumCameraDistanceAbsolute = 1.0e-6f; // 极小场景仍保留1e-6世界单位观察距离，属于数值安全下限而非用户缩放范围。
const float MinimumPanDistanceRatio = 1.0e-3f; // 极限放大时平移交互至少按场景半径千分之一的观察距离换算，避免每像素世界位移趋近于零。

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

// 根据视口尺寸和当前鼠标移动幅度返回旋转角速度，小幅移动更精细，大幅移动仍保持足够响应。
float rotationDegreesPerPixel(const QSize& viewportSize, const QPoint& delta, bool precision)
{
    const float viewportReference = (std::max)(RotationReferencePixels, static_cast<float>((std::min)(viewportSize.width(), viewportSize.height())));
    const float distance = std::sqrt(static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y()));
    const float gainRatio = clampFloat(distance / RotationGainDistance, 0.0f, 1.0f);
    const float gain = RotationMinimumGain + (RotationMaximumGain - RotationMinimumGain) * gainRatio;
    const float precisionMultiplier = precision ? PrecisionRotationMultiplier : 1.0f;
    return RotationDegreesPerReference / viewportReference * gain * precisionMultiplier;
}

// 返回将一个单位方向旋转到另一个单位方向的最短弧四元数。
QQuaternion rotationBetweenDirections(const QVector3D& sourceDirection, const QVector3D& targetDirection)
{
    const QVector3D source = sourceDirection.normalized();
    const QVector3D target = targetDirection.normalized();
    const float dot = clampFloat(QVector3D::dotProduct(source, target), -1.0f, 1.0f);
    QVector3D axis = QVector3D::crossProduct(source, target);

    if (axis.lengthSquared() <= CameraDirectionEpsilon * CameraDirectionEpsilon)
    {
        if (dot > 0.0f)
        {
            return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }

        const QVector3D reference = std::fabs(source.x()) < 0.9f ? QVector3D(1.0f, 0.0f, 0.0f) : QVector3D(0.0f, 1.0f, 0.0f); // 反向向量没有唯一旋转轴，选择与源方向不平行的基础轴。
        axis = QVector3D::crossProduct(source, reference).normalized();
        return QQuaternion::fromAxisAndAngle(axis, 180.0f);
    }

    const float angle = std::acos(dot) * 180.0f / Pi;
    return QQuaternion::fromAxisAndAngle(axis.normalized(), angle);
}

// 根据观察方向和期望屏幕向上方向构造完整相机姿态，不使用欧拉角。
QQuaternion cameraRotationFromDirectionUp(const QVector3D& direction, const QVector3D& upReference)
{
    const QVector3D cameraDirection = direction.normalized();
    QVector3D desiredUp = upReference - cameraDirection * QVector3D::dotProduct(upReference, cameraDirection);

    if (desiredUp.lengthSquared() <= CameraDirectionEpsilon * CameraDirectionEpsilon)
    {
        const QVector3D fallbackUp = std::fabs(cameraDirection.z()) < 0.9f ? QVector3D(0.0f, 0.0f, 1.0f) : QVector3D(0.0f, 1.0f, 0.0f); // 期望Up与观察方向共线时选择稳定备用轴。
        desiredUp = fallbackUp - cameraDirection * QVector3D::dotProduct(fallbackUp, cameraDirection);
    }

    desiredUp.normalize();
    const QQuaternion directionRotation = rotationBetweenDirections(QVector3D(0.0f, 0.0f, 1.0f), cameraDirection);
    const QVector3D currentUp = directionRotation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)).normalized();
    const float rollSin = QVector3D::dotProduct(QVector3D::crossProduct(currentUp, desiredUp), cameraDirection);
    const float rollCos = clampFloat(QVector3D::dotProduct(currentUp, desiredUp), -1.0f, 1.0f);
    const float rollAngle = std::atan2(rollSin, rollCos) * 180.0f / Pi;
    const QQuaternion rollRotation = QQuaternion::fromAxisAndAngle(cameraDirection, rollAngle);
    return (rollRotation * directionRotation).normalized();
}

// 返回当前相机参考半径对应的最小有效观察距离，仅防止视图矩阵数值退化。
float minimumCameraDistance(float radius)
{
    return (std::max)(radius * MinimumCameraDistanceRatio, MinimumCameraDistanceAbsolute);
}

// 返回左键平移使用的交互参考距离；正常缩放跟随真实相机距离，极限放大时仅限制交互灵敏度继续衰减。
float panInteractionDistance(float radius, float cameraScale)
{
    const float actualDistance = (std::max)(radius * cameraScale, minimumCameraDistance(radius));
    return (std::max)(actualDistance, radius * MinimumPanDistanceRatio);
}

// 返回当前观察中心平面上一个屏幕像素对应的世界空间距离，使左键平移与45度透视投影严格匹配。
float worldUnitsPerPixel(float cameraDistance, const QSize& viewportSize)
{
    const float viewportHeight = static_cast<float>((std::max)(viewportSize.height(), 1));
    return 2.0f * cameraDistance * std::tan(degreeToRadian(CameraVerticalFieldOfView * 0.5f)) / viewportHeight;
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
    : resourceKind(MyVoxel::Display_ResourceKind::Mesh)
    , usage(MyVoxel::Display_ObjectUsage::Static)
    , stateVersion(0)
    , visible(true)
    , lineWidth(1.0f)
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
    , m_cameraRotation(cameraRotationFromDirectionUp(QVector3D(1.0f, 1.0f, 1.0f), QVector3D(0.0f, 0.0f, 1.0f)))
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

bool VoxelOpenGLWidget::submitDisplaySnapshot(const MyVoxel::Display_ObjectSnapshot& snapshot)
{
    if (!snapshot.isValid())
    {
        return false;
    }

    const SnapshotClock::time_point begin = SnapshotClock::now();

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo*& slot = m_sceneObjects[snapshot.objectId];

        if (!slot)
        {
            slot = new SceneObjectInfo();
        }

        SceneObjectInfo& object = *slot;
        object.resourceKind = snapshot.resourceKind;
        object.usage = snapshot.usage;
        object.stateVersion = snapshot.stateVersion;
        object.modelMatrix = toQMatrix4x4(snapshot.localToWorld);
        object.visible = snapshot.visible;
        object.lineWidth = snapshot.lineWidth;
        object.parts.clear();

        for (std::size_t index = 0; index < snapshot.parts.size(); ++index)
        {
            const MyVoxel::Display_ObjectPartSnapshot& source = snapshot.parts[index];
            PartInfo part;
            part.version = source.version;
            part.active = true;
            setPartResourceInfo(part, *source.resource);
            object.parts.insert(std::make_pair(source.partId, part));
        }

        rebuildObjectBounds(object);
        m_lastUpdatedMeshPartCount = snapshot.resourceKind == MyVoxel::Display_ResourceKind::Mesh ? snapshot.parts.size() : 0;
    }

    m_renderThread->enqueueReplaceObject(snapshot, elapsedMilliseconds(begin, SnapshotClock::now()), 0.0);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::submitDisplayPartsUpdate(const MyVoxel::Display_ObjectPartsUpdate& update)
{
    if (!update.isValid())
    {
        return false;
    }

    const SnapshotClock::time_point begin = SnapshotClock::now();
    MyVoxel::Display_ObjectPartsUpdate filtered;
    filtered.objectId = update.objectId;
    filtered.resourceKind = update.resourceKind;

    {
        QMutexLocker locker(&m_sceneMutex);
        SceneObjectInfo* object = findSceneObjectLocked(update.objectId);

        if (!object || object->resourceKind != update.resourceKind)
        {
            m_lastUpdatedMeshPartCount = 0;
            return false;
        }

        filtered.parts.reserve(update.parts.size());

        for (std::size_t index = 0; index < update.parts.size(); ++index)
        {
            const MyVoxel::Display_ObjectPartUpdate& source = update.parts[index];
            SceneObjectInfo::PartMap::iterator iterator = object->parts.find(source.partId);

            if (iterator != object->parts.end() && source.version <= iterator->second.version)
            {
                continue;
            }

            PartInfo& part = object->parts[source.partId];
            part.version = source.version;

            if (source.operation == MyVoxel::Display_ObjectPartOperation::Replace)
            {
                part.active = true;
                setPartResourceInfo(part, *source.resource);
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
        m_lastUpdatedMeshPartCount = update.resourceKind == MyVoxel::Display_ResourceKind::Mesh ? filtered.parts.size() : 0;
    }

    if (filtered.parts.empty())
    {
        return true;
    }

    m_renderThread->enqueueUpdateParts(filtered, elapsedMilliseconds(begin, SnapshotClock::now()), 0.0);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::submitDisplayStateUpdate(const MyVoxel::Display_ObjectStateUpdate& update)
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

        if (update.stateVersion <= object->stateVersion)
        {
            return true;
        }

        if (update.hasLineWidth && object->resourceKind != MyVoxel::Display_ResourceKind::Line)
        {
            return false;
        }

        if (update.hasUsage)
        {
            object->usage = update.usage;
        }

        if (update.hasLocalToWorld)
        {
            object->modelMatrix = toQMatrix4x4(update.localToWorld);
        }

        if (update.hasVisible)
        {
            object->visible = update.visible;
        }

        if (update.hasLineWidth)
        {
            object->lineWidth = update.lineWidth;
        }

        object->stateVersion = update.stateVersion;
    }

    m_renderThread->enqueueStateUpdate(update);
    requestBoundsUpdate();
    return true;
}

bool VoxelOpenGLWidget::removeDisplayObject(MyVoxel::Display_ObjectId objectId)
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

bool VoxelOpenGLWidget::containsDisplayObject(MyVoxel::Display_ObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    return findSceneObjectLocked(objectId) != nullptr;
}

std::size_t VoxelOpenGLWidget::displayObjectCount() const
{
    QMutexLocker locker(&m_sceneMutex);
    return m_sceneObjects.size();
}

std::size_t VoxelOpenGLWidget::displayObjectPartCount(MyVoxel::Display_ObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object)
    {
        return 0;
    }

    std::size_t count = 0;

    for (SceneObjectInfo::PartMap::const_iterator iterator = object->parts.begin(); iterator != object->parts.end(); ++iterator)
    {
        if (iterator->second.active)
        {
            ++count;
        }
    }

    return count;
}

std::size_t VoxelOpenGLWidget::displayObjectTriangleCount(MyVoxel::Display_ObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object || object->resourceKind != MyVoxel::Display_ResourceKind::Mesh)
    {
        return 0;
    }

    std::size_t count = 0;

    for (SceneObjectInfo::PartMap::const_iterator iterator = object->parts.begin(); iterator != object->parts.end(); ++iterator)
    {
        if (iterator->second.active)
        {
            count += iterator->second.triangleCount;
        }
    }

    return count;
}

std::size_t VoxelOpenGLWidget::displayObjectSegmentCount(MyVoxel::Display_ObjectId objectId) const
{
    QMutexLocker locker(&m_sceneMutex);
    const SceneObjectInfo* object = findSceneObjectLocked(objectId);

    if (!object || object->resourceKind != MyVoxel::Display_ResourceKind::Line)
    {
        return 0;
    }

    std::size_t count = 0;

    for (SceneObjectInfo::PartMap::const_iterator iterator = object->parts.begin(); iterator != object->parts.end(); ++iterator)
    {
        if (iterator->second.active)
        {
            count += iterator->second.segmentCount;
        }
    }

    return count;
}

bool VoxelOpenGLWidget::isDisplayObjectVisible(MyVoxel::Display_ObjectId objectId) const
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

const MeshUpdateStatistics& VoxelOpenGLWidget::lastMeshUpdateStatistics() const
{
    return m_lastMeshUpdateStatistics;
}

const QString& VoxelOpenGLWidget::backgroundRenderError() const
{
    return m_backgroundRenderError;
}

/// 显示模式

void VoxelOpenGLWidget::setWireframe(bool enabled)
{
    if (m_wireframe == enabled)
    {
        return;
    }

    m_wireframe = enabled;
    m_renderThread->enqueueWireframe(enabled);
}

bool VoxelOpenGLWidget::isWireframe() const
{
    return m_wireframe;
}

/// 相机控制

void VoxelOpenGLWidget::fitAll()
{
    assert(QThread::currentThread() == thread());
    updateBounds();
    m_viewOffset = QVector3D(0.0f, 0.0f, 0.0f);
    m_cameraScale = 2.8f;
    submitCameraState();
}

void VoxelOpenGLWidget::setIsometricView()
{
    m_cameraRotation = cameraRotationFromDirectionUp(QVector3D(1.0f, 1.0f, 1.0f), QVector3D(0.0f, 0.0f, 1.0f));
    fitAll();
}

void VoxelOpenGLWidget::setFrontView()
{
    m_cameraRotation = cameraRotationFromDirectionUp(QVector3D(0.0f, 1.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f));
    fitAll();
}

void VoxelOpenGLWidget::setTopView()
{
    m_cameraRotation = cameraRotationFromDirectionUp(QVector3D(0.0f, 0.0f, 1.0f), QVector3D(0.0f, -1.0f, 0.0f));
    fitAll();
}

void VoxelOpenGLWidget::setRightView()
{
    m_cameraRotation = cameraRotationFromDirectionUp(QVector3D(1.0f, 0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f));
    fitAll();
}

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
    if (m_functions)
    {
        m_functions->glViewport(0, 0, width, height);
    }

    m_viewportSize = QSize((std::max)(width, 1), (std::max)(height, 1));
    submitCameraState();
}

void VoxelOpenGLWidget::paintGL()
{
    if (!m_functions)
    {
        return;
    }

    m_functions->glClear(GL_COLOR_BUFFER_BIT);

    if (!m_glInitialized || m_presentTextureId == 0)
    {
        return;
    }

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

    if (delta.isNull())
    {
        event->accept();
        return;
    }

    if (event->buttons() & Qt::MiddleButton)
    {
        const bool precision = (event->modifiers() & Qt::ShiftModifier) != 0;
        const float rotateScale = rotationDegreesPerPixel(m_viewportSize, delta, precision);
        const float horizontalAngle = -static_cast<float>(delta.x()) * rotateScale;
        const float verticalAngle = -static_cast<float>(delta.y()) * rotateScale;
        const QVector3D rotationVector = cameraUp() * horizontalAngle + cameraRight() * verticalAngle;
        const float rotationAngle = rotationVector.length();

        if (rotationAngle > 0.0f)
        {
            const QQuaternion incrementalRotation = QQuaternion::fromAxisAndAngle(rotationVector / rotationAngle, rotationAngle);
            m_cameraRotation = (incrementalRotation * m_cameraRotation).normalized();
            submitCameraState();
        }
    }
    else if (event->buttons() & Qt::LeftButton)
    {
        const float moveScale = worldUnitsPerPixel(panInteractionDistance(m_radius, m_cameraScale), m_viewportSize);
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
    const float wheelSteps = static_cast<float>(event->angleDelta().y()) / 120.0f; // Qt标准滚轮一格为120单位，同时兼容高分辨率滚轮的部分步进。

    if (wheelSteps != 0.0f)
    {
        const float oldCameraDistance = (std::max)(m_radius * m_cameraScale, minimumCameraDistance(m_radius));
        const float zoomedScale = m_cameraScale * std::pow(WheelZoomFactorPerStep, wheelSteps);
        const float safeScale = minimumCameraDistance(m_radius) / (std::max)(m_radius, MinimumRadius);

        if (zoomedScale > 0.0f && zoomedScale <= (std::numeric_limits<float>::max)())
        {
            const float newCameraScale = (std::max)(zoomedScale, safeScale);
            const float newCameraDistance = (std::max)(m_radius * newCameraScale, minimumCameraDistance(m_radius));
            const float oldWorldUnitsPerPixel = worldUnitsPerPixel(oldCameraDistance, m_viewportSize);
            const float newWorldUnitsPerPixel = worldUnitsPerPixel(newCameraDistance, m_viewportSize);
            const float worldUnitsDifference = oldWorldUnitsPerPixel - newWorldUnitsPerPixel;
            const float mouseX = static_cast<float>(event->pos().x()) - static_cast<float>(m_viewportSize.width()) * 0.5f;
            const float mouseY = static_cast<float>(event->pos().y()) - static_cast<float>(m_viewportSize.height()) * 0.5f;

            m_viewOffset += cameraRight() * mouseX * worldUnitsDifference;
            m_viewOffset -= cameraUp() * mouseY * worldUnitsDifference;
            m_cameraScale = newCameraScale;
            submitCameraState();
        }
    }

    event->accept();
}

/// CPU场景元数据

VoxelOpenGLWidget::SceneObjectInfo* VoxelOpenGLWidget::findSceneObjectLocked(MyVoxel::Display_ObjectId objectId)
{
    SceneObjectMap::iterator iterator = m_sceneObjects.find(objectId);
    return iterator == m_sceneObjects.end() ? nullptr : iterator->second;
}

const VoxelOpenGLWidget::SceneObjectInfo* VoxelOpenGLWidget::findSceneObjectLocked(MyVoxel::Display_ObjectId objectId) const
{
    SceneObjectMap::const_iterator iterator = m_sceneObjects.find(objectId);
    return iterator == m_sceneObjects.end() ? nullptr : iterator->second;
}

void VoxelOpenGLWidget::setPartResourceInfo(PartInfo& part, const MyVoxel::Display_Resource& resource)
{
    part.triangleCount = 0;
    part.segmentCount = 0;
    part.localBounds = resource.localBounds();

    if (resource.kind() == MyVoxel::Display_ResourceKind::Mesh)
    {
        part.triangleCount = static_cast<const MyVoxel::Display_MeshResource&>(resource).triangleCount();
    }
    else
    {
        part.segmentCount = static_cast<const MyVoxel::Display_LineResource&>(resource).segmentCount();
    }
}

void VoxelOpenGLWidget::rebuildObjectBounds(SceneObjectInfo& object)
{
    object.localBounds.clear();

    for (SceneObjectInfo::PartMap::const_iterator iterator = object.parts.begin(); iterator != object.parts.end(); ++iterator)
    {
        if (iterator->second.active)
        {
            object.localBounds.include(iterator->second.localBounds);
        }
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
    if (m_renderThread)
    {
        m_renderThread->stopRendering();
    }

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
    if (!m_glInitialized && !m_renderContext && !m_offscreenSurface && !m_presentVao.isCreated())
    {
        return;
    }

    stopBackgroundRenderer();
    QCoreApplication::removePostedEvents(this, RenderFrameReadyEvent::eventType());
    QCoreApplication::removePostedEvents(this, RenderFailureEvent::eventType());

    if (context())
    {
        makeCurrent();

        if (m_presentVao.isCreated())
        {
            m_presentVao.destroy();
        }

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

            if (object.visible)
            {
                expandWorldBounds(object.localBounds, object.modelMatrix, hasPoint, minimum, maximum);
            }
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
    state.rotation = m_cameraRotation;
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
    return m_cameraRotation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
}

QVector3D VoxelOpenGLWidget::cameraRight() const
{
    return m_cameraRotation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f)).normalized();
}

QVector3D VoxelOpenGLWidget::cameraUp() const
{
    return m_cameraRotation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)).normalized();
}

}