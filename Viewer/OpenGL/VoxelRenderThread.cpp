#include "VoxelRenderThread.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <QCoreApplication>
#include <QMutexLocker>
#include <QOffscreenSurface>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QString>

namespace
{

const float Pi = 3.14159265358979323846f; // 角度与弧度转换使用的圆周率。
const float MinimumRadius = 0.001f; // 空场景和极小场景使用的最小包围球半径。
const std::size_t MeshVertexFloatCount = 10; // Mesh顶点包含位置3、法线3和颜色4个float。
const int MeshPositionFloatOffset = 0; // Mesh顶点位置属性float偏移。
const int MeshNormalFloatOffset = 3; // Mesh顶点法线属性float偏移。
const int MeshColorFloatOffset = 6; // Mesh顶点颜色属性float偏移。
const std::size_t LineVertexFloatCount = 7; // Line顶点包含位置3和颜色4个float。
const int LinePositionFloatOffset = 0; // Line顶点位置属性float偏移。
const int LineColorFloatOffset = 3; // Line顶点颜色属性float偏移。

static_assert(sizeof(MyVoxel::Display_MeshVertex) == MeshVertexFloatCount * sizeof(float),
              "Display_MeshVertex must contain exactly ten consecutive float values.");
static_assert(sizeof(MyVoxel::Display_LineVertex) == LineVertexFloatCount * sizeof(float),
              "Display_LineVertex must contain exactly seven consecutive float values.");

typedef std::chrono::steady_clock RenderClock;

// 返回两个稳定时钟时间点之间的毫秒数。
double elapsedMilliseconds(const RenderClock::time_point& begin, const RenderClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 将角度转换为弧度。
float degreeToRadian(float degree)
{
    return degree * Pi / 180.0f;
}

// 将字节数量转换为QOpenGLBuffer使用的有符号整数。
int checkedBufferByteSize(std::size_t byteSize)
{
    assert(byteSize <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
    return static_cast<int>(byteSize);
}

// 返回相机中心指向相机位置的单位方向。
QVector3D cameraDirection(const MyVoxelViewer::RenderCameraState& camera)
{
    const float yaw = degreeToRadian(camera.yaw);
    const float pitch = degreeToRadian(camera.pitch);
    return QVector3D(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), std::sin(pitch)).normalized();
}

}

namespace MyVoxelViewer
{

RenderFrameReadyEvent::RenderFrameReadyEvent(std::uint64_t frameId, unsigned int textureId, std::uint64_t frameVersion,
                                             const QSize& textureSize, const MeshUpdateStatistics& statistics)
    : QEvent(eventType())
    , m_frameId(frameId)
    , m_textureId(textureId)
    , m_frameVersion(frameVersion)
    , m_textureSize(textureSize)
    , m_statistics(statistics)
{
}

QEvent::Type RenderFrameReadyEvent::eventType()
{
    static const int type = QEvent::registerEventType();
    return static_cast<QEvent::Type>(type);
}

std::uint64_t RenderFrameReadyEvent::frameId() const { return m_frameId; }
unsigned int RenderFrameReadyEvent::textureId() const { return m_textureId; }
std::uint64_t RenderFrameReadyEvent::frameVersion() const { return m_frameVersion; }
const QSize& RenderFrameReadyEvent::textureSize() const { return m_textureSize; }
const MeshUpdateStatistics& RenderFrameReadyEvent::statistics() const { return m_statistics; }

RenderFailureEvent::RenderFailureEvent(const QString& message)
    : QEvent(eventType())
    , m_message(message)
{
}

QEvent::Type RenderFailureEvent::eventType()
{
    static const int type = QEvent::registerEventType();
    return static_cast<QEvent::Type>(type);
}

const QString& RenderFailureEvent::message() const { return m_message; }

VoxelRenderThread::PendingObjectChange::PendingObjectChange()
    : action(PendingNone)
    , kind(RenderMeshObject)
    , dynamicUsage(false)
    , hasModelMatrix(false)
    , visible(true)
    , hasVisible(false)
    , lineWidth(1.0f)
{
    modelMatrix.setToIdentity();
}

VoxelRenderThread::PendingBatch::PendingBatch()
    : clearObjects(false)
    , hasCamera(false)
    , hasWireframe(false)
    , wireframe(false)
    , requestFrame(false)
    , cacheMilliseconds(0.0)
    , cpuCopyMilliseconds(0.0)
    , stagedCpuPartCount(0)
{
}

bool VoxelRenderThread::PendingBatch::isEmpty() const
{
    return !clearObjects && objects.empty() && !hasCamera && !hasWireframe && !requestFrame &&
           cacheMilliseconds == 0.0 && cpuCopyMilliseconds == 0.0 && stagedCpuPartCount == 0;
}

void VoxelRenderThread::PendingBatch::clear()
{
    clearObjects = false;
    objects.clear();
    hasCamera = false;
    hasWireframe = false;
    requestFrame = false;
    cacheMilliseconds = 0.0;
    cpuCopyMilliseconds = 0.0;
    stagedCpuPartCount = 0;
}

struct VoxelRenderThread::RenderState
{
    struct RenderPart
    {
        RenderPart()
            : vertexBuffer(QOpenGLBuffer::VertexBuffer)
            , vertexCount(0)
            , triangleCount(0)
            , segmentCount(0)
        {
        }

        QOpenGLVertexArrayObject vertexArray; // 当前分片顶点属性状态。
        QOpenGLBuffer vertexBuffer; // 当前分片连续顶点数据。
        int vertexCount; // 当前分片需要绘制的OpenGL顶点数量。
        std::size_t triangleCount; // Mesh分片包含的三角形数量。
        std::size_t segmentCount; // Line分片包含的独立线段数量。
    };

    struct RenderObject
    {
        typedef std::map<std::uint64_t, RenderPart*> PartMap;

        RenderObject()
            : kind(RenderMeshObject)
            , dynamicUsage(false)
            , visible(true)
            , lineWidth(1.0f)
        {
            modelMatrix.setToIdentity();
        }

        RenderObjectKind kind; // 当前对象是Mesh还是Line。
        bool dynamicUsage; // 当前对象GPU缓冲是否使用DynamicDraw。
        QMatrix4x4 modelMatrix; // 对象局部空间到显示世界空间的模型矩阵。
        bool visible; // 对象是否参与后台绘制。
        float lineWidth; // Line对象期望OpenGL线宽。
        PartMap parts; // 当前对象全部GPU分片。
    };

    struct FrameResource
    {
        FrameResource()
            : id(0)
            , framebuffer(0)
            , colorTexture(0)
            , depthStencilBuffer(0)
            , ready(false)
            , displayed(false)
            , retired(false)
        {
        }

        std::uint64_t id;
        QSize size;
        unsigned int framebuffer;
        unsigned int colorTexture;
        unsigned int depthStencilBuffer;
        bool ready;
        bool displayed;
        bool retired;
    };

    typedef std::map<std::uint64_t, RenderObject*> ObjectMap;
    typedef std::map<std::uint64_t, FrameResource> FrameMap;

    RenderState()
        : functions(nullptr)
        , initialized(false)
        , wireframe(false)
        , nextFrameId(1)
        , nextFrameVersion(1)
        , readyFrameId(0)
        , displayedFrameId(0)
    {
    }

    bool initialize(QOpenGLContext* context, QString& errorMessage)
    {
        functions = context->versionFunctions<QOpenGLFunctions_3_3_Core>();

        if (!functions)
        {
            errorMessage = QStringLiteral("OpenGL 3.3 Core function table is unavailable.");
            return false;
        }

        functions->initializeOpenGLFunctions();
        initialized = true;
        functions->glEnable(GL_DEPTH_TEST);
        functions->glDepthFunc(GL_LEQUAL);
        functions->glEnable(GL_BLEND);
        functions->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        functions->glDisable(GL_CULL_FACE);
        functions->glClearColor(0.045f, 0.055f, 0.075f, 1.0f);

        if (!createShaderPrograms(errorMessage))
        {
            return false;
        }

        if (!backgroundVao.create())
        {
            errorMessage = QStringLiteral("Failed to create the background OpenGL vertex array object.");
            return false;
        }

        return true;
    }

    void cleanup()
    {
        if (!initialized)
        {
            return;
        }

        clearObjects(nullptr);

        for (FrameMap::iterator iterator = frames.begin(); iterator != frames.end(); ++iterator)
        {
            destroyFrame(iterator->second);
        }

        frames.clear();
        currentFrameIds.clear();
        readyFrameId = 0;
        displayedFrameId = 0;

        if (backgroundVao.isCreated())
        {
            backgroundVao.destroy();
        }

        backgroundProgram.removeAllShaders();
        meshProgram.removeAllShaders();
        lineProgram.removeAllShaders();
        initialized = false;
    }

    bool createShaderPrograms(QString& errorMessage)
    {
        const char* backgroundVertexShader =
            "#version 330 core\n"
            "out vec2 v_uv;\n"
            "void main()\n"
            "{\n"
            "    vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));\n"
            "    vec2 position = positions[gl_VertexID];\n"
            "    gl_Position = vec4(position, 0.0, 1.0);\n"
            "    v_uv = position * 0.5 + 0.5;\n"
            "}\n";

        const char* backgroundFragmentShader =
            "#version 330 core\n"
            "in vec2 v_uv;\n"
            "out vec4 fragColor;\n"
            "void main()\n"
            "{\n"
            "    vec3 bottomColor = vec3(0.040, 0.050, 0.070);\n"
            "    vec3 topColor = vec3(0.205, 0.235, 0.285);\n"
            "    float vertical = smoothstep(0.0, 1.0, clamp(v_uv.y, 0.0, 1.0));\n"
            "    vec3 color = mix(bottomColor, topColor, vertical);\n"
            "    vec2 centered = v_uv - vec2(0.50, 0.46);\n"
            "    float radial = length(centered * vec2(1.0, 0.82));\n"
            "    color += vec3(0.030, 0.034, 0.042) * (1.0 - smoothstep(0.05, 0.82, radial));\n"
            "    color *= 1.0 - smoothstep(0.35, 0.95, radial) * 0.24;\n"
            "    fragColor = vec4(color, 1.0);\n"
            "}\n";

        const char* meshVertexShader =
            "#version 330 core\n"
            "layout(location = 0) in vec3 a_position;\n"
            "layout(location = 1) in vec3 a_normal;\n"
            "layout(location = 2) in vec4 a_color;\n"
            "uniform mat4 u_model;\n"
            "uniform mat4 u_mvp;\n"
            "uniform mat3 u_normalMatrix;\n"
            "out vec3 v_worldPosition;\n"
            "out vec3 v_normal;\n"
            "out vec4 v_color;\n"
            "void main()\n"
            "{\n"
            "    vec4 worldPosition = u_model * vec4(a_position, 1.0);\n"
            "    gl_Position = u_mvp * vec4(a_position, 1.0);\n"
            "    v_worldPosition = worldPosition.xyz;\n"
            "    v_normal = normalize(u_normalMatrix * a_normal);\n"
            "    v_color = a_color;\n"
            "}\n";

        const char* meshFragmentShader =
            "#version 330 core\n"
            "in vec3 v_worldPosition;\n"
            "in vec3 v_normal;\n"
            "in vec4 v_color;\n"
            "uniform vec3 u_cameraPosition;\n"
            "out vec4 fragColor;\n"
            "void main()\n"
            "{\n"
            "    vec3 normal = normalize(v_normal);\n"
            "    if (!gl_FrontFacing) normal = -normal;\n"
            "    vec3 viewDirection = normalize(u_cameraPosition - v_worldPosition);\n"
            "    vec3 keyDirection = normalize(vec3(0.45, -0.35, 0.82));\n"
            "    vec3 fillDirection = normalize(vec3(-0.65, 0.55, 0.35));\n"
            "    float keyDiffuse = max(dot(normal, keyDirection), 0.0);\n"
            "    float fillDiffuse = max(dot(normal, fillDirection), 0.0);\n"
            "    vec3 halfDirection = normalize(keyDirection + viewDirection);\n"
            "    float specular = pow(max(dot(normal, halfDirection), 0.0), 48.0);\n"
            "    float rim = pow(1.0 - max(dot(normal, viewDirection), 0.0), 2.0);\n"
            "    vec3 baseColor = pow(max(v_color.rgb, vec3(0.0)), vec3(2.2));\n"
            "    vec3 linearColor = baseColor * 0.24;\n"
            "    linearColor += baseColor * keyDiffuse * 0.78;\n"
            "    linearColor += baseColor * fillDiffuse * 0.20;\n"
            "    linearColor += vec3(1.0, 0.97, 0.90) * specular * 0.28;\n"
            "    linearColor += mix(baseColor, vec3(0.70, 0.82, 1.0), 0.45) * rim * 0.12;\n"
            "    linearColor = linearColor / (linearColor + vec3(0.35));\n"
            "    vec3 displayColor = pow(max(linearColor, vec3(0.0)), vec3(1.0 / 2.2));\n"
            "    fragColor = vec4(displayColor, v_color.a);\n"
            "}\n";

        const char* lineVertexShader =
            "#version 330 core\n"
            "layout(location = 0) in vec3 a_position;\n"
            "layout(location = 1) in vec4 a_color;\n"
            "uniform mat4 u_mvp;\n"
            "out vec4 v_color;\n"
            "void main()\n"
            "{\n"
            "    gl_Position = u_mvp * vec4(a_position, 1.0);\n"
            "    v_color = a_color;\n"
            "}\n";

        const char* lineFragmentShader =
            "#version 330 core\n"
            "in vec4 v_color;\n"
            "out vec4 fragColor;\n"
            "void main()\n"
            "{\n"
            "    fragColor = v_color;\n"
            "}\n";

        if (!backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, backgroundVertexShader) ||
            !backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, backgroundFragmentShader) ||
            !backgroundProgram.link())
        {
            errorMessage = QStringLiteral("Background shader initialization failed: ") + backgroundProgram.log();
            return false;
        }

        if (!meshProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, meshVertexShader) ||
            !meshProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, meshFragmentShader) ||
            !meshProgram.link())
        {
            errorMessage = QStringLiteral("Mesh shader initialization failed: ") + meshProgram.log();
            return false;
        }

        if (!lineProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, lineVertexShader) ||
            !lineProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, lineFragmentShader) ||
            !lineProgram.link())
        {
            errorMessage = QStringLiteral("Line shader initialization failed: ") + lineProgram.log();
            return false;
        }

        return true;
    }

    void applyBatch(const PendingBatch& batch, MeshUpdateStatistics& statistics)
    {
        statistics.cacheAndVersionMilliseconds += batch.cacheMilliseconds;
        statistics.cpuStagingCopyMilliseconds += batch.cpuCopyMilliseconds;
        statistics.stagedCpuPartCount += batch.stagedCpuPartCount;

        if (batch.clearObjects)
        {
            clearObjects(&statistics);
        }

        for (std::map<std::uint64_t, PendingObjectChange>::const_iterator iterator = batch.objects.begin();
             iterator != batch.objects.end(); ++iterator)
        {
            applyObjectChange(iterator->first, iterator->second, statistics);
        }

        if (batch.hasCamera)
        {
            camera = batch.camera;
            ensureFramePool(camera.viewportSize);
        }

        if (batch.hasWireframe)
        {
            wireframe = batch.wireframe;
        }
    }

    void applyObjectChange(std::uint64_t objectId, const PendingObjectChange& change, MeshUpdateStatistics& statistics)
    {
        ObjectMap::iterator objectIterator = objects.find(objectId);

        if (change.action == PendingRemove)
        {
            if (objectIterator != objects.end())
            {
                clearObject(*objectIterator->second, &statistics);
                delete objectIterator->second;
                objects.erase(objectIterator);
            }

            return;
        }

        if (change.action == PendingReplace)
        {
            if (objectIterator != objects.end())
            {
                clearObject(*objectIterator->second, &statistics);
                delete objectIterator->second;
                objects.erase(objectIterator);
            }

            RenderObject* object = new RenderObject();
            object->kind = change.kind;
            object->dynamicUsage = change.dynamicUsage;
            object->modelMatrix = change.modelMatrix;
            object->visible = change.visible;
            object->lineWidth = change.lineWidth;
            const std::pair<ObjectMap::iterator, bool> inserted = objects.insert(std::make_pair(objectId, object));

            if (!inserted.second)
            {
                delete object;
                return;
            }

            objectIterator = inserted.first;
        }

        if (objectIterator == objects.end())
        {
            return;
        }

        RenderObject& object = *objectIterator->second;

        if (change.hasModelMatrix)
        {
            object.modelMatrix = change.modelMatrix;
        }

        if (change.hasVisible)
        {
            object.visible = change.visible;
        }

        if (object.kind == RenderMeshObject)
        {
            for (std::map<std::uint64_t, MyVoxel::Display_MeshPartUpdate>::const_iterator iterator =
                     change.meshParts.begin(); iterator != change.meshParts.end(); ++iterator)
            {
                const MyVoxel::Display_MeshPartUpdate& part = iterator->second;

                if (part.operation == MyVoxel::Display_MeshPartOperation::Remove)
                {
                    removePart(object, part.partId, &statistics);
                }
                else
                {
                    uploadMeshPart(object, part.partId, *part.resource, statistics);
                }
            }
        }
        else
        {
            for (std::map<std::uint64_t, MyVoxel::Foundation::RefPtr<const MyVoxel::Display_LineResource> >::const_iterator iterator =
                     change.lineParts.begin(); iterator != change.lineParts.end(); ++iterator)
            {
                uploadLinePart(object, iterator->first, *iterator->second, statistics);
            }
        }
    }

    RenderPart* createOrFindPart(RenderObject& object, std::uint64_t partId, bool& created)
    {
        RenderObject::PartMap::iterator iterator = object.parts.find(partId);

        if (iterator != object.parts.end())
        {
            created = false;
            return iterator->second;
        }

        RenderPart* part = new RenderPart();
        created = true;

        if (!part->vertexArray.create() || !part->vertexBuffer.create())
        {
            if (part->vertexBuffer.isCreated()) part->vertexBuffer.destroy();
            if (part->vertexArray.isCreated()) part->vertexArray.destroy();
            delete part;
            return nullptr;
        }

        return part;
    }

    bool commitCreatedPart(RenderObject& object, std::uint64_t partId, RenderPart* part, bool created,
                           MeshUpdateStatistics& statistics)
    {
        if (!created)
        {
            ++statistics.reusedGpuPartCount;
            return true;
        }

        const std::pair<RenderObject::PartMap::iterator, bool> inserted =
            object.parts.insert(std::make_pair(partId, part));

        if (!inserted.second)
        {
            if (part->vertexBuffer.isCreated()) part->vertexBuffer.destroy();
            if (part->vertexArray.isCreated()) part->vertexArray.destroy();
            delete part;
            return false;
        }

        ++statistics.createdGpuPartCount;
        return true;
    }

    void uploadMeshPart(RenderObject& object, std::uint64_t partId,
                        const MyVoxel::Display_MeshResource& resource, MeshUpdateStatistics& statistics)
    {
        assert(resource.isValid() && resource.vertexCount() > 0);
        const RenderClock::time_point uploadStart = RenderClock::now();
        bool created = false;
        RenderPart* part = createOrFindPart(object, partId, created);

        if (!part)
        {
            return;
        }

        const std::vector<MyVoxel::Display_MeshVertex>& vertices = resource.vertices();
        part->vertexArray.bind();
        part->vertexBuffer.bind();

        if (created)
        {
            part->vertexBuffer.setUsagePattern(object.dynamicUsage ? QOpenGLBuffer::DynamicDraw : QOpenGLBuffer::StaticDraw);
        }

        part->vertexBuffer.allocate(&vertices[0], checkedBufferByteSize(resource.memoryByteSize()));

        if (created)
        {
            meshProgram.bind();
            meshProgram.enableAttributeArray(0);
            meshProgram.setAttributeBuffer(0, GL_FLOAT, MeshPositionFloatOffset * static_cast<int>(sizeof(float)), 3,
                                           MeshVertexFloatCount * static_cast<int>(sizeof(float)));
            meshProgram.enableAttributeArray(1);
            meshProgram.setAttributeBuffer(1, GL_FLOAT, MeshNormalFloatOffset * static_cast<int>(sizeof(float)), 3,
                                           MeshVertexFloatCount * static_cast<int>(sizeof(float)));
            meshProgram.enableAttributeArray(2);
            meshProgram.setAttributeBuffer(2, GL_FLOAT, MeshColorFloatOffset * static_cast<int>(sizeof(float)), 4,
                                           MeshVertexFloatCount * static_cast<int>(sizeof(float)));
            meshProgram.release();
        }

        part->vertexBuffer.release();
        part->vertexArray.release();
        assert(resource.vertexCount() <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
        part->vertexCount = static_cast<int>(resource.vertexCount());
        part->triangleCount = resource.triangleCount();
        part->segmentCount = 0;

        if (!commitCreatedPart(object, partId, part, created, statistics))
        {
            return;
        }

        statistics.gpuUploadMilliseconds += elapsedMilliseconds(uploadStart, RenderClock::now());
        ++statistics.uploadedPartCount;
    }

    void uploadLinePart(RenderObject& object, std::uint64_t partId,
                        const MyVoxel::Display_LineResource& resource, MeshUpdateStatistics& statistics)
    {
        assert(resource.isValid() && resource.vertexCount() > 0);
        const RenderClock::time_point uploadStart = RenderClock::now();
        bool created = false;
        RenderPart* part = createOrFindPart(object, partId, created);

        if (!part)
        {
            return;
        }

        const std::vector<MyVoxel::Display_LineVertex>& vertices = resource.vertices();
        part->vertexArray.bind();
        part->vertexBuffer.bind();

        if (created)
        {
            part->vertexBuffer.setUsagePattern(object.dynamicUsage ? QOpenGLBuffer::DynamicDraw : QOpenGLBuffer::StaticDraw);
        }

        part->vertexBuffer.allocate(&vertices[0], checkedBufferByteSize(resource.memoryByteSize()));

        if (created)
        {
            lineProgram.bind();
            lineProgram.enableAttributeArray(0);
            lineProgram.setAttributeBuffer(0, GL_FLOAT, LinePositionFloatOffset * static_cast<int>(sizeof(float)), 3,
                                           LineVertexFloatCount * static_cast<int>(sizeof(float)));
            lineProgram.enableAttributeArray(1);
            lineProgram.setAttributeBuffer(1, GL_FLOAT, LineColorFloatOffset * static_cast<int>(sizeof(float)), 4,
                                           LineVertexFloatCount * static_cast<int>(sizeof(float)));
            lineProgram.release();
        }

        part->vertexBuffer.release();
        part->vertexArray.release();
        assert(resource.vertexCount() <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
        part->vertexCount = static_cast<int>(resource.vertexCount());
        part->triangleCount = 0;
        part->segmentCount = resource.segmentCount();

        if (!commitCreatedPart(object, partId, part, created, statistics))
        {
            return;
        }

        statistics.gpuUploadMilliseconds += elapsedMilliseconds(uploadStart, RenderClock::now());
        ++statistics.uploadedPartCount;
    }

    void removePart(RenderObject& object, std::uint64_t partId, MeshUpdateStatistics* statistics)
    {
        RenderObject::PartMap::iterator iterator = object.parts.find(partId);

        if (iterator == object.parts.end())
        {
            return;
        }

        const RenderClock::time_point removalStart = RenderClock::now();
        if (iterator->second->vertexBuffer.isCreated()) iterator->second->vertexBuffer.destroy();
        if (iterator->second->vertexArray.isCreated()) iterator->second->vertexArray.destroy();
        delete iterator->second;
        object.parts.erase(iterator);

        if (statistics)
        {
            statistics->gpuRemovalMilliseconds += elapsedMilliseconds(removalStart, RenderClock::now());
            ++statistics->removedPartCount;
        }
    }

    void clearObject(RenderObject& object, MeshUpdateStatistics* statistics)
    {
        while (!object.parts.empty())
        {
            removePart(object, object.parts.begin()->first, statistics);
        }
    }

    void clearObjects(MeshUpdateStatistics* statistics)
    {
        for (ObjectMap::iterator iterator = objects.begin(); iterator != objects.end(); ++iterator)
        {
            clearObject(*iterator->second, statistics);
            delete iterator->second;
        }

        objects.clear();
    }

    void ensureFramePool(const QSize& requestedSize)
    {
        const QSize size((std::max)(requestedSize.width(), 1), (std::max)(requestedSize.height(), 1));

        if (currentFrameSize == size && currentFrameIds.size() == 2)
        {
            return;
        }

        currentFrameSize = size;

        for (FrameMap::iterator iterator = frames.begin(); iterator != frames.end(); ++iterator)
        {
            iterator->second.retired = true;
        }

        currentFrameIds.clear();

        for (int index = 0; index < 2; ++index)
        {
            const std::uint64_t frameId = createFrame(size);

            if (frameId != 0)
            {
                currentFrameIds.push_back(frameId);
            }
        }

        cleanupRetiredFrames();
    }

    std::uint64_t createFrame(const QSize& size)
    {
        FrameResource frame;
        frame.id = nextFrameId++;
        frame.size = size;
        functions->glGenTextures(1, &frame.colorTexture);
        functions->glBindTexture(GL_TEXTURE_2D, frame.colorTexture);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        functions->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        functions->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, size.width(), size.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        functions->glBindTexture(GL_TEXTURE_2D, 0);

        functions->glGenRenderbuffers(1, &frame.depthStencilBuffer);
        functions->glBindRenderbuffer(GL_RENDERBUFFER, frame.depthStencilBuffer);
        functions->glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, size.width(), size.height());
        functions->glBindRenderbuffer(GL_RENDERBUFFER, 0);

        functions->glGenFramebuffers(1, &frame.framebuffer);
        functions->glBindFramebuffer(GL_FRAMEBUFFER, frame.framebuffer);
        const unsigned int drawBuffer = GL_COLOR_ATTACHMENT0;
        functions->glDrawBuffers(1, &drawBuffer);
        functions->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, frame.colorTexture, 0);
        functions->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, frame.depthStencilBuffer);
        const unsigned int status = functions->glCheckFramebufferStatus(GL_FRAMEBUFFER);
        functions->glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (status != GL_FRAMEBUFFER_COMPLETE)
        {
            destroyFrame(frame);
            return 0;
        }

        frames.insert(std::make_pair(frame.id, frame));
        return frame.id;
    }

    void destroyFrame(FrameResource& frame)
    {
        if (frame.framebuffer != 0)
        {
            functions->glDeleteFramebuffers(1, &frame.framebuffer);
            frame.framebuffer = 0;
        }

        if (frame.depthStencilBuffer != 0)
        {
            functions->glDeleteRenderbuffers(1, &frame.depthStencilBuffer);
            frame.depthStencilBuffer = 0;
        }

        if (frame.colorTexture != 0)
        {
            functions->glDeleteTextures(1, &frame.colorTexture);
            frame.colorTexture = 0;
        }
    }

    void cleanupRetiredFrames()
    {
        FrameMap::iterator iterator = frames.begin();

        while (iterator != frames.end())
        {
            if (iterator->second.retired && !iterator->second.ready && !iterator->second.displayed)
            {
                destroyFrame(iterator->second);
                frames.erase(iterator++);
            }
            else
            {
                ++iterator;
            }
        }
    }

    FrameResource* freeCurrentFrame()
    {
        for (std::size_t index = 0; index < currentFrameIds.size(); ++index)
        {
            FrameMap::iterator iterator = frames.find(currentFrameIds[index]);

            if (iterator != frames.end() && !iterator->second.ready && !iterator->second.displayed && !iterator->second.retired)
            {
                return &iterator->second;
            }
        }

        return nullptr;
    }

    bool renderFrame(MeshUpdateStatistics& statistics, std::uint64_t& frameId, unsigned int& textureId,
                     std::uint64_t& frameVersion, QSize& textureSize)
    {
        ensureFramePool(camera.viewportSize);
        FrameResource* frame = freeCurrentFrame();

        if (!frame)
        {
            return false;
        }

        const RenderClock::time_point renderStart = RenderClock::now();
        functions->glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
        functions->glViewport(0, 0, frame->size.width(), frame->size.height());
        functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        drawBackground();
        drawObjects(statistics);
        functions->glBindFramebuffer(GL_FRAMEBUFFER, 0);
        functions->glFinish(); // GPU等待仅发生在后台线程，确保GUI收到事件后可直接无阻塞采样共享纹理。
        statistics.renderMilliseconds += elapsedMilliseconds(renderStart, RenderClock::now());
        statistics.totalMilliseconds = statistics.cacheAndVersionMilliseconds + statistics.cpuStagingCopyMilliseconds +
            statistics.contextAcquireMilliseconds + statistics.vertexExpansionMilliseconds + statistics.gpuUploadMilliseconds +
            statistics.gpuRemovalMilliseconds + statistics.contextReleaseMilliseconds + statistics.renderMilliseconds;
        statistics.frameVersion = nextFrameVersion++;

        frame->ready = true;
        readyFrameId = frame->id;
        frameId = frame->id;
        textureId = frame->colorTexture;
        frameVersion = statistics.frameVersion;
        textureSize = frame->size;
        return true;
    }

    void acceptPresentedFrame(std::uint64_t frameId)
    {
        FrameMap::iterator newIterator = frames.find(frameId);

        if (newIterator == frames.end())
        {
            return;
        }

        if (displayedFrameId != 0 && displayedFrameId != frameId)
        {
            FrameMap::iterator oldIterator = frames.find(displayedFrameId);

            if (oldIterator != frames.end())
            {
                oldIterator->second.displayed = false;
            }
        }

        newIterator->second.ready = false;
        newIterator->second.displayed = true;
        displayedFrameId = frameId;

        if (readyFrameId == frameId)
        {
            readyFrameId = 0;
        }

        cleanupRetiredFrames();
    }

    void drawBackground()
    {
        functions->glDisable(GL_DEPTH_TEST);
        functions->glDepthMask(GL_FALSE);
        functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        backgroundProgram.bind();
        backgroundVao.bind();
        functions->glDrawArrays(GL_TRIANGLES, 0, 3);
        backgroundVao.release();
        backgroundProgram.release();
        functions->glDepthMask(GL_TRUE);
        functions->glEnable(GL_DEPTH_TEST);
    }

    void drawObjects(MeshUpdateStatistics& statistics)
    {
        const QVector3D viewCenter = camera.center + camera.viewOffset;
        const float radius = (std::max)(camera.radius, MinimumRadius);
        const float cameraDistance = (std::max)(radius * camera.cameraScale, 0.1f);
        const QVector3D eye = viewCenter + cameraDirection(camera) * cameraDistance;
        const float aspect = camera.viewportSize.height() > 0
            ? static_cast<float>(camera.viewportSize.width()) / static_cast<float>(camera.viewportSize.height()) : 1.0f;
        const float nearPlane = (std::max)(0.001f, cameraDistance - radius * 1.75f);
        const float farPlane = (std::max)(nearPlane + 1.0f, cameraDistance + radius * 3.50f);

        QMatrix4x4 projection;
        projection.perspective(45.0f, aspect, nearPlane, farPlane);
        QMatrix4x4 view;
        view.lookAt(eye, viewCenter, QVector3D(0.0f, 0.0f, 1.0f));

        meshProgram.bind();
        meshProgram.setUniformValue("u_cameraPosition", eye);
        functions->glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);

        for (ObjectMap::const_iterator objectIterator = objects.begin(); objectIterator != objects.end(); ++objectIterator)
        {
            const RenderObject& object = *objectIterator->second;

            if (object.kind != RenderMeshObject || !object.visible || object.parts.empty())
            {
                continue;
            }

            const QMatrix4x4 mvp = projection * view * object.modelMatrix;
            meshProgram.setUniformValue("u_model", object.modelMatrix);
            meshProgram.setUniformValue("u_mvp", mvp);
            meshProgram.setUniformValue("u_normalMatrix", object.modelMatrix.normalMatrix());

            for (RenderObject::PartMap::const_iterator partIterator = object.parts.begin(); partIterator != object.parts.end(); ++partIterator)
            {
                partIterator->second->vertexArray.bind();
                functions->glDrawArrays(GL_TRIANGLES, 0, partIterator->second->vertexCount);
                partIterator->second->vertexArray.release();
                ++statistics.drawCallCount;
                statistics.triangleCount += partIterator->second->triangleCount;
            }
        }

        functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        meshProgram.release();

        lineProgram.bind();

        for (ObjectMap::const_iterator objectIterator = objects.begin(); objectIterator != objects.end(); ++objectIterator)
        {
            const RenderObject& object = *objectIterator->second;

            if (object.kind != RenderLineObject || !object.visible || object.parts.empty())
            {
                continue;
            }

            lineProgram.setUniformValue("u_mvp", projection * view * object.modelMatrix);
            functions->glLineWidth(object.lineWidth);

            for (RenderObject::PartMap::const_iterator partIterator = object.parts.begin(); partIterator != object.parts.end(); ++partIterator)
            {
                partIterator->second->vertexArray.bind();
                functions->glDrawArrays(GL_LINES, 0, partIterator->second->vertexCount);
                partIterator->second->vertexArray.release();
                ++statistics.drawCallCount;
                ++statistics.lineDrawCallCount;
                statistics.lineSegmentCount += partIterator->second->segmentCount;
            }
        }

        functions->glLineWidth(1.0f);
        lineProgram.release();
    }

    QOpenGLFunctions_3_3_Core* functions;
    bool initialized;
    bool wireframe;
    RenderCameraState camera;
    QOpenGLShaderProgram backgroundProgram;
    QOpenGLShaderProgram meshProgram;
    QOpenGLShaderProgram lineProgram;
    QOpenGLVertexArrayObject backgroundVao;
    ObjectMap objects;
    FrameMap frames;
    std::vector<std::uint64_t> currentFrameIds;
    QSize currentFrameSize;
    std::uint64_t nextFrameId;
    std::uint64_t nextFrameVersion;
    std::uint64_t readyFrameId;
    std::uint64_t displayedFrameId;
};

VoxelRenderThread::VoxelRenderThread(QObject* parent)
    : QThread(parent)
    , m_stopRequested(false)
    , m_started(false)
    , m_readyFrameOutstanding(false)
    , m_frameAcceptancePending(false)
    , m_readyFrameId(0)
    , m_readyFrameVersion(0)
    , m_acceptedFrameId(0)
    , m_acceptedFrameVersion(0)
    , m_context(nullptr)
    , m_surface(nullptr)
    , m_frameReceiver(nullptr)
    , m_guiThread(nullptr)
{
}

VoxelRenderThread::~VoxelRenderThread()
{
    stopRendering();
}

bool VoxelRenderThread::startRendering(QOpenGLContext* context, QOffscreenSurface* surface,
                                       QObject* frameReceiver, QThread* guiThread)
{
    QMutexLocker locker(&m_mutex);

    if (m_started || !context || !surface || !surface->isValid() || !frameReceiver || !guiThread)
    {
        return false;
    }

    context->moveToThread(this);
    m_context = context;
    m_surface = surface;
    m_frameReceiver = frameReceiver;
    m_guiThread = guiThread;
    m_stopRequested = false;
    m_readyFrameOutstanding = false;
    m_frameAcceptancePending = false;
    m_readyFrameId = 0;
    m_readyFrameVersion = 0;
    m_acceptedFrameId = 0;
    m_acceptedFrameVersion = 0;
    m_started = true;
    start();
    return true;
}

void VoxelRenderThread::stopRendering()
{
    const bool running = isRunning();

    {
        QMutexLocker locker(&m_mutex);

        if (m_started)
        {
            m_stopRequested = true;
            m_waitCondition.wakeAll();
        }
    }

    if (running && QThread::currentThread() != this)
    {
        wait();
    }
}

bool VoxelRenderThread::isRendering() const
{
    QMutexLocker locker(&m_mutex);
    return m_started && !m_stopRequested;
}

void VoxelRenderThread::enqueueReplaceObject(const MyVoxel::Display_MeshObjectSnapshot& snapshot,
                                             double cacheMilliseconds, double cpuCopyMilliseconds)
{
    assert(snapshot.isValid());
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[snapshot.objectId];
    change = PendingObjectChange();
    change.action = PendingReplace;
    change.kind = RenderMeshObject;
    change.dynamicUsage = snapshot.usage == MyVoxel::Display_MeshUsage::Dynamic;
    change.modelMatrix = toQMatrix4x4(snapshot.localToWorld);
    change.hasModelMatrix = true;
    change.visible = snapshot.visible;
    change.hasVisible = true;

    for (std::size_t index = 0; index < snapshot.parts.size(); ++index)
    {
        const MyVoxel::Display_MeshPartSnapshot& part = snapshot.parts[index];
        change.meshParts[part.partId] =
            MyVoxel::Display_MeshPartUpdate::replacement(part.partId, part.version, part.resource);
        ++m_pending.stagedCpuPartCount;
    }

    m_pending.cacheMilliseconds += cacheMilliseconds;
    m_pending.cpuCopyMilliseconds += cpuCopyMilliseconds;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueReplaceObject(const MyVoxel::Display_LineObjectSnapshot& snapshot,
                                             double cacheMilliseconds, double cpuCopyMilliseconds)
{
    assert(snapshot.isValid());
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[snapshot.objectId];
    change = PendingObjectChange();
    change.action = PendingReplace;
    change.kind = RenderLineObject;
    change.dynamicUsage = snapshot.usage == MyVoxel::Display_LineUsage::Dynamic;
    change.modelMatrix = toQMatrix4x4(snapshot.localToWorld);
    change.hasModelMatrix = true;
    change.visible = snapshot.visible;
    change.hasVisible = true;
    change.lineWidth = snapshot.width;

    for (std::size_t index = 0; index < snapshot.parts.size(); ++index)
    {
        change.lineParts[snapshot.parts[index].partId] = snapshot.parts[index].resource;
        ++m_pending.stagedCpuPartCount;
    }

    m_pending.cacheMilliseconds += cacheMilliseconds;
    m_pending.cpuCopyMilliseconds += cpuCopyMilliseconds;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueUpdateParts(const MyVoxel::Display_MeshUpdate& update,
                                           double cacheMilliseconds, double cpuCopyMilliseconds)
{
    assert(update.isValid());
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[update.objectId];

    if (change.action == PendingRemove)
    {
        return;
    }

    for (std::size_t index = 0; index < update.parts.size(); ++index)
    {
        change.meshParts[update.parts[index].partId] = update.parts[index];

        if (update.parts[index].operation == MyVoxel::Display_MeshPartOperation::Replace)
        {
            ++m_pending.stagedCpuPartCount;
        }
    }

    m_pending.cacheMilliseconds += cacheMilliseconds;
    m_pending.cpuCopyMilliseconds += cpuCopyMilliseconds;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueStateUpdate(const MyVoxel::Display_MeshStateUpdate& update)
{
    assert(update.isValid());
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[update.objectId];

    if (change.action == PendingRemove)
    {
        return;
    }

    if (update.hasLocalToWorld)
    {
        change.modelMatrix = toQMatrix4x4(update.localToWorld);
        change.hasModelMatrix = true;
    }

    if (update.hasVisible)
    {
        change.visible = update.visible;
        change.hasVisible = true;
    }

    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueRemoveObject(std::uint64_t objectId)
{
    QMutexLocker locker(&m_mutex);
    PendingObjectChange& change = m_pending.objects[objectId];
    change = PendingObjectChange();
    change.action = PendingRemove;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueClearObjects()
{
    QMutexLocker locker(&m_mutex);
    m_pending.clearObjects = true;
    m_pending.objects.clear();
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueCameraState(const RenderCameraState& state)
{
    QMutexLocker locker(&m_mutex);
    m_pending.camera = state;
    m_pending.hasCamera = true;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::enqueueWireframe(bool enabled)
{
    QMutexLocker locker(&m_mutex);
    m_pending.wireframe = enabled;
    m_pending.hasWireframe = true;
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::requestFrame()
{
    QMutexLocker locker(&m_mutex);
    m_pending.requestFrame = true;
    m_waitCondition.wakeOne();
}

void VoxelRenderThread::acceptPresentedFrame(std::uint64_t frameId, std::uint64_t frameVersion)
{
    QMutexLocker locker(&m_mutex);

    if (!m_readyFrameOutstanding || frameId != m_readyFrameId || frameVersion != m_readyFrameVersion)
    {
        return;
    }

    m_readyFrameOutstanding = false;
    m_frameAcceptancePending = true;
    m_acceptedFrameId = frameId;
    m_acceptedFrameVersion = frameVersion;
    m_waitCondition.wakeOne();
}

VoxelRenderThread::PendingBatch VoxelRenderThread::takePendingBatch()
{
    PendingBatch batch;
    batch.clearObjects = m_pending.clearObjects;
    batch.objects.swap(m_pending.objects);
    batch.hasCamera = m_pending.hasCamera;
    batch.camera = m_pending.camera;
    batch.hasWireframe = m_pending.hasWireframe;
    batch.wireframe = m_pending.wireframe;
    batch.requestFrame = m_pending.requestFrame;
    batch.cacheMilliseconds = m_pending.cacheMilliseconds;
    batch.cpuCopyMilliseconds = m_pending.cpuCopyMilliseconds;
    batch.stagedCpuPartCount = m_pending.stagedCpuPartCount;
    m_pending.clear();
    return batch;
}

bool VoxelRenderThread::hasRunnableWorkLocked() const
{
    return m_stopRequested || !m_pending.isEmpty() || m_frameAcceptancePending;
}

void VoxelRenderThread::postFailure(const QString& message) const
{
    if (m_frameReceiver)
    {
        QCoreApplication::postEvent(m_frameReceiver, new RenderFailureEvent(message));
    }
}

void VoxelRenderThread::run()
{
    const RenderClock::time_point acquireStart = RenderClock::now();

    if (!m_context || !m_surface || !m_context->makeCurrent(m_surface))
    {
        postFailure(QStringLiteral("Failed to make the background OpenGL context current."));

        if (m_context && m_guiThread)
        {
            m_context->moveToThread(m_guiThread);
        }

        QMutexLocker locker(&m_mutex);
        m_started = false;
        return;
    }

    MeshUpdateStatistics accumulatedStatistics;
    accumulatedStatistics.contextAcquireMilliseconds = elapsedMilliseconds(acquireStart, RenderClock::now());
    RenderState state;
    QString errorMessage;

    if (!state.initialize(m_context, errorMessage))
    {
        postFailure(errorMessage);
        state.cleanup();
        m_context->doneCurrent();

        if (m_guiThread)
        {
            m_context->moveToThread(m_guiThread);
        }

        QMutexLocker locker(&m_mutex);
        m_started = false;
        return;
    }

    bool renderRequested = true;

    while (true)
    {
        PendingBatch batch;
        bool stopRequested = false;
        bool acceptFrame = false;
        std::uint64_t acceptedFrameId = 0;

        {
            QMutexLocker locker(&m_mutex);

            while (!m_stopRequested && m_pending.isEmpty() && !m_frameAcceptancePending &&
                   !(renderRequested && !m_readyFrameOutstanding))
            {
                m_waitCondition.wait(&m_mutex);
            }

            stopRequested = m_stopRequested;

            if (!stopRequested)
            {
                batch = takePendingBatch();
                acceptFrame = m_frameAcceptancePending;
                acceptedFrameId = m_acceptedFrameId;
                m_frameAcceptancePending = false;
            }
        }

        if (stopRequested)
        {
            break;
        }

        if (acceptFrame)
        {
            state.acceptPresentedFrame(acceptedFrameId);
        }

        if (!batch.isEmpty())
        {
            state.applyBatch(batch, accumulatedStatistics);
            renderRequested = renderRequested || batch.requestFrame;
        }

        bool canPublish = false;

        {
            QMutexLocker locker(&m_mutex);
            canPublish = !m_readyFrameOutstanding;
        }

        if (renderRequested && canPublish)
        {
            std::uint64_t frameId = 0;
            unsigned int textureId = 0;
            std::uint64_t frameVersion = 0;
            QSize textureSize;

            if (state.renderFrame(accumulatedStatistics, frameId, textureId, frameVersion, textureSize))
            {
                {
                    QMutexLocker locker(&m_mutex);
                    m_readyFrameOutstanding = true;
                    m_readyFrameId = frameId;
                    m_readyFrameVersion = frameVersion;
                }

                if (m_frameReceiver)
                {
                    QCoreApplication::postEvent(m_frameReceiver,
                        new RenderFrameReadyEvent(frameId, textureId, frameVersion, textureSize, accumulatedStatistics));
                }

                accumulatedStatistics.clear();
                renderRequested = false;
            }
        }
    }

    state.cleanup();
    const RenderClock::time_point releaseStart = RenderClock::now();
    m_context->doneCurrent();
    accumulatedStatistics.contextReleaseMilliseconds += elapsedMilliseconds(releaseStart, RenderClock::now());

    if (m_guiThread)
    {
        m_context->moveToThread(m_guiThread);
    }

    QMutexLocker locker(&m_mutex);
    m_started = false;
    m_stopRequested = false;
    m_context = nullptr;
    m_surface = nullptr;
    m_frameReceiver = nullptr;
    m_guiThread = nullptr;
}

}
