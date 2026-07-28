#include "VoxelMeshViewWidget.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLExtraFunctions>
#include <QSurfaceFormat>
#include <QWheelEvent>

namespace
{

// 将浮点值限制在指定范围内。
float clampFloat(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

// 将角度转换为弧度。
float degreeToRadian(float degree)
{
    return degree * static_cast<float>(3.1415926) / static_cast<float>(180.0);
}

// 返回当前相机方向，方向由观察中心指向相机。
QVector3D cameraDirection(float yawDegree, float pitchDegree)
{
    const float yaw = degreeToRadian(yawDegree);
    const float pitch = degreeToRadian(pitchDegree);
    return QVector3D(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), std::sin(pitch)).normalized();
}

// 返回当前屏幕水平方向。
QVector3D cameraRight(float yawDegree, float pitchDegree)
{
    const QVector3D forward = -cameraDirection(yawDegree, pitchDegree);
    const QVector3D worldUp(0.0f, 0.0f, 1.0f);
    return QVector3D::crossProduct(forward, worldUp).normalized();
}

// 返回当前屏幕竖直方向。
QVector3D cameraUp(float yawDegree, float pitchDegree)
{
    const QVector3D forward = -cameraDirection(yawDegree, pitchDegree);
    const QVector3D right = cameraRight(yawDegree, pitchDegree);
    return QVector3D::crossProduct(right, forward).normalized();
}

// 将无符号字节数量转换为QOpenGLBuffer使用的有符号整数。
int checkedBufferByteSize(std::size_t byteSize)
{
    assert(byteSize <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
    return static_cast<int>(byteSize);
}

}

namespace MyVoxelViewer
{

VoxelMeshViewWidget::VoxelMeshViewWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_vertexBuffer(QOpenGLBuffer::VertexBuffer)
    , m_indexBuffer(QOpenGLBuffer::IndexBuffer)
{
    // OpenGL 3.3 Core和4倍多重采样用于当前独立网格显示窗口。
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
    setFormat(format);

    setFocusPolicy(Qt::StrongFocus);
}

void VoxelMeshViewWidget::setMesh(const DisplayMesh& mesh)
{
    m_mesh = mesh;
    applyMeshChange();
}

void VoxelMeshViewWidget::setMesh(DisplayMesh&& mesh)
{
    m_mesh = std::move(mesh);
    applyMeshChange();
}

const DisplayMesh& VoxelMeshViewWidget::mesh() const
{
    return m_mesh;
}

void VoxelMeshViewWidget::initializeGL()
{
    m_functions = context()->extraFunctions();
    m_functions->initializeOpenGLFunctions();

    m_functions->glEnable(GL_DEPTH_TEST);
    m_functions->glDepthFunc(GL_LEQUAL);
    m_functions->glEnable(GL_MULTISAMPLE);

    // 暂不启用背面剔除，避免左手坐标系或暂未统一的网格绕序导致表面消失。
    m_functions->glDisable(GL_CULL_FACE);

    // 清屏色只作为背景Shader未执行时的后备颜色。
    m_functions->glClearColor(0.045f, 0.055f, 0.075f, 1.0f);

    createBackgroundShaderProgram();
    createMeshShaderProgram();
    createBuffers();

    m_initialized = true;
}

void VoxelMeshViewWidget::resizeGL(int width, int height)
{
    m_functions->glViewport(0, 0, width, height);
}

void VoxelMeshViewWidget::paintGL()
{
    m_functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawBackground();

    if (m_mesh.isEmpty())
    {
        return;
    }

    const QVector3D viewCenter = m_center + m_viewOffset;
    const float cameraDistance = std::max(m_radius * m_cameraScale, 0.1f);
    const QVector3D eye = viewCenter + cameraDirection(m_yaw, m_pitch) * cameraDistance;

    drawMesh(eye, viewCenter);
}

void VoxelMeshViewWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Escape)
    {
        close();
        return;
    }

    QOpenGLWidget::keyPressEvent(event);
}

void VoxelMeshViewWidget::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePosition = event->pos();
    event->accept();
}

void VoxelMeshViewWidget::mouseMoveEvent(QMouseEvent* event)
{
    const QPoint delta = event->pos() - m_lastMousePosition;
    m_lastMousePosition = event->pos();

    if (event->buttons() & Qt::LeftButton)
    {
        m_yaw -= static_cast<float>(delta.x()) * 0.35f;
        m_pitch += static_cast<float>(delta.y()) * 0.35f;
        m_pitch = clampFloat(m_pitch, -85.0f, 85.0f);
        update();
    }
    else if (event->buttons() & Qt::RightButton)
    {
        const float cameraDistance = std::max(m_radius * m_cameraScale, 0.1f);
        const float moveScale = cameraDistance * 0.0015f;
        const QVector3D right = cameraRight(m_yaw, m_pitch);
        const QVector3D up = cameraUp(m_yaw, m_pitch);

        m_viewOffset -= right * static_cast<float>(delta.x()) * moveScale;
        m_viewOffset += up * static_cast<float>(delta.y()) * moveScale;
        update();
    }

    event->accept();
}

void VoxelMeshViewWidget::wheelEvent(QWheelEvent* event)
{
    if (event->angleDelta().y() > 0)
    {
        m_cameraScale *= 0.90f;
    }
    else if (event->angleDelta().y() < 0)
    {
        m_cameraScale *= 1.10f;
    }

    m_cameraScale = clampFloat(m_cameraScale, 0.35f, 20.0f);
    update();
    event->accept();
}

void VoxelMeshViewWidget::createBackgroundShaderProgram()
{
    const char* vertexShader =
        "#version 330 core\n"
        "out vec2 v_uv;\n"
        "void main()\n"
        "{\n"
        "    vec2 positions[3] = vec2[3](\n"
        "        vec2(-1.0, -1.0),\n"
        "        vec2( 3.0, -1.0),\n"
        "        vec2(-1.0,  3.0));\n"
        "    vec2 position = positions[gl_VertexID];\n"
        "    gl_Position = vec4(position, 0.0, 1.0);\n"
        "    v_uv = position * 0.5 + 0.5;\n"
        "}\n";

    const char* fragmentShader =
        "#version 330 core\n"
        "in vec2 v_uv;\n"
        "uniform vec3 u_topColor;\n"
        "uniform vec3 u_bottomColor;\n"
        "out vec4 fragColor;\n"
        "void main()\n"
        "{\n"
        "    float vertical = smoothstep(0.0, 1.0, clamp(v_uv.y, 0.0, 1.0));\n"
        "    vec3 color = mix(u_bottomColor, u_topColor, vertical);\n"
        "\n"
        "    vec2 centered = v_uv - vec2(0.50, 0.46);\n"
        "    float radialDistance = length(centered * vec2(1.0, 0.82));\n"
        "    float centerGlow = 1.0 - smoothstep(0.05, 0.82, radialDistance);\n"
        "    float vignette = smoothstep(0.35, 0.95, radialDistance);\n"
        "\n"
        "    color += vec3(0.030, 0.034, 0.042) * centerGlow;\n"
        "    color *= 1.0 - vignette * 0.24;\n"
        "    fragColor = vec4(color, 1.0);\n"
        "}\n";

    const bool vertexOk = m_backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader);
    const bool fragmentOk = m_backgroundProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader);
    const bool linkOk = m_backgroundProgram.link();

    assert(vertexOk);
    assert(fragmentOk);
    assert(linkOk);
}

void VoxelMeshViewWidget::createMeshShaderProgram()
{
    const char* vertexShader =
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_position;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec4 a_color;\n"
        "\n"
        "uniform mat4 u_mvp;\n"
        "\n"
        "out vec3 v_worldPosition;\n"
        "out vec3 v_normal;\n"
        "out vec4 v_color;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_Position = u_mvp * vec4(a_position, 1.0);\n"
        "    v_worldPosition = a_position;\n"
        "    v_normal = normalize(a_normal);\n"
        "    v_color = a_color;\n"
        "}\n";

    const char* fragmentShader =
        "#version 330 core\n"
        "in vec3 v_worldPosition;\n"
        "in vec3 v_normal;\n"
        "in vec4 v_color;\n"
        "\n"
        "uniform vec3 u_cameraPosition;\n"
        "uniform vec3 u_keyLightDirection;\n"
        "uniform vec3 u_fillLightDirection;\n"
        "uniform float u_ambientStrength;\n"
        "uniform float u_keyLightStrength;\n"
        "uniform float u_fillLightStrength;\n"
        "uniform float u_specularStrength;\n"
        "uniform float u_shininess;\n"
        "uniform float u_rimStrength;\n"
        "\n"
        "out vec4 fragColor;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    vec3 normal = normalize(v_normal);\n"
        "\n"
        "    if (!gl_FrontFacing)\n"
        "    {\n"
        "        normal = -normal;\n"
        "    }\n"
        "\n"
        "    vec3 viewDirection = normalize(u_cameraPosition - v_worldPosition);\n"
        "    vec3 keyDirection = normalize(u_keyLightDirection);\n"
        "    vec3 fillDirection = normalize(u_fillLightDirection);\n"
        "    vec3 halfDirection = normalize(keyDirection + viewDirection);\n"
        "\n"
        "    float keyDiffuse = max(dot(normal, keyDirection), 0.0);\n"
        "    float fillDiffuse = max(dot(normal, fillDirection), 0.0);\n"
        "    float specular = pow(max(dot(normal, halfDirection), 0.0), u_shininess);\n"
        "    float rim = pow(1.0 - max(dot(normal, viewDirection), 0.0), 2.2);\n"
        "    float upperAmbient = max(normal.z, 0.0) * 0.10;\n"
        "    float lowerAmbient = max(-normal.z, 0.0) * 0.035;\n"
        "\n"
        "    vec3 baseColor = pow(max(v_color.rgb, vec3(0.0)), vec3(2.2));\n"
        "    vec3 linearColor = baseColor * (u_ambientStrength + upperAmbient + lowerAmbient);\n"
        "    linearColor += baseColor * keyDiffuse * u_keyLightStrength;\n"
        "    linearColor += baseColor * fillDiffuse * u_fillLightStrength;\n"
        "    linearColor += vec3(1.0, 0.97, 0.90) * specular * u_specularStrength;\n"
        "    linearColor += mix(baseColor, vec3(0.70, 0.82, 1.0), 0.45) * rim * u_rimStrength;\n"
        "\n"
        "    linearColor = linearColor / (linearColor + vec3(0.35));\n"
        "    vec3 displayColor = pow(max(linearColor, vec3(0.0)), vec3(1.0 / 2.2));\n"
        "    fragColor = vec4(displayColor, v_color.a);\n"
        "}\n";

    const bool vertexOk = m_meshProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader);
    const bool fragmentOk = m_meshProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader);
    const bool linkOk = m_meshProgram.link();

    assert(vertexOk);
    assert(fragmentOk);
    assert(linkOk);
}

void VoxelMeshViewWidget::createBuffers()
{
    m_backgroundVao.create();

    m_meshVao.create();
    m_meshVao.bind();

    m_vertexBuffer.create();
    m_vertexBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_vertexBuffer.bind();

    const int vertexByteSize = checkedBufferByteSize(m_mesh.vertices().size() * sizeof(DisplayVertex));

    if (vertexByteSize > 0)
    {
        m_vertexBuffer.allocate(&m_mesh.vertices()[0], vertexByteSize);
    }
    else
    {
        m_vertexBuffer.allocate(nullptr, 0);
    }

    m_vertexBufferCapacity = vertexByteSize;

    m_meshProgram.enableAttributeArray(0);
    m_meshProgram.setAttributeBuffer(0, GL_FLOAT, offsetof(DisplayVertex, x), 3, sizeof(DisplayVertex));

    m_meshProgram.enableAttributeArray(1);
    m_meshProgram.setAttributeBuffer(1, GL_FLOAT, offsetof(DisplayVertex, normalX), 3, sizeof(DisplayVertex));

    m_meshProgram.enableAttributeArray(2);
    m_meshProgram.setAttributeBuffer(2, GL_FLOAT, offsetof(DisplayVertex, color), 4, sizeof(DisplayVertex));

    m_indexBuffer.create();
    m_indexBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_indexBuffer.bind();

    const int indexByteSize = checkedBufferByteSize(m_mesh.indices().size() * sizeof(std::uint32_t));

    if (indexByteSize > 0)
    {
        m_indexBuffer.allocate(&m_mesh.indices()[0], indexByteSize);
    }
    else
    {
        m_indexBuffer.allocate(nullptr, 0);
    }

    m_indexBufferCapacity = indexByteSize;

    m_meshVao.release();
    m_vertexBuffer.release();
    m_indexBuffer.release();
}

void VoxelMeshViewWidget::updateMeshBuffer()
{
    if (!m_vertexBuffer.isCreated() || !m_indexBuffer.isCreated() || !m_meshVao.isCreated())
    {
        return;
    }

    m_meshVao.bind();

    m_vertexBuffer.bind();

    const int vertexByteSize = checkedBufferByteSize(m_mesh.vertices().size() * sizeof(DisplayVertex));

    if (vertexByteSize > m_vertexBufferCapacity)
    {
        m_vertexBuffer.allocate(&m_mesh.vertices()[0], vertexByteSize);
        m_vertexBufferCapacity = vertexByteSize;
    }
    else if (vertexByteSize > 0)
    {
        m_vertexBuffer.write(0, &m_mesh.vertices()[0], vertexByteSize);
    }

    m_indexBuffer.bind();

    const int indexByteSize = checkedBufferByteSize(m_mesh.indices().size() * sizeof(std::uint32_t));

    if (indexByteSize > m_indexBufferCapacity)
    {
        m_indexBuffer.allocate(&m_mesh.indices()[0], indexByteSize);
        m_indexBufferCapacity = indexByteSize;
    }
    else if (indexByteSize > 0)
    {
        m_indexBuffer.write(0, &m_mesh.indices()[0], indexByteSize);
    }

    m_meshVao.release();
    m_vertexBuffer.release();
    m_indexBuffer.release();
}

void VoxelMeshViewWidget::drawBackground()
{
    assert(m_functions);

    m_functions->glDisable(GL_DEPTH_TEST);
    m_functions->glDepthMask(GL_FALSE);
    m_functions->glDisable(GL_CULL_FACE);

    m_backgroundProgram.bind();

    // 顶部和底部颜色组成偏冷的工业CAD背景。
    m_backgroundProgram.setUniformValue("u_topColor", QVector3D(0.205f, 0.235f, 0.285f));
    m_backgroundProgram.setUniformValue("u_bottomColor", QVector3D(0.040f, 0.050f, 0.070f));

    m_backgroundVao.bind();
    m_functions->glDrawArrays(GL_TRIANGLES, 0, 3);
    m_backgroundVao.release();

    m_backgroundProgram.release();

    m_functions->glDepthMask(GL_TRUE);
    m_functions->glEnable(GL_DEPTH_TEST);
}

void VoxelMeshViewWidget::drawMesh(const QVector3D& eye, const QVector3D& viewCenter)
{
    assert(m_functions);
    assert(!m_mesh.isEmpty());

    const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    const float cameraDistance = (eye - viewCenter).length();

    // 近远裁剪面根据当前模型尺寸动态计算，以提高深度缓冲精度。
    const float nearPlane = std::max(0.01f, cameraDistance - m_radius * 1.50f);
    const float farPlane = std::max(nearPlane + 1.0f, cameraDistance + m_radius * 3.00f);

    QMatrix4x4 projection;
    projection.perspective(45.0f, aspect, nearPlane, farPlane);

    QMatrix4x4 view;
    view.lookAt(eye, viewCenter, QVector3D(0.0f, 0.0f, 1.0f));

    const QVector3D cameraDirectionValue = (eye - viewCenter).normalized();
    const QVector3D right = cameraRight(m_yaw, m_pitch);
    const QVector3D up = cameraUp(m_yaw, m_pitch);

    // 主光位于观察方向左上方，辅光位于右下方，用于减少体素台阶处的纯黑阴影。
    const QVector3D keyLightDirection = (cameraDirectionValue - right * 0.35f + up * 0.55f).normalized();
    const QVector3D fillLightDirection = (cameraDirectionValue + right * 0.75f - up * 0.15f).normalized();

    m_meshProgram.bind();
    m_meshProgram.setUniformValue("u_mvp", projection * view);
    m_meshProgram.setUniformValue("u_cameraPosition", eye);
    m_meshProgram.setUniformValue("u_keyLightDirection", keyLightDirection);
    m_meshProgram.setUniformValue("u_fillLightDirection", fillLightDirection);

    // 当前材质参数用于形成偏金属的CAM仿真显示效果。
    m_meshProgram.setUniformValue("u_ambientStrength", 0.24f);
    m_meshProgram.setUniformValue("u_keyLightStrength", 0.78f);
    m_meshProgram.setUniformValue("u_fillLightStrength", 0.20f);
    m_meshProgram.setUniformValue("u_specularStrength", 0.32f);
    m_meshProgram.setUniformValue("u_shininess", 64.0f);
    m_meshProgram.setUniformValue("u_rimStrength", 0.14f);

    m_meshVao.bind();
    m_functions->glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_mesh.indexCount()), GL_UNSIGNED_INT, nullptr);
    m_meshVao.release();

    m_meshProgram.release();
}

void VoxelMeshViewWidget::updateBounds()
{
    if (m_mesh.vertices().empty())
    {
        m_center = QVector3D(0.0f, 0.0f, 0.0f);
        m_radius = 1.0f;
        return;
    }

    const DisplayVertex& first = m_mesh.vertices()[0];

    QVector3D minimum(first.x, first.y, first.z);
    QVector3D maximum(first.x, first.y, first.z);

    for (const DisplayVertex& vertex : m_mesh.vertices())
    {
        minimum.setX(std::min(minimum.x(), vertex.x));
        minimum.setY(std::min(minimum.y(), vertex.y));
        minimum.setZ(std::min(minimum.z(), vertex.z));

        maximum.setX(std::max(maximum.x(), vertex.x));
        maximum.setY(std::max(maximum.y(), vertex.y));
        maximum.setZ(std::max(maximum.z(), vertex.z));
    }

    m_center = (minimum + maximum) * 0.5f;
    m_radius = std::max((maximum - minimum).length() * 0.5f, 1.0f);
}

void VoxelMeshViewWidget::applyMeshChange()
{
    updateBounds();

    if (m_initialized)
    {
        makeCurrent();
        updateMeshBuffer();
        doneCurrent();
    }

    update();
}

}