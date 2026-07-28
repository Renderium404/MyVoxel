#include "VoxelInstanceViewWidget.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

#include <QKeyEvent>
#include <QMatrix4x4>
#include <QOpenGLExtraFunctions>
#include <QSurfaceFormat>
#include <QTimer>

namespace
{

struct CubeVertex
{
    CubeVertex() = default;

    CubeVertex(double xValue, double yValue, double zValue, double nxValue, double nyValue, double nzValue)
        : x(static_cast<float>(xValue))
        , y(static_cast<float>(yValue))
        , z(static_cast<float>(zValue))
        , nx(static_cast<float>(nxValue))
        , ny(static_cast<float>(nyValue))
        , nz(static_cast<float>(nzValue))
    {
    }

    float x;
    float y;
    float z;
    float nx;
    float ny;
    float nz;
};

const CubeVertex CubeVertices[] =
{
    CubeVertex(-0.5, -0.5,  0.5,  0.0,  0.0,  1.0),
    CubeVertex( 0.5, -0.5,  0.5,  0.0,  0.0,  1.0),
    CubeVertex( 0.5,  0.5,  0.5,  0.0,  0.0,  1.0),
    CubeVertex(-0.5,  0.5,  0.5,  0.0,  0.0,  1.0),

    CubeVertex( 0.5, -0.5, -0.5,  0.0,  0.0, -1.0),
    CubeVertex(-0.5, -0.5, -0.5,  0.0,  0.0, -1.0),
    CubeVertex(-0.5,  0.5, -0.5,  0.0,  0.0, -1.0),
    CubeVertex( 0.5,  0.5, -0.5,  0.0,  0.0, -1.0),

    CubeVertex(-0.5, -0.5, -0.5, -1.0,  0.0,  0.0),
    CubeVertex(-0.5, -0.5,  0.5, -1.0,  0.0,  0.0),
    CubeVertex(-0.5,  0.5,  0.5, -1.0,  0.0,  0.0),
    CubeVertex(-0.5,  0.5, -0.5, -1.0,  0.0,  0.0),

    CubeVertex( 0.5, -0.5,  0.5,  1.0,  0.0,  0.0),
    CubeVertex( 0.5, -0.5, -0.5,  1.0,  0.0,  0.0),
    CubeVertex( 0.5,  0.5, -0.5,  1.0,  0.0,  0.0),
    CubeVertex( 0.5,  0.5,  0.5,  1.0,  0.0,  0.0),

    CubeVertex(-0.5,  0.5,  0.5,  0.0,  1.0,  0.0),
    CubeVertex( 0.5,  0.5,  0.5,  0.0,  1.0,  0.0),
    CubeVertex( 0.5,  0.5, -0.5,  0.0,  1.0,  0.0),
    CubeVertex(-0.5,  0.5, -0.5,  0.0,  1.0,  0.0),

    CubeVertex(-0.5, -0.5, -0.5,  0.0, -1.0,  0.0),
    CubeVertex( 0.5, -0.5, -0.5,  0.0, -1.0,  0.0),
    CubeVertex( 0.5, -0.5,  0.5,  0.0, -1.0,  0.0),
    CubeVertex(-0.5, -0.5,  0.5,  0.0, -1.0,  0.0)
};

const unsigned int CubeIndices[] =
{
    0, 1, 2, 0, 2, 3,
    4, 5, 6, 4, 6, 7,
    8, 9, 10, 8, 10, 11,
    12, 13, 14, 12, 14, 15,
    16, 17, 18, 16, 18, 19,
    20, 21, 22, 20, 22, 23
};

const float CubeCoordinates[] =
{
    -0.5f,
    0.5f
};

// 使用实例矩阵变换单位立方体中的点。
QVector3D transformCubePoint(const MyVoxelViewer::VoxelDisplayInstance& instance, float x, float y, float z)
{
    const float* matrix = instance.transform;

    return QVector3D(matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12],
                     matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13],
                     matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14]);
}

}

namespace MyVoxelViewer
{

VoxelInstanceViewWidget::VoxelInstanceViewWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_vertexBuffer(QOpenGLBuffer::VertexBuffer)
    , m_indexBuffer(QOpenGLBuffer::IndexBuffer)
    , m_instanceBuffer(QOpenGLBuffer::VertexBuffer)
{
    setFocusPolicy(Qt::StrongFocus);

    QTimer* timer = new QTimer(this);
    QObject::connect(timer, &QTimer::timeout, [this]()
    {
        if (m_autoRotate)
        {
            m_angle += m_rotateStep;
            update();
        }
    });
    timer->start(16);
}

void VoxelInstanceViewWidget::setInstances(const std::vector<VoxelDisplayInstance>& instances)
{
    m_instances = instances;
    updateBounds();

    if (m_initialized)
    {
        makeCurrent();
        updateInstanceBuffer();
        doneCurrent();
        update();
    }
}

const std::vector<VoxelDisplayInstance>& VoxelInstanceViewWidget::instances() const
{
    return m_instances;
}

void VoxelInstanceViewWidget::setAutoRotate(bool enabled)
{
    m_autoRotate = enabled;
}

void VoxelInstanceViewWidget::setRotateStep(double degrees)
{
    m_rotateStep = static_cast<float>(degrees);
}

void VoxelInstanceViewWidget::initializeGL()
{
    m_functions = context()->extraFunctions();
    m_functions->initializeOpenGLFunctions();

    m_functions->glEnable(GL_DEPTH_TEST);
    m_functions->glEnable(GL_CULL_FACE);
    m_functions->glCullFace(GL_BACK);
    m_functions->glClearColor(0.08f, 0.09f, 0.11f, 1.0f);

    createShaderProgram();
    createBuffers();

    m_initialized = true;
}

void VoxelInstanceViewWidget::resizeGL(int width, int height)
{
    m_functions->glViewport(0, 0, width, height);
}

void VoxelInstanceViewWidget::paintGL()
{
    m_functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (m_instances.empty())
    {
        return;
    }

    const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    const float cameraDistance = std::max(12.0f, m_radius * 2.4f);
    const float radians = m_angle * 3.1415926f / 180.0f;

    QMatrix4x4 projection;
    projection.perspective(45.0f, aspect, 0.1f, 10000.0f);

    const QVector3D eye(m_center.x() + std::cos(radians) * cameraDistance,
                        m_center.y() - std::sin(radians) * cameraDistance,
                        m_center.z() + cameraDistance * 0.65f);

    QMatrix4x4 view;
    view.lookAt(eye, m_center, QVector3D(0.0f, 0.0f, 1.0f));

    m_program.bind();
    m_program.setUniformValue("u_mvp", projection * view);

    m_vao.bind();
    m_functions->glDrawElementsInstanced(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr, static_cast<GLsizei>(m_instances.size()));
    m_vao.release();

    m_program.release();
}

void VoxelInstanceViewWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Escape)
    {
        close();
        return;
    }

    QOpenGLWidget::keyPressEvent(event);
}

void VoxelInstanceViewWidget::createShaderProgram()
{
    const char* vertexShader =
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_position;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in mat4 a_instanceTransform;\n"
        "layout(location = 6) in vec4 a_instanceColor;\n"
        "uniform mat4 u_mvp;\n"
        "out vec3 v_normal;\n"
        "out vec4 v_color;\n"
        "void main()\n"
        "{\n"
        "    vec4 worldPosition = a_instanceTransform * vec4(a_position, 1.0);\n"
        "    mat3 normalMatrix = transpose(inverse(mat3(a_instanceTransform)));\n"
        "    gl_Position = u_mvp * worldPosition;\n"
        "    v_normal = normalize(normalMatrix * a_normal);\n"
        "    v_color = a_instanceColor;\n"
        "}\n";

    const char* fragmentShader =
        "#version 330 core\n"
        "in vec3 v_normal;\n"
        "in vec4 v_color;\n"
        "out vec4 fragColor;\n"
        "void main()\n"
        "{\n"
        "    vec3 lightDirection = normalize(vec3(0.4, -0.6, 0.8));\n"
        "    float diffuse = max(dot(normalize(v_normal), lightDirection), 0.18);\n"
        "    fragColor = vec4(v_color.rgb * diffuse, v_color.a);\n"
        "}\n";

    const bool vertexOk = m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader);
    const bool fragmentOk = m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader);
    const bool linkOk = m_program.link();

    assert(vertexOk);
    assert(fragmentOk);
    assert(linkOk);
}

void VoxelInstanceViewWidget::createBuffers()
{
    m_vao.create();
    m_vao.bind();

    m_vertexBuffer.create();
    m_vertexBuffer.bind();
    m_vertexBuffer.allocate(CubeVertices, static_cast<int>(sizeof(CubeVertices)));

    m_program.enableAttributeArray(0);
    m_program.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(CubeVertex));

    m_program.enableAttributeArray(1);
    m_program.setAttributeBuffer(1, GL_FLOAT, 3 * static_cast<int>(sizeof(float)), 3, sizeof(CubeVertex));

    m_indexBuffer.create();
    m_indexBuffer.bind();
    m_indexBuffer.allocate(CubeIndices, static_cast<int>(sizeof(CubeIndices)));

    m_instanceBuffer.create();
    m_instanceBuffer.bind();

    if (m_instances.empty())
    {
        m_instanceBuffer.allocate(nullptr, 0);
    }
    else
    {
        m_instanceBuffer.allocate(&m_instances[0], static_cast<int>(m_instances.size() * sizeof(VoxelDisplayInstance)));
    }

    const std::size_t transformOffset = offsetof(VoxelDisplayInstance, transform);

    for (int column = 0; column < 4; ++column)
    {
        const unsigned int location = static_cast<unsigned int>(2 + column);
        const std::size_t columnOffset = transformOffset + static_cast<std::size_t>(column * 4) * sizeof(float);

        m_functions->glEnableVertexAttribArray(location);
        m_functions->glVertexAttribPointer(location, 4, GL_FLOAT, GL_FALSE, sizeof(VoxelDisplayInstance), reinterpret_cast<void*>(columnOffset));
        m_functions->glVertexAttribDivisor(location, 1);
    }

    m_functions->glEnableVertexAttribArray(6);
    m_functions->glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(VoxelDisplayInstance), reinterpret_cast<void*>(offsetof(VoxelDisplayInstance, color)));
    m_functions->glVertexAttribDivisor(6, 1);

    m_vao.release();
    m_vertexBuffer.release();
    m_indexBuffer.release();
    m_instanceBuffer.release();
}

void VoxelInstanceViewWidget::updateInstanceBuffer()
{
    if (!m_instanceBuffer.isCreated())
    {
        return;
    }

    m_instanceBuffer.bind();

    if (m_instances.empty())
    {
        m_instanceBuffer.allocate(nullptr, 0);
    }
    else
    {
        m_instanceBuffer.allocate(&m_instances[0], static_cast<int>(m_instances.size() * sizeof(VoxelDisplayInstance)));
    }

    m_instanceBuffer.release();
}

void VoxelInstanceViewWidget::updateBounds()
{
    if (m_instances.empty())
    {
        m_center = QVector3D(0.0f, 0.0f, 0.0f);
        m_radius = 1.0f;
        return;
    }

    const QVector3D firstPoint = transformCubePoint(m_instances[0], CubeCoordinates[0], CubeCoordinates[0], CubeCoordinates[0]);
    QVector3D minimum = firstPoint;
    QVector3D maximum = firstPoint;

    for (const VoxelDisplayInstance& instance : m_instances)
    {
        for (int z = 0; z < 2; ++z)
        {
            for (int y = 0; y < 2; ++y)
            {
                for (int x = 0; x < 2; ++x)
                {
                    const QVector3D point = transformCubePoint(instance, CubeCoordinates[x], CubeCoordinates[y], CubeCoordinates[z]);

                    minimum.setX(std::min(minimum.x(), point.x()));
                    minimum.setY(std::min(minimum.y(), point.y()));
                    minimum.setZ(std::min(minimum.z(), point.z()));

                    maximum.setX(std::max(maximum.x(), point.x()));
                    maximum.setY(std::max(maximum.y(), point.y()));
                    maximum.setZ(std::max(maximum.z(), point.z()));
                }
            }
        }
    }

    m_center = (minimum + maximum) * 0.5f;
    m_radius = std::max((maximum - minimum).length() * 0.5f, 1.0f);
}

}