#include <cassert>
#include <cstdlib>
#include <exception>

#include <QAction>
#include <QApplication>
#include <QDebug>
#include <QMatrix4x4>
#include <QStatusBar>
#include <QString>
#include <QToolBar>

#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/PrimitiveModeling.h"
#include "MyVoxel/Modeling/ShapeVoxelization.h"
#include "MyVoxel/Surface/VoxelSurfaceMesher.h"
#include "MyVoxel/Volume/VolumeFieldBuilder.h"
#include "MyVoxel/Volume/VolumeFieldView.h"
#include "Viewer/OpenGL/VoxelOpenGLWidget.h"
#include "Viewer/OpenGL/VoxelViewerWindow.h"
#include "MyVoxel/Modeling/ModelingBuilder.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，圆弧角度统一使用弧度制。

}

// 创建带顶部圆角的轴对称回转实体。
MyVoxel::Geometry::Shape createRevolvedBody()
{
    using MyMath::Vector3;
    using MyVoxel::Foundation::RefPtr;
    using MyVoxel::Geometry::Curve;
    using MyVoxel::Geometry::CurveLoop;
    using MyVoxel::Geometry::Shape;
    using MyVoxel::Modeling::ModelingBuilder;

    const Vector3 axisBottom(0.0, -30.0, 0.0); // 旋转轴下端。
    const Vector3 outerBottom(12.0, -30.0, 0.0); // 外圆柱底部。
    const Vector3 outerShoulder(12.0, 10.0, 0.0); // 顶部圆角起点。
    const Vector3 roundedTop(8.0, 14.0, 0.0); // 顶部圆角终点。
    const Vector3 axisTop(0.0, 14.0, 0.0); // 旋转轴上端。

    std::vector<RefPtr<const Curve>> curves;
    curves.reserve(5);
    curves.push_back(ModelingBuilder::line(axisBottom, outerBottom));
    curves.push_back(ModelingBuilder::line(outerBottom, outerShoulder));
    curves.push_back(ModelingBuilder::arc(Vector3(8.0, 10.0, 0.0), 4.0, 0.0, Pi * 0.5));
    curves.push_back(ModelingBuilder::line(roundedTop, axisTop));
    curves.push_back(ModelingBuilder::line(axisTop, axisBottom));

    const CurveLoop profile = ModelingBuilder::curveLoop(curves);
    return ModelingBuilder::revolve(profile);
}
namespace
{

const double BaseVoxelEdgeLength = 8.0; // 第0层体素边长为64毫米，最高层边长由细分层级决定。
const MyVoxel::VoxelLevel MaximumVoxelLevel = static_cast<MyVoxel::VoxelLevel>(6); // 六次细分后最高层体素边长为1毫米。
const double SphereRadius = 18.0; // 使用半径18毫米的球体观察曲面平滑程度。
const double DisplaySpacing = 48.0; // 三种网格沿X方向间隔48毫米并排显示。
const double BandWidth = 3.0; // 距离场内外各保留三个最高层体素宽度，满足中心差分梯度读取。

const MyVoxel::Geometry::MeshColor VoxelColor(0.88, 0.48, 0.18, 1.0); // 左侧阶梯体素表面使用橙色。
const MyVoxel::Geometry::MeshColor StandardColor(0.28, 0.76, 0.42, 1.0); // 中间标准Surface Nets使用绿色。
const MyVoxel::Geometry::MeshColor FeatureColor(0.24, 0.52, 0.92, 1.0); // 右侧特征敏感QEF表面使用蓝色。

// 创建只包含X方向平移的模型矩阵。
QMatrix4x4 translationMatrix(float x)
{
    QMatrix4x4 matrix;
    matrix.setToIdentity();
    matrix.translate(x, 0.0f, 0.0f);
    return matrix;
}

// 并排显示体素阶梯面、标准Surface Nets和特征敏感QEF曲面。
class VolumeFieldComparisonWindow : public MyVoxelViewer::VoxelViewerWindow
{
public:
    explicit VolumeFieldComparisonWindow(QWidget* parent = nullptr)
        : MyVoxelViewer::VoxelViewerWindow(parent)
    {
        setWindowTitle(QStringLiteral("MyVoxel VolumeField Smooth Surface Comparison"));
        m_ready = initializeScene();
    }

    // 判断体素化、距离场构建、网格化和OpenGL对象创建是否全部成功。
    bool isReady() const
    {
        return m_ready;
    }

private:
    /// 场景创建

    // 创建测试球体并生成三种用于视觉对比的表面网格。
    bool initializeScene()
    {
        try
        {
            const MyVoxel::Geometry::Shape sphere = createRevolvedBody();
            MyVoxel::VoxelShape voxelShape = MyVoxel::Modeling::voxelize(sphere, BaseVoxelEdgeLength, MaximumVoxelLevel);

            if (!voxelShape.isValid() || voxelShape.isEmpty())
            {
                qCritical() << "Sphere voxelization produced an invalid or empty VoxelShape.";
                statusBar()->showMessage(QStringLiteral("Sphere voxelization failed."));
                return false;
            }

            const MyVoxel::Geometry::Mesh voxelMesh = MyVoxel::VoxelSurfaceMesher::build(voxelShape, VoxelColor);

            MyVoxel::VolumeFieldBuildOptions fieldOptions;
            fieldOptions.exteriorBandWidth = BandWidth;
            fieldOptions.interiorBandWidth = BandWidth;
            MyVoxel::VolumeFieldBuilder::build(voxelShape, fieldOptions);

            const MyVoxel::VolumeFieldView fieldView(voxelShape);

            if (!fieldView.isCurrent())
            {
                qCritical() << "VolumeFieldBuilder did not produce a current distance field.";
                statusBar()->showMessage(QStringLiteral("Distance-field construction failed."));
                return false;
            }

            MyVoxel::Meshing::VolumeMeshingOptions standardOptions;
            standardOptions.color = StandardColor;
            standardOptions.mode = MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets;
            const MyVoxel::Geometry::Mesh standardMesh = MyVoxel::Meshing::VolumeMesher::build(fieldView, standardOptions);

            MyVoxel::Meshing::VolumeMeshingOptions featureOptions;
            featureOptions.color = FeatureColor;
            featureOptions.mode = MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets;
            const MyVoxel::Geometry::Mesh featureMesh = MyVoxel::Meshing::VolumeMesher::build(fieldView, featureOptions);

            if (!voxelMesh.isRenderable() || !standardMesh.isRenderable() || !featureMesh.isRenderable())
            {
                qCritical() << "At least one comparison mesh is not renderable.";
                statusBar()->showMessage(QStringLiteral("Comparison mesh generation failed."));
                return false;
            }

            m_voxelObjectId = addMesh(voxelMesh, translationMatrix(static_cast<float>(-DisplaySpacing)));
            m_standardObjectId = addMesh(standardMesh, translationMatrix(0.0f));
            m_featureObjectId = addMesh(featureMesh, translationMatrix(static_cast<float>(DisplaySpacing)));

            if (m_voxelObjectId == MyVoxelViewer::VoxelOpenGLWidget::invalidMeshObjectId() ||
                m_standardObjectId == MyVoxelViewer::VoxelOpenGLWidget::invalidMeshObjectId() ||
                m_featureObjectId == MyVoxelViewer::VoxelOpenGLWidget::invalidMeshObjectId())
            {
                qCritical() << "OpenGL viewer failed to create one or more mesh objects.";
                statusBar()->showMessage(QStringLiteral("OpenGL object creation failed."));
                return false;
            }

            createComparisonToolBar();
            viewer()->setIsometricView();
            viewer()->fitAll();

            statusBar()->showMessage(QStringLiteral("Left: voxel surface | Center: standard Surface Nets | Right: feature-sensitive QEF | Minimum voxel edge: %1 mm").arg(voxelShape.grid().minimumCellEdgeLength(), 0, 'f', 3));
            qDebug().noquote() << QStringLiteral("Voxel surface: %1 vertices / %2 triangles").arg(static_cast<qulonglong>(voxelMesh.vertexCount())).arg(static_cast<qulonglong>(voxelMesh.triangleCount()));
            qDebug().noquote() << QStringLiteral("Standard Surface Nets: %1 vertices / %2 triangles").arg(static_cast<qulonglong>(standardMesh.vertexCount())).arg(static_cast<qulonglong>(standardMesh.triangleCount()));
            qDebug().noquote() << QStringLiteral("Feature-sensitive QEF: %1 vertices / %2 triangles").arg(static_cast<qulonglong>(featureMesh.vertexCount())).arg(static_cast<qulonglong>(featureMesh.triangleCount()));
            return true;
        }
        catch (const std::exception& exception)
        {
            qCritical() << "Volume-field OpenGL comparison failed:" << exception.what();
        }
        catch (...)
        {
            qCritical() << "Volume-field OpenGL comparison failed with an unknown exception.";
        }

        statusBar()->showMessage(QStringLiteral("Volume-field OpenGL comparison failed."));
        return false;
    }

    // 创建三种表面的独立显示开关。
    void createComparisonToolBar()
    {
        QToolBar* toolBar = addToolBar(QStringLiteral("Comparison"));
        toolBar->setMovable(false);

        QAction* voxelAction = toolBar->addAction(QStringLiteral("Voxel"));
        QAction* standardAction = toolBar->addAction(QStringLiteral("Standard"));
        QAction* featureAction = toolBar->addAction(QStringLiteral("Feature"));
        voxelAction->setCheckable(true);
        standardAction->setCheckable(true);
        featureAction->setCheckable(true);
        voxelAction->setChecked(true);
        standardAction->setChecked(true);
        featureAction->setChecked(true);

        connect(voxelAction, &QAction::toggled, this, [this](bool visible)
        {
            setMeshObjectVisible(m_voxelObjectId, visible);
            viewer()->fitAll();
        });

        connect(standardAction, &QAction::toggled, this, [this](bool visible)
        {
            setMeshObjectVisible(m_standardObjectId, visible);
            viewer()->fitAll();
        });

        connect(featureAction, &QAction::toggled, this, [this](bool visible)
        {
            setMeshObjectVisible(m_featureObjectId, visible);
            viewer()->fitAll();
        });
    }

private:
    MyVoxelViewer::MeshObjectId m_voxelObjectId = 0; // 左侧原始体素阶梯表面对象。
    MyVoxelViewer::MeshObjectId m_standardObjectId = 0; // 中间标准Surface Nets对象。
    MyVoxelViewer::MeshObjectId m_featureObjectId = 0; // 右侧特征敏感QEF对象。
    bool m_ready = false; // 场景是否已经完整创建。
};

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    VolumeFieldComparisonWindow window;

    if (!window.isReady())
    {
        return EXIT_FAILURE;
    }

    window.show();
    return application.exec();
}