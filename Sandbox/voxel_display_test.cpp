#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <io.h>
#include <vector>

#include <QApplication>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QWidget>

#include "MyMath/Vector3.h"
#include "MyVoxel/Builder/ShapeBuilder.h"
#include "MyVoxel/Builder/ShapeVoxelizer.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "MyVoxel/Operation/ShapeTransform.h"
#include "MyVoxel/Shape/Shape.h"
#include "MyVoxel/Surface/VoxelSurfaceExtractor.h"
#include "MyVoxel/Surface/VoxelSurfaceGeneration.h"
#include "MyVoxel/Surface/VoxelSurfaceMesh.h"
#include "Viewer/Builder/ShapeDisplayBuilder.h"
#include "Viewer/Builder/VoxelSurfaceDisplayBuilder.h"
#include "Viewer/OpenGL/DisplayColor.h"
#include "Viewer/OpenGL/DisplayMesh.h"
#include "Viewer/OpenGL/VoxelDisplayInstance.h"
#include "Viewer/OpenGL/VoxelInstanceCollector.h"
#include "Viewer/Widget/VoxelInstanceViewWidget.h"
#include "Viewer/Widget/VoxelMeshViewWidget.h"

namespace
{

using Clock = std::chrono::steady_clock;

const double Pi = 3.14159265358979323846; // 圆周率，用于构造砂轮显示姿态。

// 设置Windows控制台使用宽字符输出，避免中文乱码。
void initializeConsole()
{
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_U16TEXT);
#endif
}

// 配置Viewer测试使用的OpenGL格式。
void configureOpenGL()
{
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);
}

// 返回两个时间点之间的毫秒数。
double elapsedMilliseconds(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// 检查显示顶点数据是否有限。
bool isFiniteVertex(const MyVoxelViewer::DisplayVertex& vertex)
{
    return std::isfinite(vertex.x) && std::isfinite(vertex.y) && std::isfinite(vertex.z) &&
           std::isfinite(vertex.normalX) && std::isfinite(vertex.normalY) && std::isfinite(vertex.normalZ) &&
           std::isfinite(vertex.color.red) && std::isfinite(vertex.color.green) &&
           std::isfinite(vertex.color.blue) && std::isfinite(vertex.color.alpha);
}

// 检查显示顶点法线是否有效。
bool hasValidNormal(const MyVoxelViewer::DisplayVertex& vertex)
{
    const double lengthSquared = static_cast<double>(vertex.normalX) * vertex.normalX +
                                 static_cast<double>(vertex.normalY) * vertex.normalY +
                                 static_cast<double>(vertex.normalZ) * vertex.normalZ;

    return std::isfinite(lengthSquared) && lengthSquared > 1.0e-12;
}

// 检查显示网格的基本拓扑和数据是否有效。
bool isValidDisplayMesh(const MyVoxelViewer::DisplayMesh& mesh)
{
    if (mesh.isEmpty() || mesh.indexCount() % 3 != 0)
    {
        return false;
    }

    for (const MyVoxelViewer::DisplayVertex& vertex : mesh.vertices())
    {
        if (!isFiniteVertex(vertex) || !hasValidNormal(vertex))
        {
            return false;
        }
    }

    for (std::uint32_t index : mesh.indices())
    {
        if (index >= mesh.vertexCount())
        {
            return false;
        }
    }

    return true;
}

// 检查体素表面网格数据是否有效。
bool isValidSurfaceMesh(const MyVoxel::VoxelSurfaceMesh& mesh)
{
    if (mesh.isEmpty() || mesh.indexCount() % 3 != 0)
    {
        return false;
    }

    for (const MyVoxel::VoxelSurfaceVertex& vertex : mesh.vertices())
    {
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z) ||
            !std::isfinite(vertex.nx) || !std::isfinite(vertex.ny) || !std::isfinite(vertex.nz) ||
            !std::isfinite(vertex.color.red) || !std::isfinite(vertex.color.green) ||
            !std::isfinite(vertex.color.blue) || !std::isfinite(vertex.color.alpha))
        {
            return false;
        }

        const double normalLengthSquared = static_cast<double>(vertex.nx) * vertex.nx +
                                           static_cast<double>(vertex.ny) * vertex.ny +
                                           static_cast<double>(vertex.nz) * vertex.nz;

        if (!std::isfinite(normalLengthSquared) || normalLengthSquared <= 1.0e-12)
        {
            return false;
        }
    }

    for (std::uint32_t index : mesh.indices())
    {
        if (index >= mesh.vertexCount())
        {
            return false;
        }
    }

    return true;
}

// 检查体素显示实例数据是否有效。
bool isValidInstance(const MyVoxelViewer::VoxelDisplayInstance& instance)
{
    for (int i = 0; i < 16; ++i)
    {
        if (!std::isfinite(instance.transform[i]))
        {
            return false;
        }
    }

    return std::isfinite(instance.color.red) && std::isfinite(instance.color.green) &&
           std::isfinite(instance.color.blue) && std::isfinite(instance.color.alpha);
}

// 检查全部体素实例是否有效。
bool areValidInstances(const std::vector<MyVoxelViewer::VoxelDisplayInstance>& instances)
{
    if (instances.empty())
    {
        return false;
    }

    for (const MyVoxelViewer::VoxelDisplayInstance& instance : instances)
    {
        if (!isValidInstance(instance))
        {
            return false;
        }
    }

    return true;
}

// 创建带标题的显示面板。
QGroupBox* createPanel(const QString& title, QWidget* view)
{
    QGroupBox* panel = new QGroupBox(title);
    QVBoxLayout* layout = new QVBoxLayout(panel);

    layout->setContentsMargins(6, 6, 6, 6);
    layout->addWidget(view);
    return panel;
}

}

int main(int argc, char* argv[])
{
    initializeConsole();
    configureOpenGL();

    QApplication application(argc, argv);

    const double baseVoxelEdgeLength = 1.0; // 第0层体素边长，单位为毫米。
    const MyVoxel::VoxelLevel maximumLevel = 5; // 显示测试使用的最高细分层级。
    const double toolRotation = Pi / 6.0; // 砂轮绕Y轴旋转30度。

    /// 标准Shape与切削结果

    const MyVoxel::Shape workpieceShape = MyVoxel::ShapeBuilder::makeBox(-3.0, -2.0, -1.5, 3.0, 2.0, 1.5);
    const MyVoxel::Shape sourceToolShape = MyVoxel::ShapeBuilder::makeCylinder(0.0, 0.0, -3.0, 3.0, 1.2);
    const MyVoxel::Shape rotatedToolShape = MyVoxel::rotate(sourceToolShape, MyMath::Vector3::unitY(), toolRotation);
    const MyVoxel::Shape cuttingToolShape = MyVoxel::translate(rotatedToolShape, MyMath::Vector3(0.5, 0.0, 0.0));

    const MyVoxel::VoxelShape workpiece = MyVoxel::voxelize(workpieceShape, baseVoxelEdgeLength, maximumLevel);
    const MyVoxel::VoxelShape cutResult = MyVoxel::cut(workpiece, cuttingToolShape);

    /// 标准Shape显示网格

    const MyVoxelViewer::DisplayColor toolColor(0.28, 0.58, 0.88);
    const MyVoxelViewer::DisplayMesh toolDisplayMesh = MyVoxelViewer::ShapeDisplayBuilder::build(cuttingToolShape, toolColor, 96);

    /// 阶梯贪心表面网格

    const MyVoxel::VoxelSurfaceColor greedySurfaceColor(0.72, 0.75, 0.80);

    MyVoxel::VoxelSurfaceGenerationOptions greedyOptions;
    greedyOptions.mode = MyVoxel::VoxelSurfaceGenerationMode::GreedyVoxel;

    const Clock::time_point greedyExtractionStart = Clock::now();
    const MyVoxel::VoxelSurfaceMesh greedySurfaceMesh = MyVoxel::VoxelSurfaceExtractor::extract(cutResult, greedySurfaceColor, greedyOptions);
    const Clock::time_point greedyExtractionEnd = Clock::now();

    const MyVoxelViewer::DisplayMesh greedyDisplayMesh = MyVoxelViewer::VoxelSurfaceDisplayBuilder::build(greedySurfaceMesh);

    /// 连续光滑表面网格

    const MyVoxel::VoxelSurfaceColor smoothSurfaceColor(0.76, 0.78, 0.82);

    MyVoxel::VoxelSurfaceGenerationOptions smoothOptions;
    smoothOptions.mode = MyVoxel::VoxelSurfaceGenerationMode::SmoothMarchingTetrahedra;
    smoothOptions.isoValue = 0.5;
    smoothOptions.boundaryPadding = 0;

    const Clock::time_point smoothExtractionStart = Clock::now();
    const MyVoxel::VoxelSurfaceMesh smoothSurfaceMesh = MyVoxel::VoxelSurfaceExtractor::extract(cutResult, smoothSurfaceColor, smoothOptions);
    const Clock::time_point smoothExtractionEnd = Clock::now();

    const MyVoxelViewer::DisplayMesh smoothDisplayMesh = MyVoxelViewer::VoxelSurfaceDisplayBuilder::build(smoothSurfaceMesh);

    /// 体素实例显示数据

    const MyVoxelViewer::DisplayColor instanceColor(0.72, 0.75, 0.80);
    const std::vector<MyVoxelViewer::VoxelDisplayInstance> instances = MyVoxelViewer::VoxelInstanceCollector::collect(cutResult, instanceColor);

    /// 数据验证

    const bool toolMeshValid = isValidDisplayMesh(toolDisplayMesh);
    const bool greedySurfaceMeshValid = isValidSurfaceMesh(greedySurfaceMesh);
    const bool greedyDisplayMeshValid = isValidDisplayMesh(greedyDisplayMesh);
    const bool smoothSurfaceMeshValid = isValidSurfaceMesh(smoothSurfaceMesh);
    const bool smoothDisplayMeshValid = isValidDisplayMesh(smoothDisplayMesh);
    const bool instancesValid = areValidInstances(instances);

    const double greedyExtractionMilliseconds = elapsedMilliseconds(greedyExtractionStart, greedyExtractionEnd);
    const double smoothExtractionMilliseconds = elapsedMilliseconds(smoothExtractionStart, smoothExtractionEnd);

    std::wcout << std::fixed << std::setprecision(3);

    std::wcout << L"========================================" << std::endl;
    std::wcout << L"可视化数据验证" << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"标准Shape显示网格       : " << (toolMeshValid ? L"通过" : L"失败") << std::endl;
    std::wcout << L"阶梯贪心表面提取        : " << (greedySurfaceMeshValid ? L"通过" : L"失败") << std::endl;
    std::wcout << L"阶梯贪心显示网格        : " << (greedyDisplayMeshValid ? L"通过" : L"失败") << std::endl;
    std::wcout << L"连续光滑表面提取        : " << (smoothSurfaceMeshValid ? L"通过" : L"失败") << std::endl;
    std::wcout << L"连续光滑显示网格        : " << (smoothDisplayMeshValid ? L"通过" : L"失败") << std::endl;
    std::wcout << L"体素实例数据            : " << (instancesValid ? L"通过" : L"失败") << std::endl;

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"显示网格统计" << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"最高细分层级            : " << static_cast<int>(maximumLevel) << std::endl;
    std::wcout << L"最高层体素边长          : " << cutResult.voxelEdgeLength(maximumLevel) << std::endl;
    std::wcout << L"标准Shape顶点数量       : " << toolDisplayMesh.vertexCount() << std::endl;
    std::wcout << L"标准Shape三角形数量     : " << toolDisplayMesh.indexCount() / 3 << std::endl;
    std::wcout << L"阶梯表面顶点数量        : " << greedySurfaceMesh.vertexCount() << std::endl;
    std::wcout << L"阶梯表面三角形数量      : " << greedySurfaceMesh.triangleCount() << std::endl;
    std::wcout << L"阶梯表面提取耗时        : " << greedyExtractionMilliseconds << L" ms" << std::endl;
    std::wcout << L"光滑表面顶点数量        : " << smoothSurfaceMesh.vertexCount() << std::endl;
    std::wcout << L"光滑表面三角形数量      : " << smoothSurfaceMesh.triangleCount() << std::endl;
    std::wcout << L"光滑表面提取耗时        : " << smoothExtractionMilliseconds << L" ms" << std::endl;
    std::wcout << L"体素显示实例数量        : " << instances.size() << std::endl;

    if (!toolMeshValid || !greedySurfaceMeshValid || !greedyDisplayMeshValid ||
        !smoothSurfaceMeshValid || !smoothDisplayMeshValid || !instancesValid)
    {
        return 1;
    }

    /// 创建显示窗口

    MyVoxelViewer::VoxelMeshViewWidget* shapeView = new MyVoxelViewer::VoxelMeshViewWidget;
    shapeView->setMesh(toolDisplayMesh);
    shapeView->setMinimumSize(320, 520);

    MyVoxelViewer::VoxelMeshViewWidget* greedySurfaceView = new MyVoxelViewer::VoxelMeshViewWidget;
    greedySurfaceView->setMesh(greedyDisplayMesh);
    greedySurfaceView->setMinimumSize(320, 520);

    MyVoxelViewer::VoxelMeshViewWidget* smoothSurfaceView = new MyVoxelViewer::VoxelMeshViewWidget;
    smoothSurfaceView->setMesh(smoothDisplayMesh);
    smoothSurfaceView->setMinimumSize(320, 520);

    MyVoxelViewer::VoxelInstanceViewWidget* instanceView = new MyVoxelViewer::VoxelInstanceViewWidget;
    instanceView->setInstances(instances);
    instanceView->setAutoRotate(false);
    instanceView->setMinimumSize(320, 520);

    QWidget window;
    QHBoxLayout* mainLayout = new QHBoxLayout(&window);

    mainLayout->addWidget(createPanel(QStringLiteral("Standard Shape"), shapeView));
    mainLayout->addWidget(createPanel(QStringLiteral("Greedy Voxel Surface"), greedySurfaceView));
    mainLayout->addWidget(createPanel(QStringLiteral("Smooth Surface"), smoothSurfaceView));
    mainLayout->addWidget(createPanel(QStringLiteral("Voxel Instances"), instanceView));

    window.setWindowTitle(QStringLiteral("MyVoxel Surface Generation Test"));
    window.resize(1680, 620);
    window.show();

    return application.exec();
}