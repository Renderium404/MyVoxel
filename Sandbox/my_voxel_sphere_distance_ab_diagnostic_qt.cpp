#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDebug>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QStatusBar>
#include <QString>
#include <QSurfaceFormat>
#include <QToolBar>
#include <QVector3D>
#include <QWheelEvent>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Storage/VolumeBlock.h"
#include "MyVoxel/Core/Volume/VolumeField.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Meshing/VolumeMesher.h"
#include "MyVoxel/Modeling/PrimitiveModeling.h"
#include "MyVoxel/Modeling/ShapeVoxelization.h"
#include "MyVoxel/Surface/VoxelSurfaceMesher.h"
#include "MyVoxel/Volume/VolumeFieldBuilder.h"
#include "MyVoxel/Volume/VolumeFieldView.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 解析球参数化使用的圆周率。
const double BaseVoxelEdgeLength = 64.0; // 第0层体素边长为64毫米。
const double SphereRadius = 18.0; // 所有对比对象使用半径18毫米的球体。
const double BandWidth = 3.0; // 距离场内外各保留三个最高层样本宽度。
const double DisplaySpacing = 43.0; // 五个A/B诊断对象沿X方向间隔43毫米。
const double ErrorExaggeration = 10.0; // 夸张网格将径向误差放大十倍。
const unsigned int ReferenceSliceCount = 128; // 解析参考球圆周使用128段。
const unsigned int ReferenceStackCount = 64; // 解析参考球纬向使用64层。
const float MinimumSceneRadius = 0.001f; // 空场景使用的最小观察半径。
const float MinimumCameraScale = 0.30f; // 滚轮缩放允许的最小相机距离比例。
const float MaximumCameraScale = 50.0f; // 滚轮缩放允许的最大相机距离比例。
const float DiagnosticRed = 0.72f; // 所有诊断对象使用相同中性灰色，避免颜色差异干扰判断。
const float DiagnosticGreen = 0.74f;
const float DiagnosticBlue = 0.78f;
const float DiagnosticAlpha = 1.0f;

const MyVoxel::Geometry::MeshColor DiagnosticColor(DiagnosticRed, DiagnosticGreen, DiagnosticBlue, DiagnosticAlpha);

enum class DiagnosticRenderMode
{
    Unlit = 0,
    SmoothDiffuse = 1,
    SmoothSpecular = 2,
    FlatDiffuse = 3,
    NormalRgb = 4
};

// 返回带指定法线的网格顶点。
MyVoxel::Geometry::MeshVertex makeVertex(const MyMath::Vector3& point, const MyMath::Vector3& normal)
{
    return MyVoxel::Geometry::MeshVertex(point.x(), point.y(), point.z(), normal.x(), normal.y(), normal.z());
}

// 返回MeshVertex的位置向量。
MyMath::Vector3 vertexPosition(const MyVoxel::Geometry::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.x), static_cast<double>(vertex.y), static_cast<double>(vertex.z));
}

// 返回MeshVertex的法线向量。
MyMath::Vector3 vertexNormal(const MyVoxel::Geometry::MeshVertex& vertex)
{
    return MyMath::Vector3(static_cast<double>(vertex.normalX), static_cast<double>(vertex.normalY), static_cast<double>(vertex.normalZ));
}

// 创建具有解析单位法线的UV参考球，用于排除渲染器自身产生的摩尔纹。
MyVoxel::Geometry::Mesh makeReferenceSphere(double radius, unsigned int sliceCount, unsigned int stackCount)
{
    assert(radius > 0.0);
    assert(sliceCount >= 3);
    assert(stackCount >= 2);

    MyVoxel::Geometry::Mesh mesh;
    const std::size_t ringCount = static_cast<std::size_t>(stackCount - 1);
    const std::size_t vertexCount = static_cast<std::size_t>(2) + ringCount * static_cast<std::size_t>(sliceCount);
    const std::size_t triangleCount = static_cast<std::size_t>(2) * static_cast<std::size_t>(sliceCount) * static_cast<std::size_t>(stackCount - 1);
    mesh.reserve(vertexCount, triangleCount * 3);

    const std::uint32_t topIndex = mesh.appendVertex(MyVoxel::Geometry::MeshVertex(0.0, 0.0, radius, 0.0, 0.0, 1.0));

    for (unsigned int stack = 1; stack < stackCount; ++stack)
    {
        const double theta = Pi * static_cast<double>(stack) / static_cast<double>(stackCount);
        const double sinTheta = std::sin(theta);
        const double cosTheta = std::cos(theta);

        for (unsigned int slice = 0; slice < sliceCount; ++slice)
        {
            const double phi = 2.0 * Pi * static_cast<double>(slice) / static_cast<double>(sliceCount);
            const MyMath::Vector3 normal(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
            mesh.appendVertex(makeVertex(normal * radius, normal));
        }
    }

    const std::uint32_t bottomIndex = mesh.appendVertex(MyVoxel::Geometry::MeshVertex(0.0, 0.0, -radius, 0.0, 0.0, -1.0));
    const std::uint32_t firstRingIndex = 1;

    for (unsigned int slice = 0; slice < sliceCount; ++slice)
    {
        const std::uint32_t current = firstRingIndex + slice;
        const std::uint32_t next = firstRingIndex + (slice + 1) % sliceCount;
        mesh.appendTriangle(topIndex, current, next, DiagnosticColor);
    }

    for (unsigned int stack = 0; stack + 1 < stackCount - 1; ++stack)
    {
        const std::uint32_t upperRing = firstRingIndex + stack * sliceCount;
        const std::uint32_t lowerRing = upperRing + sliceCount;

        for (unsigned int slice = 0; slice < sliceCount; ++slice)
        {
            const std::uint32_t nextSlice = (slice + 1) % sliceCount;
            const std::uint32_t upperCurrent = upperRing + slice;
            const std::uint32_t upperNext = upperRing + nextSlice;
            const std::uint32_t lowerCurrent = lowerRing + slice;
            const std::uint32_t lowerNext = lowerRing + nextSlice;
            mesh.appendTriangle(upperCurrent, lowerCurrent, lowerNext, DiagnosticColor);
            mesh.appendTriangle(upperCurrent, lowerNext, upperNext, DiagnosticColor);
        }
    }

    const std::uint32_t lastRingIndex = firstRingIndex + static_cast<std::uint32_t>(stackCount - 2) * sliceCount;

    for (unsigned int slice = 0; slice < sliceCount; ++slice)
    {
        const std::uint32_t current = lastRingIndex + slice;
        const std::uint32_t next = lastRingIndex + (slice + 1) % sliceCount;
        mesh.appendTriangle(current, bottomIndex, next, DiagnosticColor);
    }

    assert(mesh.isRenderable());
    return mesh;
}

// 按相邻三角形面积加权重新计算共享顶点法线，几何位置和三角形拓扑保持不变。
MyVoxel::Geometry::Mesh makeAreaWeightedNormalMesh(const MyVoxel::Geometry::Mesh& source)
{
    assert(source.isRenderable());

    MyVoxel::Geometry::Mesh result = source;
    std::vector<MyMath::Vector3> accumulatedNormals(source.vertexCount(), MyMath::Vector3::zero());
    const std::vector<MyVoxel::Geometry::MeshVertex>& vertices = source.vertices();
    const std::vector<std::uint32_t>& indices = source.indices();

    for (std::size_t triangleIndex = 0; triangleIndex < source.triangleCount(); ++triangleIndex)
    {
        const std::size_t offset = triangleIndex * 3;
        const std::uint32_t index0 = indices[offset];
        const std::uint32_t index1 = indices[offset + 1];
        const std::uint32_t index2 = indices[offset + 2];
        const MyMath::Vector3 point0 = vertexPosition(vertices[index0]);
        const MyMath::Vector3 point1 = vertexPosition(vertices[index1]);
        const MyMath::Vector3 point2 = vertexPosition(vertices[index2]);
        const MyMath::Vector3 weightedNormal = MyMath::Vector3::cross(point1 - point0, point2 - point0);
        accumulatedNormals[index0] += weightedNormal;
        accumulatedNormals[index1] += weightedNormal;
        accumulatedNormals[index2] += weightedNormal;
    }

    for (std::size_t vertexIndex = 0; vertexIndex < source.vertexCount(); ++vertexIndex)
    {
        MyMath::Vector3 normal = accumulatedNormals[vertexIndex];

        if (!normal.normalize())
        {
            normal = vertexNormal(vertices[vertexIndex]);

            if (!normal.normalize())
            {
                normal = MyMath::Vector3::unitZ();
            }
        }

        result.setVertex(static_cast<std::uint32_t>(vertexIndex), makeVertex(vertexPosition(vertices[vertexIndex]), normal));
    }

    assert(result.isRenderable());
    return result;
}


// 保存球面网格相对解析球面的径向误差统计。
struct SphereErrorStatistics
{
    SphereErrorStatistics()
        : minimumError(0.0)
        , maximumError(0.0)
        , meanError(0.0)
        , meanAbsoluteError(0.0)
        , rootMeanSquareError(0.0)
        , percentile95AbsoluteError(0.0)
        , edgeRootMeanSquareDelta(0.0)
        , maximumEdgeDelta(0.0)
        , meanNormalAngleDegrees(0.0)
        , maximumNormalAngleDegrees(0.0)
    {
    }

    double minimumError; // 最大内凹误差，负值表示位于解析球内部。
    double maximumError; // 最大外凸误差，正值表示位于解析球外部。
    double meanError; // 全部顶点径向误差的有符号平均值。
    double meanAbsoluteError; // 全部顶点径向误差绝对值平均值。
    double rootMeanSquareError; // 全部顶点径向误差均方根。
    double percentile95AbsoluteError; // 95%顶点不超过的径向绝对误差。
    double edgeRootMeanSquareDelta; // 相邻网格顶点径向误差差值的均方根。
    double maximumEdgeDelta; // 相邻网格顶点径向误差差值的最大值。
    double meanNormalAngleDegrees; // 顶点法线相对解析径向法线的平均夹角。
    double maximumNormalAngleDegrees; // 顶点法线相对解析径向法线的最大夹角。
};


// 保存当前构建距离样本相对解析球有符号距离的误差统计。
struct FieldSampleErrorStatistics
{
    FieldSampleErrorStatistics()
        : sampleCount(0)
        , nearSurfaceSampleCount(0)
        , minimumError(0.0)
        , maximumError(0.0)
        , meanError(0.0)
        , meanAbsoluteError(0.0)
        , rootMeanSquareError(0.0)
        , nearSurfaceMeanAbsoluteError(0.0)
        , nearSurfaceRootMeanSquareError(0.0)
    {
    }

    std::size_t sampleCount; // 当前全部显式距离样本数量。
    std::size_t nearSurfaceSampleCount; // 解析距离绝对值不超过一个最高层体素边长的样本数量。
    double minimumError; // 当前距离减去解析距离的最小误差。
    double maximumError; // 当前距离减去解析距离的最大误差。
    double meanError; // 全部显式样本的有符号平均误差。
    double meanAbsoluteError; // 全部显式样本的平均绝对误差。
    double rootMeanSquareError; // 全部显式样本的均方根误差。
    double nearSurfaceMeanAbsoluteError; // 近表面样本的平均绝对误差。
    double nearSurfaceRootMeanSquareError; // 近表面样本的均方根误差。
};

// 返回指定局部空间位置到解析球面的有符号距离。
double analyticSphereDistance(const MyMath::Vector3& point, double radius)
{
    return std::sqrt(point.lengthSquared()) - radius;
}

// 使用当前VolumeField的稀疏块布局写入解析球有符号距离，材料真值和背景距离保持不变。
bool buildAnalyticSphereField(MyVoxel::VoxelShape& targetShape, const MyVoxel::VolumeField& layoutField, double radius)
{
    assert(targetShape.isValid());
    assert(layoutField.isValid());
    assert(radius > 0.0);

    MyVoxel::VolumeField* targetField = targetShape.editVolumeField();

    if (!targetField)
    {
        return false;
    }

    targetField->resetPreservingStorage();
    targetField->setBackgroundDistances(layoutField.interiorBackgroundDistance(), layoutField.exteriorBackgroundDistance());

    const MyVoxel::VolumeField::BlockIndexMap& blockIndices = layoutField.blockIndices();

    for (MyVoxel::VolumeField::BlockIndexMap::const_iterator iterator = blockIndices.begin(); iterator != blockIndices.end(); ++iterator)
    {
        const MyVoxel::VoxelCellAddress blockAddress(iterator->first, layoutField.blockLevel());
        MyVoxel::VolumeBlock& block = targetField->ensureBlock(blockAddress);

        for (unsigned int sampleIndex = 0; sampleIndex < MyVoxel::VolumeBlockSampleCount; ++sampleIndex)
        {
            const MyVoxel::VoxelCellAddress sampleAddress = targetField->sampleAddress(blockAddress, sampleIndex);
            const MyMath::Vector3 samplePosition = targetShape.grid().cellCenter(sampleAddress);
            MyVoxel::volumeDistance(block, sampleIndex) = static_cast<float>(analyticSphereDistance(samplePosition, radius));
        }
    }

    targetField->markAllValid();
    return targetField->isValid() && targetField->isCurrent() && targetField->blockCount() == layoutField.blockCount();
}

// 统计当前构建距离场相对解析球距离的直接样本误差。
FieldSampleErrorStatistics calculateFieldSampleErrorStatistics(const MyVoxel::VolumeFieldView& view, double radius)
{
    assert(view.isCurrent());
    assert(radius > 0.0);

    FieldSampleErrorStatistics statistics;
    const MyVoxel::VolumeField& field = view.field();
    const MyVoxel::VolumeField::BlockIndexMap& blockIndices = field.blockIndices();
    const double nearSurfaceWidth = view.grid().minimumCellEdgeLength();
    double errorSum = 0.0;
    double absoluteErrorSum = 0.0;
    double squaredErrorSum = 0.0;
    double nearSurfaceAbsoluteErrorSum = 0.0;
    double nearSurfaceSquaredErrorSum = 0.0;

    for (MyVoxel::VolumeField::BlockIndexMap::const_iterator iterator = blockIndices.begin(); iterator != blockIndices.end(); ++iterator)
    {
        const MyVoxel::VoxelCellAddress blockAddress(iterator->first, field.blockLevel());
        const MyVoxel::VolumeBlock* block = field.findBlock(blockAddress);
        assert(block);

        for (unsigned int sampleIndex = 0; sampleIndex < MyVoxel::VolumeBlockSampleCount; ++sampleIndex)
        {
            const MyVoxel::VoxelCellAddress sampleAddress = field.sampleAddress(blockAddress, sampleIndex);
            const MyMath::Vector3 point = view.samplePosition(sampleAddress);
            const double exactDistance = analyticSphereDistance(point, radius);
            const double error = static_cast<double>(MyVoxel::volumeDistance(*block, sampleIndex)) - exactDistance;

            if (statistics.sampleCount == 0)
            {
                statistics.minimumError = error;
                statistics.maximumError = error;
            }
            else
            {
                statistics.minimumError = (std::min)(statistics.minimumError, error);
                statistics.maximumError = (std::max)(statistics.maximumError, error);
            }

            ++statistics.sampleCount;
            errorSum += error;
            absoluteErrorSum += std::fabs(error);
            squaredErrorSum += error * error;

            if (std::fabs(exactDistance) <= nearSurfaceWidth)
            {
                ++statistics.nearSurfaceSampleCount;
                nearSurfaceAbsoluteErrorSum += std::fabs(error);
                nearSurfaceSquaredErrorSum += error * error;
            }
        }
    }

    if (statistics.sampleCount > 0)
    {
        const double count = static_cast<double>(statistics.sampleCount);
        statistics.meanError = errorSum / count;
        statistics.meanAbsoluteError = absoluteErrorSum / count;
        statistics.rootMeanSquareError = std::sqrt(squaredErrorSum / count);
    }

    if (statistics.nearSurfaceSampleCount > 0)
    {
        const double count = static_cast<double>(statistics.nearSurfaceSampleCount);
        statistics.nearSurfaceMeanAbsoluteError = nearSurfaceAbsoluteErrorSum / count;
        statistics.nearSurfaceRootMeanSquareError = std::sqrt(nearSurfaceSquaredErrorSum / count);
    }

    return statistics;
}

// 输出当前构建距离场相对解析球距离的直接样本误差。
void printFieldSampleErrorStatistics(MyVoxel::VoxelLevel level,
                                     double minimumVoxelEdgeLength,
                                     const FieldSampleErrorStatistics& statistics)
{
    qDebug().noquote() << QStringLiteral("VolumeField sample error against analytic sphere | level=%1 | edge=%2 mm | samples=%3 | near=%4")
        .arg(static_cast<unsigned int>(level))
        .arg(minimumVoxelEdgeLength, 0, 'f', 6)
        .arg(static_cast<qulonglong>(statistics.sampleCount))
        .arg(static_cast<qulonglong>(statistics.nearSurfaceSampleCount));
    qDebug().noquote() << QStringLiteral("  all stored samples mm: min=%1 | max=%2 | mean=%3 | MAE=%4 | RMS=%5")
        .arg(statistics.minimumError, 0, 'f', 9)
        .arg(statistics.maximumError, 0, 'f', 9)
        .arg(statistics.meanError, 0, 'f', 9)
        .arg(statistics.meanAbsoluteError, 0, 'f', 9)
        .arg(statistics.rootMeanSquareError, 0, 'f', 9);
    qDebug().noquote() << QStringLiteral("  near surface (|exact|<=1 voxel) mm: MAE=%1 | RMS=%2 | normalized MAE=%3 | normalized RMS=%4")
        .arg(statistics.nearSurfaceMeanAbsoluteError, 0, 'f', 9)
        .arg(statistics.nearSurfaceRootMeanSquareError, 0, 'f', 9)
        .arg(statistics.nearSurfaceMeanAbsoluteError / minimumVoxelEdgeLength, 0, 'f', 9)
        .arg(statistics.nearSurfaceRootMeanSquareError / minimumVoxelEdgeLength, 0, 'f', 9);
}

// 返回两个颜色按照指定比例线性插值的结果。
MyVoxel::Geometry::MeshColor interpolateColor(const MyVoxel::Geometry::MeshColor& first,
                                              const MyVoxel::Geometry::MeshColor& second,
                                              double factor)
{
    const double t = (std::max)(0.0, (std::min)(1.0, factor));
    return MyVoxel::Geometry::MeshColor(
        first.red + (second.red - first.red) * t,
        first.green + (second.green - first.green) * t,
        first.blue + (second.blue - first.blue) * t,
        first.alpha + (second.alpha - first.alpha) * t);
}

// 将径向误差映射为蓝色内凹、绿色准确、红色外凸的热力图颜色。
MyVoxel::Geometry::MeshColor radialErrorColor(double error, double colorRange)
{
    assert(colorRange > 0.0);
    const MyVoxel::Geometry::MeshColor negativeColor(0.08, 0.24, 0.96, 1.0);
    const MyVoxel::Geometry::MeshColor zeroColor(0.12, 0.82, 0.22, 1.0);
    const MyVoxel::Geometry::MeshColor positiveColor(0.98, 0.16, 0.08, 1.0);
    const double normalized = (std::max)(-1.0, (std::min)(1.0, error / colorRange));
    return normalized < 0.0
        ? interpolateColor(zeroColor, negativeColor, -normalized)
        : interpolateColor(zeroColor, positiveColor, normalized);
}

// 计算指定球面重建网格的径向误差、局部跳变和法线方向误差。
SphereErrorStatistics calculateSphereErrorStatistics(const MyVoxel::Geometry::Mesh& mesh,
                                                     double radius,
                                                     std::vector<double>& vertexErrors)
{
    assert(mesh.isRenderable());
    assert(radius > 0.0);

    SphereErrorStatistics statistics;
    vertexErrors.resize(mesh.vertexCount());
    std::vector<double> absoluteErrors(mesh.vertexCount());
    double errorSum = 0.0;
    double absoluteErrorSum = 0.0;
    double squaredErrorSum = 0.0;
    double normalAngleSum = 0.0;
    std::size_t validNormalCount = 0;

    for (std::size_t vertexIndex = 0; vertexIndex < mesh.vertexCount(); ++vertexIndex)
    {
        const MyMath::Vector3 point = vertexPosition(mesh.vertices()[vertexIndex]);
        const double pointRadius = std::sqrt(point.lengthSquared());
        const double error = pointRadius - radius;
        vertexErrors[vertexIndex] = error;
        absoluteErrors[vertexIndex] = std::fabs(error);

        if (vertexIndex == 0)
        {
            statistics.minimumError = error;
            statistics.maximumError = error;
        }
        else
        {
            statistics.minimumError = (std::min)(statistics.minimumError, error);
            statistics.maximumError = (std::max)(statistics.maximumError, error);
        }

        errorSum += error;
        absoluteErrorSum += std::fabs(error);
        squaredErrorSum += error * error;

        if (pointRadius > (std::numeric_limits<double>::epsilon)())
        {
            MyMath::Vector3 normal = vertexNormal(mesh.vertices()[vertexIndex]);

            if (normal.normalize())
            {
                const MyMath::Vector3 radialNormal = point / pointRadius;
                const double cosine = (std::max)(-1.0, (std::min)(1.0, MyMath::Vector3::dot(normal, radialNormal)));
                const double angleDegrees = std::acos(cosine) * 180.0 / Pi;
                normalAngleSum += angleDegrees;
                statistics.maximumNormalAngleDegrees = (std::max)(statistics.maximumNormalAngleDegrees, angleDegrees);
                ++validNormalCount;
            }
        }
    }

    const double vertexCount = static_cast<double>(mesh.vertexCount());
    statistics.meanError = errorSum / vertexCount;
    statistics.meanAbsoluteError = absoluteErrorSum / vertexCount;
    statistics.rootMeanSquareError = std::sqrt(squaredErrorSum / vertexCount);
    statistics.meanNormalAngleDegrees = validNormalCount > 0 ? normalAngleSum / static_cast<double>(validNormalCount) : 0.0;
    std::sort(absoluteErrors.begin(), absoluteErrors.end());
    const std::size_t percentileIndex = (std::min)(absoluteErrors.size() - 1, static_cast<std::size_t>(std::ceil(static_cast<double>(absoluteErrors.size()) * 0.95)) - 1);
    statistics.percentile95AbsoluteError = absoluteErrors[percentileIndex];

    std::set<std::pair<std::uint32_t, std::uint32_t> > edges;
    const std::vector<std::uint32_t>& indices = mesh.indices();

    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangleCount(); ++triangleIndex)
    {
        const std::size_t offset = triangleIndex * 3;
        const std::uint32_t triangle[3] = { indices[offset], indices[offset + 1], indices[offset + 2] };

        for (unsigned int edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            std::uint32_t first = triangle[edgeIndex];
            std::uint32_t second = triangle[(edgeIndex + 1) % 3];

            if (first > second)
            {
                std::swap(first, second);
            }

            edges.insert(std::make_pair(first, second));
        }
    }

    double squaredEdgeDeltaSum = 0.0;

    for (std::set<std::pair<std::uint32_t, std::uint32_t> >::const_iterator iterator = edges.begin(); iterator != edges.end(); ++iterator)
    {
        const double delta = std::fabs(vertexErrors[iterator->first] - vertexErrors[iterator->second]);
        squaredEdgeDeltaSum += delta * delta;
        statistics.maximumEdgeDelta = (std::max)(statistics.maximumEdgeDelta, delta);
    }

    statistics.edgeRootMeanSquareDelta = edges.empty() ? 0.0 : std::sqrt(squaredEdgeDeltaSum / static_cast<double>(edges.size()));
    return statistics;
}

// 使用原始径向误差生成误差热力图，并按指定倍率放大几何径向误差。
MyVoxel::Geometry::Mesh makeSphereErrorMesh(const MyVoxel::Geometry::Mesh& source,
                                            const std::vector<double>& vertexErrors,
                                            double radius,
                                            double exaggeration,
                                            double colorRange)
{
    assert(source.isRenderable());
    assert(vertexErrors.size() == source.vertexCount());
    assert(radius > 0.0);
    assert(exaggeration >= 0.0);
    assert(colorRange > 0.0);

    MyVoxel::Geometry::Mesh result;
    result.reserve(source.vertexCount(), source.indexCount());

    for (std::size_t vertexIndex = 0; vertexIndex < source.vertexCount(); ++vertexIndex)
    {
        const MyMath::Vector3 sourcePoint = vertexPosition(source.vertices()[vertexIndex]);
        MyMath::Vector3 radialDirection = sourcePoint;

        if (!radialDirection.normalize())
        {
            radialDirection = MyMath::Vector3::unitZ();
        }

        const MyMath::Vector3 point = radialDirection * (radius + vertexErrors[vertexIndex] * exaggeration);
        result.appendVertex(makeVertex(point, radialDirection));
    }

    const std::vector<std::uint32_t>& indices = source.indices();

    for (std::size_t triangleIndex = 0; triangleIndex < source.triangleCount(); ++triangleIndex)
    {
        const std::size_t offset = triangleIndex * 3;
        const std::uint32_t index0 = indices[offset];
        const std::uint32_t index1 = indices[offset + 1];
        const std::uint32_t index2 = indices[offset + 2];
        const double error = (vertexErrors[index0] + vertexErrors[index1] + vertexErrors[index2]) / 3.0;
        result.appendTriangle(index0, index1, index2, radialErrorColor(error, colorRange));
    }

    return makeAreaWeightedNormalMesh(result);
}

// 输出一个球面重建网格的完整误差报告。
void printSphereErrorStatistics(const QString& name,
                                MyVoxel::VoxelLevel level,
                                double minimumVoxelEdgeLength,
                                const MyVoxel::Geometry::Mesh& mesh,
                                const SphereErrorStatistics& statistics)
{
    qDebug().noquote() << QStringLiteral("%1 | level=%2 | edge=%3 mm | vertices=%4 | triangles=%5")
        .arg(name)
        .arg(static_cast<unsigned int>(level))
        .arg(minimumVoxelEdgeLength, 0, 'f', 6)
        .arg(static_cast<qulonglong>(mesh.vertexCount()))
        .arg(static_cast<qulonglong>(mesh.triangleCount()));
    qDebug().noquote() << QStringLiteral("  radial error mm: min=%1 | max=%2 | mean=%3 | MAE=%4 | RMS=%5 | P95=%6 | peak-to-peak=%7")
        .arg(statistics.minimumError, 0, 'f', 9)
        .arg(statistics.maximumError, 0, 'f', 9)
        .arg(statistics.meanError, 0, 'f', 9)
        .arg(statistics.meanAbsoluteError, 0, 'f', 9)
        .arg(statistics.rootMeanSquareError, 0, 'f', 9)
        .arg(statistics.percentile95AbsoluteError, 0, 'f', 9)
        .arg(statistics.maximumError - statistics.minimumError, 0, 'f', 9);
    qDebug().noquote() << QStringLiteral("  local continuity mm: edge RMS delta=%1 | edge max delta=%2")
        .arg(statistics.edgeRootMeanSquareDelta, 0, 'f', 9)
        .arg(statistics.maximumEdgeDelta, 0, 'f', 9);
    qDebug().noquote() << QStringLiteral("  normal deviation deg: mean=%1 | max=%2")
        .arg(statistics.meanNormalAngleDegrees, 0, 'f', 6)
        .arg(statistics.maximumNormalAngleDegrees, 0, 'f', 6);
    qDebug().noquote() << QStringLiteral("  normalized by voxel edge: MAE=%1 | RMS=%2 | edge RMS delta=%3")
        .arg(statistics.meanAbsoluteError / minimumVoxelEdgeLength, 0, 'f', 9)
        .arg(statistics.rootMeanSquareError / minimumVoxelEdgeLength, 0, 'f', 9)
        .arg(statistics.edgeRootMeanSquareDelta / minimumVoxelEdgeLength, 0, 'f', 9);
}

// 创建只包含X方向平移的模型矩阵。
QMatrix4x4 translationMatrix(float x)
{
    QMatrix4x4 matrix;
    matrix.setToIdentity();
    matrix.translate(x, 0.0f, 0.0f);
    return matrix;
}

// 将字节数量转换为OpenGL使用的有符号整数。
int checkedByteSize(std::size_t byteSize)
{
    assert(byteSize <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
    return static_cast<int>(byteSize);
}

// 将浮点值限制在指定闭区间。
float clampFloat(float value, float minimum, float maximum)
{
    return (std::max)(minimum, (std::min)(maximum, value));
}

// 展开网格为OpenGL逐三角形位置、顶点法线和颜色数组。
void buildVertexData(const MyVoxel::Geometry::Mesh& mesh, std::vector<float>& data)
{
    const std::size_t FloatCountPerVertex = 10; // 每个顶点保存位置3、法线3和颜色4个float。
    data.resize(mesh.indexCount() * FloatCountPerVertex);
    std::size_t output = 0;

    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangleCount(); ++triangleIndex)
    {
        const MyVoxel::Geometry::MeshColor& color = mesh.triangleColor(triangleIndex);
        const std::size_t indexOffset = triangleIndex * 3;

        for (unsigned int corner = 0; corner < 3; ++corner)
        {
            const MyVoxel::Geometry::MeshVertex& vertex = mesh.vertices()[mesh.indices()[indexOffset + corner]];
            data[output++] = vertex.x;
            data[output++] = vertex.y;
            data[output++] = vertex.z;
            data[output++] = vertex.normalX;
            data[output++] = vertex.normalY;
            data[output++] = vertex.normalZ;
            data[output++] = color.red;
            data[output++] = color.green;
            data[output++] = color.blue;
            data[output++] = color.alpha;
        }
    }

    assert(output == data.size());
}

// 将局部包围盒八个角点变换到世界空间并扩展场景范围。
void expandBounds(const MyVoxel::Bounds3& bounds, const QMatrix4x4& model, bool& hasPoint, QVector3D& minimum, QVector3D& maximum)
{
    if (!bounds.isValid())
    {
        return;
    }

    const MyMath::Vector3& localMinimum = bounds.minimum();
    const MyMath::Vector3& localMaximum = bounds.maximum();
    const float x[2] = { static_cast<float>(localMinimum.x()), static_cast<float>(localMaximum.x()) };
    const float y[2] = { static_cast<float>(localMinimum.y()), static_cast<float>(localMaximum.y()) };
    const float z[2] = { static_cast<float>(localMinimum.z()), static_cast<float>(localMaximum.z()) };

    for (unsigned int ix = 0; ix < 2; ++ix)
    {
        for (unsigned int iy = 0; iy < 2; ++iy)
        {
            for (unsigned int iz = 0; iz < 2; ++iz)
            {
                const QVector3D point = model.map(QVector3D(x[ix], y[iy], z[iz]));

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

class DiagnosticOpenGLWidget : public QOpenGLWidget
{
public:
    explicit DiagnosticOpenGLWidget(QWidget* parent = nullptr)
        : QOpenGLWidget(parent)
    {
        QSurfaceFormat format;
        format.setVersion(3, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
        format.setDepthBufferSize(24);
        format.setStencilBufferSize(8);
        format.setSamples(4);
        setFormat(format);
        setFocusPolicy(Qt::StrongFocus);
        setMinimumSize(900, 560);
    }

    ~DiagnosticOpenGLWidget() override
    {
        if (!context())
        {
            return;
        }

        makeCurrent();

        for (std::size_t index = 0; index < m_objects.size(); ++index)
        {
            destroyObjectResources(*m_objects[index]);
        }

        doneCurrent();
    }

    // 添加一个诊断网格对象并返回对象索引。
    std::size_t addMesh(const MyVoxel::Geometry::Mesh& mesh, const QMatrix4x4& model)
    {
        assert(mesh.isRenderable());
        std::unique_ptr<RenderObject> object(new RenderObject(mesh, model));
        m_objects.push_back(std::move(object));
        updateSceneBounds();
        update();
        return m_objects.size() - 1;
    }

    // 设置指定对象是否参与显示。
    void setObjectVisible(std::size_t objectIndex, bool visible)
    {
        assert(objectIndex < m_objects.size());
        m_objects[objectIndex]->visible = visible;
        updateSceneBounds();
        update();
    }

    // 设置当前诊断渲染模式。
    void setRenderMode(DiagnosticRenderMode mode)
    {
        m_renderMode = mode;
        update();
    }

    // 设置是否以线框方式绘制。
    void setWireframe(bool enabled)
    {
        m_wireframe = enabled;
        update();
    }

    // 设置是否启用多重采样。
    void setMultisampling(bool enabled)
    {
        m_multisampling = enabled;
        update();
    }

    // 切换到等轴测观察方向并适应全部对象。
    void setIsometricView()
    {
        m_yaw = 45.0f;
        m_pitch = 35.264f;
        fitAll();
    }

    // 切换到前视方向并适应全部对象。
    void setFrontView()
    {
        m_yaw = 90.0f;
        m_pitch = 0.0f;
        fitAll();
    }

    // 切换到顶视方向并适应全部对象。
    void setTopView()
    {
        m_yaw = 90.0f;
        m_pitch = 89.0f;
        fitAll();
    }

    // 切换到右视方向并适应全部对象。
    void setRightView()
    {
        m_yaw = 0.0f;
        m_pitch = 0.0f;
        fitAll();
    }

    // 重新适应全部可见对象。
    void fitAll()
    {
        updateSceneBounds();
        m_viewOffset = QVector3D(0.0f, 0.0f, 0.0f);
        m_cameraScale = 2.35f;
        update();
    }

protected:
    void initializeGL() override
    {
        m_functions = context()->versionFunctions<QOpenGLFunctions_3_3_Core>();
        assert(m_functions);
        m_functions->initializeOpenGLFunctions();
        m_functions->glEnable(GL_DEPTH_TEST);
        m_functions->glDepthFunc(GL_LEQUAL);
        m_functions->glEnable(GL_MULTISAMPLE);
        m_functions->glDisable(GL_CULL_FACE);
        m_functions->glClearColor(0.055f, 0.060f, 0.070f, 1.0f);
        createShaderProgram();

        for (std::size_t index = 0; index < m_objects.size(); ++index)
        {
            uploadObject(*m_objects[index]);
        }
    }

    void resizeGL(int width, int height) override
    {
        m_functions->glViewport(0, 0, width, height);
    }

    void paintGL() override
    {
        if (m_multisampling)
        {
            m_functions->glEnable(GL_MULTISAMPLE);
        }
        else
        {
            m_functions->glDisable(GL_MULTISAMPLE);
        }

        m_functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (m_objects.empty())
        {
            return;
        }

        const QVector3D viewCenter = m_center + m_viewOffset;
        const float cameraDistance = (std::max)(m_radius * m_cameraScale, 0.1f);
        const QVector3D eye = viewCenter + cameraDirection() * cameraDistance;
        const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
        const float nearPlane = (std::max)(0.001f, cameraDistance - m_radius * 1.70f);
        const float farPlane = (std::max)(nearPlane + 1.0f, cameraDistance + m_radius * 3.50f);

        QMatrix4x4 projection;
        projection.perspective(45.0f, aspect, nearPlane, farPlane);
        QMatrix4x4 view;
        view.lookAt(eye, viewCenter, QVector3D(0.0f, 0.0f, 1.0f));

        m_program.bind();
        m_program.setUniformValue("u_cameraPosition", eye);
        m_program.setUniformValue("u_lightDirection", QVector3D(0.36f, -0.42f, 0.83f).normalized());
        m_program.setUniformValue("u_renderMode", static_cast<int>(m_renderMode));
        m_functions->glPolygonMode(GL_FRONT_AND_BACK, m_wireframe ? GL_LINE : GL_FILL);

        for (std::size_t index = 0; index < m_objects.size(); ++index)
        {
            RenderObject& object = *m_objects[index];

            if (!object.visible || object.vertexCount == 0)
            {
                continue;
            }

            m_program.setUniformValue("u_model", object.model);
            m_program.setUniformValue("u_mvp", projection * view * object.model);
            m_program.setUniformValue("u_normalMatrix", object.model.normalMatrix());
            object.vao.bind();
            m_functions->glDrawArrays(GL_TRIANGLES, 0, object.vertexCount);
            object.vao.release();
        }

        m_functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        m_program.release();
    }

    void keyPressEvent(QKeyEvent* event) override
    {
        switch (event->key())
        {
        case Qt::Key_F:
            fitAll();
            event->accept();
            return;
        case Qt::Key_1:
            setIsometricView();
            event->accept();
            return;
        case Qt::Key_2:
            setFrontView();
            event->accept();
            return;
        case Qt::Key_3:
            setTopView();
            event->accept();
            return;
        case Qt::Key_4:
            setRightView();
            event->accept();
            return;
        case Qt::Key_W:
            setWireframe(!m_wireframe);
            event->accept();
            return;
        default:
            break;
        }

        QOpenGLWidget::keyPressEvent(event);
    }

    void mousePressEvent(QMouseEvent* event) override
    {
        m_lastMousePosition = event->pos();
        setFocus();
        event->accept();
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        const QPoint delta = event->pos() - m_lastMousePosition;
        m_lastMousePosition = event->pos();

        if (event->buttons() & Qt::LeftButton)
        {
            m_yaw -= static_cast<float>(delta.x()) * 0.35f;
            m_pitch += static_cast<float>(delta.y()) * 0.35f;
            m_pitch = clampFloat(m_pitch, -89.0f, 89.0f);
            update();
        }
        else if (event->buttons() & Qt::RightButton)
        {
            const float cameraDistance = (std::max)(m_radius * m_cameraScale, 0.1f);
            const float moveScale = cameraDistance * 0.0015f;
            m_viewOffset -= cameraRight() * static_cast<float>(delta.x()) * moveScale;
            m_viewOffset += cameraUp() * static_cast<float>(delta.y()) * moveScale;
            update();
        }

        event->accept();
    }

    void mouseDoubleClickEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            fitAll();
            event->accept();
            return;
        }

        QOpenGLWidget::mouseDoubleClickEvent(event);
    }

    void wheelEvent(QWheelEvent* event) override
    {
        if (event->angleDelta().y() > 0)
        {
            m_cameraScale *= 0.90f;
        }
        else if (event->angleDelta().y() < 0)
        {
            m_cameraScale *= 1.10f;
        }

        m_cameraScale = clampFloat(m_cameraScale, MinimumCameraScale, MaximumCameraScale);
        update();
        event->accept();
    }

private:
    struct RenderObject
    {
        RenderObject(const MyVoxel::Geometry::Mesh& meshValue, const QMatrix4x4& modelValue)
            : mesh(meshValue)
            , model(modelValue)
            , vbo(QOpenGLBuffer::VertexBuffer)
        {
        }

        MyVoxel::Geometry::Mesh mesh; // OpenGL初始化前保存的CPU网格。
        QMatrix4x4 model; // 对象局部空间到诊断世界空间的模型矩阵。
        bool visible = true; // 对象是否参与绘制和适应窗口。
        QOpenGLVertexArrayObject vao; // 当前对象顶点属性状态。
        QOpenGLBuffer vbo; // 当前对象展开后的逐三角形顶点缓冲区。
        int vertexCount = 0; // 当前对象上传到GPU的顶点数量。
    };

    // 创建支持五种诊断模式的单一网格Shader。
    void createShaderProgram()
    {
        const char* vertexShader =
            "#version 330 core\n"
            "layout(location = 0) in vec3 a_position;\n"
            "layout(location = 1) in vec3 a_normal;\n"
            "layout(location = 2) in vec4 a_color;\n"
            "uniform mat4 u_model;\n"
            "uniform mat4 u_mvp;\n"
            "uniform mat3 u_normalMatrix;\n"
            "out vec3 v_worldPosition;\n"
            "out vec3 v_smoothNormal;\n"
            "out vec4 v_color;\n"
            "void main()\n"
            "{\n"
            "    vec4 worldPosition = u_model * vec4(a_position, 1.0);\n"
            "    v_worldPosition = worldPosition.xyz;\n"
            "    v_smoothNormal = normalize(u_normalMatrix * a_normal);\n"
            "    v_color = a_color;\n"
            "    gl_Position = u_mvp * vec4(a_position, 1.0);\n"
            "}\n";

        const char* fragmentShader =
            "#version 330 core\n"
            "in vec3 v_worldPosition;\n"
            "in vec3 v_smoothNormal;\n"
            "in vec4 v_color;\n"
            "uniform vec3 u_cameraPosition;\n"
            "uniform vec3 u_lightDirection;\n"
            "uniform int u_renderMode;\n"
            "out vec4 fragColor;\n"
            "vec3 orientedNormal(vec3 normalValue)\n"
            "{\n"
            "    vec3 normal = normalize(normalValue);\n"
            "    return gl_FrontFacing ? normal : -normal;\n"
            "}\n"
            "vec3 diffuseColor(vec3 normal, vec3 baseColor)\n"
            "{\n"
            "    float diffuse = max(dot(normal, normalize(u_lightDirection)), 0.0);\n"
            "    return baseColor * (0.18 + 0.82 * diffuse);\n"
            "}\n"
            "void main()\n"
            "{\n"
            "    vec3 smoothNormal = orientedNormal(v_smoothNormal);\n"
            "    if (u_renderMode == 0)\n"
            "    {\n"
            "        fragColor = v_color;\n"
            "        return;\n"
            "    }\n"
            "    if (u_renderMode == 4)\n"
            "    {\n"
            "        fragColor = vec4(smoothNormal * 0.5 + 0.5, 1.0);\n"
            "        return;\n"
            "    }\n"
            "    vec3 normal = smoothNormal;\n"
            "    if (u_renderMode == 3)\n"
            "    {\n"
            "        normal = orientedNormal(cross(dFdx(v_worldPosition), dFdy(v_worldPosition)));\n"
            "    }\n"
            "    vec3 color = diffuseColor(normal, v_color.rgb);\n"
            "    if (u_renderMode == 2)\n"
            "    {\n"
            "        vec3 viewDirection = normalize(u_cameraPosition - v_worldPosition);\n"
            "        vec3 halfDirection = normalize(normalize(u_lightDirection) + viewDirection);\n"
            "        float specular = pow(max(dot(normal, halfDirection), 0.0), 48.0);\n"
            "        color += vec3(0.38) * specular;\n"
            "    }\n"
            "    fragColor = vec4(color, v_color.a);\n"
            "}\n";

        if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader) ||
            !m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader) ||
            !m_program.link())
        {
            qFatal("Diagnostic OpenGL shader creation failed: %s", qPrintable(m_program.log()));
        }
    }

    // 上传一个CPU网格到独立VAO和VBO。
    void uploadObject(RenderObject& object)
    {
        assert(object.mesh.isRenderable());
        std::vector<float> data;
        buildVertexData(object.mesh, data);
        object.vao.create();
        object.vbo.create();
        object.vao.bind();
        object.vbo.bind();
        object.vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
        object.vbo.allocate(data.empty() ? nullptr : &data[0], checkedByteSize(data.size() * sizeof(float)));
        m_program.bind();
        const int stride = static_cast<int>(10 * sizeof(float));
        m_program.enableAttributeArray(0);
        m_program.setAttributeBuffer(0, GL_FLOAT, 0, 3, stride);
        m_program.enableAttributeArray(1);
        m_program.setAttributeBuffer(1, GL_FLOAT, static_cast<int>(3 * sizeof(float)), 3, stride);
        m_program.enableAttributeArray(2);
        m_program.setAttributeBuffer(2, GL_FLOAT, static_cast<int>(6 * sizeof(float)), 4, stride);
        m_program.release();
        object.vbo.release();
        object.vao.release();
        assert(object.mesh.indexCount() <= static_cast<std::size_t>((std::numeric_limits<int>::max)()));
        object.vertexCount = static_cast<int>(object.mesh.indexCount());
    }

    // 删除指定对象的GPU资源。
    void destroyObjectResources(RenderObject& object)
    {
        if (object.vbo.isCreated())
        {
            object.vbo.destroy();
        }

        if (object.vao.isCreated())
        {
            object.vao.destroy();
        }

        object.vertexCount = 0;
    }

    // 根据全部可见对象更新观察中心和包围球半径。
    void updateSceneBounds()
    {
        bool hasPoint = false;
        QVector3D minimum;
        QVector3D maximum;

        for (std::size_t index = 0; index < m_objects.size(); ++index)
        {
            const RenderObject& object = *m_objects[index];

            if (object.visible)
            {
                expandBounds(object.mesh.localBounds(), object.model, hasPoint, minimum, maximum);
            }
        }

        if (!hasPoint)
        {
            m_center = QVector3D(0.0f, 0.0f, 0.0f);
            m_radius = 1.0f;
            return;
        }

        m_center = (minimum + maximum) * 0.5f;
        m_radius = (std::max)((maximum - minimum).length() * 0.5f, MinimumSceneRadius);
    }

    // 返回当前由观察中心指向相机的单位方向。
    QVector3D cameraDirection() const
    {
        const float yaw = m_yaw * static_cast<float>(Pi / 180.0);
        const float pitch = m_pitch * static_cast<float>(Pi / 180.0);
        const float cosPitch = std::cos(pitch);
        return QVector3D(cosPitch * std::cos(yaw), cosPitch * std::sin(yaw), std::sin(pitch)).normalized();
    }

    // 返回当前屏幕水平方向。
    QVector3D cameraRight() const
    {
        return QVector3D::crossProduct(cameraDirection(), QVector3D(0.0f, 0.0f, 1.0f)).normalized();
    }

    // 返回当前屏幕竖直方向。
    QVector3D cameraUp() const
    {
        return QVector3D::crossProduct(cameraRight(), cameraDirection()).normalized();
    }

private:
    QOpenGLFunctions_3_3_Core* m_functions = nullptr; // 当前OpenGL 3.3 Core函数表。
    QOpenGLShaderProgram m_program; // 五种诊断模式共用的网格Shader。
    std::vector<std::unique_ptr<RenderObject> > m_objects; // 当前全部诊断网格对象。
    DiagnosticRenderMode m_renderMode = DiagnosticRenderMode::Unlit; // 默认关闭光照，直接观察径向误差热力图。
    bool m_wireframe = false; // 是否使用OpenGL线框模式。
    bool m_multisampling = true; // 是否启用多重采样抗锯齿。
    QVector3D m_center = QVector3D(0.0f, 0.0f, 0.0f); // 当前可见对象世界包围中心。
    QVector3D m_viewOffset = QVector3D(0.0f, 0.0f, 0.0f); // 用户平移产生的观察中心偏移。
    float m_radius = 1.0f; // 当前可见对象世界包围球半径。
    float m_cameraScale = 2.35f; // 相机距离相对包围球半径的比例。
    float m_yaw = 45.0f; // 相机绕世界Z轴的方位角，单位为度。
    float m_pitch = 35.264f; // 相机俯仰角，单位为度。
    QPoint m_lastMousePosition; // 上一次鼠标位置。
};

class SphereDistanceAbDiagnosticWindow : public QMainWindow
{
public:
    explicit SphereDistanceAbDiagnosticWindow(MyVoxel::VoxelLevel maximumLevel, QWidget* parent = nullptr)
        : QMainWindow(parent)
        , m_viewer(new DiagnosticOpenGLWidget(this))
        , m_maximumLevel(maximumLevel)
    {
        setWindowTitle(QStringLiteral("MyVoxel Sphere Distance Field A/B Diagnostic - Level %1").arg(static_cast<unsigned int>(maximumLevel)));
        setCentralWidget(m_viewer);
        resize(1500, 760);
        m_ready = initializeScene();
    }

    // 判断场景生成和诊断对象创建是否成功。
    bool isReady() const
    {
        return m_ready;
    }

private:
    // 对比当前Builder距离与解析球直接采样距离在同一VolumeMesher中的网格结果。
    bool initializeScene()
    {
        try
        {
            const MyVoxel::Geometry::Shape sphere = MyVoxel::Modeling::makeSphere(SphereRadius);
            MyVoxel::VoxelShape builderShape = MyVoxel::Modeling::voxelize(sphere, BaseVoxelEdgeLength, m_maximumLevel);

            if (!builderShape.isValid() || builderShape.isEmpty())
            {
                qCritical() << "Sphere voxelization produced an invalid or empty VoxelShape.";
                return false;
            }

            MyVoxel::VolumeFieldBuildOptions fieldOptions;
            fieldOptions.exteriorBandWidth = BandWidth;
            fieldOptions.interiorBandWidth = BandWidth;
            MyVoxel::VolumeFieldBuilder::build(builderShape, fieldOptions);
            const MyVoxel::VolumeFieldView builderView(builderShape);

            if (!builderView.isCurrent())
            {
                qCritical() << "VolumeFieldBuilder did not produce a current distance field.";
                return false;
            }

            MyVoxel::VoxelShape analyticShape = builderShape;

            if (!buildAnalyticSphereField(analyticShape, builderView.field(), SphereRadius))
            {
                qCritical() << "Analytic sphere distance field construction failed.";
                return false;
            }

            const MyVoxel::VolumeFieldView analyticView(analyticShape);

            if (!analyticView.isCurrent())
            {
                qCritical() << "Analytic sphere distance field is not current.";
                return false;
            }

            MyVoxel::Meshing::VolumeMeshingOptions standardOptions;
            standardOptions.color = DiagnosticColor;
            standardOptions.mode = MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets;
            MyVoxel::Meshing::VolumeMeshingOptions featureOptions = standardOptions;
            featureOptions.mode = MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets;

            const MyVoxel::Geometry::Mesh builderStandardMesh = MyVoxel::Meshing::VolumeMesher::build(builderView, standardOptions);
            const MyVoxel::Geometry::Mesh analyticStandardMesh = MyVoxel::Meshing::VolumeMesher::build(analyticView, standardOptions);
            const MyVoxel::Geometry::Mesh builderFeatureMesh = MyVoxel::Meshing::VolumeMesher::build(builderView, featureOptions);
            const MyVoxel::Geometry::Mesh analyticFeatureMesh = MyVoxel::Meshing::VolumeMesher::build(analyticView, featureOptions);
            const MyVoxel::Geometry::Mesh referenceMesh = makeReferenceSphere(SphereRadius, ReferenceSliceCount, ReferenceStackCount);

            if (!referenceMesh.isRenderable() || !builderStandardMesh.isRenderable() || !analyticStandardMesh.isRenderable() ||
                !builderFeatureMesh.isRenderable() || !analyticFeatureMesh.isRenderable())
            {
                qCritical() << "At least one sphere A/B diagnostic mesh is not renderable.";
                return false;
            }

            std::vector<double> builderStandardErrors;
            std::vector<double> analyticStandardErrors;
            std::vector<double> builderFeatureErrors;
            std::vector<double> analyticFeatureErrors;
            const SphereErrorStatistics builderStandardStatistics = calculateSphereErrorStatistics(builderStandardMesh, SphereRadius, builderStandardErrors);
            const SphereErrorStatistics analyticStandardStatistics = calculateSphereErrorStatistics(analyticStandardMesh, SphereRadius, analyticStandardErrors);
            const SphereErrorStatistics builderFeatureStatistics = calculateSphereErrorStatistics(builderFeatureMesh, SphereRadius, builderFeatureErrors);
            const SphereErrorStatistics analyticFeatureStatistics = calculateSphereErrorStatistics(analyticFeatureMesh, SphereRadius, analyticFeatureErrors);
            const FieldSampleErrorStatistics fieldStatistics = calculateFieldSampleErrorStatistics(builderView, SphereRadius);
            const double minimumVoxelEdgeLength = builderShape.grid().minimumCellEdgeLength();

            const MyVoxel::Geometry::Mesh builderStandardErrorMesh = makeSphereErrorMesh(builderStandardMesh, builderStandardErrors, SphereRadius, ErrorExaggeration, minimumVoxelEdgeLength);
            const MyVoxel::Geometry::Mesh analyticStandardErrorMesh = makeSphereErrorMesh(analyticStandardMesh, analyticStandardErrors, SphereRadius, ErrorExaggeration, minimumVoxelEdgeLength);
            const MyVoxel::Geometry::Mesh builderFeatureErrorMesh = makeSphereErrorMesh(builderFeatureMesh, builderFeatureErrors, SphereRadius, ErrorExaggeration, minimumVoxelEdgeLength);
            const MyVoxel::Geometry::Mesh analyticFeatureErrorMesh = makeSphereErrorMesh(analyticFeatureMesh, analyticFeatureErrors, SphereRadius, ErrorExaggeration, minimumVoxelEdgeLength);

            m_referenceObject = m_viewer->addMesh(referenceMesh, translationMatrix(static_cast<float>(-2.0 * DisplaySpacing)));
            m_builderStandardObject = m_viewer->addMesh(builderStandardErrorMesh, translationMatrix(static_cast<float>(-DisplaySpacing)));
            m_analyticStandardObject = m_viewer->addMesh(analyticStandardErrorMesh, translationMatrix(0.0f));
            m_builderFeatureObject = m_viewer->addMesh(builderFeatureErrorMesh, translationMatrix(static_cast<float>(DisplaySpacing)));
            m_analyticFeatureObject = m_viewer->addMesh(analyticFeatureErrorMesh, translationMatrix(static_cast<float>(2.0 * DisplaySpacing)));
            createToolBars();
            m_viewer->setRenderMode(DiagnosticRenderMode::Unlit);
            m_viewer->setIsometricView();
            updateStatus(QStringLiteral("10倍径向误差热力图"));

            qDebug().noquote() << QStringLiteral("Sphere distance A/B | radius=%1 mm | heat range=+/-%2 mm | geometry=%3x")
                .arg(SphereRadius, 0, 'f', 3)
                .arg(minimumVoxelEdgeLength, 0, 'f', 6)
                .arg(ErrorExaggeration, 0, 'f', 1);
            printFieldSampleErrorStatistics(m_maximumLevel, minimumVoxelEdgeLength, fieldStatistics);
            printSphereErrorStatistics(QStringLiteral("Builder + Standard Surface Nets"), m_maximumLevel, minimumVoxelEdgeLength, builderStandardMesh, builderStandardStatistics);
            printSphereErrorStatistics(QStringLiteral("Analytic SDF + Standard Surface Nets"), m_maximumLevel, minimumVoxelEdgeLength, analyticStandardMesh, analyticStandardStatistics);
            printSphereErrorStatistics(QStringLiteral("Builder + Feature-sensitive QEF"), m_maximumLevel, minimumVoxelEdgeLength, builderFeatureMesh, builderFeatureStatistics);
            printSphereErrorStatistics(QStringLiteral("Analytic SDF + Feature-sensitive QEF"), m_maximumLevel, minimumVoxelEdgeLength, analyticFeatureMesh, analyticFeatureStatistics);
            qDebug().noquote() << QStringLiteral("CSV_AB,%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14")
                .arg(static_cast<unsigned int>(m_maximumLevel))
                .arg(minimumVoxelEdgeLength, 0, 'f', 9)
                .arg(fieldStatistics.nearSurfaceMeanAbsoluteError, 0, 'f', 9)
                .arg(fieldStatistics.nearSurfaceRootMeanSquareError, 0, 'f', 9)
                .arg(builderStandardStatistics.meanAbsoluteError, 0, 'f', 9)
                .arg(builderStandardStatistics.rootMeanSquareError, 0, 'f', 9)
                .arg(builderStandardStatistics.edgeRootMeanSquareDelta, 0, 'f', 9)
                .arg(analyticStandardStatistics.meanAbsoluteError, 0, 'f', 9)
                .arg(analyticStandardStatistics.rootMeanSquareError, 0, 'f', 9)
                .arg(analyticStandardStatistics.edgeRootMeanSquareDelta, 0, 'f', 9)
                .arg(builderFeatureStatistics.meanAbsoluteError, 0, 'f', 9)
                .arg(builderFeatureStatistics.edgeRootMeanSquareDelta, 0, 'f', 9)
                .arg(analyticFeatureStatistics.meanAbsoluteError, 0, 'f', 9)
                .arg(analyticFeatureStatistics.edgeRootMeanSquareDelta, 0, 'f', 9);
            return true;
        }
        catch (const std::exception& exception)
        {
            qCritical() << "Sphere distance A/B diagnostic failed:" << exception.what();
        }
        catch (...)
        {
            qCritical() << "Sphere distance A/B diagnostic failed with an unknown exception.";
        }

        statusBar()->showMessage(QStringLiteral("Sphere distance A/B diagnostic initialization failed."));
        return false;
    }

    // 创建对象可见性、渲染模式和抗混叠控制工具栏。
    void createToolBars()
    {
        QToolBar* objectToolBar = addToolBar(QStringLiteral("Objects"));
        objectToolBar->setMovable(false);
        addVisibilityAction(objectToolBar, QStringLiteral("解析球"), m_referenceObject);
        addVisibilityAction(objectToolBar, QStringLiteral("Builder标准"), m_builderStandardObject);
        addVisibilityAction(objectToolBar, QStringLiteral("解析SDF标准"), m_analyticStandardObject);
        addVisibilityAction(objectToolBar, QStringLiteral("Builder QEF"), m_builderFeatureObject);
        addVisibilityAction(objectToolBar, QStringLiteral("解析SDF QEF"), m_analyticFeatureObject);

        QToolBar* renderToolBar = addToolBar(QStringLiteral("Rendering"));
        renderToolBar->setMovable(false);
        QActionGroup* renderGroup = new QActionGroup(this);
        renderGroup->setExclusive(true);
        QAction* unlitAction = addRenderModeAction(renderToolBar, renderGroup, QStringLiteral("误差色"), DiagnosticRenderMode::Unlit, QStringLiteral("10倍径向误差热力图"));
        addRenderModeAction(renderToolBar, renderGroup, QStringLiteral("漫反射"), DiagnosticRenderMode::SmoothDiffuse, QStringLiteral("面积加权网格法线，仅漫反射"));
        addRenderModeAction(renderToolBar, renderGroup, QStringLiteral("高光"), DiagnosticRenderMode::SmoothSpecular, QStringLiteral("面积加权网格法线，漫反射和镜面高光"));
        addRenderModeAction(renderToolBar, renderGroup, QStringLiteral("平面法线"), DiagnosticRenderMode::FlatDiffuse, QStringLiteral("每个三角形使用独立平面法线"));
        addRenderModeAction(renderToolBar, renderGroup, QStringLiteral("法线RGB"), DiagnosticRenderMode::NormalRgb, QStringLiteral("将面积加权网格法线直接映射到RGB"));
        unlitAction->setChecked(true);
        renderToolBar->addSeparator();

        QAction* wireframeAction = renderToolBar->addAction(QStringLiteral("线框"));
        wireframeAction->setCheckable(true);
        connect(wireframeAction, &QAction::toggled, this, [this](bool enabled)
        {
            m_viewer->setWireframe(enabled);
        });

        QAction* msaaAction = renderToolBar->addAction(QStringLiteral("MSAA"));
        msaaAction->setCheckable(true);
        msaaAction->setChecked(true);
        connect(msaaAction, &QAction::toggled, this, [this](bool enabled)
        {
            m_viewer->setMultisampling(enabled);
        });
    }

    // 添加一个默认选中的对象可见性动作。
    void addVisibilityAction(QToolBar* toolBar, const QString& text, std::size_t objectIndex)
    {
        QAction* action = toolBar->addAction(text);
        action->setCheckable(true);
        action->setChecked(true);
        connect(action, &QAction::toggled, this, [this, objectIndex](bool visible)
        {
            m_viewer->setObjectVisible(objectIndex, visible);
            m_viewer->fitAll();
        });
    }

    // 添加一个互斥诊断渲染模式动作并返回动作指针。
    QAction* addRenderModeAction(QToolBar* toolBar, QActionGroup* group, const QString& text,
                                 DiagnosticRenderMode mode, const QString& statusText)
    {
        QAction* action = toolBar->addAction(text);
        action->setCheckable(true);
        group->addAction(action);
        connect(action, &QAction::triggered, this, [this, mode, statusText]()
        {
            m_viewer->setRenderMode(mode);
            updateStatus(statusText);
        });
        return action;
    }

    // 更新状态栏中的对象顺序、层级和当前模式。
    void updateStatus(const QString& modeText)
    {
        statusBar()->showMessage(QStringLiteral("左至右：解析球 | Builder标准 | 解析SDF标准 | Builder QEF | 解析SDF QEF | 全部显示10倍径向误差 | Level %1 | %2")
                                 .arg(static_cast<unsigned int>(m_maximumLevel)).arg(modeText));
    }

private:
    DiagnosticOpenGLWidget* m_viewer = nullptr; // 独立诊断OpenGL窗口。
    MyVoxel::VoxelLevel m_maximumLevel; // 当前体素最高细分层级。
    std::size_t m_referenceObject = 0; // 解析参考球对象索引。
    std::size_t m_builderStandardObject = 0; // 当前Builder距离经过标准Surface Nets后的十倍误差对象。
    std::size_t m_analyticStandardObject = 0; // 解析球距离经过标准Surface Nets后的十倍误差对象。
    std::size_t m_builderFeatureObject = 0; // 当前Builder距离经过QEF后的十倍误差对象。
    std::size_t m_analyticFeatureObject = 0; // 解析球距离经过QEF后的十倍误差对象。
    bool m_ready = false; // 场景是否已经完整创建。
};

// 从命令行读取体素最高层级，未指定时使用第8层。
MyVoxel::VoxelLevel parseMaximumLevel(int argc, char* argv[])
{
    int level = 8;

    if (argc > 1)
    {
        bool converted = false;
        const int requestedLevel = QString::fromLocal8Bit(argv[1]).toInt(&converted);

        if (converted && requestedLevel >= 2 && requestedLevel <= 10)
        {
            level = requestedLevel;
        }
        else
        {
            qWarning() << "Maximum voxel level must be in range [2, 10]; using level 8.";
        }
    }

    return static_cast<MyVoxel::VoxelLevel>(level);
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    SphereDistanceAbDiagnosticWindow window(parseMaximumLevel(argc, argv));

    if (!window.isReady())
    {
        return EXIT_FAILURE;
    }

    window.show();
    return application.exec();
}