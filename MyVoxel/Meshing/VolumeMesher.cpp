#include "VolumeMesher.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/Curve/Geometry_Arc.h"
#include "MyVoxel/Geometry/Curve/Geometry_Line.h"
#include "MyVoxel/Topology/Edge/Topology_Edge.h"
#include "MyVoxel/Topology/Vertex/Topology_Vertex.h"
#include "CellMeshingState.h"
#include "FeaturePointSolver.h"
#include "VolumeMeshingTopology.h"
#include "VolumeMeshingWorkspace.h"
#include "VoxelTsdfAccessor.h"

namespace
{

const double Pi = 3.14159265358979323846; // 特征角从度转换为弧度使用的圆周率。
const double TwoPi = Pi * 2.0; // 完整圆角度规范化使用的弧度值。
const double FeatureVertexOwnerOffsetScale = 0.5; // 将几何点映射到唯一Surface Nets单元时从采样中心格向负方向偏移半个采样间距。
const double FeatureQueryToleranceScale = 1.0e-9; // Feature与单元边界接触判断允许的相对采样间距容差。

// 返回添加整数偏移后的VoxelIndex分量并检查数值范围。
MyVoxel::VoxelIndex offsetComponent(MyVoxel::VoxelIndex value, int offset)
{
    const std::int64_t result = static_cast<std::int64_t>(value) + static_cast<std::int64_t>(offset);
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());
    MYVOXEL_ASSERT_MESSAGE(result >= minimum && result <= maximum, "VolumeMesher sample index exceeds VoxelIndex range.");
    return static_cast<MyVoxel::VoxelIndex>(result);
}

// 返回带整数偏移的最高层TSDF采样索引。
MyVoxel::VoxelCellIndex offsetIndex(const MyVoxel::VoxelCellIndex& index, int offsetX, int offsetY, int offsetZ)
{
    return MyVoxel::VoxelCellIndex(offsetComponent(index.x, offsetX), offsetComponent(index.y, offsetY), offsetComponent(index.z, offsetZ));
}

// 在六个方向扩展最高层有限采样范围。
MyVoxel::VoxelCellRange expandedRange(const MyVoxel::VoxelCellRange& range, int padding)
{
    MYVOXEL_ASSERT_MESSAGE(range.isValid(), "VolumeMesher range expansion requires a valid range.");
    MYVOXEL_ASSERT_MESSAGE(padding >= 0, "VolumeMesher range padding must be non-negative.");
    return MyVoxel::VoxelCellRange(
        MyVoxel::VoxelCellIndex(offsetComponent(range.minimum.x, -padding), offsetComponent(range.minimum.y, -padding),
                                offsetComponent(range.minimum.z, -padding)),
        MyVoxel::VoxelCellIndex(offsetComponent(range.maximum.x, padding), offsetComponent(range.maximum.y, padding),
                                offsetComponent(range.maximum.z, padding)),
        range.level);
}

// 返回一个Root在最高层采样索引中的单轴宽度。
std::int64_t rootSampleScale(const MyVoxel::VoxelGrid& grid)
{
    MYVOXEL_ASSERT_MESSAGE(grid.maximumLevel() >= MyVoxel::BaseVoxelLevel,
                           "VolumeMesher grid maximum level must not be below BaseVoxelLevel.");
    const unsigned int difference = static_cast<unsigned int>(grid.maximumLevel()) - static_cast<unsigned int>(MyVoxel::BaseVoxelLevel);
    MYVOXEL_ASSERT_MESSAGE(difference < 63U, "VolumeMesher root sample scale exceeds int64 range.");
    return static_cast<std::int64_t>(1ULL << difference);
}

// 将Root索引转换为其最高层最小采样索引。
MyVoxel::VoxelIndex rootMinimumSample(MyVoxel::VoxelIndex rootIndex, std::int64_t scale)
{
    const std::int64_t root = static_cast<std::int64_t>(rootIndex);

    if (root > 0)
    {
        MYVOXEL_ASSERT_MESSAGE(root <= (std::numeric_limits<std::int64_t>::max)() / scale,
                               "VolumeMesher positive root sample coordinate overflowed int64.");
    }
    else if (root < 0)
    {
        MYVOXEL_ASSERT_MESSAGE(root >= (std::numeric_limits<std::int64_t>::min)() / scale,
                               "VolumeMesher negative root sample coordinate overflowed int64.");
    }

    const std::int64_t result = root * scale;
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());
    MYVOXEL_ASSERT_MESSAGE(result >= minimum && result <= maximum, "VolumeMesher root minimum sample exceeds VoxelIndex range.");
    return static_cast<MyVoxel::VoxelIndex>(result);
}

// 返回Root最高层最大采样索引。
MyVoxel::VoxelIndex rootMaximumSample(MyVoxel::VoxelIndex rootIndex, std::int64_t scale)
{
    const std::int64_t minimum = static_cast<std::int64_t>(rootMinimumSample(rootIndex, scale));
    const std::int64_t result = minimum + scale - 1;
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());
    MYVOXEL_ASSERT_MESSAGE(result <= maximum, "VolumeMesher root maximum sample exceeds VoxelIndex range.");
    return static_cast<MyVoxel::VoxelIndex>(result);
}

// 判断标量值是否位于当前等值面内部，严格小于isoValue与OpenVDB符号规则一致。
bool isInside(float value, double isoValue)
{
    return static_cast<double>(value) < isoValue;
}

// 检查TSDF和网格化选项。
void validateShape(const MyVoxel::VoxelShape& shape, const MyVoxel::Meshing::VolumeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "VolumeMesher requires a valid VoxelShape.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.isoValue), "VolumeMesher isoValue must be finite.");
    MYVOXEL_ASSERT_MESSAGE(options.isoValue > -static_cast<double>(shape.backgroundDistance()) &&
                           options.isoValue < static_cast<double>(shape.backgroundDistance()),
                           "VolumeMesher isoValue must lie strictly inside (-B,+B).");
    MYVOXEL_ASSERT_MESSAGE(options.color.isValid(), "VolumeMesher requires a valid display color.");
    MYVOXEL_ASSERT_MESSAGE(options.mode == MyVoxel::Meshing::VolumeMeshingMode::StandardSurfaceNets ||
                           options.mode == MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets ||
                           options.mode == MyVoxel::Meshing::VolumeMeshingMode::FeatureConstrainedSurfaceNets,
                           "VolumeMesher encountered an invalid meshing mode.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.featureAngleDegrees) &&
                           options.featureAngleDegrees > 0.0 && options.featureAngleDegrees < 180.0,
                           "VolumeMesher feature angle must lie strictly inside (0,180) degrees.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.featureSnapDistanceScale) && options.featureSnapDistanceScale > 0.0,
                           "VolumeMesher explicit Feature snap-distance scale must be finite and positive.");
}

// 检查有限采样范围是否能够用于当前Shape最高层TSDF。
void validateSampleRange(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellRange& sampleRange)
{
    MYVOXEL_ASSERT_MESSAGE(sampleRange.isValid(), "VolumeMesher requires a valid finite sample range.");
    MYVOXEL_ASSERT_MESSAGE(sampleRange.level == shape.grid().maximumLevel(),
                           "VolumeMesher sample range must use the VoxelShape highest level.");
    MYVOXEL_ASSERT_MESSAGE(sampleRange.countX() >= 2 && sampleRange.countY() >= 2 && sampleRange.countZ() >= 2,
                           "VolumeMesher requires at least two TSDF samples in every direction.");
}

// 读取一个Surface Nets单元的八个OpenVDB顺序角点索引和值。
void gatherCellValues(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, const MyVoxel::VoxelCellIndex& cubeIndex,
                      std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerIndices,
                      std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerValues)
{
    for (unsigned int cornerIndex = 0; cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount; ++cornerIndex)
    {
        cornerIndices[cornerIndex] = offsetIndex(cubeIndex,
            MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetX(cornerIndex),
            MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetY(cornerIndex),
            MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetZ(cornerIndex));
        cornerValues[cornerIndex] = accessor.value(cornerIndices[cornerIndex]);
    }
}

// 独立读取八个角点中心位置。
void gatherCornerPositionsStandard(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor,
                                   const std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerIndices,
                                   std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerPoints)
{
    for (unsigned int cornerIndex = 0; cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount; ++cornerIndex)
    {
        cornerPoints[cornerIndex] = accessor.samplePosition(cornerIndices[cornerIndex]);
    }
}

// 从当前单元第0角点和固定最高层采样间距推导八个角点中心位置。
void gatherCornerPositionsFast(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, const MyVoxel::VoxelCellIndex& cubeIndex,
                               std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerPoints)
{
    const MyMath::Vector3 basePoint = accessor.samplePosition(cubeIndex);
    const double spacing = accessor.sampleSpacing();

    for (unsigned int cornerIndex = 0; cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount; ++cornerIndex)
    {
        cornerPoints[cornerIndex] = basePoint + MyMath::Vector3(
            static_cast<double>(MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetX(cornerIndex)) * spacing,
            static_cast<double>(MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetY(cornerIndex)) * spacing,
            static_cast<double>(MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetZ(cornerIndex)) * spacing);
    }
}

// 读取八个角点的中心差分TSDF梯度；当前先不引入梯度缓存，确保Feature模式只改变几何语义。
void gatherCornerGradients(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor,
                           const std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerIndices,
                           std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerGradients)
{
    for (unsigned int cornerIndex = 0; cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount; ++cornerIndex)
    {
        cornerGradients[cornerIndex] = accessor.gradient(cornerIndices[cornerIndex]);
    }
}

// 返回线性等值面交点参数并限制到闭区间[0,1]。
double isoInterpolationFactor(float value0, float value1, double isoValue)
{
    const double first = static_cast<double>(value0);
    const double second = static_cast<double>(value1);
    const double denominator = second - first;

    if (denominator == 0.0)
    {
        return 0.5;
    }

    const double factor = (isoValue - first) / denominator;
    return (std::max)(0.0, (std::min)(1.0, factor));
}

// 在线性TSDF采样边上返回指定等值面的交点。
MyMath::Vector3 interpolateIsoPoint(const MyMath::Vector3& point0, const MyMath::Vector3& point1,
                                    float value0, float value1, double isoValue)
{
    return point0 + (point1 - point0) * isoInterpolationFactor(value0, value1, isoValue);
}

// 在线性交点处插值两个角点梯度并归一化。
bool interpolateIsoNormal(const MyMath::Vector3& gradient0, const MyMath::Vector3& gradient1,
                          double factor, MyMath::Vector3& normal)
{
    normal = gradient0 + (gradient1 - gradient0) * factor;
    return normal.normalize();
}

// 返回八角点三线性插值函数在指定单元局部uvw处的解析梯度。
MyMath::Vector3 trilinearGradient(const std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& values,
                                 double u, double v, double w, double spacing)
{
    u = (std::max)(0.0, (std::min)(1.0, u));
    v = (std::max)(0.0, (std::min)(1.0, v));
    w = (std::max)(0.0, (std::min)(1.0, w));

    const double c0 = static_cast<double>(values[0]);
    const double c1 = static_cast<double>(values[1]);
    const double c2 = static_cast<double>(values[2]);
    const double c3 = static_cast<double>(values[3]);
    const double c4 = static_cast<double>(values[4]);
    const double c5 = static_cast<double>(values[5]);
    const double c6 = static_cast<double>(values[6]);
    const double c7 = static_cast<double>(values[7]);
    const double oneMinusU = 1.0 - u;
    const double oneMinusV = 1.0 - v;
    const double oneMinusW = 1.0 - w;

    const double derivativeU = oneMinusV * oneMinusW * (c1 - c0) + oneMinusV * w * (c2 - c3) +
                               v * oneMinusW * (c5 - c4) + v * w * (c6 - c7);
    const double derivativeV = oneMinusU * oneMinusW * (c4 - c0) + u * oneMinusW * (c5 - c1) +
                               u * w * (c6 - c2) + oneMinusU * w * (c7 - c3);
    const double derivativeW = oneMinusU * oneMinusV * (c3 - c0) + u * oneMinusV * (c2 - c1) +
                               u * v * (c6 - c5) + oneMinusU * v * (c7 - c4);
    const double inverseSpacing = 1.0 / spacing;
    return MyMath::Vector3(derivativeU * inverseSpacing, derivativeV * inverseSpacing, derivativeW * inverseSpacing);
}

// 返回八角点跨相对面的平均变化方向，作为病态三线性梯度的后备法线。
MyMath::Vector3 cubeGradient(const std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& values)
{
    const double gradientX = static_cast<double>(values[1] + values[2] + values[5] + values[6]) -
                             static_cast<double>(values[0] + values[3] + values[4] + values[7]);
    const double gradientY = static_cast<double>(values[4] + values[5] + values[6] + values[7]) -
                             static_cast<double>(values[0] + values[1] + values[2] + values[3]);
    const double gradientZ = static_cast<double>(values[2] + values[3] + values[6] + values[7]) -
                             static_cast<double>(values[0] + values[1] + values[4] + values[5]);
    MyMath::Vector3 normal(gradientX, gradientY, gradientZ);

    if (!normal.normalize())
    {
        normal = MyMath::Vector3::unitZ();
    }

    return normal;
}

// 返回最终单元顶点处的三线性TSDF梯度法线；Feature模式本阶段仍复用该法线，避免同时改变着色语义。
MyMath::Vector3 cellVertexNormal(const std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& values,
                                const std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& points,
                                const MyMath::Vector3& position, double spacing)
{
    const MyMath::Vector3& origin = points[0];
    const double inverseSpacing = 1.0 / spacing;
    MyMath::Vector3 normal = trilinearGradient(values,
        (position.x() - origin.x()) * inverseSpacing,
        (position.y() - origin.y()) * inverseSpacing,
        (position.z() - origin.z()) * inverseSpacing,
        spacing);

    if (!normal.normalize())
    {
        normal = cubeGradient(values);
    }

    return normal;
}

// 将QEF结果限制在当前Surface Nets单元内部，避免病态求解产生远距离尖峰。
MyMath::Vector3 clampPointToCell(const MyMath::Vector3& point, const MyMath::Vector3& minimum, const MyMath::Vector3& maximum)
{
    return MyMath::Vector3(
        (std::max)(minimum.x(), (std::min)(maximum.x(), point.x())),
        (std::max)(minimum.y(), (std::min)(maximum.y(), point.y())),
        (std::max)(minimum.z(), (std::min)(maximum.z(), point.z())));
}

// 判断同一边组零交叉法线中是否存在达到特征角阈值的方向变化。
bool hasSharpFeature(const std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::EdgeCount>& normals,
                     unsigned int normalCount, double minimumDot)
{
    if (normalCount < 2)
    {
        return false;
    }

    for (unsigned int first = 0; first + 1 < normalCount; ++first)
    {
        for (unsigned int second = first + 1; second < normalCount; ++second)
        {
            if (MyMath::Vector3::dot(normals[first], normals[second]) <= minimumDot)
            {
                return true;
            }
        }
    }

    return false;
}

// 将角度规范化到[0,2π)。
double normalizedAngle(double angle)
{
    double result = std::fmod(angle, TwoPi);

    if (result < 0.0)
    {
        result += TwoPi;
    }

    return result;
}

// 返回从起始角沿逆时针方向到目标角的非负角距离。
double positiveAngleDistance(double startAngle, double targetAngle)
{
    return normalizedAngle(targetAngle - startAngle);
}

// 判断两个VoxelCellIndex是否完全一致。
bool sameCellIndex(const MyVoxel::VoxelCellIndex& first, const MyVoxel::VoxelCellIndex& second)
{
    return first.x == second.x && first.y == second.y && first.z == second.z;
}

// 返回Topology_Edge在指定端点处指向远离该Vertex方向的单位切向量。
bool outwardEdgeTangent(const MyVoxel::Topology_Edge& edge, const MyVoxel::Topology_Vertex& vertex, MyMath::Vector3& tangent)
{
    const bool atStart = edge.startVertex().isSame(vertex);
    const bool atEnd = edge.endVertex().isSame(vertex);

    if (atStart && atEnd)
    {
        return false;
    }

    if (atStart)
    {
        tangent = edge.tangentAt(0.0);
        return tangent.normalize();
    }

    if (atEnd)
    {
        tangent = edge.tangentAt(1.0) * -1.0;
        return tangent.normalize();
    }

    return false;
}

// 判断Topology_Vertex是否代表需要锁定网格位置的真实几何尖角，而不是完整圆的平滑拓扑seam。
bool isGeometricFeatureVertex(const MyVoxel::VoxelFeatureSet& features, const MyVoxel::Topology_Vertex& vertex, double smoothMinimumDot)
{
    std::vector<MyVoxel::Topology_Edge> incidentEdges;
    features.incidentEdges(vertex, incidentEdges);

    if (incidentEdges.empty())
    {
        return true;
    }

    if (incidentEdges.size() == 1)
    {
        const MyVoxel::Topology_Edge& edge = incidentEdges[0];

        if (edge.startVertex().isSame(vertex) && edge.endVertex().isSame(vertex))
        {
            return false;
        }

        return true;
    }

    if (incidentEdges.size() >= 3)
    {
        return true;
    }

    MyMath::Vector3 firstTangent;
    MyMath::Vector3 secondTangent;

    if (!outwardEdgeTangent(incidentEdges[0], vertex, firstTangent) ||
        !outwardEdgeTangent(incidentEdges[1], vertex, secondTangent))
    {
        return true;
    }

    // 两条Edge在该点平滑延续时，两个“向外”切向应近似相反；偏离直线超过featureAngle则视为真实折角。
    return MyMath::Vector3::dot(firstTangent, secondTangent) > -smoothMinimumDot;
}

// 返回几何FeatureVertex唯一归属的Surface Nets单元，避免边界角点同时锁定多个相邻单元产生退化三角形。
MyVoxel::VoxelCellIndex featureVertexOwnerCell(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, const MyMath::Vector3& point)
{
    const double offset = accessor.sampleSpacing() * FeatureVertexOwnerOffsetScale;
    const MyMath::Vector3 shiftedPoint = point - MyMath::Vector3(offset, offset, offset);
    return accessor.grid().cellIndex(shiftedPoint, accessor.sampleLevel());
}

// 返回点在线段Topology_Edge上的最近点。
MyMath::Vector3 closestPointOnLineEdge(const MyVoxel::Topology_Edge& edge, const MyMath::Vector3& point)
{
    const MyMath::Vector3 start = edge.pointAt(0.0);
    const MyMath::Vector3 end = edge.pointAt(1.0);
    const MyMath::Vector3 direction = end - start;
    const double lengthSquared = direction.lengthSquared();

    if (lengthSquared <= 0.0)
    {
        return start;
    }

    const double parameter = MyMath::Vector3::dot(point - start, direction) / lengthSquared;
    const double clampedParameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return start + direction * clampedParameter;
}

// 返回点在Geometry_Arc有限圆弧上的最近点。
MyMath::Vector3 closestPointOnArcEdge(const MyVoxel::Topology_Edge& edge, const MyVoxel::Geometry_Arc& arc, const MyMath::Vector3& point)
{
    const MyMath::Vector3 relative = point - arc.center();
    const double localX = MyMath::Vector3::dot(relative, arc.xAxis());
    const double localY = MyMath::Vector3::dot(relative, arc.yAxis());

    if (localX == 0.0 && localY == 0.0)
    {
        const MyMath::Vector3 start = edge.pointAt(0.0);
        const MyMath::Vector3 end = edge.pointAt(1.0);
        return point.distanceTo(start) <= point.distanceTo(end) ? start : end;
    }

    const double angle = std::atan2(localY, localX);
    const double absoluteSweep = std::fabs(arc.sweepAngle());
    const double angleTolerance = (std::numeric_limits<double>::epsilon)() * 64.0; // 仅覆盖角度规范化和三角函数舍入误差。

    if (absoluteSweep >= TwoPi - angleTolerance || arc.containsAngle(angle, angleTolerance))
    {
        return arc.center() + arc.xAxis() * (arc.radius() * std::cos(angle)) + arc.yAxis() * (arc.radius() * std::sin(angle));
    }

    const MyMath::Vector3 start = edge.pointAt(0.0);
    const MyMath::Vector3 end = edge.pointAt(1.0);
    return point.distanceTo(start) <= point.distanceTo(end) ? start : end;
}

// 返回当前已支持解析Curve类型上的最近点。
bool closestPointOnFeatureEdge(const MyVoxel::Topology_Edge& edge, const MyMath::Vector3& point, MyMath::Vector3& result)
{
    switch (edge.geometry().kind())
    {
    case MyVoxel::CurveKind::Line:
        result = closestPointOnLineEdge(edge, point);
        return true;
    case MyVoxel::CurveKind::Arc:
        result = closestPointOnArcEdge(edge, static_cast<const MyVoxel::Geometry_Arc&>(edge.geometry()), point);
        return true;
    default:
        break;
    }

    return false;
}

// 尝试使用当前VoxelShape完整显式Feature约束Surface Nets候选点；真实FeatureVertex优先于Feature Edge。
bool constrainToExplicitFeature(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, const MyVoxel::VoxelCellIndex& cubeIndex,
                                const MyVoxel::Bounds3& cellBounds, const MyMath::Vector3& candidate,
                                const MyVoxel::Meshing::VolumeMeshingOptions& options, double smoothMinimumDot,
                                MyMath::Vector3& result)
{
    const MyVoxel::VoxelShape& shape = accessor.shape();

    if (!shape.hasCompleteFeatures() || shape.features().isEmpty())
    {
        return false;
    }

    const double maximumDistance = accessor.sampleSpacing() * options.featureSnapDistanceScale;
    const double queryTolerance = accessor.sampleSpacing() * FeatureQueryToleranceScale;
    const MyVoxel::VoxelFeatureSet& features = shape.features();
    std::vector<MyVoxel::Topology_Vertex> vertices;
    features.queryVertices(cellBounds, vertices, queryTolerance);

    double bestVertexDistance = (std::numeric_limits<double>::max)();
    bool foundVertex = false;

    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        const MyVoxel::Topology_Vertex& vertex = vertices[index];

        if (!isGeometricFeatureVertex(features, vertex, smoothMinimumDot) ||
            !sameCellIndex(featureVertexOwnerCell(accessor, vertex.point()), cubeIndex))
        {
            continue;
        }

        const double distance = candidate.distanceTo(vertex.point());

        if (distance <= maximumDistance && distance < bestVertexDistance)
        {
            bestVertexDistance = distance;
            result = vertex.point();
            foundVertex = true;
        }
    }

    if (foundVertex)
    {
        return true;
    }

    std::vector<MyVoxel::Topology_Edge> edges;
    features.queryEdges(cellBounds, edges, queryTolerance);
    double bestEdgeDistance = (std::numeric_limits<double>::max)();
    bool foundEdge = false;

    for (std::size_t index = 0; index < edges.size(); ++index)
    {
        MyMath::Vector3 projectedPoint;

        if (!closestPointOnFeatureEdge(edges[index], candidate, projectedPoint) ||
            !cellBounds.contains(projectedPoint, queryTolerance))
        {
            continue;
        }

        const double distance = candidate.distanceTo(projectedPoint);

        if (distance <= maximumDistance && distance < bestEdgeDistance)
        {
            bestEdgeDistance = distance;
            result = projectedPoint;
            foundEdge = true;
        }
    }

    return foundEdge;
}

// 返回指定单元直接读取TSDF值生成的八位符号掩码。
std::uint8_t directCellSignMask(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor,
                                const MyVoxel::VoxelCellIndex& cubeIndex, double isoValue)
{
    std::uint8_t signMask = 0;

    for (unsigned int cornerIndex = 0; cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount; ++cornerIndex)
    {
        const MyVoxel::VoxelCellIndex sampleIndex = offsetIndex(cubeIndex,
            MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetX(cornerIndex),
            MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetY(cornerIndex),
            MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetZ(cornerIndex));

        if (isInside(accessor.value(sampleIndex), isoValue))
        {
            signMask = static_cast<std::uint8_t>(signMask | static_cast<std::uint8_t>(1U << cornerIndex));
        }
    }

    return signMask;
}

// 预计算每个最高层TSDF采样点相对于等值面的符号。
void buildSampleSigns(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, double isoValue,
                      MyVoxel::Meshing::VolumeMeshingWorkspace& workspace)
{
    const MyVoxel::VoxelCellRange& range = workspace.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z <= static_cast<std::int64_t>(range.maximum.z); ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y <= static_cast<std::int64_t>(range.maximum.y); ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x <= static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex sampleIndex(static_cast<MyVoxel::VoxelIndex>(x),
                                                          static_cast<MyVoxel::VoxelIndex>(y),
                                                          static_cast<MyVoxel::VoxelIndex>(z));
                workspace.setSampleInside(sampleIndex, isInside(accessor.value(sampleIndex), isoValue));
            }

    MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteSampleSigns(), "VolumeMesher failed to precompute all sample signs.");
}

// 使用预计算采样符号生成每个Surface Nets单元的八位原始符号掩码。
void buildCellSignMasks(MyVoxel::Meshing::VolumeMeshingWorkspace& workspace)
{
    MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteSampleSigns(), "VolumeMesher cell sign-mask stage requires complete sample signs.");
    const MyVoxel::VoxelCellRange& range = workspace.cellRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z <= static_cast<std::int64_t>(range.maximum.z); ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y <= static_cast<std::int64_t>(range.maximum.y); ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x <= static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex cubeIndex(static_cast<MyVoxel::VoxelIndex>(x),
                                                        static_cast<MyVoxel::VoxelIndex>(y),
                                                        static_cast<MyVoxel::VoxelIndex>(z));
                std::uint8_t signMask = 0;

                for (unsigned int cornerIndex = 0; cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount; ++cornerIndex)
                {
                    const MyVoxel::VoxelCellIndex sampleIndex = offsetIndex(cubeIndex,
                        MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetX(cornerIndex),
                        MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetY(cornerIndex),
                        MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetZ(cornerIndex));

                    if (workspace.sampleInside(sampleIndex))
                    {
                        signMask = static_cast<std::uint8_t>(signMask | static_cast<std::uint8_t>(1U << cornerIndex));
                    }
                }

                workspace.setCellSignMask(cubeIndex, signMask);
            }

    MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteCellSignMasks(), "VolumeMesher failed to precompute all cell sign masks.");
}

// 使用相邻单元规则修正共享二义面的边连接方向。
std::uint8_t correctedTopologySignMask(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor,
                                       const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                                       const MyVoxel::VoxelCellIndex& cubeIndex, std::uint8_t rawSignMask,
                                       double isoValue, MyVoxel::Meshing::VolumeSignMode signMode)
{
    const std::uint8_t face = MyVoxel::Meshing::VolumeMeshingTopology::ambiguousFace(rawSignMask);

    if (face == 0)
    {
        return rawSignMask;
    }

    MyVoxel::VoxelCellIndex neighborIndex = cubeIndex;
    std::uint8_t oppositeFace = 0;

    switch (face)
    {
    case 1: neighborIndex = offsetIndex(cubeIndex, 0, 0, -1); oppositeFace = 3; break;
    case 2: neighborIndex = offsetIndex(cubeIndex, 1, 0, 0); oppositeFace = 4; break;
    case 3: neighborIndex = offsetIndex(cubeIndex, 0, 0, 1); oppositeFace = 1; break;
    case 4: neighborIndex = offsetIndex(cubeIndex, -1, 0, 0); oppositeFace = 2; break;
    case 5: neighborIndex = offsetIndex(cubeIndex, 0, -1, 0); oppositeFace = 6; break;
    case 6: neighborIndex = offsetIndex(cubeIndex, 0, 1, 0); oppositeFace = 5; break;
    default:
        MYVOXEL_ASSERT_MESSAGE(false, "VolumeMesher encountered an invalid ambiguous face index.");
        return rawSignMask;
    }

    const std::uint8_t neighborSignMask =
        signMode == MyVoxel::Meshing::VolumeSignMode::FastPrecomputed && workspace.containsCell(neighborIndex)
            ? workspace.cellSignMask(neighborIndex)
            : directCellSignMask(accessor, neighborIndex, isoValue);

    if (MyVoxel::Meshing::VolumeMeshingTopology::ambiguousFace(neighborSignMask) == oppositeFace)
    {
        return static_cast<std::uint8_t>(~rawSignMask);
    }

    return rawSignMask;
}

// 根据配置选择动态拓扑或固定查找表。
MyVoxel::Meshing::VolumeCellTopology resolveTopology(std::uint8_t signMask, MyVoxel::Meshing::VolumeTopologyMode topologyMode)
{
    return topologyMode == MyVoxel::Meshing::VolumeTopologyMode::FastLookup
        ? MyVoxel::Meshing::VolumeMeshingTopology::lookup(signMask)
        : MyVoxel::Meshing::VolumeMeshingTopology::build(signMask);
}

// 生成全部混合符号单元及其一个或多个边组顶点。
void buildCellStates(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor,
                     const MyVoxel::Meshing::VolumeMeshingOptions& options,
                     MyVoxel::Meshing::VolumeMeshingWorkspace& workspace, MyVoxel::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = workspace.cellRange();
    const bool featureSensitive = options.mode == MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets ||
                                  options.mode == MyVoxel::Meshing::VolumeMeshingMode::FeatureConstrainedSurfaceNets;
    const bool explicitFeatureConstrained = options.mode == MyVoxel::Meshing::VolumeMeshingMode::FeatureConstrainedSurfaceNets;
    const bool fastSampling = options.samplingMode == MyVoxel::Meshing::VolumeSamplingMode::FastCached;
    const bool precomputedSigns = options.signMode == MyVoxel::Meshing::VolumeSignMode::FastPrecomputed;
    const double featureMinimumDot = std::cos(options.featureAngleDegrees * Pi / 180.0);

    if (precomputedSigns)
    {
        MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteCellSignMasks(), "VolumeMesher fast sign path requires complete cell sign masks.");
    }

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z <= static_cast<std::int64_t>(range.maximum.z); ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y <= static_cast<std::int64_t>(range.maximum.y); ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x <= static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex cubeIndex(static_cast<MyVoxel::VoxelIndex>(x),
                                                        static_cast<MyVoxel::VoxelIndex>(y),
                                                        static_cast<MyVoxel::VoxelIndex>(z));
                std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerIndices;
                std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerValues;
                std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerPoints;
                std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerGradients;
                std::uint8_t rawSignMask = 0;

                if (precomputedSigns)
                {
                    rawSignMask = workspace.cellSignMask(cubeIndex);
                    if (rawSignMask == 0 || rawSignMask == 0xFFU) continue;
                    gatherCellValues(accessor, cubeIndex, cornerIndices, cornerValues);
                }
                else
                {
                    gatherCellValues(accessor, cubeIndex, cornerIndices, cornerValues);
                    rawSignMask = MyVoxel::Meshing::VolumeMeshingTopology::computeSignMask(cornerValues, options.isoValue);
                    if (rawSignMask == 0 || rawSignMask == 0xFFU) continue;
                }

                if (fastSampling) gatherCornerPositionsFast(accessor, cubeIndex, cornerPoints);
                else gatherCornerPositionsStandard(accessor, cornerIndices, cornerPoints);

                if (featureSensitive) gatherCornerGradients(accessor, cornerIndices, cornerGradients);

                const std::uint8_t topologySignMask =
                    correctedTopologySignMask(accessor, workspace, cubeIndex, rawSignMask, options.isoValue, options.signMode);
                const MyVoxel::Meshing::VolumeCellTopology topology = resolveTopology(topologySignMask, options.topologyMode);

                MyVoxel::Meshing::CellMeshingState state;
                state.rawSignMask = rawSignMask;
                state.topologySignMask = topologySignMask;
                state.edgeGroupCount = topology.edgeGroupCount;
                state.edgeGroups = topology.edgeGroups;

                MYVOXEL_ASSERT_MESSAGE(state.edgeGroupCount > 0, "Mixed-sign Surface Nets cell must contain at least one edge group.");

                for (std::uint8_t edgeGroup = 1; edgeGroup <= topology.edgeGroupCount; ++edgeGroup)
                {
                    MyVoxel::Meshing::FeaturePointSolver featureSolver;
                    std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::EdgeCount> crossingNormals;
                    MyMath::Vector3 averagePosition = MyMath::Vector3::zero();
                    unsigned int crossingCount = 0;
                    unsigned int normalCount = 0;

                    for (unsigned int edgeIndex = 0; edgeIndex < MyVoxel::Meshing::VolumeMeshingTopology::EdgeCount; ++edgeIndex)
                    {
                        if (topology.edgeGroups[edgeIndex] != edgeGroup) continue;

                        const unsigned int corner0 = MyVoxel::Meshing::VolumeMeshingTopology::edgeCorner0(edgeIndex);
                        const unsigned int corner1 = MyVoxel::Meshing::VolumeMeshingTopology::edgeCorner1(edgeIndex);
                        const double factor = isoInterpolationFactor(cornerValues[corner0], cornerValues[corner1], options.isoValue);
                        const MyMath::Vector3 crossingPoint =
                            cornerPoints[corner0] + (cornerPoints[corner1] - cornerPoints[corner0]) * factor;

                        averagePosition += crossingPoint;
                        ++crossingCount;

                        if (featureSensitive)
                        {
                            MyMath::Vector3 crossingNormal;

                            if (interpolateIsoNormal(cornerGradients[corner0], cornerGradients[corner1], factor, crossingNormal))
                            {
                                crossingNormals[normalCount] = crossingNormal;
                                ++normalCount;
                                featureSolver.addSample(crossingPoint, crossingNormal);
                            }
                        }
                    }

                    MYVOXEL_ASSERT_MESSAGE(crossingCount > 0, "VolumeMesher edge group must contain at least one crossing edge.");
                    averagePosition /= static_cast<double>(crossingCount);
                    MyMath::Vector3 position = averagePosition;

                    if (featureSensitive && hasSharpFeature(crossingNormals, normalCount, featureMinimumDot))
                    {
                        MyMath::Vector3 featurePoint;

                        if (featureSolver.solve(featurePoint))
                        {
                            position = clampPointToCell(featurePoint, cornerPoints[0], cornerPoints[6]);
                        }
                    }

                    if (explicitFeatureConstrained)
                    {
                        const MyVoxel::Bounds3 cellBounds(cornerPoints[0], cornerPoints[6]);
                        MyMath::Vector3 constrainedPosition;

                        if (constrainToExplicitFeature(accessor, cubeIndex, cellBounds, position, options, featureMinimumDot, constrainedPosition))
                        {
                            position = clampPointToCell(constrainedPosition, cornerPoints[0], cornerPoints[6]);
                        }
                    }

                    const MyMath::Vector3 normal = cellVertexNormal(cornerValues, cornerPoints, position, accessor.sampleSpacing());
                    const std::uint32_t vertexIndex = mesh.appendVertex(
                        MyVoxel::MeshVertex(position.x(), position.y(), position.z(), normal.x(), normal.y(), normal.z()));

                    if (edgeGroup == 1) state.firstVertexIndex = vertexIndex;
                    else
                    {
                        MYVOXEL_ASSERT_MESSAGE(vertexIndex == state.firstVertexIndex + static_cast<std::uint32_t>(edgeGroup - 1),
                                               "VolumeMesher cell edge-group vertices must remain contiguous.");
                    }
                }

                workspace.setState(cubeIndex, state);
            }
}

// 返回指定单元中与局部边对应的全局Surface Nets顶点。
bool findEdgeVertex(const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                    const MyVoxel::VoxelCellIndex& cubeIndex, unsigned int edgeIndex, std::uint32_t& vertexIndex)
{
    const MyVoxel::Meshing::CellMeshingState* state = workspace.find(cubeIndex);

    if (!state || state->edgeGroup(edgeIndex) == 0)
    {
        return false;
    }

    vertexIndex = state->vertexIndex(edgeIndex);
    return true;
}

// 按TSDF由内向外方向追加共享顶点四边形并固定拆分为两个三角形。
void appendIndexedQuad(MyVoxel::Mesh& mesh, std::uint32_t index0, std::uint32_t index1,
                       std::uint32_t index2, std::uint32_t index3, bool positiveOrientation,
                       const MyVoxel::Display_Color& color)
{
    if (positiveOrientation)
    {
        mesh.appendTriangle(index0, index1, index2, color);
        mesh.appendTriangle(index0, index2, index3, color);
    }
    else
    {
        mesh.appendTriangle(index0, index2, index1, color);
        mesh.appendTriangle(index0, index3, index2, color);
    }
}

// 连接所有X方向异号采样边周围四个单元的对应边组顶点。
void appendXEdgeQuads(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, double isoValue,
                      const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                      const MyVoxel::Display_Color& color, MyVoxel::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = workspace.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z) + 1; z < static_cast<std::int64_t>(range.maximum.z); ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y) + 1; y < static_cast<std::int64_t>(range.maximum.y); ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x); x < static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex sample0(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y),
                                                      static_cast<MyVoxel::VoxelIndex>(z));
                const MyVoxel::VoxelCellIndex sample1 = offsetIndex(sample0, 1, 0, 0);
                const bool inside0 = isInside(accessor.value(sample0), isoValue);
                const bool inside1 = isInside(accessor.value(sample1), isoValue);
                if (inside0 == inside1) continue;

                const MyVoxel::VoxelCellIndex cells[4] =
                {
                    sample0, offsetIndex(sample0, 0, -1, 0), offsetIndex(sample0, 0, -1, -1), offsetIndex(sample0, 0, 0, -1)
                };
                const unsigned int edges[4] = {0, 4, 6, 2};
                std::uint32_t indices[4];

                if (!findEdgeVertex(workspace, cells[0], edges[0], indices[0]) ||
                    !findEdgeVertex(workspace, cells[1], edges[1], indices[1]) ||
                    !findEdgeVertex(workspace, cells[2], edges[2], indices[2]) ||
                    !findEdgeVertex(workspace, cells[3], edges[3], indices[3])) continue;

                appendIndexedQuad(mesh, indices[0], indices[1], indices[2], indices[3], inside0 && !inside1, color);
            }
}

// 连接所有Y方向异号采样边周围四个单元的对应边组顶点。
void appendYEdgeQuads(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, double isoValue,
                      const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                      const MyVoxel::Display_Color& color, MyVoxel::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = workspace.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z) + 1; z < static_cast<std::int64_t>(range.maximum.z); ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y); y < static_cast<std::int64_t>(range.maximum.y); ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x) + 1; x < static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex sample0(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y),
                                                      static_cast<MyVoxel::VoxelIndex>(z));
                const MyVoxel::VoxelCellIndex sample1 = offsetIndex(sample0, 0, 1, 0);
                const bool inside0 = isInside(accessor.value(sample0), isoValue);
                const bool inside1 = isInside(accessor.value(sample1), isoValue);
                if (inside0 == inside1) continue;

                const MyVoxel::VoxelCellIndex cells[4] =
                {
                    sample0, offsetIndex(sample0, 0, 0, -1), offsetIndex(sample0, -1, 0, -1), offsetIndex(sample0, -1, 0, 0)
                };
                const unsigned int edges[4] = {8, 11, 10, 9};
                std::uint32_t indices[4];

                if (!findEdgeVertex(workspace, cells[0], edges[0], indices[0]) ||
                    !findEdgeVertex(workspace, cells[1], edges[1], indices[1]) ||
                    !findEdgeVertex(workspace, cells[2], edges[2], indices[2]) ||
                    !findEdgeVertex(workspace, cells[3], edges[3], indices[3])) continue;

                appendIndexedQuad(mesh, indices[0], indices[1], indices[2], indices[3], inside0 && !inside1, color);
            }
}

// 连接所有Z方向异号采样边周围四个单元的对应边组顶点。
void appendZEdgeQuads(const MyVoxel::Meshing::VoxelTsdfAccessor& accessor, double isoValue,
                      const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                      const MyVoxel::Display_Color& color, MyVoxel::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = workspace.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z); z < static_cast<std::int64_t>(range.maximum.z); ++z)
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y) + 1; y < static_cast<std::int64_t>(range.maximum.y); ++y)
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x) + 1; x < static_cast<std::int64_t>(range.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex sample0(static_cast<MyVoxel::VoxelIndex>(x), static_cast<MyVoxel::VoxelIndex>(y),
                                                      static_cast<MyVoxel::VoxelIndex>(z));
                const MyVoxel::VoxelCellIndex sample1 = offsetIndex(sample0, 0, 0, 1);
                const bool inside0 = isInside(accessor.value(sample0), isoValue);
                const bool inside1 = isInside(accessor.value(sample1), isoValue);
                if (inside0 == inside1) continue;

                const MyVoxel::VoxelCellIndex cells[4] =
                {
                    offsetIndex(sample0, -1, -1, 0), offsetIndex(sample0, 0, -1, 0), sample0, offsetIndex(sample0, -1, 0, 0)
                };
                const unsigned int edges[4] = {5, 7, 3, 1};
                std::uint32_t indices[4];

                if (!findEdgeVertex(workspace, cells[0], edges[0], indices[0]) ||
                    !findEdgeVertex(workspace, cells[1], edges[1], indices[1]) ||
                    !findEdgeVertex(workspace, cells[2], edges[2], indices[2]) ||
                    !findEdgeVertex(workspace, cells[3], edges[3], indices[3])) continue;

                appendIndexedQuad(mesh, indices[0], indices[1], indices[2], indices[3], inside0 && !inside1, color);
            }
}

}

namespace MyVoxel
{
namespace Meshing
{

VoxelCellRange VolumeMesher::defaultSampleRange(const VoxelShape& shape)
{
    validateShape(shape, VolumeMeshingOptions());

    if (shape.isEmpty())
    {
        return VoxelCellRange();
    }

    const std::int64_t scale = rootSampleScale(shape.grid());
    bool initialized = false;
    VoxelCellIndex minimumIndex;
    VoxelCellIndex maximumIndex;

    shape.forest().forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelCellIndex rootMinimum(rootMinimumSample(rootAddress.index.x, scale),
                                             rootMinimumSample(rootAddress.index.y, scale),
                                             rootMinimumSample(rootAddress.index.z, scale));
            const VoxelCellIndex rootMaximum(rootMaximumSample(rootAddress.index.x, scale),
                                             rootMaximumSample(rootAddress.index.y, scale),
                                             rootMaximumSample(rootAddress.index.z, scale));

            if (!initialized)
            {
                minimumIndex = rootMinimum;
                maximumIndex = rootMaximum;
                initialized = true;
                return;
            }

            minimumIndex.x = (std::min)(minimumIndex.x, rootMinimum.x);
            minimumIndex.y = (std::min)(minimumIndex.y, rootMinimum.y);
            minimumIndex.z = (std::min)(minimumIndex.z, rootMinimum.z);
            maximumIndex.x = (std::max)(maximumIndex.x, rootMaximum.x);
            maximumIndex.y = (std::max)(maximumIndex.y, rootMaximum.y);
            maximumIndex.z = (std::max)(maximumIndex.z, rootMaximum.z);
        });

    MYVOXEL_ASSERT_MESSAGE(initialized, "Non-empty VoxelShape must contain at least one explicit root.");
    return expandedRange(VoxelCellRange(minimumIndex, maximumIndex, shape.grid().maximumLevel()), 1);
}

Mesh VolumeMesher::build(const VoxelShape& shape, const VolumeMeshingOptions& options)
{
    validateShape(shape, options);

    if (shape.isEmpty())
    {
        return Mesh();
    }

    return build(shape, defaultSampleRange(shape), options);
}

Mesh VolumeMesher::build(const VoxelShape& shape, const VoxelCellRange& sampleRange, const VolumeMeshingOptions& options)
{
    validateShape(shape, options);
    validateSampleRange(shape, sampleRange);
    VolumeMeshingWorkspace workspace(sampleRange);
    return buildWithWorkspace(shape, workspace, options);
}

Mesh VolumeMesher::buildWithWorkspace(const VoxelShape& shape, VolumeMeshingWorkspace& workspace,
                                      const VolumeMeshingOptions& options)
{
    validateShape(shape, options);
    MYVOXEL_ASSERT_MESSAGE(workspace.isValid(), "VolumeMesher requires a valid VolumeMeshingWorkspace.");
    validateSampleRange(shape, workspace.sampleRange());

    workspace.clear();
    VoxelTsdfAccessor accessor(shape);

    if (options.signMode == VolumeSignMode::FastPrecomputed)
    {
        buildSampleSigns(accessor, options.isoValue, workspace);
        buildCellSignMasks(workspace);
    }

    Mesh mesh;
    buildCellStates(accessor, options, workspace, mesh);

    if (workspace.isEmpty())
    {
        MYVOXEL_ASSERT_MESSAGE(mesh.vertexCount() == 0, "Empty Surface Nets workspace must not contain vertices.");
        return mesh;
    }

    appendXEdgeQuads(accessor, options.isoValue, workspace, options.color, mesh);
    appendYEdgeQuads(accessor, options.isoValue, workspace, options.color, mesh);
    appendZEdgeQuads(accessor, options.isoValue, workspace, options.color, mesh);

    MYVOXEL_ASSERT_MESSAGE(mesh.isValid(), "VolumeMesher produced an invalid Surface Nets mesh.");
    MYVOXEL_ASSERT_MESSAGE(mesh.isEmpty() || mesh.hasValidNormals(), "VolumeMesher non-empty mesh must contain complete valid normals.");
    MYVOXEL_ASSERT_MESSAGE(mesh.isEmpty() || mesh.hasTriangleColors(), "VolumeMesher non-empty mesh must contain complete triangle colors.");
    return mesh;
}

}
}