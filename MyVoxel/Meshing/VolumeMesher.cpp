#include "VolumeMesher.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Meshing/CellMeshingState.h"
#include "MyVoxel/Meshing/FeaturePointSolver.h"
#include "MyVoxel/Meshing/VolumeMeshingTopology.h"
#include "MyVoxel/Meshing/VolumeMeshingWorkspace.h"

namespace
{

// 等值面拓扑和边组方案参考OpenVDB tools/VolumeToMesh.h。
// Copyright Contributors to the OpenVDB Project.
// SPDX-License-Identifier: Apache-2.0

// 返回带整数偏移的体素索引。
MyVoxel::VoxelCellIndex offsetIndex(const MyVoxel::VoxelCellIndex& index, int offsetX, int offsetY, int offsetZ)
{
    return MyVoxel::VoxelCellIndex(
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(index.x) + offsetX),
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(index.y) + offsetY),
        static_cast<MyVoxel::VoxelIndex>(static_cast<std::int64_t>(index.z) + offsetZ));
}

// 判断标量值是否位于等值面内部。
bool isInside(float value, double isoValue)
{
    return static_cast<double>(value) < isoValue;
}

// 返回两个异号采样值之间的等值面插值比例。
double isoInterpolationFactor(float value0, float value1, double isoValue)
{
    const double firstValue = static_cast<double>(value0);
    const double secondValue = static_cast<double>(value1);
    const double denominator = secondValue - firstValue;

    MYVOXEL_ASSERT_MESSAGE(denominator != 0.0, "VolumeMesher crossing edge requires distinct scalar values.");

    const double factor = (isoValue - firstValue) / denominator;
    return (std::max)(0.0, (std::min)(1.0, factor));
}

// 返回两个异号采样点之间的线性交点。
MyMath::Vector3 interpolateIsoPoint(const MyMath::Vector3& point0,
                                    const MyMath::Vector3& point1,
                                    float value0,
                                    float value1,
                                    double isoValue)
{
    return point0 + (point1 - point0) * isoInterpolationFactor(value0, value1, isoValue);
}

// 使用中心差分计算指定采样点的标量梯度。
MyMath::Vector3 sampleGradient(const MyVoxel::LevelSetVolume& volume, const MyVoxel::VoxelCellIndex& index)
{
    const double gradientX =
        static_cast<double>(volume.value(offsetIndex(index, 1, 0, 0))) -
        static_cast<double>(volume.value(offsetIndex(index, -1, 0, 0)));

    const double gradientY =
        static_cast<double>(volume.value(offsetIndex(index, 0, 1, 0))) -
        static_cast<double>(volume.value(offsetIndex(index, 0, -1, 0)));

    const double gradientZ =
        static_cast<double>(volume.value(offsetIndex(index, 0, 0, 1))) -
        static_cast<double>(volume.value(offsetIndex(index, 0, 0, -1)));

    return MyMath::Vector3(gradientX, gradientY, gradientZ);
}

// 在线性边交点处插值两个角点的梯度并归一化。
bool interpolateIsoNormal(const MyMath::Vector3& gradient0,
                          const MyMath::Vector3& gradient1,
                          double factor,
                          MyMath::Vector3& normal)
{
    normal = gradient0 + (gradient1 - gradient0) * factor;
    return normal.normalize();
}

// 根据八个角点值估计当前单元中心处的梯度。
MyMath::Vector3 cubeGradient(const std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& values)
{
    const double gradientX =
        (static_cast<double>(values[1]) + static_cast<double>(values[2]) +
         static_cast<double>(values[5]) + static_cast<double>(values[6]) -
         static_cast<double>(values[0]) - static_cast<double>(values[3]) -
         static_cast<double>(values[4]) - static_cast<double>(values[7])) * 0.25;

    const double gradientY =
        (static_cast<double>(values[4]) + static_cast<double>(values[5]) +
         static_cast<double>(values[6]) + static_cast<double>(values[7]) -
         static_cast<double>(values[0]) - static_cast<double>(values[1]) -
         static_cast<double>(values[2]) - static_cast<double>(values[3])) * 0.25;

    const double gradientZ =
        (static_cast<double>(values[2]) + static_cast<double>(values[3]) +
         static_cast<double>(values[6]) + static_cast<double>(values[7]) -
         static_cast<double>(values[0]) - static_cast<double>(values[1]) -
         static_cast<double>(values[4]) - static_cast<double>(values[5])) * 0.25;

    MyMath::Vector3 normal(gradientX, gradientY, gradientZ);

    if (!normal.normalize())
    {
        normal = MyMath::Vector3::unitZ();
    }

    return normal;
}

// 将QEF结果限制在当前单元内部，避免退化求解产生远距离尖峰。
MyMath::Vector3 clampPointToCell(const MyMath::Vector3& point,
                                const MyMath::Vector3& minimum,
                                const MyMath::Vector3& maximum)
{
    return MyMath::Vector3(
        (std::max)(minimum.x(), (std::min)(maximum.x(), point.x())),
        (std::max)(minimum.y(), (std::min)(maximum.y(), point.y())),
        (std::max)(minimum.z(), (std::min)(maximum.z(), point.z())));
}

// 读取指定单元的八个OpenVDB顺序角点索引和值。
void gatherCellValues(const MyVoxel::LevelSetVolume& volume,
                      const MyVoxel::VoxelCellIndex& cubeIndex,
                      std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerIndices,
                      std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerValues)
{
    for (unsigned int cornerIndex = 0;
         cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount;
         ++cornerIndex)
    {
        cornerIndices[cornerIndex] =
            offsetIndex(
                cubeIndex,
                MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetX(cornerIndex),
                MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetY(cornerIndex),
                MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetZ(cornerIndex));

        cornerValues[cornerIndex] = volume.value(cornerIndices[cornerIndex]);
    }
}

// 使用独立标准路径读取八个角点的世界坐标。
void gatherCornerPositionsStandard(
    const MyVoxel::LevelSetVolume& volume,
    const std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerIndices,
    std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerPoints)
{
    for (unsigned int cornerIndex = 0;
         cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount;
         ++cornerIndex)
    {
        cornerPoints[cornerIndex] = volume.samplePosition(cornerIndices[cornerIndex]);
    }
}

// 从当前单元第一个角点和最高层体素边长推导八个角点位置。
void gatherCornerPositionsFast(
    const MyVoxel::LevelSetVolume& volume,
    const MyVoxel::VoxelCellIndex& cubeIndex,
    std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerPoints)
{
    const MyMath::Vector3 basePoint = volume.samplePosition(cubeIndex);
    const double spacing = volume.grid().minimumCellEdgeLength();

    for (unsigned int cornerIndex = 0;
         cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount;
         ++cornerIndex)
    {
        cornerPoints[cornerIndex] =
            basePoint +
            MyMath::Vector3(
                static_cast<double>(MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetX(cornerIndex)) * spacing,
                static_cast<double>(MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetY(cornerIndex)) * spacing,
                static_cast<double>(MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetZ(cornerIndex)) * spacing);
    }
}

// 使用独立标准路径为八个角点分别计算中心差分梯度。
void gatherCornerGradientsStandard(
    const MyVoxel::LevelSetVolume& volume,
    const std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerIndices,
    std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerGradients)
{
    for (unsigned int cornerIndex = 0;
         cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount;
         ++cornerIndex)
    {
        cornerGradients[cornerIndex] = sampleGradient(volume, cornerIndices[cornerIndex]);
    }
}

// 从连续工作区取得八个角点的懒计算梯度。
void gatherCornerGradientsFast(
    const MyVoxel::LevelSetVolume& volume,
    MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
    const std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerIndices,
    std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount>& cornerGradients)
{
    for (unsigned int cornerIndex = 0;
         cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount;
         ++cornerIndex)
    {
        cornerGradients[cornerIndex] = workspace.gradient(volume, cornerIndices[cornerIndex]);
    }
}

// 返回指定单元直接读取标量值生成的八位符号掩码。
std::uint8_t directCellSignMask(const MyVoxel::LevelSetVolume& volume,
                                const MyVoxel::VoxelCellIndex& cubeIndex,
                                double isoValue)
{
    std::uint8_t signMask = 0;

    for (unsigned int cornerIndex = 0;
         cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount;
         ++cornerIndex)
    {
        const MyVoxel::VoxelCellIndex sampleIndex =
            offsetIndex(
                cubeIndex,
                MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetX(cornerIndex),
                MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetY(cornerIndex),
                MyVoxel::Meshing::VolumeMeshingTopology::cornerOffsetZ(cornerIndex));

        if (isInside(volume.value(sampleIndex), isoValue))
        {
            signMask = static_cast<std::uint8_t>(signMask | static_cast<std::uint8_t>(1U << cornerIndex));
        }
    }

    return signMask;
}

// 预计算每个采样点相对于当前等值面的内外符号。
void buildSampleSigns(const MyVoxel::LevelSetVolume& volume,
                      double isoValue,
                      MyVoxel::Meshing::VolumeMeshingWorkspace& workspace)
{
    const MyVoxel::VoxelCellRange& range = workspace.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z);
         z <= static_cast<std::int64_t>(range.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y);
             y <= static_cast<std::int64_t>(range.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x);
                 x <= static_cast<std::int64_t>(range.maximum.x);
                 ++x)
            {
                const MyVoxel::VoxelCellIndex sampleIndex(
                    static_cast<MyVoxel::VoxelIndex>(x),
                    static_cast<MyVoxel::VoxelIndex>(y),
                    static_cast<MyVoxel::VoxelIndex>(z));

                workspace.setSampleInside(sampleIndex, isInside(volume.value(sampleIndex), isoValue));
            }
        }
    }

    MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteSampleSigns(),
                           "VolumeMesher failed to precompute all sample signs.");
}

// 使用预计算采样符号生成每个网格单元的八位符号掩码。
void buildCellSignMasks(MyVoxel::Meshing::VolumeMeshingWorkspace& workspace)
{
    MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteSampleSigns(),
                           "VolumeMesher cell sign-mask stage requires complete sample signs.");

    const MyVoxel::VoxelCellRange& range = workspace.cellRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z);
         z <= static_cast<std::int64_t>(range.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y);
             y <= static_cast<std::int64_t>(range.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x);
                 x <= static_cast<std::int64_t>(range.maximum.x);
                 ++x)
            {
                const MyVoxel::VoxelCellIndex cubeIndex(
                    static_cast<MyVoxel::VoxelIndex>(x),
                    static_cast<MyVoxel::VoxelIndex>(y),
                    static_cast<MyVoxel::VoxelIndex>(z));

                std::uint8_t signMask = 0;

                for (unsigned int cornerIndex = 0;
                     cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount;
                     ++cornerIndex)
                {
                    const MyVoxel::VoxelCellIndex sampleIndex =
                        offsetIndex(
                            cubeIndex,
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
        }
    }

    MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteCellSignMasks(),
                           "VolumeMesher failed to precompute all cell sign masks.");
}

// 使用相邻单元规则修正共享二义面的边连接方向。
std::uint8_t correctedTopologySignMask(
    const MyVoxel::LevelSetVolume& volume,
    const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
    const MyVoxel::VoxelCellIndex& cubeIndex,
    std::uint8_t rawSignMask,
    double isoValue,
    MyVoxel::Meshing::VolumeSignMode signMode)
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
    case 1:
        neighborIndex = offsetIndex(cubeIndex, 0, 0, -1);
        oppositeFace = 3;
        break;

    case 2:
        neighborIndex = offsetIndex(cubeIndex, 1, 0, 0);
        oppositeFace = 4;
        break;

    case 3:
        neighborIndex = offsetIndex(cubeIndex, 0, 0, 1);
        oppositeFace = 1;
        break;

    case 4:
        neighborIndex = offsetIndex(cubeIndex, -1, 0, 0);
        oppositeFace = 2;
        break;

    case 5:
        neighborIndex = offsetIndex(cubeIndex, 0, -1, 0);
        oppositeFace = 6;
        break;

    case 6:
        neighborIndex = offsetIndex(cubeIndex, 0, 1, 0);
        oppositeFace = 5;
        break;

    default:
        MYVOXEL_ASSERT_MESSAGE(false, "VolumeMesher encountered an invalid ambiguous face index.");
        return rawSignMask;
    }

    std::uint8_t neighborSignMask = 0;

    if (signMode == MyVoxel::Meshing::VolumeSignMode::FastPrecomputed && workspace.containsCell(neighborIndex))
    {
        neighborSignMask = workspace.cellSignMask(neighborIndex);
    }
    else
    {
        neighborSignMask = directCellSignMask(volume, neighborIndex, isoValue);
    }

    if (MyVoxel::Meshing::VolumeMeshingTopology::ambiguousFace(neighborSignMask) == oppositeFace)
    {
        return static_cast<std::uint8_t>(~rawSignMask);
    }

    return rawSignMask;
}

// 根据配置选择独立动态拓扑路径或快速查找表路径。
MyVoxel::Meshing::VolumeCellTopology resolveTopology(
    std::uint8_t signMask,
    MyVoxel::Meshing::VolumeTopologyMode topologyMode)
{
    if (topologyMode == MyVoxel::Meshing::VolumeTopologyMode::FastLookup)
    {
        return MyVoxel::Meshing::VolumeMeshingTopology::lookup(signMask);
    }

    return MyVoxel::Meshing::VolumeMeshingTopology::build(signMask);
}

// 生成全部混合符号单元及其一个或多个边组顶点。
void buildCellStates(const MyVoxel::LevelSetVolume& volume,
                     const MyVoxel::Meshing::VolumeMeshingOptions& options,
                     MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                     MyVoxel::Geometry::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = workspace.cellRange();

    const bool featureSensitive =
        options.mode == MyVoxel::Meshing::VolumeMeshingMode::FeatureSensitiveSurfaceNets;

    const bool fastSampling =
        options.samplingMode == MyVoxel::Meshing::VolumeSamplingMode::FastCached;

    const bool precomputedSigns =
        options.signMode == MyVoxel::Meshing::VolumeSignMode::FastPrecomputed;

    if (precomputedSigns)
    {
        MYVOXEL_ASSERT_MESSAGE(workspace.hasCompleteCellSignMasks(),
                               "VolumeMesher fast sign path requires complete cell sign masks.");
    }

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z);
         z <= static_cast<std::int64_t>(range.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y);
             y <= static_cast<std::int64_t>(range.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x);
                 x <= static_cast<std::int64_t>(range.maximum.x);
                 ++x)
            {
                const MyVoxel::VoxelCellIndex cubeIndex(
                    static_cast<MyVoxel::VoxelIndex>(x),
                    static_cast<MyVoxel::VoxelIndex>(y),
                    static_cast<MyVoxel::VoxelIndex>(z));

                std::array<MyVoxel::VoxelCellIndex, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerIndices;
                std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerPoints;
                std::array<MyMath::Vector3, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerGradients;
                std::array<float, MyVoxel::Meshing::VolumeMeshingTopology::CornerCount> cornerValues;

                std::uint8_t rawSignMask = 0;

                if (precomputedSigns)
                {
                    rawSignMask = workspace.cellSignMask(cubeIndex);

                    if (rawSignMask == 0 || rawSignMask == 0xFFU)
                    {
                        continue;
                    }

                    gatherCellValues(volume, cubeIndex, cornerIndices, cornerValues);
                }
                else
                {
                    gatherCellValues(volume, cubeIndex, cornerIndices, cornerValues);

                    rawSignMask =
                        MyVoxel::Meshing::VolumeMeshingTopology::computeSignMask(cornerValues, options.isoValue);

                    if (rawSignMask == 0 || rawSignMask == 0xFFU)
                    {
                        continue;
                    }
                }

                if (fastSampling)
                {
                    gatherCornerPositionsFast(volume, cubeIndex, cornerPoints);
                }
                else
                {
                    gatherCornerPositionsStandard(volume, cornerIndices, cornerPoints);
                }

                if (featureSensitive)
                {
                    if (fastSampling)
                    {
                        gatherCornerGradientsFast(volume, workspace, cornerIndices, cornerGradients);
                    }
                    else
                    {
                        gatherCornerGradientsStandard(volume, cornerIndices, cornerGradients);
                    }
                }

                const std::uint8_t topologySignMask =
                    correctedTopologySignMask(
                        volume,
                        workspace,
                        cubeIndex,
                        rawSignMask,
                        options.isoValue,
                        options.signMode);

                const MyVoxel::Meshing::VolumeCellTopology topology =
                    resolveTopology(topologySignMask, options.topologyMode);

                MyVoxel::Meshing::CellMeshingState state;
                state.rawSignMask = rawSignMask;
                state.topologySignMask = topologySignMask;
                state.edgeGroupCount = topology.edgeGroupCount;
                state.edgeGroups = topology.edgeGroups;

                const MyMath::Vector3 fallbackNormal = cubeGradient(cornerValues);

                for (std::uint8_t edgeGroup = 1; edgeGroup <= topology.edgeGroupCount; ++edgeGroup)
                {
                    MyVoxel::Meshing::FeaturePointSolver featureSolver;
                    MyMath::Vector3 averagePosition = MyMath::Vector3::zero();
                    MyMath::Vector3 averageNormal = MyMath::Vector3::zero();
                    unsigned int crossingCount = 0;
                    unsigned int normalCount = 0;

                    for (unsigned int edgeIndex = 0;
                         edgeIndex < MyVoxel::Meshing::VolumeMeshingTopology::EdgeCount;
                         ++edgeIndex)
                    {
                        if (topology.edgeGroups[edgeIndex] != edgeGroup)
                        {
                            continue;
                        }

                        const unsigned int corner0 =
                            MyVoxel::Meshing::VolumeMeshingTopology::edgeCorner0(edgeIndex);

                        const unsigned int corner1 =
                            MyVoxel::Meshing::VolumeMeshingTopology::edgeCorner1(edgeIndex);

                        const double factor =
                            isoInterpolationFactor(
                                cornerValues[corner0],
                                cornerValues[corner1],
                                options.isoValue);

                        const MyMath::Vector3 crossingPoint =
                            interpolateIsoPoint(
                                cornerPoints[corner0],
                                cornerPoints[corner1],
                                cornerValues[corner0],
                                cornerValues[corner1],
                                options.isoValue);

                        averagePosition += crossingPoint;
                        ++crossingCount;

                        if (featureSensitive)
                        {
                            MyMath::Vector3 crossingNormal;

                            if (interpolateIsoNormal(
                                    cornerGradients[corner0],
                                    cornerGradients[corner1],
                                    factor,
                                    crossingNormal))
                            {
                                averageNormal += crossingNormal;
                                ++normalCount;
                                featureSolver.addSample(crossingPoint, crossingNormal);
                            }
                        }
                    }

                    MYVOXEL_ASSERT_MESSAGE(
                        crossingCount > 0,
                        "VolumeMesher edge group must contain at least one crossing edge.");

                    averagePosition /= static_cast<double>(crossingCount);

                    MyMath::Vector3 position = averagePosition;
                    MyMath::Vector3 normal = fallbackNormal;

                    if (featureSensitive)
                    {
                        MyMath::Vector3 featurePoint;

                        if (featureSolver.solve(featurePoint))
                        {
                            position = clampPointToCell(featurePoint, cornerPoints[0], cornerPoints[6]);
                        }

                        if (normalCount > 0 && averageNormal.normalize())
                        {
                            normal = averageNormal;
                        }
                    }

                    const std::uint32_t vertexIndex =
                        mesh.appendVertex(
                            MyVoxel::Geometry::MeshVertex(
                                position.x(),
                                position.y(),
                                position.z(),
                                normal.x(),
                                normal.y(),
                                normal.z()));

                    if (edgeGroup == 1)
                    {
                        state.firstVertexIndex = vertexIndex;
                    }
                    else
                    {
                        MYVOXEL_ASSERT_MESSAGE(
                            vertexIndex == state.firstVertexIndex + static_cast<std::uint32_t>(edgeGroup - 1),
                            "VolumeMesher cell edge-group vertices must remain contiguous.");
                    }
                }

                workspace.setState(cubeIndex, state);
            }
        }
    }
}

// 返回指定单元中与指定局部边对应的全局顶点。
bool findEdgeVertex(const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                    const MyVoxel::VoxelCellIndex& cubeIndex,
                    unsigned int edgeIndex,
                    std::uint32_t& vertexIndex)
{
    const MyVoxel::Meshing::CellMeshingState* state = workspace.find(cubeIndex);

    if (!state || state->edgeGroup(edgeIndex) == 0)
    {
        return false;
    }

    vertexIndex = state->vertexIndex(edgeIndex);
    return true;
}

// 按指定方向追加一个共享顶点四边形。
void appendIndexedQuad(MyVoxel::Geometry::Mesh& mesh,
                       std::uint32_t index0,
                       std::uint32_t index1,
                       std::uint32_t index2,
                       std::uint32_t index3,
                       bool positiveOrientation,
                       const MyVoxel::Geometry::MeshColor& color)
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

// 连接所有X方向异号采样边周围的四个正确边组顶点。
void appendXEdgeQuads(const MyVoxel::LevelSetVolume& volume,
                      double isoValue,
                      const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                      const MyVoxel::Geometry::MeshColor& color,
                      MyVoxel::Geometry::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = volume.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z) + 1;
         z < static_cast<std::int64_t>(range.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y) + 1;
             y < static_cast<std::int64_t>(range.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x);
                 x < static_cast<std::int64_t>(range.maximum.x);
                 ++x)
            {
                const MyVoxel::VoxelCellIndex sample0(
                    static_cast<MyVoxel::VoxelIndex>(x),
                    static_cast<MyVoxel::VoxelIndex>(y),
                    static_cast<MyVoxel::VoxelIndex>(z));

                const MyVoxel::VoxelCellIndex sample1 = offsetIndex(sample0, 1, 0, 0);
                const bool inside0 = isInside(volume.value(sample0), isoValue);
                const bool inside1 = isInside(volume.value(sample1), isoValue);

                if (inside0 == inside1)
                {
                    continue;
                }

                const MyVoxel::VoxelCellIndex cells[4] =
                {
                    sample0,
                    offsetIndex(sample0, 0, -1, 0),
                    offsetIndex(sample0, 0, -1, -1),
                    offsetIndex(sample0, 0, 0, -1)
                };

                const unsigned int edges[4] = { 0, 4, 6, 2 };
                std::uint32_t indices[4];

                if (!findEdgeVertex(workspace, cells[0], edges[0], indices[0]) ||
                    !findEdgeVertex(workspace, cells[1], edges[1], indices[1]) ||
                    !findEdgeVertex(workspace, cells[2], edges[2], indices[2]) ||
                    !findEdgeVertex(workspace, cells[3], edges[3], indices[3]))
                {
                    continue;
                }

                appendIndexedQuad(mesh, indices[0], indices[1], indices[2], indices[3], inside0 && !inside1, color);
            }
        }
    }
}

// 连接所有Y方向异号采样边周围的四个正确边组顶点。
void appendYEdgeQuads(const MyVoxel::LevelSetVolume& volume,
                      double isoValue,
                      const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                      const MyVoxel::Geometry::MeshColor& color,
                      MyVoxel::Geometry::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = volume.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z) + 1;
         z < static_cast<std::int64_t>(range.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y);
             y < static_cast<std::int64_t>(range.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x) + 1;
                 x < static_cast<std::int64_t>(range.maximum.x);
                 ++x)
            {
                const MyVoxel::VoxelCellIndex sample0(
                    static_cast<MyVoxel::VoxelIndex>(x),
                    static_cast<MyVoxel::VoxelIndex>(y),
                    static_cast<MyVoxel::VoxelIndex>(z));

                const MyVoxel::VoxelCellIndex sample1 = offsetIndex(sample0, 0, 1, 0);
                const bool inside0 = isInside(volume.value(sample0), isoValue);
                const bool inside1 = isInside(volume.value(sample1), isoValue);

                if (inside0 == inside1)
                {
                    continue;
                }

                const MyVoxel::VoxelCellIndex cells[4] =
                {
                    sample0,
                    offsetIndex(sample0, 0, 0, -1),
                    offsetIndex(sample0, -1, 0, -1),
                    offsetIndex(sample0, -1, 0, 0)
                };

                const unsigned int edges[4] = { 8, 11, 10, 9 };
                std::uint32_t indices[4];

                if (!findEdgeVertex(workspace, cells[0], edges[0], indices[0]) ||
                    !findEdgeVertex(workspace, cells[1], edges[1], indices[1]) ||
                    !findEdgeVertex(workspace, cells[2], edges[2], indices[2]) ||
                    !findEdgeVertex(workspace, cells[3], edges[3], indices[3]))
                {
                    continue;
                }

                appendIndexedQuad(mesh, indices[0], indices[1], indices[2], indices[3], inside0 && !inside1, color);
            }
        }
    }
}

// 连接所有Z方向异号采样边周围的四个正确边组顶点。
void appendZEdgeQuads(const MyVoxel::LevelSetVolume& volume,
                      double isoValue,
                      const MyVoxel::Meshing::VolumeMeshingWorkspace& workspace,
                      const MyVoxel::Geometry::MeshColor& color,
                      MyVoxel::Geometry::Mesh& mesh)
{
    const MyVoxel::VoxelCellRange& range = volume.sampleRange();

    for (std::int64_t z = static_cast<std::int64_t>(range.minimum.z);
         z < static_cast<std::int64_t>(range.maximum.z);
         ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(range.minimum.y) + 1;
             y < static_cast<std::int64_t>(range.maximum.y);
             ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(range.minimum.x) + 1;
                 x < static_cast<std::int64_t>(range.maximum.x);
                 ++x)
            {
                const MyVoxel::VoxelCellIndex sample0(
                    static_cast<MyVoxel::VoxelIndex>(x),
                    static_cast<MyVoxel::VoxelIndex>(y),
                    static_cast<MyVoxel::VoxelIndex>(z));

                const MyVoxel::VoxelCellIndex sample1 = offsetIndex(sample0, 0, 0, 1);
                const bool inside0 = isInside(volume.value(sample0), isoValue);
                const bool inside1 = isInside(volume.value(sample1), isoValue);

                if (inside0 == inside1)
                {
                    continue;
                }

                const MyVoxel::VoxelCellIndex cells[4] =
                {
                    offsetIndex(sample0, -1, -1, 0),
                    offsetIndex(sample0, 0, -1, 0),
                    sample0,
                    offsetIndex(sample0, -1, 0, 0)
                };

                const unsigned int edges[4] = { 5, 7, 3, 1 };
                std::uint32_t indices[4];

                if (!findEdgeVertex(workspace, cells[0], edges[0], indices[0]) ||
                    !findEdgeVertex(workspace, cells[1], edges[1], indices[1]) ||
                    !findEdgeVertex(workspace, cells[2], edges[2], indices[2]) ||
                    !findEdgeVertex(workspace, cells[3], edges[3], indices[3]))
                {
                    continue;
                }

                appendIndexedQuad(mesh, indices[0], indices[1], indices[2], indices[3], inside0 && !inside1, color);
            }
        }
    }
}

// 判断两个采样范围是否完全一致。
bool sameRange(const MyVoxel::VoxelCellRange& first, const MyVoxel::VoxelCellRange& second)
{
    return
        first.level == second.level &&
        first.minimum.x == second.minimum.x &&
        first.minimum.y == second.minimum.y &&
        first.minimum.z == second.minimum.z &&
        first.maximum.x == second.maximum.x &&
        first.maximum.y == second.maximum.y &&
        first.maximum.z == second.maximum.z;
}

}

namespace MyVoxel
{
namespace Meshing
{

Geometry::Mesh VolumeMesher::build(const LevelSetVolume& volume, const VolumeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(volume.isValid(), "VolumeMesher requires a valid LevelSetVolume.");

    VolumeMeshingWorkspace workspace(volume.sampleRange());
    return buildWithWorkspace(volume, workspace, options);
}

Geometry::Mesh VolumeMesher::buildWithWorkspace(const LevelSetVolume& volume,
                                                VolumeMeshingWorkspace& workspace,
                                                const VolumeMeshingOptions& options)
{
    MYVOXEL_ASSERT_MESSAGE(volume.isValid(), "VolumeMesher requires a valid LevelSetVolume.");
    MYVOXEL_ASSERT_MESSAGE(std::isfinite(options.isoValue), "VolumeMesher isoValue must be finite.");
    MYVOXEL_ASSERT_MESSAGE(workspace.isValid(), "VolumeMesher requires a valid meshing workspace.");
    MYVOXEL_ASSERT_MESSAGE(sameRange(volume.sampleRange(), workspace.sampleRange()),
                           "VolumeMesher workspace sample range must match the source volume.");

    workspace.clear();

    if (options.signMode == VolumeSignMode::FastPrecomputed)
    {
        buildSampleSigns(volume, options.isoValue, workspace);
        buildCellSignMasks(workspace);
    }

    Geometry::Mesh mesh;

    buildCellStates(volume, options, workspace, mesh);

    appendXEdgeQuads(volume, options.isoValue, workspace, options.color, mesh);
    appendYEdgeQuads(volume, options.isoValue, workspace, options.color, mesh);
    appendZEdgeQuads(volume, options.isoValue, workspace, options.color, mesh);

    MYVOXEL_ASSERT_MESSAGE(workspace.isValid(), "VolumeMesher generated an invalid meshing workspace.");
    MYVOXEL_ASSERT_MESSAGE(mesh.isValid(), "VolumeMesher generated an invalid Geometry::Mesh.");

    return mesh;
}

}
}