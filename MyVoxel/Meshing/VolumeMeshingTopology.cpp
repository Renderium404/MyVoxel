#include "VolumeMeshingTopology.h"

#include <array>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// OpenVDB角点顺序：X为最低位方向，随后沿Z，再沿Y排列。
const int CornerOffsets[8][3] =
{
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 1, 0, 1 },
    { 0, 0, 1 },
    { 0, 1, 0 },
    { 1, 1, 0 },
    { 1, 1, 1 },
    { 0, 1, 1 }
};

// OpenVDB十二条边顺序，对应边组表中的十二条边。
const unsigned int EdgeCorners[12][2] =
{
    { 0, 1 },
    { 1, 2 },
    { 3, 2 },
    { 0, 3 },
    { 4, 5 },
    { 5, 6 },
    { 7, 6 },
    { 4, 7 },
    { 0, 4 },
    { 1, 5 },
    { 2, 6 },
    { 3, 7 }
};

// 六个立方体面按照环形顺序保存四个角点。
const unsigned int FaceCorners[6][4] =
{
    { 0, 1, 5, 4 }, // Z负方向面。
    { 1, 2, 6, 5 }, // X正方向面。
    { 3, 2, 6, 7 }, // Z正方向面。
    { 0, 3, 7, 4 }, // X负方向面。
    { 0, 1, 2, 3 }, // Y负方向面。
    { 4, 5, 6, 7 }  // Y正方向面。
};

// 六个立方体面按照与FaceCorners一致的环形顺序保存四条边。
const unsigned int FaceEdges[6][4] =
{
    { 0, 9, 4, 8 },
    { 1, 10, 5, 9 },
    { 2, 10, 6, 11 },
    { 3, 11, 7, 8 },
    { 0, 1, 2, 3 },
    { 4, 5, 6, 7 }
};

// OpenVDB用于相邻单元二义面一致性修正的256项表。
// Copyright Contributors to the OpenVDB Project.
// SPDX-License-Identifier: Apache-2.0
const std::uint8_t AmbiguousFaceTable[256] =
{
    0,0,0,0,0,5,0,0,0,0,5,0,0,0,0,0,0,0,1,0,0,5,1,0,4,0,0,0,4,0,0,0,
    0,1,0,0,2,0,0,0,0,1,5,0,2,0,0,0,0,0,0,0,2,0,0,0,4,0,0,0,0,0,0,0,
    0,0,2,2,0,5,0,0,3,3,0,0,0,0,0,0,6,6,0,0,6,0,0,0,0,0,0,0,0,0,0,0,
    0,1,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,4,0,4,3,0,3,0,0,0,5,0,0,0,0,0,0,0,1,0,3,0,0,0,0,0,0,0,0,0,0,0,
    6,0,6,0,0,0,0,0,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,4,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

// 管理十二条边的并查集。
class EdgeDisjointSet
{
public:
    EdgeDisjointSet()
    {
        for (unsigned int edgeIndex = 0; edgeIndex < MyVoxel::Meshing::VolumeMeshingTopology::EdgeCount; ++edgeIndex)
        {
            m_parents[edgeIndex] = edgeIndex;
        }
    }

    unsigned int find(unsigned int edgeIndex)
    {
        MYVOXEL_ASSERT_MESSAGE(edgeIndex < MyVoxel::Meshing::VolumeMeshingTopology::EdgeCount,
                               "Volume meshing edge index exceeds the disjoint-set range.");

        while (m_parents[edgeIndex] != edgeIndex)
        {
            m_parents[edgeIndex] = m_parents[m_parents[edgeIndex]];
            edgeIndex = m_parents[edgeIndex];
        }

        return edgeIndex;
    }

    void unite(unsigned int firstEdge, unsigned int secondEdge)
    {
        const unsigned int firstRoot = find(firstEdge);
        const unsigned int secondRoot = find(secondEdge);

        if (firstRoot != secondRoot)
        {
            m_parents[secondRoot] = firstRoot;
        }
    }

private:
    std::array<unsigned int, MyVoxel::Meshing::VolumeMeshingTopology::EdgeCount> m_parents;
};

// 判断指定角点在符号掩码中是否位于内部。
bool cornerInside(std::uint8_t signMask, unsigned int cornerIndex)
{
    MYVOXEL_ASSERT_MESSAGE(cornerIndex < MyVoxel::Meshing::VolumeMeshingTopology::CornerCount,
                           "Volume meshing corner index exceeds the valid range.");

    return (signMask & static_cast<std::uint8_t>(1U << cornerIndex)) != 0;
}

// 保存启动时由标准路径生成的256项固定查找表。
struct TopologyLookupTable
{
    TopologyLookupTable()
    {
        for (unsigned int signMask = 0; signMask < MyVoxel::Meshing::VolumeMeshingTopology::SignConfigurationCount; ++signMask)
        {
            entries[signMask] =
                MyVoxel::Meshing::VolumeMeshingTopology::build(
                    static_cast<std::uint8_t>(signMask));
        }
    }

    std::array<MyVoxel::Meshing::VolumeCellTopology,
               MyVoxel::Meshing::VolumeMeshingTopology::SignConfigurationCount> entries;
};

const TopologyLookupTable FastTopologyTable; // 在程序启动时一次性生成，此后热点中只进行数组索引。

}

namespace MyVoxel
{
namespace Meshing
{

VolumeCellTopology::VolumeCellTopology()
    : signMask(0)
    , edgeGroupCount(0)
{
    edgeGroups.fill(0);
}

/// 角点与边定义

int VolumeMeshingTopology::cornerOffsetX(unsigned int cornerIndex)
{
    MYVOXEL_ASSERT_MESSAGE(cornerIndex < CornerCount, "Volume meshing corner index exceeds the valid range.");
    return CornerOffsets[cornerIndex][0];
}

int VolumeMeshingTopology::cornerOffsetY(unsigned int cornerIndex)
{
    MYVOXEL_ASSERT_MESSAGE(cornerIndex < CornerCount, "Volume meshing corner index exceeds the valid range.");
    return CornerOffsets[cornerIndex][1];
}

int VolumeMeshingTopology::cornerOffsetZ(unsigned int cornerIndex)
{
    MYVOXEL_ASSERT_MESSAGE(cornerIndex < CornerCount, "Volume meshing corner index exceeds the valid range.");
    return CornerOffsets[cornerIndex][2];
}

unsigned int VolumeMeshingTopology::edgeCorner0(unsigned int edgeIndex)
{
    MYVOXEL_ASSERT_MESSAGE(edgeIndex < EdgeCount, "Volume meshing edge index exceeds the valid range.");
    return EdgeCorners[edgeIndex][0];
}

unsigned int VolumeMeshingTopology::edgeCorner1(unsigned int edgeIndex)
{
    MYVOXEL_ASSERT_MESSAGE(edgeIndex < EdgeCount, "Volume meshing edge index exceeds the valid range.");
    return EdgeCorners[edgeIndex][1];
}

/// 符号与拓扑

std::uint8_t VolumeMeshingTopology::computeSignMask(const std::array<float, CornerCount>& values, double isoValue)
{
    std::uint8_t signMask = 0;

    for (unsigned int cornerIndex = 0; cornerIndex < CornerCount; ++cornerIndex)
    {
        if (static_cast<double>(values[cornerIndex]) < isoValue)
        {
            signMask |= static_cast<std::uint8_t>(1U << cornerIndex);
        }
    }

    return signMask;
}

std::uint8_t VolumeMeshingTopology::ambiguousFace(std::uint8_t signMask)
{
    return AmbiguousFaceTable[signMask];
}

VolumeCellTopology VolumeMeshingTopology::build(std::uint8_t signMask)
{
    VolumeCellTopology topology;
    topology.signMask = signMask;

    if (signMask == 0 || signMask == 0xFFU)
    {
        return topology;
    }

    std::array<bool, EdgeCount> crossingEdges;

    for (unsigned int edgeIndex = 0; edgeIndex < EdgeCount; ++edgeIndex)
    {
        crossingEdges[edgeIndex] =
            cornerInside(signMask, EdgeCorners[edgeIndex][0]) !=
            cornerInside(signMask, EdgeCorners[edgeIndex][1]);
    }

    EdgeDisjointSet groups;

    for (unsigned int faceIndex = 0; faceIndex < FaceCount; ++faceIndex)
    {
        unsigned int faceCrossingEdges[4];
        unsigned int crossingCount = 0;

        for (unsigned int faceEdgeIndex = 0; faceEdgeIndex < 4; ++faceEdgeIndex)
        {
            const unsigned int edgeIndex = FaceEdges[faceIndex][faceEdgeIndex];

            if (crossingEdges[edgeIndex])
            {
                faceCrossingEdges[crossingCount] = edgeIndex;
                ++crossingCount;
            }
        }

        if (crossingCount == 2)
        {
            groups.unite(faceCrossingEdges[0], faceCrossingEdges[1]);
            continue;
        }

        if (crossingCount != 4)
        {
            continue;
        }

        const unsigned int edge0 = FaceEdges[faceIndex][0];
        const unsigned int edge1 = FaceEdges[faceIndex][1];
        const unsigned int edge2 = FaceEdges[faceIndex][2];
        const unsigned int edge3 = FaceEdges[faceIndex][3];

        // 二义面沿两个外部角点分别连接相邻相交边。
        if (cornerInside(signMask, FaceCorners[faceIndex][0]))
        {
            groups.unite(edge0, edge1);
            groups.unite(edge2, edge3);
        }
        else
        {
            groups.unite(edge3, edge0);
            groups.unite(edge1, edge2);
        }
    }

    std::array<std::uint8_t, EdgeCount> rootGroups;
    rootGroups.fill(0);

    for (unsigned int edgeIndex = 0; edgeIndex < EdgeCount; ++edgeIndex)
    {
        if (!crossingEdges[edgeIndex])
        {
            continue;
        }

        const unsigned int rootEdge = groups.find(edgeIndex);

        if (rootGroups[rootEdge] == 0)
        {
            ++topology.edgeGroupCount;

            MYVOXEL_ASSERT_MESSAGE(topology.edgeGroupCount <= MaximumEdgeGroupCount,
                                   "Volume meshing cell requires more than four edge groups.");

            rootGroups[rootEdge] = topology.edgeGroupCount;
        }

        topology.edgeGroups[edgeIndex] = rootGroups[rootEdge];
    }

    MYVOXEL_ASSERT_MESSAGE(topology.edgeGroupCount > 0,
                           "Mixed-sign volume cell must contain at least one edge group.");

    return topology;
}

const VolumeCellTopology& VolumeMeshingTopology::lookup(std::uint8_t signMask)
{
    return FastTopologyTable.entries[signMask];
}

}
}