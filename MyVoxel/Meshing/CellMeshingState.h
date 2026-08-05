#ifndef MYVOXEL_MESHING_CELLMESHINGSTATE_H
#define MYVOXEL_MESHING_CELLMESHINGSTATE_H

#include <array>
#include <cassert>
#include <cstdint>

#include "VolumeMeshingTopology.h"

namespace MyVoxel
{
namespace Meshing
{

// 保存一个混合符号单元在当前网格提取过程中的临时状态。
struct CellMeshingState
{
    CellMeshingState()
        : rawSignMask(0)
        , topologySignMask(0)
        , edgeGroupCount(0)
        , firstVertexIndex(0)
    {
        edgeGroups.fill(0);
    }

    // 判断OpenVDB角点0是否位于等值面内部。
    bool originInside() const
    {
        return (rawSignMask & 0x01U) != 0;
    }

    // 返回指定边所属的局部顶点组。
    std::uint8_t edgeGroup(unsigned int edgeIndex) const
    {
        assert(edgeIndex < static_cast<unsigned int>(VolumeMeshingTopology::EdgeCount));
        return edgeGroups[edgeIndex];
    }

    // 返回指定相交边对应的全局顶点索引。
    std::uint32_t vertexIndex(unsigned int edgeIndex) const
    {
        const std::uint8_t group = edgeGroup(edgeIndex);

        assert(group > 0);
        assert(group <= edgeGroupCount);
        return firstVertexIndex + static_cast<std::uint32_t>(group - 1);
    }

    std::uint8_t rawSignMask; // 未执行二义性修正的真实角点符号。
    std::uint8_t topologySignMask; // 用于边组划分的修正后符号。
    std::uint8_t edgeGroupCount; // 当前单元生成的连续顶点数量。
    std::array<std::uint8_t, VolumeMeshingTopology::EdgeCount> edgeGroups; // 十二条边对应的局部顶点组。
    std::uint32_t firstVertexIndex; // 当前单元第一组顶点的全局索引。
};

}
}

#endif // MYVOXEL_MESHING_CELLMESHINGSTATE_H