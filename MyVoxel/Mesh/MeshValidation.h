#ifndef MYVOXEL_MESH_MESHVALIDATION_H
#define MYVOXEL_MESH_MESHVALIDATION_H

#include <cstddef>

#include "MyVoxel/Mesh/Mesh.h"

namespace MyVoxel
{

// 保存一次索引三角网格几何和拓扑检查结果。
//
// 拓扑检查基于顶点索引而不是坐标焊接：几何位置相同但索引不同的顶点仍被视为不同顶点。
// 当前报告不检查三角形自相交，也不要求三角形绕序一致。
struct MeshValidationReport
{
    MeshValidationReport();

    /// 综合判断

    // 判断源网格的位置和索引是否构成有效索引三角网格。
    bool hasValidGeometry() const;

    // 判断网格是否为非空、无退化和重复三角形、无边界边、无非流形边及非流形顶点的闭合流形。
    //
    // 该判断不包含三角形自相交检查，也不要求网格只有一个连通分量。
    bool isClosedManifold() const;

    // 判断网格是否为只有一个三角形连通分量的闭合流形。
    bool isSingleComponentClosedManifold() const;

    /// 基础统计

    bool geometryValid; // 源网格的位置和索引是否有效。
    std::size_t vertexCount; // 源网格顶点数量。
    std::size_t usedVertexCount; // 至少被一个三角形索引引用的顶点数量。
    std::size_t unusedVertexCount; // 没有被任何三角形引用的顶点数量。
    std::size_t triangleCount; // 源网格三角形数量。

    /// 几何问题

    std::size_t degenerateTriangleCount; // 二倍面积不大于指定阈值的三角形数量。
    std::size_t duplicateTriangleCount; // 与更早三角形使用相同三个顶点索引的重复三角形数量。

    /// 拓扑统计

    std::size_t uniqueEdgeCount; // 按无向顶点索引对去重后的边数量。
    std::size_t boundaryEdgeCount; // 只被一个三角形使用的边数量。
    std::size_t manifoldEdgeCount; // 恰好被两个三角形使用的边数量。
    std::size_t nonManifoldEdgeCount; // 被三个或更多三角形使用的边数量。
    std::size_t nonManifoldVertexCount; // 入射三角形不能围绕顶点形成单个连通扇区的顶点数量。
    std::size_t connectedComponentCount; // 通过共享边连接得到的三角形连通分量数量。
};

// 检查指定网格的几何和索引拓扑，并返回独立报告。
//
// doubledAreaTolerance用于退化三角形判断，含义与Mesh::degenerateTriangleCount一致。
// 该函数不修改网格，不检查三角形自相交，也不要求三角形绕序一致。
MeshValidationReport validateMesh(
    const Mesh& mesh,
    double doubledAreaTolerance = 1.0e-12);

}

#endif // MYVOXEL_MESH_MESHVALIDATION_H