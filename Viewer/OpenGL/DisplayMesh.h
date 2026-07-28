#ifndef MYVOXELVIEWER_DISPLAYMESH_H
#define MYVOXELVIEWER_DISPLAYMESH_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "DisplayColor.h"

namespace MyVoxelViewer
{

// 表示一个可直接上传到GPU的显示顶点。
struct DisplayVertex
{
    DisplayVertex() = default;
    DisplayVertex(double xValue, double yValue, double zValue, double normalXValue, double normalYValue, double normalZValue, const DisplayColor& colorValue);

    float x = 0.0f; // 顶点X坐标。
    float y = 0.0f; // 顶点Y坐标。
    float z = 0.0f; // 顶点Z坐标。
    float normalX = 0.0f; // 法向X分量。
    float normalY = 0.0f; // 法向Y分量。
    float normalZ = 1.0f; // 法向Z分量。
    DisplayColor color; // 顶点颜色。
};

// 表示可视化层统一使用的三角网格。
class DisplayMesh
{
public:
    // 清空全部顶点和索引。
    void clear();

    // 为后续顶点和索引写入预留空间。
    void reserve(std::size_t vertexCount, std::size_t indexCount);

    // 检查当前网格是否没有可绘制三角形。
    bool isEmpty() const;

    // 返回当前顶点数量。
    std::size_t vertexCount() const;

    // 返回当前索引数量。
    std::size_t indexCount() const;

    // 返回可修改顶点集合。
    std::vector<DisplayVertex>& vertices();

    // 返回只读顶点集合。
    const std::vector<DisplayVertex>& vertices() const;

    // 返回可修改索引集合。
    std::vector<std::uint32_t>& indices();

    // 返回只读索引集合。
    const std::vector<std::uint32_t>& indices() const;

private:
    std::vector<DisplayVertex> m_vertices; // 显示网格顶点。
    std::vector<std::uint32_t> m_indices; // 三角形索引。
};

}

#endif // MYVOXELVIEWER_DISPLAYMESH_H