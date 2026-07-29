#ifndef MYVOXEL_SURFACE_VOXELSURFACEMESH_H
#define MYVOXEL_SURFACE_VOXELSURFACEMESH_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace MyVoxel
{

// 表示体素表面的RGBA颜色。
struct VoxelSurfaceColor
{
    VoxelSurfaceColor();
    VoxelSurfaceColor(double redValue, double greenValue, double blueValue, double alphaValue = 1.0);

    float red; // 红色分量。
    float green; // 绿色分量。
    float blue; // 蓝色分量。
    float alpha; // 透明度分量。
};

// 表示体素表面三角网格中的一个局部空间顶点。
struct VoxelSurfaceVertex
{
    VoxelSurfaceVertex();
    VoxelSurfaceVertex(double xValue, double yValue, double zValue, double normalXValue, double normalYValue, double normalZValue, const VoxelSurfaceColor& colorValue);

    float x; // 体素形体局部坐标系中的X坐标。
    float y; // 体素形体局部坐标系中的Y坐标。
    float z; // 体素形体局部坐标系中的Z坐标。
    float normalX; // 局部空间法线X分量。
    float normalY; // 局部空间法线Y分量。
    float normalZ; // 局部空间法线Z分量。
    VoxelSurfaceColor color; // 当前顶点颜色。
};

// 保存体素外表面的局部空间三角网格。
class VoxelSurfaceMesh
{
public:
    /// 网格状态

    // 清空全部顶点和索引。
    void clear();

    // 预留指定数量的顶点和索引空间。
    void reserve(std::size_t vertexCount, std::size_t indexCount);

    // 判断当前网格是否没有有效三角形。
    bool isEmpty() const;

    // 返回当前顶点数量。
    std::size_t vertexCount() const;

    // 返回当前索引数量。
    std::size_t indexCount() const;

    // 返回当前三角形数量。
    std::size_t triangleCount() const;

    /// 网格数据

    // 返回只读顶点数组。
    const std::vector<VoxelSurfaceVertex>& vertices() const;

    // 返回只读三角形索引数组。
    const std::vector<std::uint32_t>& indices() const;

    /// 网格构建

    // 追加另一个独立网格，并修正其全部索引偏移。
    void append(const VoxelSurfaceMesh& mesh);

    // 追加一个顶点并返回其索引。
    std::uint32_t appendVertex(const VoxelSurfaceVertex& vertex);

    // 使用三个已有顶点索引追加一个三角形。
    void appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2);

    // 追加三个独立顶点组成的三角形。
    void appendTriangle(const VoxelSurfaceVertex& vertex0, const VoxelSurfaceVertex& vertex1, const VoxelSurfaceVertex& vertex2);

    // 追加一个四边形面，并按0-1-2和0-2-3拆分为两个三角形。
    void appendQuad(const VoxelSurfaceVertex& vertex0, const VoxelSurfaceVertex& vertex1, const VoxelSurfaceVertex& vertex2, const VoxelSurfaceVertex& vertex3);

private:
    std::vector<VoxelSurfaceVertex> m_vertices; // 当前网格的全部局部空间顶点。
    std::vector<std::uint32_t> m_indices; // 当前网格的全部三角形索引。
};

}

#endif // MYVOXEL_SURFACE_VOXELSURFACEMESH_H