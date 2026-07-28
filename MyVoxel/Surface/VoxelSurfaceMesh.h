#ifndef MYVOXEL_VOXELSURFACEMESH_H
#define MYVOXEL_VOXELSURFACEMESH_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace MyVoxel
{

// 表面网格颜色。
struct VoxelSurfaceColor
{
    VoxelSurfaceColor();
    VoxelSurfaceColor(double redValue, double greenValue, double blueValue, double alphaValue = 1.0);

    float red = 1.0f; // 红色分量。
    float green = 1.0f; // 绿色分量。
    float blue = 1.0f; // 蓝色分量。
    float alpha = 1.0f; // 透明度分量。
};

// 表面网格顶点。
struct VoxelSurfaceVertex
{
    VoxelSurfaceVertex();
    VoxelSurfaceVertex(double xValue, double yValue, double zValue, double nxValue, double nyValue, double nzValue, const VoxelSurfaceColor& colorValue);

    float x = 0.0f; // 顶点X坐标。
    float y = 0.0f; // 顶点Y坐标。
    float z = 0.0f; // 顶点Z坐标。
    float nx = 0.0f; // 法线X分量。
    float ny = 0.0f; // 法线Y分量。
    float nz = 1.0f; // 法线Z分量。
    VoxelSurfaceColor color; // 顶点颜色。
};

// 体素外表面三角网格。
class VoxelSurfaceMesh
{
public:
    // 清空网格数据。
    void clear();

    // 预留指定数量的顶点和索引空间。
    void reserve(std::size_t vertexCount, std::size_t indexCount);

    // 返回网格是否为空。
    bool isEmpty() const;

    // 返回顶点数量。
    std::size_t vertexCount() const;

    // 返回索引数量。
    std::size_t indexCount() const;

    // 返回三角形数量。
    std::size_t triangleCount() const;

    // 返回顶点数组。
    const std::vector<VoxelSurfaceVertex>& vertices() const;

    // 返回索引数组。
    const std::vector<std::uint32_t>& indices() const;

    /// 网格追加

    // 追加另一个独立网格，并修正其全部索引偏移。
    void append(const VoxelSurfaceMesh& mesh);

    /// 顶点与三角形

    // 追加一个顶点并返回其索引。
    std::uint32_t appendVertex(const VoxelSurfaceVertex& vertex);

    // 使用三个已有顶点索引追加一个三角形。
    void appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2);

    // 追加三个独立顶点组成的三角形。
    void appendTriangle(const VoxelSurfaceVertex& vertex0, const VoxelSurfaceVertex& vertex1, const VoxelSurfaceVertex& vertex2);

    // 追加一个四边形面，内部拆分为两个三角形。
    void appendQuad(const VoxelSurfaceVertex& vertex0, const VoxelSurfaceVertex& vertex1, const VoxelSurfaceVertex& vertex2, const VoxelSurfaceVertex& vertex3);

private:
    std::vector<VoxelSurfaceVertex> m_vertices; // 网格顶点。
    std::vector<std::uint32_t> m_indices; // 三角形索引。
};

}

#endif // MYVOXEL_VOXELSURFACEMESH_H