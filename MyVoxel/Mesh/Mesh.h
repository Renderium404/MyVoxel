#ifndef MYVOXEL_MESH_MESH_H
#define MYVOXEL_MESH_MESH_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Base/Display_Normal.h"

namespace MyVoxel
{

// 表示三角网格中的一个局部空间顶点。
//
// 顶点位置是网格几何的必要数据，Display_Normal是可选显示属性。
// 零Display_Normal表示当前顶点没有可用显示法线。
struct MeshVertex
{
    MeshVertex();

    // 使用指定位置创建不包含有效显示法线的顶点。
    MeshVertex(double xValue, double yValue, double zValue);

    // 使用指定位置和显示法线创建顶点。
    MeshVertex(double xValue, double yValue, double zValue, const Display_Normal& normalValue);

    // 使用指定位置和显示法线分量创建顶点。
    MeshVertex(double xValue, double yValue, double zValue, double normalXValue, double normalYValue, double normalZValue);

    float x; // 局部空间X坐标。
    float y; // 局部空间Y坐标。
    float z; // 局部空间Z坐标。
    Display_Normal normal; // 可选局部显示法线，零向量表示没有有效法线。
};

// 保存局部空间索引三角网格及其可选显示属性。
//
// 每三个连续索引组成一个三角形。
// 顶点位置和三角形索引定义网格几何；Display_Normal和Display_Color不参与几何有效性判断。
// 顶点法线必须全部存在或全部不存在，三角形颜色也必须全部存在或全部不存在。
// 当前网格不保存实例变换、显示对象状态或GPU资源。
class Mesh
{
public:
    /// 网格状态

    // 清空全部顶点、索引和逐三角形颜色。
    void clear();
    // 为指定顶点数量和索引数量预留连续空间，indexCount必须是3的整数倍。
    void reserve(std::size_t vertexCount, std::size_t indexCount);
    // 在尚未写入三角形时，将顶点数组调整到指定大小。
    void resizeVertices(std::size_t vertexCount);
    // 判断当前网格是否没有任何三角形。
    bool isEmpty() const;
    // 检查位置、索引和三角形数量是否构成有效索引三角网格。
    bool isGeometryValid() const;
    // 检查几何和可选显示属性是否有效，法线与颜色必须分别全部存在或全部不存在。
    bool isValid() const;
    // 判断当前非空网格是否具有完整单位法线和完整逐三角形颜色，可直接建立Display_MeshResource。
    bool isRenderable() const;
    // 判断全部顶点是否具有有限且非零的有效显示法线。
    bool hasValidNormals(double epsilon = 1.0e-12) const;
    // 判断全部顶点显示法线是否在指定误差内为单位向量。
    bool hasUnitNormals(double epsilon = 1.0e-5) const;
    // 判断当前非空网格是否具有完整有效的逐三角形颜色。
    bool hasTriangleColors() const;
    // 判断当前网格是否包含二倍面积不大于指定阈值的退化三角形。
    bool hasDegenerateTriangles(double doubledAreaTolerance = 1.0e-12) const;
    // 返回二倍面积不大于指定阈值的退化三角形数量。
    std::size_t degenerateTriangleCount(double doubledAreaTolerance = 1.0e-12) const;

    /// 网格属性

    // 返回当前顶点数量。
    std::size_t vertexCount() const;
    // 返回当前索引数量。
    std::size_t indexCount() const;
    // 返回当前三角形数量。
    std::size_t triangleCount() const;
    // 返回当前实际使用的顶点、索引和颜色数据字节数，不包含vector预留空间和对象自身开销。
    std::size_t memoryByteSize() const;
    // 返回当前实际三角形使用顶点形成的局部轴对齐包围盒，空网格返回无效包围盒。
    Bounds3 localBounds() const;

    /// 网格数据

    // 返回只读顶点数组。
    const std::vector<MeshVertex>& vertices() const;
    // 返回只读三角形索引数组。
    const std::vector<std::uint32_t>& indices() const;
    // 返回与三角形一一对应的只读显示颜色数组，没有颜色时返回空数组。
    const std::vector<Display_Color>& triangleColors() const;
    // 返回指定三角形的显示颜色，当前网格必须具有完整逐三角形颜色。
    const Display_Color& triangleColor(std::size_t triangleIndex) const;

    /// 网格变换

    // 返回经过指定可逆仿射变换的新网格，不修改当前网格。
    Mesh transformed(const MyMath::Matrix4& transform) const;
    // 使用指定可逆仿射变换修改全部顶点和有效法线，镜像变换会同步修正三角形绕序。
    void transformInPlace(const MyMath::Matrix4& transform);

    /// 网格构建

    // 追加另一个独立网格，并修正其全部索引偏移。
    //
    // 当两个网格都包含三角形时，它们必须使用相同的法线和逐三角形颜色布局。
    void append(const Mesh& mesh);
    // 变换另一个网格后追加，并修正其全部索引偏移。
    void appendTransformed(const Mesh& mesh, const MyMath::Matrix4& transform);
    // 追加一个顶点并返回其索引。
    std::uint32_t appendVertex(const MeshVertex& vertex);
    // 覆盖指定索引处的已有顶点。
    void setVertex(std::uint32_t vertexIndex, const MeshVertex& vertex);
    // 使用三个已有顶点索引追加一个不包含颜色的三角形。
    void appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2);
    // 使用三个已有顶点索引追加一个指定显示颜色的三角形。
    void appendTriangle(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2, const Display_Color& color);
    // 追加三个独立顶点组成的不包含颜色三角形。
    void appendTriangle(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2);
    // 追加三个独立顶点组成的指定显示颜色三角形。
    void appendTriangle(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2, const Display_Color& color);
    // 追加一个不包含颜色的四边形，并按0-1-2和0-2-3拆分为两个三角形。
    void appendQuad(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2, const MeshVertex& vertex3);
    // 追加一个指定显示颜色的四边形，并按0-1-2和0-2-3拆分为两个三角形。
    void appendQuad(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2, const MeshVertex& vertex3,
                    const Display_Color& color);

private:
    // 检查三角形索引并追加到索引数组。
    void appendTriangleIndices(std::uint32_t index0, std::uint32_t index1, std::uint32_t index2);

private:
    std::vector<MeshVertex> m_vertices; // 当前网格的全部局部空间顶点。
    std::vector<std::uint32_t> m_indices; // 当前网格的全部三角形索引。
    std::vector<Display_Color> m_triangleColors; // 可选逐三角形显示颜色，必须为空或与三角形一一对应。
};

}

#endif // MYVOXEL_MESH_MESH_H
