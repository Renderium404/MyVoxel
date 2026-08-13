#ifndef MYVOXEL_DISPLAY_MESH_DISPLAY_MESHRESOURCE_H
#define MYVOXEL_DISPLAY_MESH_DISPLAY_MESHRESOURCE_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Mesh/Display_MeshTypes.h"
#include "MyVoxel/Display/Resource/Display_Resource.h"

namespace MyVoxel
{

class Mesh;

// 指定索引Mesh展开为逐角点显示顶点时的法线处理方式。
enum class Display_MeshNormalMode
{
    SourceNormals, // 完全保留Mesh已有逐顶点显示法线。
    CreaseAware // 根据三角面夹角建立平滑组，在锐边两侧生成独立逐角点显示法线。
};

// 控制Display_MeshResource建立时的显示法线策略。
struct Display_MeshResourceOptions
{
    Display_MeshResourceOptions();

    Display_MeshNormalMode normalMode; // 默认保留源Mesh法线。
    double creaseAngleDegrees; // CreaseAware模式下达到该夹角的相邻三角面断开平滑组。
};

// 保存已经展开为逐角点顶点的不可变CPU显示网格资源。
//
// 创建时将索引Mesh展开为可直接上传的连续显示顶点，显示后端不再依赖Mesh、法线布局或逐三角形颜色布局。
// 当前资源要求源Mesh非空且具有完整单位法线和逐三角形颜色。
// CreaseAware模式只改变显示法线，不改变Mesh几何、索引、颜色或局部包围盒。
class Display_MeshResource : public Display_Resource
{
public:
    // 根据可渲染三角网格建立不可变显示资源，并完全保留源Mesh法线。
    explicit Display_MeshResource(const Mesh& mesh);
    // 根据可渲染三角网格和指定显示法线策略建立不可变显示资源。
    Display_MeshResource(const Mesh& mesh, const Display_MeshResourceOptions& options);

    /// 资源属性

    // 返回网格显示资源类型。
    Display_ResourceKind kind() const override;
    // 判断当前资源是否已经根据可渲染非空Mesh完整建立。
    bool isValid() const override;
    // 返回当前资源局部轴对齐包围盒。
    const Bounds3& localBounds() const override;
    // 返回当前连续显示顶点数组占用的字节数，不包含vector预留空间和对象自身开销。
    std::size_t memoryByteSize() const override;

    /// 网格数据

    // 返回按三角形顺序连续排列的逐角点显示顶点。
    const std::vector<Display_MeshVertex>& vertices() const;
    // 返回显示顶点数量，恒等于三角形数量乘以三。
    std::size_t vertexCount() const;
    // 返回当前资源三角形数量。
    std::size_t triangleCount() const;

protected:
    ~Display_MeshResource() override = default;

private:
    // 根据源Mesh和显示选项建立完整逐角点资源。
    void build(const Mesh& mesh, const Display_MeshResourceOptions& options);

private:
    std::vector<Display_MeshVertex> m_vertices; // 可直接上传的连续逐角点显示顶点。
    Bounds3 m_localBounds; // 源Mesh实际三角形形成的局部轴对齐包围盒。
    std::size_t m_triangleCount; // 当前资源包含的三角形数量。
    bool m_valid; // 当前资源是否已经完整建立。
};

}

#endif // MYVOXEL_DISPLAY_MESH_DISPLAY_MESHRESOURCE_H