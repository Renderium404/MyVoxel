#ifndef MYVOXEL_DISPLAY_LINE_DISPLAY_LINERESOURCE_H
#define MYVOXEL_DISPLAY_LINE_DISPLAY_LINERESOURCE_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Line/Display_LineTypes.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"

namespace MyVoxel
{

// 保存按GL_LINES语义排列的不可变CPU线显示资源，每两个连续顶点组成一个独立线段。
class Display_LineResource : public Foundation::ReferenceCounted
{
public:
    // 根据线段端点序列和统一颜色建立不可变线资源，points必须包含偶数个有限点且至少形成一个线段。
    Display_LineResource(const std::vector<MyMath::Vector3>& points, const Display_Color& color);

    /// 状态判断

    // 判断当前资源是否包含至少一个完整有效线段。
    bool isValid() const;

    /// 资源数据

    // 返回按线段顺序连续排列的显示顶点。
    const std::vector<Display_LineVertex>& vertices() const;
    // 返回显示顶点数量，恒等于线段数量乘以二。
    std::size_t vertexCount() const;
    // 返回独立线段数量。
    std::size_t segmentCount() const;
    // 返回全部线段形成的局部轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回连续显示顶点数组占用字节数，不包含vector预留空间和对象自身。
    std::size_t memoryByteSize() const;

protected:
    ~Display_LineResource() override = default;

private:
    std::vector<Display_LineVertex> m_vertices; // 可直接上传并按GL_LINES绘制的连续线顶点。
    Bounds3 m_localBounds; // 全部线段端点形成的局部轴对齐包围盒。
    std::size_t m_segmentCount; // 当前资源包含的独立线段数量。
    bool m_valid; // 当前资源是否已经完整建立。
};

}

#endif // MYVOXEL_DISPLAY_LINE_DISPLAY_LINERESOURCE_H
