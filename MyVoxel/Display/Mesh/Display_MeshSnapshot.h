#ifndef MYVOXEL_DISPLAY_MESH_DISPLAY_MESHSNAPSHOT_H
#define MYVOXEL_DISPLAY_MESH_DISPLAY_MESHSNAPSHOT_H

#include <cstdint>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

// 保存完整对象快照中的一个不可变非空网格分片。
struct Display_MeshPartSnapshot
{
    Display_MeshPartSnapshot();
    Display_MeshPartSnapshot(Display_MeshPartId partIdValue, std::uint64_t versionValue,
                             const Foundation::RefPtr<const Display_MeshResource>& resourceValue);

    // 判断分片标识、版本和显示资源是否完整有效。
    bool isValid() const;

    Display_MeshPartId partId; // 对象内部稳定分片标识，零值允许作为普通对象唯一分片。
    std::uint64_t version; // 当前分片单调递增版本，零值无效。
    Foundation::RefPtr<const Display_MeshResource> resource; // 当前分片不可变CPU显示资源。
};

// 保存完整替换一个显示网格对象所需的前端快照。
struct Display_MeshObjectSnapshot
{
    Display_MeshObjectSnapshot();

    // 判断对象标识、变换和全部非重复分片是否有效。
    bool isValid() const;
    // 返回全部非空分片形成的局部轴对齐包围盒，没有分片时返回无效包围盒。
    Bounds3 localBounds() const;
    // 返回全部分片三角形数量。
    std::size_t triangleCount() const;

    Display_MeshObjectId objectId; // 全局显示对象标识，零值无效。
    Display_MeshUsage usage; // 当前对象GPU缓冲的预期更新频率。
    MyMath::Matrix4 localToWorld; // 对象局部空间到显示世界空间的可逆仿射变换。
    bool visible; // 当前对象是否参与绘制。
    std::vector<Display_MeshPartSnapshot> parts; // 当前对象全部非空分片。
};

}

#endif // MYVOXEL_DISPLAY_MESH_DISPLAY_MESHSNAPSHOT_H
