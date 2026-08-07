#ifndef MYVOXEL_DISPLAY_MESH_DISPLAY_MESHUPDATE_H
#define MYVOXEL_DISPLAY_MESH_DISPLAY_MESHUPDATE_H

#include <cstdint>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

// 保存一个对象分片的替换或删除操作。
struct Display_MeshPartUpdate
{
    Display_MeshPartUpdate();

    // 创建指定分片的资源替换操作。
    static Display_MeshPartUpdate replacement(Display_MeshPartId partId, std::uint64_t version,
                                              const Foundation::RefPtr<const Display_MeshResource>& resource);
    // 创建指定分片的删除操作。
    static Display_MeshPartUpdate removal(Display_MeshPartId partId, std::uint64_t version);

    // 判断操作、版本和可选资源是否匹配。
    bool isValid() const;

    Display_MeshPartId partId; // 对象内部稳定分片标识。
    std::uint64_t version; // 当前分片单调递增版本，零值无效。
    Display_MeshPartOperation operation; // 替换或删除操作。
    Foundation::RefPtr<const Display_MeshResource> resource; // 替换操作使用的不可变资源，删除操作必须为空。
};

// 保存一个显示对象的网格分片增量更新。
struct Display_MeshUpdate
{
    Display_MeshUpdate();

    // 判断对象标识、全部操作和分片唯一性是否有效。
    bool isValid() const;

    Display_MeshObjectId objectId; // 目标显示对象标识，零值无效。
    std::vector<Display_MeshPartUpdate> parts; // 当前批次最终分片操作，同一分片不得重复。
};

// 保存显示对象变换和可见状态的可选增量更新。
struct Display_MeshStateUpdate
{
    Display_MeshStateUpdate();

    // 判断对象标识、可选变换和至少一项状态修改是否有效。
    bool isValid() const;

    Display_MeshObjectId objectId; // 目标显示对象标识，零值无效。
    bool hasLocalToWorld; // 是否更新对象局部到世界变换。
    MyMath::Matrix4 localToWorld; // 新的可逆仿射变换。
    bool hasVisible; // 是否更新对象可见状态。
    bool visible; // 新的对象可见状态。
};

}

#endif // MYVOXEL_DISPLAY_MESH_DISPLAY_MESHUPDATE_H
