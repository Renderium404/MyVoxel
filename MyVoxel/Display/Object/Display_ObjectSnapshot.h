#ifndef MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTSNAPSHOT_H
#define MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTSNAPSHOT_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Object/Display_ObjectTypes.h"
#include "MyVoxel/Display/Resource/Display_Resource.h"
#include "MyVoxel/Display/Resource/Display_ResourceTypes.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

// 保存完整显示对象快照中的一个不可变资源分片。
struct Display_ObjectPartSnapshot
{
    Display_ObjectPartSnapshot();
    Display_ObjectPartSnapshot(Display_ObjectPartId partIdValue, std::uint64_t versionValue, Display_ResourceId resourceIdValue,
                               const Foundation::RefPtr<const Display_Resource>& resourceValue);

    // 判断分片标识、版本和资源引用是否完整有效。
    bool isValid() const;

    Display_ObjectPartId partId;                            // 对象内部稳定分片标识，零值允许作为普通对象唯一分片。
    std::uint64_t version;                                  // 当前分片单调递增版本，零值无效。
    Display_ResourceId resourceId;                          // 当前不可变显示资源标识，零值无效。
    Foundation::RefPtr<const Display_Resource> resource;    // 当前分片实际不可变CPU显示资源。
};

// 保存完整重建一个Display_Object所需的不可变前端快照。
struct Display_ObjectSnapshot
{
    Display_ObjectSnapshot();

    /// 状态判断

    // 判断对象身份、资源类型、状态、变换和全部非重复资源分片是否完整有效。
    bool isValid() const;

    /// 空间与资源统计

    // 返回全部资源分片形成的局部轴对齐包围盒，没有分片时返回无效包围盒。
    Bounds3 localBounds() const;
    // 返回全部资源分片形成的世界轴对齐包围盒，没有分片时返回无效包围盒。
    Bounds3 worldBounds() const;
    // 返回全部资源连续数据占用的字节数。
    std::size_t memoryByteSize() const;

    Display_ObjectId objectId;          // 全局显示对象标识，零值无效。
    Display_ResourceKind resourceKind;  // 当前对象统一资源类型。
    Display_ObjectUsage usage;          // GPU资源预期更新频率。
    std::uint64_t stateVersion;         // 当前对象状态版本，零值无效。
    MyMath::Matrix4 localToWorld;       // 对象局部空间到显示世界空间的可逆仿射变换。
    bool visible;                       // 当前对象是否参与绘制。
    float lineWidth;                    // Line对象期望线宽，其他资源类型忽略。
    std::vector<Display_ObjectPartSnapshot> parts; // 当前全部活动资源分片。
};

}

#endif // MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTSNAPSHOT_H