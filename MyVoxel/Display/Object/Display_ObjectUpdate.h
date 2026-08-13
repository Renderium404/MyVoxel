#ifndef MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTUPDATE_H
#define MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTUPDATE_H

#include <cstdint>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Display/Object/Display_ObjectTypes.h"
#include "MyVoxel/Display/Resource/Display_Resource.h"
#include "MyVoxel/Display/Resource/Display_ResourceTypes.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

// 标识显示对象资源分片的替换或删除操作。
enum class Display_ObjectPartOperation
{
    Replace,
    Remove
};

// 保存一个Display_Object资源分片的增量修改。
struct Display_ObjectPartUpdate
{
    Display_ObjectPartUpdate();

    // 创建指定分片的不可变资源替换操作。
    static Display_ObjectPartUpdate replacement(Display_ObjectPartId partId, std::uint64_t version, Display_ResourceId resourceId,
                                                const Foundation::RefPtr<const Display_Resource>& resource);
    // 创建指定分片的删除操作。
    static Display_ObjectPartUpdate removal(Display_ObjectPartId partId, std::uint64_t version);

    // 判断操作、版本和可选资源数据是否完整匹配。
    bool isValid() const;

    Display_ObjectPartId partId; // 目标对象内部稳定分片标识。
    std::uint64_t version; // 当前分片新版本，零值无效。
    Display_ObjectPartOperation operation; // 当前分片替换或删除操作。
    Display_ResourceId resourceId; // Replace使用的新资源标识，Remove时必须为零。
    Foundation::RefPtr<const Display_Resource> resource; // Replace使用的新不可变资源，Remove时必须为空。
};

// 保存一个Display_Object的资源分片增量修改。
struct Display_ObjectPartsUpdate
{
    Display_ObjectPartsUpdate();

    // 判断对象标识、资源类型以及全部非重复分片操作是否有效。
    bool isValid() const;

    Display_ObjectId objectId; // 目标显示对象标识。
    Display_ResourceKind resourceKind; // 目标对象统一资源类型。
    std::vector<Display_ObjectPartUpdate> parts; // 当前批次全部最终分片修改。
};

// 保存Display_Object自身非资源状态的可选增量修改。
struct Display_ObjectStateUpdate
{
    Display_ObjectStateUpdate();

    // 判断对象标识、状态版本和至少一项状态修改是否有效。
    bool isValid() const;

    Display_ObjectId objectId; // 目标显示对象标识。
    std::uint64_t stateVersion; // 应用本次修改后的对象状态版本。

    bool hasUsage; // 是否修改GPU资源预期更新频率。
    Display_ObjectUsage usage; // 新Usage。

    bool hasLocalToWorld; // 是否修改对象空间变换。
    MyMath::Matrix4 localToWorld; // 新的可逆仿射局部到世界变换。

    bool hasVisible; // 是否修改对象可见性。
    bool visible; // 新可见状态。

    bool hasLineWidth; // 是否修改Line显示线宽。
    float lineWidth; // 新Line显示线宽。
};

}

#endif // MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTUPDATE_H