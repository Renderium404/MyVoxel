#ifndef MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECT_H
#define MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECT_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>
#include "MyMath/Matrix4.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Display/Object/Display_ObjectTypes.h"
#include "MyVoxel/Display/Resource/Display_Resource.h"
#include "MyVoxel/Display/Resource/Display_ResourceTypes.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

class Display_ObjectManager;

// 表示显示对象当前使用的一个不可变资源分片。
//
// resourceId标识不可变资源身份，resource负责保证资源实际生命周期；
// version由Display_ObjectManager管理，每次替换或删除该partId时单调递增。
struct Display_ObjectPart
{
    Display_ObjectPart();
    Display_ObjectPart(Display_ObjectPartId partIdValue, std::uint64_t versionValue, Display_ResourceId resourceIdValue,
                       const Foundation::RefPtr<const Display_Resource>& resourceValue);

    // 判断分片版本、资源标识和不可变资源是否完整有效。
    bool isValid() const;

    Display_ObjectPartId partId; // 对象内部稳定分片标识，零值允许作为普通对象唯一分片。
    std::uint64_t version; // 当前分片单调递增版本，零值无效。
    Display_ResourceId resourceId; // 当前不可变CPU显示资源的全局资源标识，零值无效。
    Foundation::RefPtr<const Display_Resource> resource; // 当前分片实际持有的不可变CPU资源。
};

// 表示一个同类型显示资源集合在世界空间中的逻辑显示对象。
//
// Display_Object只保存CPU侧场景状态，不保存GPU句柄。
// 一个对象内部的全部资源必须具有相同Display_ResourceKind。
// Display_Object由Display_ObjectManager创建和修改，对外作为可复制只读状态使用。
class Display_Object
{
public:
    // 构造无效空显示对象。
    Display_Object();
    Display_Object(const Display_Object&) = default;
    Display_Object& operator=(const Display_Object&) = default;

    /// 状态判断

    // 判断对象标识、资源类型、状态版本、变换和全部资源分片是否完整有效。
    bool isValid() const;
    // 判断当前对象是否为无效空对象。
    bool isNull() const;
    // 判断当前对象是否完整有效。
    explicit operator bool() const;

    /// 对象身份

    // 返回全局显示对象标识。
    Display_ObjectId objectId() const;
    // 返回当前对象允许使用的统一显示资源类型。
    Display_ResourceKind resourceKind() const;
    // 返回当前对象状态版本。
    std::uint64_t stateVersion() const;

    /// 对象状态

    // 返回对象资源的预期GPU更新频率。
    Display_ObjectUsage usage() const;
    // 返回对象局部空间到显示世界空间的可逆仿射变换。
    const MyMath::Matrix4& localToWorld() const;
    // 判断当前对象是否参与绘制。
    bool visible() const;
    // 返回Line资源使用的期望显示线宽，其他资源类型忽略该值。
    float lineWidth() const;
    /// 资源分片

    // 返回当前活动资源分片数量。
    std::size_t partCount() const;
    // 判断指定分片当前是否存在。
    bool hasPart(Display_ObjectPartId partId) const;
    // 返回指定活动分片，调用者必须保证分片存在。
    const Display_ObjectPart& part(Display_ObjectPartId partId) const;
    // 返回指定分片最后一次替换或删除后的版本，从未使用过该partId时返回零。
    std::uint64_t partVersion(Display_ObjectPartId partId) const;

    /// 空间范围

    // 返回当前全部资源分片形成的局部轴对齐包围盒，没有分片时返回无效包围盒。
    Bounds3 localBounds() const;
    // 返回当前全部资源分片经对象变换后的世界轴对齐包围盒，没有分片时返回无效包围盒。
    Bounds3 worldBounds() const;

private:
    typedef std::map<Display_ObjectPartId, Display_ObjectPart> PartMap;
    typedef std::map<Display_ObjectPartId, std::uint64_t> PartVersionMap;

    // 由Display_ObjectManager使用指定身份和资源类型创建有效空对象。
    Display_Object(Display_ObjectId objectId, Display_ResourceKind resourceKind, Display_ObjectUsage usage);

private:
    Display_ObjectId m_objectId; // 全局显示对象身份。
    Display_ResourceKind m_resourceKind; // 当前对象全部分片必须使用的统一资源类型。
    Display_ObjectUsage m_usage; // 当前对象GPU资源预期更新频率。
    MyMath::Matrix4 m_localToWorld; // 当前对象局部空间到世界空间的可逆仿射变换。
    bool m_visible; // 当前对象是否参与绘制。
    float m_lineWidth; // Line资源使用的期望显示线宽，其他类型忽略。
    std::uint64_t m_stateVersion; // 对象状态单调递增版本，零值表示无效对象。
    PartMap m_parts; // 当前存在的全部不可变资源分片。
    PartVersionMap m_partVersions; // 所有曾使用partId的最后版本，包括已经删除的分片。

    friend class Display_ObjectManager;
};

}

#endif // MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECT_H