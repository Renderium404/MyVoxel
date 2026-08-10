#include "Display_ObjectSnapshot.h"

#include <cmath>
#include <set>

namespace
{

// 判断矩阵是否为可逆仿射变换。
bool isInvertibleAffine(const MyMath::Matrix4& matrix)
{
    if (!matrix.isAffine())
    {
        return false;
    }

    MyMath::Matrix4 inverse;
    return matrix.inverted(inverse);
}

// 判断Line对象使用的显示线宽是否有效。
bool isValidLineWidth(float width)
{
    return std::isfinite(static_cast<double>(width)) && width > 0.0f;
}

}

namespace MyVoxel
{

Display_ObjectPartSnapshot::Display_ObjectPartSnapshot()
    : partId(0)
    , version(0)
    , resourceId(0)
{
}

Display_ObjectPartSnapshot::Display_ObjectPartSnapshot(Display_ObjectPartId partIdValue, std::uint64_t versionValue,
                                                       Display_ResourceId resourceIdValue,
                                                       const Foundation::RefPtr<const Display_Resource>& resourceValue)
    : partId(partIdValue)
    , version(versionValue)
    , resourceId(resourceIdValue)
    , resource(resourceValue)
{
}

bool Display_ObjectPartSnapshot::isValid() const
{
    return version != 0 && resourceId != 0 && resource && resource->isValid();
}

Display_ObjectSnapshot::Display_ObjectSnapshot()
    : objectId(0)
    , resourceKind(Display_ResourceKind::Mesh)
    , usage(Display_ObjectUsage::Static)
    , stateVersion(0)
    , localToWorld(MyMath::Matrix4::identity())
    , visible(true)
    , lineWidth(1.0f)
{
}

/// 状态判断

bool Display_ObjectSnapshot::isValid() const
{
    if (objectId == 0 || stateVersion == 0 || !isInvertibleAffine(localToWorld) || !isValidLineWidth(lineWidth))
    {
        return false;
    }

    std::set<Display_ObjectPartId> partIds;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (!parts[index].isValid() || parts[index].resource->kind() != resourceKind || !partIds.insert(parts[index].partId).second)
        {
            return false;
        }
    }

    return true;
}

/// 空间与资源统计

Bounds3 Display_ObjectSnapshot::localBounds() const
{
    Bounds3 result;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (parts[index].isValid())
        {
            result.include(parts[index].resource->localBounds());
        }
    }

    return result;
}

Bounds3 Display_ObjectSnapshot::worldBounds() const
{
    const Bounds3 bounds = localBounds();
    return bounds.isValid() ? bounds.transformed(localToWorld) : Bounds3();
}

std::size_t Display_ObjectSnapshot::memoryByteSize() const
{
    std::size_t result = 0;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (parts[index].isValid())
        {
            result += parts[index].resource->memoryByteSize();
        }
    }

    return result;
}

}