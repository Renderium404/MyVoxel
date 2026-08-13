#include "Display_ObjectUpdate.h"

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

}

namespace MyVoxel
{

Display_ObjectPartUpdate::Display_ObjectPartUpdate()
    : partId(0)
    , version(0)
    , operation(Display_ObjectPartOperation::Remove)
    , resourceId(0)
{
}

Display_ObjectPartUpdate Display_ObjectPartUpdate::replacement(Display_ObjectPartId partId, std::uint64_t version,
                                                               Display_ResourceId resourceId,
                                                               const Foundation::RefPtr<const Display_Resource>& resource)
{
    Display_ObjectPartUpdate result;
    result.partId = partId;
    result.version = version;
    result.operation = Display_ObjectPartOperation::Replace;
    result.resourceId = resourceId;
    result.resource = resource;
    return result;
}

Display_ObjectPartUpdate Display_ObjectPartUpdate::removal(Display_ObjectPartId partId, std::uint64_t version)
{
    Display_ObjectPartUpdate result;
    result.partId = partId;
    result.version = version;
    result.operation = Display_ObjectPartOperation::Remove;
    return result;
}

bool Display_ObjectPartUpdate::isValid() const
{
    if (version == 0)
    {
        return false;
    }

    if (operation == Display_ObjectPartOperation::Replace)
    {
        return resourceId != 0 && resource && resource->isValid();
    }

    return operation == Display_ObjectPartOperation::Remove && resourceId == 0 && !resource;
}

Display_ObjectPartsUpdate::Display_ObjectPartsUpdate()
    : objectId(0)
    , resourceKind(Display_ResourceKind::Mesh)
{
}

bool Display_ObjectPartsUpdate::isValid() const
{
    if (objectId == 0 || parts.empty())
    {
        return false;
    }

    std::set<Display_ObjectPartId> partIds;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (!parts[index].isValid() || !partIds.insert(parts[index].partId).second)
        {
            return false;
        }

        if (parts[index].operation == Display_ObjectPartOperation::Replace &&
            parts[index].resource->kind() != resourceKind)
        {
            return false;
        }
    }

    return true;
}

Display_ObjectStateUpdate::Display_ObjectStateUpdate()
    : objectId(0)
    , stateVersion(0)
    , hasUsage(false)
    , usage(Display_ObjectUsage::Static)
    , hasLocalToWorld(false)
    , localToWorld(MyMath::Matrix4::identity())
    , hasVisible(false)
    , visible(true)
    , hasLineWidth(false)
    , lineWidth(1.0f)
{
}

bool Display_ObjectStateUpdate::isValid() const
{
    if (objectId == 0 || stateVersion == 0 || (!hasUsage && !hasLocalToWorld && !hasVisible && !hasLineWidth))
    {
        return false;
    }

    if (hasLocalToWorld && !isInvertibleAffine(localToWorld))
    {
        return false;
    }

    return !hasLineWidth || (std::isfinite(static_cast<double>(lineWidth)) && lineWidth > 0.0f);
}

}