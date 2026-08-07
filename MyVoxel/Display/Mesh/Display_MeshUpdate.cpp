#include "Display_MeshUpdate.h"

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

Display_MeshPartUpdate::Display_MeshPartUpdate()
    : partId(0)
    , version(0)
    , operation(Display_MeshPartOperation::Remove)
{
}

Display_MeshPartUpdate Display_MeshPartUpdate::replacement(Display_MeshPartId partId, std::uint64_t version,
                                                           const Foundation::RefPtr<const Display_MeshResource>& resource)
{
    Display_MeshPartUpdate result;
    result.partId = partId;
    result.version = version;
    result.operation = Display_MeshPartOperation::Replace;
    result.resource = resource;
    return result;
}

Display_MeshPartUpdate Display_MeshPartUpdate::removal(Display_MeshPartId partId, std::uint64_t version)
{
    Display_MeshPartUpdate result;
    result.partId = partId;
    result.version = version;
    result.operation = Display_MeshPartOperation::Remove;
    return result;
}

bool Display_MeshPartUpdate::isValid() const
{
    if (version == 0)
    {
        return false;
    }

    if (operation == Display_MeshPartOperation::Replace)
    {
        return resource && resource->isValid();
    }

    return operation == Display_MeshPartOperation::Remove && !resource;
}

Display_MeshUpdate::Display_MeshUpdate()
    : objectId(0)
{
}

bool Display_MeshUpdate::isValid() const
{
    if (objectId == 0 || parts.empty())
    {
        return false;
    }

    std::set<Display_MeshPartId> partIds;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (!parts[index].isValid() || !partIds.insert(parts[index].partId).second)
        {
            return false;
        }
    }

    return true;
}

Display_MeshStateUpdate::Display_MeshStateUpdate()
    : objectId(0)
    , hasLocalToWorld(false)
    , localToWorld(MyMath::Matrix4::identity())
    , hasVisible(false)
    , visible(true)
{
}

bool Display_MeshStateUpdate::isValid() const
{
    return objectId != 0 && (hasLocalToWorld || hasVisible) && (!hasLocalToWorld || isInvertibleAffine(localToWorld));
}

}
