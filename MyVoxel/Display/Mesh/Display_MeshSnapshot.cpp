#include "Display_MeshSnapshot.h"

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

Display_MeshPartSnapshot::Display_MeshPartSnapshot()
    : partId(0)
    , version(0)
{
}

Display_MeshPartSnapshot::Display_MeshPartSnapshot(Display_MeshPartId partIdValue, std::uint64_t versionValue,
                                                   const Foundation::RefPtr<const Display_MeshResource>& resourceValue)
    : partId(partIdValue)
    , version(versionValue)
    , resource(resourceValue)
{
}

bool Display_MeshPartSnapshot::isValid() const
{
    return version != 0 && resource && resource->isValid();
}

Display_MeshObjectSnapshot::Display_MeshObjectSnapshot()
    : objectId(0)
    , usage(Display_MeshUsage::Static)
    , localToWorld(MyMath::Matrix4::identity())
    , visible(true)
{
}

bool Display_MeshObjectSnapshot::isValid() const
{
    if (objectId == 0 || !isInvertibleAffine(localToWorld))
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

Bounds3 Display_MeshObjectSnapshot::localBounds() const
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

std::size_t Display_MeshObjectSnapshot::triangleCount() const
{
    std::size_t result = 0;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (parts[index].isValid())
        {
            result += parts[index].resource->triangleCount();
        }
    }

    return result;
}

}
