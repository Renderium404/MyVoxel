#include "Display_LineSnapshot.h"

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

Display_LinePartSnapshot::Display_LinePartSnapshot()
    : partId(0)
    , version(0)
{
}

Display_LinePartSnapshot::Display_LinePartSnapshot(Display_LinePartId partIdValue, std::uint64_t versionValue,
                                                   const Foundation::RefPtr<const Display_LineResource>& resourceValue)
    : partId(partIdValue)
    , version(versionValue)
    , resource(resourceValue)
{
}

bool Display_LinePartSnapshot::isValid() const
{
    return version != 0 && resource && resource->isValid();
}

Display_LineObjectSnapshot::Display_LineObjectSnapshot()
    : objectId(0)
    , usage(Display_LineUsage::Static)
    , localToWorld(MyMath::Matrix4::identity())
    , visible(true)
    , width(1.0f)
{
}

bool Display_LineObjectSnapshot::isValid() const
{
    if (objectId == 0 || !isInvertibleAffine(localToWorld) || !std::isfinite(static_cast<double>(width)) || width <= 0.0f)
    {
        return false;
    }

    std::set<Display_LinePartId> partIds;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (!parts[index].isValid() || !partIds.insert(parts[index].partId).second)
        {
            return false;
        }
    }

    return true;
}

Bounds3 Display_LineObjectSnapshot::localBounds() const
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

std::size_t Display_LineObjectSnapshot::segmentCount() const
{
    std::size_t result = 0;

    for (std::size_t index = 0; index < parts.size(); ++index)
    {
        if (parts[index].isValid())
        {
            result += parts[index].resource->segmentCount();
        }
    }

    return result;
}

}
