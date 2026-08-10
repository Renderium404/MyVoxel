#include "Display_Object.h"

#include <cmath>
#include <set>

#include "MyVoxel/Foundation/Diagnostic.h"

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

// 判断显示线宽是否为有效正有限值。
bool isValidLineWidth(float width)
{
    return std::isfinite(static_cast<double>(width)) && width > 0.0f;
}

}

namespace MyVoxel
{

Display_ObjectPart::Display_ObjectPart()
    : partId(0)
    , version(0)
    , resourceId(0)
{
}

Display_ObjectPart::Display_ObjectPart(Display_ObjectPartId partIdValue, std::uint64_t versionValue, Display_ResourceId resourceIdValue,
                                       const Foundation::RefPtr<const Display_Resource>& resourceValue)
    : partId(partIdValue)
    , version(versionValue)
    , resourceId(resourceIdValue)
    , resource(resourceValue)
{
}

bool Display_ObjectPart::isValid() const
{
    return version != 0 && resourceId != 0 && resource && resource->isValid();
}

Display_Object::Display_Object()
    : m_objectId(0)
    , m_resourceKind(Display_ResourceKind::Mesh)
    , m_usage(Display_ObjectUsage::Static)
    , m_localToWorld(MyMath::Matrix4::identity())
    , m_visible(true)
    , m_lineWidth(1.0f)
    , m_stateVersion(0)
{
}

Display_Object::Display_Object(Display_ObjectId objectId, Display_ResourceKind resourceKind, Display_ObjectUsage usage)
    : m_objectId(objectId)
    , m_resourceKind(resourceKind)
    , m_usage(usage)
    , m_localToWorld(MyMath::Matrix4::identity())
    , m_visible(true)
    , m_lineWidth(1.0f)
    , m_stateVersion(1)
{
    MYVOXEL_ASSERT_MESSAGE(objectId != 0, "Display object id must be non-zero.");
}

/// 状态判断

bool Display_Object::isValid() const
{
    if (m_objectId == 0 || m_stateVersion == 0 || !isInvertibleAffine(m_localToWorld))
    {
        return false;
    }

    if (m_resourceKind == Display_ResourceKind::Line && !isValidLineWidth(m_lineWidth))
    {
        return false;
    }

    for (PartMap::const_iterator iterator = m_parts.begin(); iterator != m_parts.end(); ++iterator)
    {
        if (!iterator->second.isValid() || iterator->second.partId != iterator->first ||
            iterator->second.resource->kind() != m_resourceKind)
        {
            return false;
        }

        const PartVersionMap::const_iterator versionIterator = m_partVersions.find(iterator->first);

        if (versionIterator == m_partVersions.end() || versionIterator->second != iterator->second.version)
        {
            return false;
        }
    }

    return true;
}

bool Display_Object::isNull() const
{
    return m_objectId == 0;
}

Display_Object::operator bool() const
{
    return isValid();
}

/// 对象身份

Display_ObjectId Display_Object::objectId() const
{
    return m_objectId;
}

Display_ResourceKind Display_Object::resourceKind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access resource kind of an invalid Display_Object.");
    return m_resourceKind;
}

std::uint64_t Display_Object::stateVersion() const
{
    return m_stateVersion;
}

/// 对象状态

Display_ObjectUsage Display_Object::usage() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access usage of an invalid Display_Object.");
    return m_usage;
}

const MyMath::Matrix4& Display_Object::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access transform of an invalid Display_Object.");
    return m_localToWorld;
}

bool Display_Object::visible() const
{
    return m_visible;
}

float Display_Object::lineWidth() const
{
    return m_lineWidth;
}
/// 资源分片

std::size_t Display_Object::partCount() const
{
    return m_parts.size();
}

bool Display_Object::hasPart(Display_ObjectPartId partId) const
{
    return m_parts.find(partId) != m_parts.end();
}

const Display_ObjectPart& Display_Object::part(Display_ObjectPartId partId) const
{
    const PartMap::const_iterator iterator = m_parts.find(partId);
    MYVOXEL_ASSERT_MESSAGE(iterator != m_parts.end(), "Display object part does not exist.");
    return iterator->second;
}

std::uint64_t Display_Object::partVersion(Display_ObjectPartId partId) const
{
    const PartVersionMap::const_iterator iterator = m_partVersions.find(partId);
    return iterator != m_partVersions.end() ? iterator->second : 0;
}

/// 空间范围

Bounds3 Display_Object::localBounds() const
{
    Bounds3 result;

    for (PartMap::const_iterator iterator = m_parts.begin(); iterator != m_parts.end(); ++iterator)
    {
        result.include(iterator->second.resource->localBounds());
    }

    return result;
}

Bounds3 Display_Object::worldBounds() const
{
    const Bounds3 bounds = localBounds();
    return bounds.isValid() ? bounds.transformed(m_localToWorld) : Bounds3();
}

}