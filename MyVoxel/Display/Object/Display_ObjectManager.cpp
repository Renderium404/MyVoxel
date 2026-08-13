#include "Display_ObjectManager.h"

#include <cmath>
#include <limits>
#include <utility>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Display_ObjectManager::Display_ObjectManager(const Display_ResourceManager& resourceManager)
    : m_resourceManager(&resourceManager)
    , m_nextObjectId(1)
{
}

Display_ObjectManager::~Display_ObjectManager()
{
}

/// 对象创建

Display_ObjectId Display_ObjectManager::createObject(Display_ResourceKind resourceKind, Display_ObjectUsage usage)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    const Display_ObjectId objectId = allocateObjectId();
    const std::pair<ObjectMap::iterator, bool> inserted =
        m_objects.insert(std::make_pair(objectId, Display_Object(objectId, resourceKind, usage)));

    MYVOXEL_REQUIRE_MESSAGE(inserted.second, "Display object manager failed to insert a newly allocated object id.");
    return inserted.second ? objectId : 0;
}

Display_ObjectId Display_ObjectManager::createObject(Display_ResourceId resourceId, Display_ObjectUsage usage, Display_ObjectPartId partId)
{
    const Foundation::RefPtr<const Display_Resource> resourceValue = m_resourceManager->resource(resourceId);

    if (!resourceValue || !resourceValue->isValid())
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    const Display_ObjectId objectId = allocateObjectId();
    Display_Object objectValue(objectId, resourceValue->kind(), usage);

    objectValue.m_partVersions[partId] = 1;
    objectValue.m_parts.insert(std::make_pair(partId, Display_ObjectPart(partId, 1, resourceId, resourceValue)));

    const std::pair<ObjectMap::iterator, bool> inserted = m_objects.insert(std::make_pair(objectId, objectValue));
    MYVOXEL_REQUIRE_MESSAGE(inserted.second, "Display object manager failed to insert a newly allocated object id.");
    return inserted.second ? objectId : 0;
}

/// 对象查询

bool Display_ObjectManager::contains(Display_ObjectId objectId) const
{
    if (objectId == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_objects.find(objectId) != m_objects.end();
}

Display_Object Display_ObjectManager::object(Display_ObjectId objectId) const
{
    if (objectId == 0)
    {
        return Display_Object();
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    const ObjectMap::const_iterator iterator = m_objects.find(objectId);

    if (iterator == m_objects.end())
    {
        return Display_Object();
    }

    return iterator->second;
}

/// 对象状态

bool Display_ObjectManager::setUsage(Display_ObjectId objectId, Display_ObjectUsage usage)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    ObjectMap::iterator iterator = m_objects.find(objectId);

    if (iterator == m_objects.end())
    {
        return false;
    }

    Display_Object& objectValue = iterator->second;

    if (objectValue.m_usage == usage)
    {
        return true;
    }

    objectValue.m_usage = usage;
    objectValue.m_stateVersion = nextVersion(objectValue.m_stateVersion);
    return true;
}

bool Display_ObjectManager::setLocalToWorld(Display_ObjectId objectId, const MyMath::Matrix4& localToWorld)
{
    if (!isValidTransform(localToWorld))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    ObjectMap::iterator iterator = m_objects.find(objectId);

    if (iterator == m_objects.end())
    {
        return false;
    }

    Display_Object& objectValue = iterator->second;

    if (objectValue.m_localToWorld.isEqualTo(localToWorld, 0.001))
    {
        return true;
    }

    objectValue.m_localToWorld = localToWorld;
    objectValue.m_stateVersion = nextVersion(objectValue.m_stateVersion);
    return true;
}

bool Display_ObjectManager::setVisible(Display_ObjectId objectId, bool visible)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    ObjectMap::iterator iterator = m_objects.find(objectId);

    if (iterator == m_objects.end())
    {
        return false;
    }

    Display_Object& objectValue = iterator->second;

    if (objectValue.m_visible == visible)
    {
        return true;
    }

    objectValue.m_visible = visible;
    objectValue.m_stateVersion = nextVersion(objectValue.m_stateVersion);
    return true;
}

bool Display_ObjectManager::setLineWidth(Display_ObjectId objectId, double width)
{
    if (!std::isfinite(width) || width <= 0.0)
    {
        return false;
    }

    const float targetWidth = static_cast<float>(width);

    if (!std::isfinite(static_cast<double>(targetWidth)) || targetWidth <= 0.0f)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    ObjectMap::iterator iterator = m_objects.find(objectId);

    if (iterator == m_objects.end() || iterator->second.m_resourceKind != Display_ResourceKind::Line)
    {
        return false;
    }

    Display_Object& objectValue = iterator->second;

    if (objectValue.m_lineWidth == targetWidth)
    {
        return true;
    }

    objectValue.m_lineWidth = targetWidth;
    objectValue.m_stateVersion = nextVersion(objectValue.m_stateVersion);
    return true;
}

/// 资源分片

bool Display_ObjectManager::setPart(Display_ObjectId objectId, Display_ObjectPartId partId, Display_ResourceId resourceId)
{
    const Foundation::RefPtr<const Display_Resource> resourceValue = m_resourceManager->resource(resourceId);

    if (!resourceValue || !resourceValue->isValid())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    ObjectMap::iterator objectIterator = m_objects.find(objectId);

    if (objectIterator == m_objects.end())
    {
        return false;
    }

    Display_Object& objectValue = objectIterator->second;

    if (objectValue.m_resourceKind != resourceValue->kind())
    {
        return false;
    }

    Display_Object::PartMap::iterator partIterator = objectValue.m_parts.find(partId);

    if (partIterator != objectValue.m_parts.end() && partIterator->second.resourceId == resourceId)
    {
        return true;
    }

    const std::uint64_t version = nextVersion(objectValue.partVersion(partId));

    objectValue.m_partVersions[partId] = version;
    objectValue.m_parts[partId] = Display_ObjectPart(partId, version, resourceId, resourceValue);
    return true;
}

bool Display_ObjectManager::removePart(Display_ObjectId objectId, Display_ObjectPartId partId)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    ObjectMap::iterator objectIterator = m_objects.find(objectId);

    if (objectIterator == m_objects.end())
    {
        return false;
    }

    Display_Object& objectValue = objectIterator->second;
    Display_Object::PartMap::iterator partIterator = objectValue.m_parts.find(partId);

    if (partIterator == objectValue.m_parts.end())
    {
        return false;
    }

    objectValue.m_partVersions[partId] = nextVersion(objectValue.partVersion(partId));
    objectValue.m_parts.erase(partIterator);
    return true;
}

bool Display_ObjectManager::clearParts(Display_ObjectId objectId)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    ObjectMap::iterator objectIterator = m_objects.find(objectId);

    if (objectIterator == m_objects.end())
    {
        return false;
    }

    Display_Object& objectValue = objectIterator->second;

    for (Display_Object::PartMap::const_iterator iterator = objectValue.m_parts.begin(); iterator != objectValue.m_parts.end(); ++iterator)
    {
        objectValue.m_partVersions[iterator->first] = nextVersion(objectValue.partVersion(iterator->first));
    }

    objectValue.m_parts.clear();
    return true;
}

/// 对象释放

bool Display_ObjectManager::remove(Display_ObjectId objectId)
{
    if (objectId == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_objects.erase(objectId) != 0;
}

void Display_ObjectManager::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_objects.clear();
}

/// 对象统计

std::size_t Display_ObjectManager::objectCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_objects.size();
}

std::size_t Display_ObjectManager::partCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::size_t result = 0;

    for (ObjectMap::const_iterator iterator = m_objects.begin(); iterator != m_objects.end(); ++iterator)
    {
        result += iterator->second.m_parts.size();
    }

    return result;
}

/// 快照

Display_ObjectSnapshot Display_ObjectManager::snapshot(Display_ObjectId objectId) const
{
    if (objectId == 0)
    {
        return Display_ObjectSnapshot();
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    const ObjectMap::const_iterator objectIterator = m_objects.find(objectId);

    if (objectIterator == m_objects.end())
    {
        return Display_ObjectSnapshot();
    }

    const Display_Object& objectValue = objectIterator->second;

    Display_ObjectSnapshot result;
    result.objectId = objectValue.m_objectId;
    result.resourceKind = objectValue.m_resourceKind;
    result.usage = objectValue.m_usage;
    result.stateVersion = objectValue.m_stateVersion;
    result.localToWorld = objectValue.m_localToWorld;
    result.visible = objectValue.m_visible;
    result.lineWidth = objectValue.m_lineWidth;
    result.parts.reserve(objectValue.m_parts.size());

    for (Display_Object::PartMap::const_iterator iterator = objectValue.m_parts.begin(); iterator != objectValue.m_parts.end(); ++iterator)
    {
        const Display_ObjectPart& partValue = iterator->second;
        result.parts.push_back(Display_ObjectPartSnapshot(partValue.partId, partValue.version, partValue.resourceId, partValue.resource));
    }

    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "Display object manager produced an invalid object snapshot.");
    return result;
}

std::vector<Display_ObjectSnapshot> Display_ObjectManager::snapshots() const
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<Display_ObjectSnapshot> result;
    result.reserve(m_objects.size());

    for (ObjectMap::const_iterator objectIterator = m_objects.begin(); objectIterator != m_objects.end(); ++objectIterator)
    {
        const Display_Object& objectValue = objectIterator->second;

        Display_ObjectSnapshot snapshotValue;
        snapshotValue.objectId = objectValue.m_objectId;
        snapshotValue.resourceKind = objectValue.m_resourceKind;
        snapshotValue.usage = objectValue.m_usage;
        snapshotValue.stateVersion = objectValue.m_stateVersion;
        snapshotValue.localToWorld = objectValue.m_localToWorld;
        snapshotValue.visible = objectValue.m_visible;
        snapshotValue.lineWidth = objectValue.m_lineWidth;
        snapshotValue.parts.reserve(objectValue.m_parts.size());

        for (Display_Object::PartMap::const_iterator partIterator = objectValue.m_parts.begin(); partIterator != objectValue.m_parts.end(); ++partIterator)
        {
            const Display_ObjectPart& partValue = partIterator->second;
            snapshotValue.parts.push_back(
                Display_ObjectPartSnapshot(partValue.partId, partValue.version, partValue.resourceId, partValue.resource));
        }

        MYVOXEL_ASSERT_MESSAGE(snapshotValue.isValid(), "Display object manager produced an invalid object snapshot.");
        result.push_back(snapshotValue);
    }

    return result;
}

/// 内部辅助

Display_ObjectId Display_ObjectManager::allocateObjectId()
{
    MYVOXEL_REQUIRE_MESSAGE(m_nextObjectId != 0, "Display object id space has been exhausted.");

    if (m_nextObjectId == 0)
    {
        return 0;
    }

    const Display_ObjectId objectId = m_nextObjectId;

    if (m_nextObjectId == (std::numeric_limits<Display_ObjectId>::max)())
    {
        m_nextObjectId = 0;
    }
    else
    {
        ++m_nextObjectId;
    }

    return objectId;
}

std::uint64_t Display_ObjectManager::nextVersion(std::uint64_t currentVersion)
{
    MYVOXEL_REQUIRE_MESSAGE(currentVersion != (std::numeric_limits<std::uint64_t>::max)(),
                            "Display object version space has been exhausted.");

    if (currentVersion == (std::numeric_limits<std::uint64_t>::max)())
    {
        return 0;
    }

    return currentVersion + 1;
}

bool Display_ObjectManager::isValidTransform(const MyMath::Matrix4& transform)
{
    if (!transform.isAffine())
    {
        return false;
    }

    MyMath::Matrix4 inverse;
    return transform.inverted(inverse);
}

}