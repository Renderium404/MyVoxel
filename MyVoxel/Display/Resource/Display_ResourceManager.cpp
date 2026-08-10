#include "Display_ResourceManager.h"

#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Mesh/Mesh.h"

namespace MyVoxel
{

Display_ResourceManager::Display_ResourceManager()
    : m_nextResourceId(1)
{
}

Display_ResourceManager::~Display_ResourceManager()
{
}

/// 资源创建

Display_ResourceId Display_ResourceManager::createMeshResource(const Mesh& mesh)
{
    const Foundation::RefPtr<Display_MeshResource> resourceValue = Foundation::makeRef<Display_MeshResource>(mesh);

    if (!resourceValue || !resourceValue->isValid())
    {
        return 0;
    }

    const Foundation::RefPtr<const Display_Resource> resourceBase = resourceValue;
    return registerResource(resourceBase);
}

Display_ResourceId Display_ResourceManager::createLineResource(const std::vector<MyMath::Vector3>& points, const Display_Color& color)
{
    const Foundation::RefPtr<Display_LineResource> resourceValue = Foundation::makeRef<Display_LineResource>(points, color);

    if (!resourceValue || !resourceValue->isValid())
    {
        return 0;
    }

    const Foundation::RefPtr<const Display_Resource> resourceBase = resourceValue;
    return registerResource(resourceBase);
}

Display_ResourceId Display_ResourceManager::registerResource(const Foundation::RefPtr<const Display_Resource>& resourceValue)
{
    MYVOXEL_ASSERT_MESSAGE(resourceValue && resourceValue->isValid(), "Display resource manager requires a valid resource.");

    if (!resourceValue || !resourceValue->isValid())
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    const Display_ResourceId resourceId = allocateResourceId();
    const std::pair<ResourceMap::iterator, bool> inserted = m_resources.insert(std::make_pair(resourceId, resourceValue));
    MYVOXEL_REQUIRE_MESSAGE(inserted.second, "Display resource manager failed to insert a newly allocated resource id.");
    return inserted.second ? resourceId : 0;
}

/// 资源查询

bool Display_ResourceManager::contains(Display_ResourceId resourceId) const
{
    if (resourceId == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_resources.find(resourceId) != m_resources.end();
}

Foundation::RefPtr<const Display_Resource> Display_ResourceManager::resource(Display_ResourceId resourceId) const
{
    if (resourceId == 0)
    {
        return Foundation::RefPtr<const Display_Resource>();
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    const ResourceMap::const_iterator iterator = m_resources.find(resourceId);
    return iterator != m_resources.end() ? iterator->second : Foundation::RefPtr<const Display_Resource>();
}

Foundation::RefPtr<const Display_MeshResource> Display_ResourceManager::meshResource(Display_ResourceId resourceId) const
{
    const Foundation::RefPtr<const Display_Resource> resourceValue = resource(resourceId);

    if (!resourceValue || resourceValue->kind() != Display_ResourceKind::Mesh)
    {
        return Foundation::RefPtr<const Display_MeshResource>();
    }

    return Foundation::RefPtr<const Display_MeshResource>(static_cast<const Display_MeshResource*>(resourceValue.get()));
}

Foundation::RefPtr<const Display_LineResource> Display_ResourceManager::lineResource(Display_ResourceId resourceId) const
{
    const Foundation::RefPtr<const Display_Resource> resourceValue = resource(resourceId);

    if (!resourceValue || resourceValue->kind() != Display_ResourceKind::Line)
    {
        return Foundation::RefPtr<const Display_LineResource>();
    }

    return Foundation::RefPtr<const Display_LineResource>(static_cast<const Display_LineResource*>(resourceValue.get()));
}

/// 资源释放

bool Display_ResourceManager::remove(Display_ResourceId resourceId)
{
    if (resourceId == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_resources.erase(resourceId) != 0;
}

void Display_ResourceManager::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_resources.clear();
}

/// 资源统计

std::size_t Display_ResourceManager::resourceCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_resources.size();
}

std::size_t Display_ResourceManager::memoryByteSize() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::size_t result = 0;

    for (ResourceMap::const_iterator iterator = m_resources.begin(); iterator != m_resources.end(); ++iterator)
    {
        result += iterator->second->memoryByteSize();
    }

    return result;
}

/// 内部辅助

Display_ResourceId Display_ResourceManager::allocateResourceId()
{
    MYVOXEL_REQUIRE_MESSAGE(m_nextResourceId != 0, "Display resource id space has been exhausted.");

    const Display_ResourceId resourceId = m_nextResourceId;

    if (m_nextResourceId == (std::numeric_limits<Display_ResourceId>::max)())
    {
        m_nextResourceId = 0;
    }
    else
    {
        ++m_nextResourceId;
    }

    return resourceId;
}

}