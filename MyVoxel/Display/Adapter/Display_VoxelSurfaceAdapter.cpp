#include "Display_VoxelSurfaceAdapter.h"

#include <limits>
#include <set>
#include <stdexcept>

#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Mesh/Mesh.h"

namespace MyVoxel
{

Display_VoxelSurfaceAdapter::PartKey::PartKey()
    : direction(VoxelFaceDirection::NegativeX)
{
}

Display_VoxelSurfaceAdapter::PartKey::PartKey(const VoxelCellIndex& rootIndexValue,
                                              VoxelFaceDirection directionValue)
    : rootIndex(rootIndexValue)
    , direction(directionValue)
{
    MYVOXEL_ASSERT_MESSAGE(isValidVoxelFaceDirection(direction), "Display voxel surface part direction must be valid.");
}

bool Display_VoxelSurfaceAdapter::PartKey::operator<(const PartKey& other) const
{
    if (rootIndex != other.rootIndex)
    {
        return rootIndex < other.rootIndex;
    }

    return static_cast<unsigned int>(direction) < static_cast<unsigned int>(other.direction);
}

Display_VoxelSurfaceAdapter::PartState::PartState()
    : partId(0)
    , sourceVersion(0)
    , displayVersion(0)
    , active(false)
{
}

Display_VoxelSurfaceAdapter::Display_VoxelSurfaceAdapter(Display_MeshObjectId objectId)
    : m_objectId(objectId)
    , m_nextPartId(1)
    , m_initialized(false)
{
    MYVOXEL_ASSERT_MESSAGE(m_objectId != 0, "Display voxel surface adapter requires a non-zero object id.");
}

/// 状态管理

void Display_VoxelSurfaceAdapter::clear()
{
    m_nextPartId = 1;
    m_initialized = false;
    m_parts.clear();
}

bool Display_VoxelSurfaceAdapter::isInitialized() const
{
    return m_initialized;
}

Display_MeshObjectId Display_VoxelSurfaceAdapter::objectId() const
{
    return m_objectId;
}

std::size_t Display_VoxelSurfaceAdapter::partCount() const
{
    return m_parts.size();
}

std::size_t Display_VoxelSurfaceAdapter::activePartCount() const
{
    std::size_t count = 0;

    for (PartStateMap::const_iterator iterator = m_parts.begin(); iterator != m_parts.end(); ++iterator)
    {
        if (iterator->second.active)
        {
            ++count;
        }
    }

    return count;
}

Display_MeshPartId Display_VoxelSurfaceAdapter::partId(const VoxelCellIndex& rootIndex,
                                                       VoxelFaceDirection direction) const
{
    MYVOXEL_ASSERT_MESSAGE(isValidVoxelFaceDirection(direction), "Display voxel surface part query direction must be valid.");

    const PartStateMap::const_iterator iterator = m_parts.find(PartKey(rootIndex, direction));
    return iterator == m_parts.end() ? static_cast<Display_MeshPartId>(0) : iterator->second.partId;
}

/// 全量显示数据

Display_MeshObjectSnapshot Display_VoxelSurfaceAdapter::buildSnapshot(const VoxelSurfaceCache& cache,
                                                                      const MyMath::Matrix4& localToWorld,
                                                                      bool visible)
{
    MYVOXEL_ASSERT_MESSAGE(cache.isInitialized(), "Display voxel surface snapshot requires an initialized surface cache.");

    PartStateMap pendingStates = m_parts;
    Display_MeshPartId pendingNextPartId = m_nextPartId;
    std::set<PartKey> activeKeys;
    Display_MeshObjectSnapshot result;

    result.objectId = m_objectId;
    result.usage = Display_MeshUsage::Dynamic;
    result.localToWorld = localToWorld;
    result.visible = visible;

    const VoxelSurfaceCache::RootEntryMap& roots = cache.rootEntries();

    for (VoxelSurfaceCache::RootEntryMap::const_iterator rootIterator = roots.begin();
         rootIterator != roots.end();
         ++rootIterator)
    {
        for (unsigned int directionValue = 0; directionValue < VoxelFaceDirectionCount; ++directionValue)
        {
            const VoxelFaceDirection direction = static_cast<VoxelFaceDirection>(directionValue);
            const Mesh& mesh = rootIterator->second.directionMeshes[directionValue];

            if (mesh.isEmpty())
            {
                continue;
            }

            const PartKey key(rootIterator->first, direction);
            PartState& state = ensurePartState(pendingStates, pendingNextPartId, key);
            const std::uint64_t sourceVersion = rootIterator->second.directionMeshVersions[directionValue];

            if (!state.active || state.sourceVersion != sourceVersion || !state.resource)
            {
                replacePartResource(state, mesh, sourceVersion);
            }

            activeKeys.insert(key);
            result.parts.push_back(Display_MeshPartSnapshot(state.partId, state.displayVersion, state.resource));
        }
    }

    for (PartStateMap::iterator iterator = pendingStates.begin(); iterator != pendingStates.end(); ++iterator)
    {
        if (activeKeys.find(iterator->first) != activeKeys.end())
        {
            continue;
        }

        iterator->second.sourceVersion = 0;
        iterator->second.active = false;
        iterator->second.resource = Foundation::RefPtr<const Display_MeshResource>();
    }

    MYVOXEL_ASSERT_MESSAGE(result.isValid(), "Display voxel surface snapshot construction produced invalid data.");

    m_parts.swap(pendingStates);
    m_nextPartId = pendingNextPartId;
    m_initialized = true;
    return result;
}

/// 增量显示数据

Display_MeshUpdate Display_VoxelSurfaceAdapter::buildUpdate(const VoxelSurfaceCache& cache,
                                                            const VoxelSurfaceCacheUpdate& surfaceUpdate)
{
    return buildUpdate(cache, surfaceUpdate.changedRootIndices);
}

Display_MeshUpdate Display_VoxelSurfaceAdapter::buildUpdate(
    const VoxelSurfaceCache& cache,
    const VoxelSurfaceCache::RootIndexSet& changedRootIndices)
{
    MYVOXEL_ASSERT_MESSAGE(cache.isInitialized(), "Display voxel surface update requires an initialized surface cache.");
    MYVOXEL_ASSERT_MESSAGE(m_initialized, "Display voxel surface update requires a preceding full snapshot.");

    PartStateMap pendingStates = m_parts;
    Display_MeshPartId pendingNextPartId = m_nextPartId;
    Display_MeshUpdate result;
    result.objectId = m_objectId;

    for (VoxelSurfaceCache::RootIndexSet::const_iterator rootIterator = changedRootIndices.begin();
         rootIterator != changedRootIndices.end();
         ++rootIterator)
    {
        for (unsigned int directionValue = 0; directionValue < VoxelFaceDirectionCount; ++directionValue)
        {
            const VoxelFaceDirection direction = static_cast<VoxelFaceDirection>(directionValue);
            const PartKey key(*rootIterator, direction);
            PartStateMap::iterator stateIterator = pendingStates.find(key);
            const Mesh* mesh = cache.rootDirectionMesh(*rootIterator, direction);

            if (mesh && !mesh->isEmpty())
            {
                const std::uint64_t sourceVersion = cache.rootDirectionMeshVersion(*rootIterator, direction);
                PartState& state = stateIterator == pendingStates.end()
                    ? ensurePartState(pendingStates, pendingNextPartId, key)
                    : stateIterator->second;

                if (state.active && state.sourceVersion == sourceVersion && state.resource)
                {
                    continue;
                }

                replacePartResource(state, *mesh, sourceVersion);
                result.parts.push_back(
                    Display_MeshPartUpdate::replacement(state.partId, state.displayVersion, state.resource));
                continue;
            }

            if (stateIterator == pendingStates.end() || !stateIterator->second.active)
            {
                continue;
            }

            PartState& state = stateIterator->second;
            advanceDisplayVersion(state.displayVersion);
            state.sourceVersion = 0;
            state.active = false;
            state.resource = Foundation::RefPtr<const Display_MeshResource>();
            result.parts.push_back(Display_MeshPartUpdate::removal(state.partId, state.displayVersion));
        }
    }

    MYVOXEL_ASSERT_MESSAGE(result.parts.empty() || result.isValid(),
                           "Display voxel surface incremental update produced invalid data.");

    m_parts.swap(pendingStates);
    m_nextPartId = pendingNextPartId;
    return result;
}

/// 内部辅助

Display_VoxelSurfaceAdapter::PartState& Display_VoxelSurfaceAdapter::ensurePartState(
    PartStateMap& states,
    Display_MeshPartId& nextPartId,
    const PartKey& key)
{
    PartStateMap::iterator iterator = states.find(key);

    if (iterator != states.end())
    {
        return iterator->second;
    }

    if (nextPartId == 0)
    {
        throw std::overflow_error("Display voxel surface part id space is exhausted.");
    }

    PartState state;
    state.partId = nextPartId;

    if (nextPartId == (std::numeric_limits<Display_MeshPartId>::max)())
    {
        nextPartId = 0;
    }
    else
    {
        ++nextPartId;
    }

    const std::pair<PartStateMap::iterator, bool> inserted = states.insert(std::make_pair(key, state));
    MYVOXEL_ASSERT_MESSAGE(inserted.second, "Display voxel surface part state insertion failed.");
    return inserted.first->second;
}

void Display_VoxelSurfaceAdapter::replacePartResource(PartState& state,
                                                      const Mesh& mesh,
                                                      std::uint64_t sourceVersion)
{
    MYVOXEL_ASSERT_MESSAGE(!mesh.isEmpty() && mesh.isRenderable(),
                           "Display voxel surface replacement requires a non-empty renderable Mesh.");
    MYVOXEL_ASSERT_MESSAGE(sourceVersion != 0,
                           "Display voxel surface replacement requires a non-zero source version.");

    Foundation::RefPtr<Display_MeshResource> mutableResource =
        Foundation::makeRef<Display_MeshResource>(mesh);
    Foundation::RefPtr<const Display_MeshResource> resource = mutableResource;

    MYVOXEL_ASSERT_MESSAGE(resource && resource->isValid(),
                           "Display voxel surface replacement failed to create a valid display resource.");

    advanceDisplayVersion(state.displayVersion);
    state.sourceVersion = sourceVersion;
    state.active = true;
    state.resource = resource;
}

void Display_VoxelSurfaceAdapter::advanceDisplayVersion(std::uint64_t& version)
{
    if (version == (std::numeric_limits<std::uint64_t>::max)())
    {
        version = static_cast<std::uint64_t>(1);
    }
    else
    {
        ++version;
    }
}

}
