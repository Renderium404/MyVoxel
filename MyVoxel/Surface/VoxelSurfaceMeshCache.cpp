#include "VoxelSurfaceMeshCache.h"

#include <cassert>
#include <cstdint>
#include <limits>
#include <utility>

#include "MyVoxel/Surface/VoxelSurfaceExtractor.h"

namespace
{

// 检查两个表面颜色是否完全一致。
bool sameColor(const MyVoxel::VoxelSurfaceColor& first, const MyVoxel::VoxelSurfaceColor& second)
{
    return first.red == second.red && first.green == second.green && first.blue == second.blue && first.alpha == second.alpha;
}

// 返回指定索引分量应用偏移后的结果，超出VoxelIndex范围时返回false。
bool offsetIndex(MyVoxel::VoxelIndex value, int offset, MyVoxel::VoxelIndex& result)
{
    const std::int64_t target = static_cast<std::int64_t>(value) + static_cast<std::int64_t>(offset);
    const std::int64_t minimum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const std::int64_t maximum = static_cast<std::int64_t>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    if (target < minimum || target > maximum)
    {
        return false;
    }

    result = static_cast<MyVoxel::VoxelIndex>(target);
    return true;
}

// 返回指定体素地址所属的第0层根索引。
MyVoxel::VoxelCellIndex rootIndex(MyVoxel::VoxelCellAddress address)
{
    while (MyVoxel::hasParentCell(address))
    {
        address = MyVoxel::parentCellAddress(address);
    }

    return address.index;
}

// 向集合追加指定X方向相邻根索引。
void appendXNeighbor(MyVoxel::VoxelSurfaceMeshCache::RootIndexSet& rootIndices, const MyVoxel::VoxelCellIndex& rootIndexValue, int offset)
{
    MyVoxel::VoxelIndex x = 0;

    if (offsetIndex(rootIndexValue.x, offset, x))
    {
        rootIndices.insert(MyVoxel::VoxelCellIndex(x, rootIndexValue.y, rootIndexValue.z));
    }
}

// 向集合追加指定Y方向相邻根索引。
void appendYNeighbor(MyVoxel::VoxelSurfaceMeshCache::RootIndexSet& rootIndices, const MyVoxel::VoxelCellIndex& rootIndexValue, int offset)
{
    MyVoxel::VoxelIndex y = 0;

    if (offsetIndex(rootIndexValue.y, offset, y))
    {
        rootIndices.insert(MyVoxel::VoxelCellIndex(rootIndexValue.x, y, rootIndexValue.z));
    }
}

// 向集合追加指定Z方向相邻根索引。
void appendZNeighbor(MyVoxel::VoxelSurfaceMeshCache::RootIndexSet& rootIndices, const MyVoxel::VoxelCellIndex& rootIndexValue, int offset)
{
    MyVoxel::VoxelIndex z = 0;

    if (offsetIndex(rootIndexValue.z, offset, z))
    {
        rootIndices.insert(MyVoxel::VoxelCellIndex(rootIndexValue.x, rootIndexValue.y, z));
    }
}

}

namespace MyVoxel
{

VoxelSurfaceMeshCache::VoxelSurfaceMeshCache()
    : m_transform(MyMath::Matrix4::identity())
{
}

void VoxelSurfaceMeshCache::clear()
{
    m_meshes.clear();
    m_baseVoxelEdgeLength = 1.0;
    m_maximumLevel = BaseVoxelLevel;
    m_transform = MyMath::Matrix4::identity();
    m_color = VoxelSurfaceColor();
    m_initialized = false;
}

void VoxelSurfaceMeshCache::rebuild(const VoxelShape& shape, const VoxelSurfaceColor& color)
{
    assert(shape.transform().isAffine());

    m_meshes.clear();
    setSource(shape, color);

    const RootIndexSet rootIndices = collectMaterialRootIndices(shape);

    for (RootIndexSet::const_iterator iterator = rootIndices.begin(); iterator != rootIndices.end(); ++iterator)
    {
        rebuildRoot(shape, *iterator, color);
    }
}

VoxelSurfaceMeshCache::RootIndexSet VoxelSurfaceMeshCache::update(const VoxelShape& shape, const VoxelChangeSet& changes, const VoxelSurfaceColor& color)
{
    assert(shape.transform().isAffine());

    if (!isCompatible(shape, color))
    {
        RootIndexSet dirtyRootIndices;

        // 全量失效时，旧缓存根必须加入脏集合，确保显示层能够删除已经消失的网格。
        for (MeshMap::const_iterator iterator = m_meshes.begin(); iterator != m_meshes.end(); ++iterator)
        {
            dirtyRootIndices.insert(iterator->first);
        }

        rebuild(shape, color);

        // 新缓存根必须加入脏集合，确保显示层能够创建或更新全部当前网格。
        for (MeshMap::const_iterator iterator = m_meshes.begin(); iterator != m_meshes.end(); ++iterator)
        {
            dirtyRootIndices.insert(iterator->first);
        }

        return dirtyRootIndices;
    }

    RootIndexSet dirtyRootIndices;
    const VoxelChangeSet::RootIndexSet& modifiedRootIndices = changes.modifiedRootIndices();

    for (VoxelChangeSet::RootIndexSet::const_iterator iterator = modifiedRootIndices.begin(); iterator != modifiedRootIndices.end(); ++iterator)
    {
        appendRootAndFaceNeighbors(dirtyRootIndices, *iterator);
    }

    for (RootIndexSet::const_iterator iterator = dirtyRootIndices.begin(); iterator != dirtyRootIndices.end(); ++iterator)
    {
        rebuildRoot(shape, *iterator, color);
    }

    return dirtyRootIndices;
}



bool VoxelSurfaceMeshCache::isEmpty() const
{
    return m_meshes.empty();
}

bool VoxelSurfaceMeshCache::isInitialized() const
{
    return m_initialized;
}

std::size_t VoxelSurfaceMeshCache::meshCount() const
{
    return m_meshes.size();
}

std::size_t VoxelSurfaceMeshCache::vertexCount() const
{
    std::size_t count = 0;

    for (MeshMap::const_iterator iterator = m_meshes.begin(); iterator != m_meshes.end(); ++iterator)
    {
        count += iterator->second.vertexCount();
    }

    return count;
}

std::size_t VoxelSurfaceMeshCache::triangleCount() const
{
    std::size_t count = 0;

    for (MeshMap::const_iterator iterator = m_meshes.begin(); iterator != m_meshes.end(); ++iterator)
    {
        count += iterator->second.triangleCount();
    }

    return count;
}

bool VoxelSurfaceMeshCache::contains(const VoxelCellIndex& rootIndexValue) const
{
    return m_meshes.find(rootIndexValue) != m_meshes.end();
}

const VoxelSurfaceMesh* VoxelSurfaceMeshCache::mesh(const VoxelCellIndex& rootIndexValue) const
{
    const MeshMap::const_iterator iterator = m_meshes.find(rootIndexValue);
    return iterator == m_meshes.end() ? nullptr : &iterator->second;
}

const VoxelSurfaceMeshCache::MeshMap& VoxelSurfaceMeshCache::meshes() const
{
    return m_meshes;
}

VoxelSurfaceMesh VoxelSurfaceMeshCache::combinedMesh() const
{
    std::size_t totalVertexCount = 0;
    std::size_t totalIndexCount = 0;

    for (MeshMap::const_iterator iterator = m_meshes.begin(); iterator != m_meshes.end(); ++iterator)
    {
        totalVertexCount += iterator->second.vertexCount();
        totalIndexCount += iterator->second.indexCount();
    }

    VoxelSurfaceMesh result;
    result.reserve(totalVertexCount, totalIndexCount);

    for (MeshMap::const_iterator iterator = m_meshes.begin(); iterator != m_meshes.end(); ++iterator)
    {
        result.append(iterator->second);
    }

    return result;
}

bool VoxelSurfaceMeshCache::isCompatible(const VoxelShape& shape, const VoxelSurfaceColor& color) const
{
    return m_initialized &&
           m_baseVoxelEdgeLength == shape.baseVoxelEdgeLength() &&
           m_maximumLevel == shape.maximumLevel() &&
           m_transform.isEqualTo(shape.transform(), 0.0) &&
           sameColor(m_color, color);
}

void VoxelSurfaceMeshCache::setSource(const VoxelShape& shape, const VoxelSurfaceColor& color)
{
    m_baseVoxelEdgeLength = shape.baseVoxelEdgeLength();
    m_maximumLevel = shape.maximumLevel();
    m_transform = shape.transform();
    m_color = color;
    m_initialized = true;
}

VoxelSurfaceMeshCache::RootIndexSet VoxelSurfaceMeshCache::collectMaterialRootIndices(const VoxelShape& shape)
{
    RootIndexSet rootIndices;

    shape.forest().forEachMaterialCell(
        [&](const VoxelCellAddress& address)
        {
            rootIndices.insert(rootIndex(address));
        });

    return rootIndices;
}

void VoxelSurfaceMeshCache::appendRootAndFaceNeighbors(RootIndexSet& rootIndices, const VoxelCellIndex& rootIndexValue)
{
    rootIndices.insert(rootIndexValue);
    appendXNeighbor(rootIndices, rootIndexValue, -1);
    appendXNeighbor(rootIndices, rootIndexValue, 1);
    appendYNeighbor(rootIndices, rootIndexValue, -1);
    appendYNeighbor(rootIndices, rootIndexValue, 1);
    appendZNeighbor(rootIndices, rootIndexValue, -1);
    appendZNeighbor(rootIndices, rootIndexValue, 1);
}

void VoxelSurfaceMeshCache::rebuildRoot(const VoxelShape& shape, const VoxelCellIndex& rootIndexValue, const VoxelSurfaceColor& color)
{
    VoxelSurfaceMesh rootMesh = VoxelSurfaceExtractor::extractRoot(shape, rootIndexValue, color);

    if (rootMesh.isEmpty())
    {
        m_meshes.erase(rootIndexValue);
        return;
    }

    MeshMap::iterator iterator = m_meshes.find(rootIndexValue);

    if (iterator == m_meshes.end())
    {
        m_meshes.insert(std::make_pair(rootIndexValue, std::move(rootMesh)));
    }
    else
    {
        iterator->second = std::move(rootMesh);
    }
}

}