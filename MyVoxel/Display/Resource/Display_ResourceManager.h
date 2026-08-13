#ifndef MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCEMANAGER_H
#define MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCEMANAGER_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Display/Line/Display_LineResource.h"
#include "MyVoxel/Display/Mesh/Display_MeshResource.h"
#include "MyVoxel/Display/Resource/Display_Resource.h"
#include "MyVoxel/Display/Resource/Display_ResourceTypes.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

class Mesh;

// 管理不可变CPU显示资源的唯一标识和共享引用，不参与GPU资源、场景对象或显示状态管理。
//
// 管理器内部操作支持并发调用；资源从管理器移除后，只要外部仍持有RefPtr，资源对象就继续保持有效。
class Display_ResourceManager
{
public:
    Display_ResourceManager();
    ~Display_ResourceManager();

    Display_ResourceManager(const Display_ResourceManager&) = delete;
    Display_ResourceManager& operator=(const Display_ResourceManager&) = delete;

    /// 资源创建

    // 根据可渲染Mesh创建并注册不可变网格显示资源，失败返回零。
    Display_ResourceId createMeshResource(const Mesh& mesh);
    // 根据GL_LINES端点序列和统一颜色创建并注册不可变线显示资源，失败返回零。
    Display_ResourceId createLineResource(const std::vector<MyMath::Vector3>& points, const Display_Color& color);
    // 注册已经建立的有效不可变显示资源并返回新的管理器内唯一标识，失败返回零。
    Display_ResourceId registerResource(const Foundation::RefPtr<const Display_Resource>& resource);

    /// 资源查询

    // 判断指定资源标识当前是否仍由管理器持有。
    bool contains(Display_ResourceId resourceId) const;
    // 返回指定不可变显示资源，资源不存在时返回空引用。
    Foundation::RefPtr<const Display_Resource> resource(Display_ResourceId resourceId) const;
    // 返回指定网格显示资源，资源不存在或类型不匹配时返回空引用。
    Foundation::RefPtr<const Display_MeshResource> meshResource(Display_ResourceId resourceId) const;
    // 返回指定线显示资源，资源不存在或类型不匹配时返回空引用。
    Foundation::RefPtr<const Display_LineResource> lineResource(Display_ResourceId resourceId) const;

    /// 资源释放

    // 移除管理器对指定资源的持有引用，返回资源是否存在。
    bool remove(Display_ResourceId resourceId);
    // 移除管理器持有的全部显示资源引用。
    void clear();

    /// 资源统计

    // 返回当前管理器持有的资源数量。
    std::size_t resourceCount() const;
    // 返回当前管理器持有资源的连续数据总字节数。
    std::size_t memoryByteSize() const;

private:
    typedef std::map<Display_ResourceId, Foundation::RefPtr<const Display_Resource> > ResourceMap;

    // 在已经持有管理器互斥锁时分配新的非零资源标识。
    Display_ResourceId allocateResourceId();

private:
    mutable std::mutex m_mutex; // 保护资源表和下一个资源标识。
    ResourceMap m_resources; // 当前由管理器持有的不可变CPU显示资源。
    Display_ResourceId m_nextResourceId; // 下一次优先分配的非零资源标识。
};

}

#endif // MYVOXEL_DISPLAY_RESOURCE_DISPLAY_RESOURCEMANAGER_H