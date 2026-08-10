#ifndef MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTMANAGER_H
#define MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTMANAGER_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>

#include "MyMath/Matrix4.h"
#include "MyVoxel/Display/Object/Display_Object.h"
#include "MyVoxel/Display/Object/Display_ObjectTypes.h"
#include "MyVoxel/Display/Resource/Display_ResourceManager.h"

namespace MyVoxel
{

// 管理CPU侧逻辑显示对象身份、状态和不可变资源分片引用。
//
// Display_ObjectManager不创建资源、不保存GPU对象，也不允许外部直接修改内部Display_Object。
// 查询接口返回Display_Object副本，其内部不可变资源通过RefPtr继续共享实际数据。
// 构造时使用的Display_ResourceManager必须在当前管理器整个生命周期内保持有效。
class Display_ObjectManager
{
public:
    // 使用指定资源管理器建立空显示对象管理器。
    explicit Display_ObjectManager(const Display_ResourceManager& resourceManager);
    ~Display_ObjectManager();

    Display_ObjectManager(const Display_ObjectManager&) = delete;
    Display_ObjectManager& operator=(const Display_ObjectManager&) = delete;

    /// 对象创建

    // 创建指定统一资源类型的空显示对象并返回新的全局对象标识。
    Display_ObjectId createObject(Display_ResourceKind resourceKind, Display_ObjectUsage usage = Display_ObjectUsage::Static);
    // 根据已有不可变资源创建包含一个初始分片的显示对象，资源不存在时返回零。
    Display_ObjectId createObject(Display_ResourceId resourceId, Display_ObjectUsage usage = Display_ObjectUsage::Static,
                                  Display_ObjectPartId partId = 0);

    /// 对象查询

    // 判断指定显示对象当前是否存在。
    bool contains(Display_ObjectId objectId) const;
    // 返回指定显示对象当前状态副本，对象不存在时返回无效空对象。
    Display_Object object(Display_ObjectId objectId) const;

    /// 对象状态

    // 修改对象资源的预期GPU更新频率。
    bool setUsage(Display_ObjectId objectId, Display_ObjectUsage usage);
    // 修改对象局部空间到世界空间的可逆仿射变换。
    bool setLocalToWorld(Display_ObjectId objectId, const MyMath::Matrix4& localToWorld);
    // 修改对象可见状态。
    bool setVisible(Display_ObjectId objectId, bool visible);
    // 修改Line对象期望显示线宽，非Line对象返回false。
    bool setLineWidth(Display_ObjectId objectId, double width);

    /// 资源分片

    // 使用指定不可变资源建立或替换对象分片；资源类型必须与对象资源类型一致。
    bool setPart(Display_ObjectId objectId, Display_ObjectPartId partId, Display_ResourceId resourceId);
    // 删除指定活动资源分片，并推进该partId版本。
    bool removePart(Display_ObjectId objectId, Display_ObjectPartId partId);
    // 删除对象当前全部活动资源分片，并分别推进对应partId版本。
    bool clearParts(Display_ObjectId objectId);

    /// 对象释放

    // 删除指定显示对象并释放管理器持有的全部资源引用。
    bool remove(Display_ObjectId objectId);
    // 删除当前全部显示对象。
    void clear();

    /// 对象统计

    // 返回当前活动显示对象数量。
    std::size_t objectCount() const;
    // 返回全部显示对象当前活动资源分片数量。
    std::size_t partCount() const;

private:
    typedef std::map<Display_ObjectId, Display_Object> ObjectMap;

    // 在已经持有互斥锁时分配新的非零显示对象标识。
    Display_ObjectId allocateObjectId();
    // 返回指定当前版本之后的非零版本号，版本空间耗尽属于不可恢复内部错误。
    static std::uint64_t nextVersion(std::uint64_t currentVersion);
    // 判断矩阵是否为可逆仿射变换。
    static bool isValidTransform(const MyMath::Matrix4& transform);

private:
    const Display_ResourceManager* m_resourceManager; // 用于根据资源标识取得不可变资源引用，必须比当前管理器长寿。
    mutable std::mutex m_mutex; // 保护对象表和对象标识分配状态。
    ObjectMap m_objects; // 当前全部逻辑显示对象。
    Display_ObjectId m_nextObjectId; // 下一次优先分配的非零全局显示对象标识。
};

}

#endif // MYVOXEL_DISPLAY_OBJECT_DISPLAY_OBJECTMANAGER_H