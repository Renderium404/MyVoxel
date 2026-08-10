#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_OBJECT_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_OBJECT_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Topology/Topology_Orientation.h"
#include "MyVoxel/Topology/Topology_TObject.h"

namespace MyVoxel
{

// 作为全部公开拓扑对象的轻量句柄基类，共享Topology_TObject身份并独立保存使用方向。
//
// Topology_Object自身不参与引用计数、不保存空间变换，也不规定具体Vertex、Edge、Wire等拓扑数据。
class Topology_Object
{
public:
    // 构造不引用任何共享拓扑实体的空句柄，默认方向为Forward。
    Topology_Object();
    Topology_Object(const Topology_Object&) = default;
    Topology_Object& operator=(const Topology_Object&) = default;

    /// 状态判断

    // 判断当前句柄是否引用共享拓扑实体。
    bool isValid() const;
    // 判断当前句柄是否未引用共享拓扑实体。
    bool isNull() const;
    // 判断当前句柄是否引用共享拓扑实体。
    explicit operator bool() const;

    /// 拓扑身份
    // 判断两个非空句柄是否引用同一个Topology_TObject，使用方向不参与拓扑身份判断。
    bool isSame(const Topology_Object& other) const;

    /// 使用方向
    // 返回当前句柄相对于底层共享拓扑实体的使用方向。
    Topology_Orientation orientation() const;
    // 判断当前句柄是否按底层拓扑实体的标准方向使用。
    bool isForward() const;
    // 判断当前句柄是否按底层拓扑实体的相反方向使用。
    bool isReversed() const;

protected:
    // 使用共享拓扑实体和明确方向构造轻量拓扑句柄。
    Topology_Object(const Foundation::RefPtr<const Topology_TObject>& object, Topology_Orientation orientation);

    /// 派生类支持

    // 返回当前句柄持有的共享Topology_TObject引用，供具体拓扑句柄访问其强类型实体。
    const Foundation::RefPtr<const Topology_TObject>& tObject() const;
    // 返回当前使用方向的相反方向，供具体拓扑句柄实现reversed()。
    Topology_Orientation reversedOrientation() const;

private:
    Foundation::RefPtr<const Topology_TObject> m_object; // 当前句柄共享的底层拓扑实体。
    Topology_Orientation m_orientation; // 当前句柄相对于底层拓扑实体的使用方向。
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_OBJECT_H