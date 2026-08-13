#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_TOBJECT_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_TOBJECT_H

#include "MyVoxel/Foundation/ReferenceCounted.h"

namespace MyVoxel
{

// 作为全部共享拓扑实体的内部生命周期基类，真正的拓扑身份由具体Topology_TObject对象本身确定。
class Topology_TObject : public Foundation::ReferenceCounted
{
protected:
    // 构造引用计数为零的共享拓扑实体。
    Topology_TObject();
    // 通过Topology_Object持有的最终共享引用释放具体拓扑实体。
    ~Topology_TObject() override;
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_TOBJECT_H