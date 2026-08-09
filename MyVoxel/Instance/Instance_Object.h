#ifndef MYVOXEL_INSTANCE_INSTANCE_OBJECT_H
#define MYVOXEL_INSTANCE_INSTANCE_OBJECT_H

#include "MyMath/Matrix4.h"

namespace MyVoxel
{

// 作为全部空间实例对象的放置基类，只负责局部空间与世界空间之间的可逆仿射变换。
//
// Instance_Object不规定实例必须引用Geometry或Topology，也不承担具体对象的有效性、包围盒或显示状态。
class Instance_Object
{
public:
    /// 空间放置

    // 判断当前实例放置是否由可逆仿射变换构成。
    bool isPlacementValid() const;
    // 返回当前实例从局部空间到世界空间的可逆仿射变换。
    const MyMath::Matrix4& localToWorld() const;
    // 返回当前实例从世界空间到局部空间的逆变换。
    const MyMath::Matrix4& worldToLocal() const;

protected:
    // 使用单位矩阵构造有效空间放置。
    Instance_Object();
    // 使用指定可逆仿射矩阵构造空间放置。
    explicit Instance_Object(const MyMath::Matrix4& localToWorld);
    ~Instance_Object();

private:
    MyMath::Matrix4 m_localToWorld; // 当前实例从局部空间到世界空间的放置变换。
    MyMath::Matrix4 m_worldToLocal; // 当前实例从世界空间到局部空间的逆放置变换。
    bool m_placementValid; // 当前局部到世界变换是否同时满足仿射和可逆条件。
};

}

#endif // MYVOXEL_INSTANCE_INSTANCE_OBJECT_H