#ifndef MYVOXEL_TOPOLOGY_SHAPE_TOPOLOGY_TSHAPE_H
#define MYVOXEL_TOPOLOGY_SHAPE_TOPOLOGY_TSHAPE_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Topology/Topology_TObject.h"

namespace MyVoxel
{

class Topology_Shape;

// 保存一个共享拓扑形体身份及其连续实体几何内核。
class Topology_TShape : public Topology_TObject
{
    friend class Topology_Shape;

public:
    /// 几何内核

    // 返回当前拓扑形体对应的连续实体几何内核。
    const Geometry_Shape& geometry() const;

protected:
    // 使用指定连续实体几何创建共享拓扑形体实体。
    explicit Topology_TShape(const Foundation::RefPtr<const Geometry_Shape>& geometry);

    // 通过Topology_Shape持有的最终共享引用释放拓扑形体实体。
    ~Topology_TShape() override = default;

private:
    Foundation::RefPtr<const Geometry_Shape> m_geometry; // 当前拓扑形体对应的不可变连续实体几何内核。
};

}

#endif // MYVOXEL_TOPOLOGY_SHAPE_TOPOLOGY_TSHAPE_H