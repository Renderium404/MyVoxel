#ifndef MYVOXEL_TOPOLOGY_SHAPE_TOPOLOGY_SHAPE_H
#define MYVOXEL_TOPOLOGY_SHAPE_TOPOLOGY_SHAPE_H

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Topology/Topology_Object.h"
#include "Topology_TShape.h"

namespace MyVoxel
{

// 作为连续Geometry_Shape几何内核的轻量拓扑形体句柄，不建立Face、Shell或Solid边界拓扑。
class Topology_Shape : public Topology_Object
{
public:
    // 构造空拓扑形体句柄。
    Topology_Shape();

    // 使用指定连续实体几何内核创建新的拓扑形体身份。
    explicit Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry);

    /// 几何内核

    // 返回当前拓扑形体对应的连续实体几何内核。
    const Geometry_Shape& geometry() const;

private:
    // 返回当前句柄引用的强类型拓扑形体实体。
    const Topology_TShape& tShape() const;
};

}

#endif // MYVOXEL_TOPOLOGY_SHAPE_TOPOLOGY_SHAPE_H