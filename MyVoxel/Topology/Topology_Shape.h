#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_SHAPE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_SHAPE_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Geometry/Shape/ShapeKind.h"
#include "MyVoxel/Geometry/Shape/ShapeRelation.h"
#include "Topology_Face.h"

namespace MyVoxel
{

// 表示持有不可变实体几何资源和可选边界Face集合的局部拓扑Shape值对象，不包含任何空间放置变换。
//
// 仅持有Geometry_Shape的旧构造方式继续保持有效，用于现有查询和体素化链路；
// 需要执行Face级离散时，应通过带Faces构造或Topology_ShapeBuilder建立完整边界拓扑。
class Topology_Shape
{
public:
    // 构造不持有几何资源的空拓扑Shape。
    Topology_Shape();
    // 使用不可变实体几何资源构造不包含边界Face的局部拓扑Shape。
    explicit Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry);
    // 使用不可变实体几何资源和非空有效边界Face集合构造完整局部拓扑Shape。
    Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry, const std::vector<Topology_Face>& faces);
    Topology_Shape(const Topology_Shape&) = default;
    Topology_Shape& operator=(const Topology_Shape&) = default;

    /// 状态判断

    // 判断当前拓扑Shape是否持有有效几何资源。
    bool isValid() const;
    // 判断当前拓扑Shape是否未持有几何资源。
    bool isNull() const;
    // 判断当前拓扑Shape是否持有有效几何资源。
    explicit operator bool() const;
    // 判断当前拓扑Shape是否与另一个拓扑Shape共享同一份几何资源。
    bool sharesGeometryWith(const Topology_Shape& other) const;
    // 判断当前拓扑Shape是否包含至少一个有效边界Face。
    bool hasFaces() const;

    /// 几何资源

    // 返回当前拓扑Shape持有的不可变几何资源，空对象调用属于调用错误。
    const Geometry_Shape& geometry() const;
    // 返回当前拓扑Shape持有的不可变几何资源普通指针，空对象返回空指针。
    const Geometry_Shape* geometryPointer() const;
    // 返回当前拓扑Shape持有的不可变几何资源引用计数指针。
    const Foundation::RefPtr<const Geometry_Shape>& geometryResource() const;
    // 返回当前拓扑Shape的标准类型，空对象调用属于调用错误。
    ShapeKind kind() const;

    /// 边界拓扑

    // 返回当前边界Face数量；仅持有几何资源的兼容Topology_Shape返回零。
    std::size_t faceCount() const;
    // 返回指定编号的边界Face。
    const Topology_Face& face(std::size_t index) const;
    // 返回当前全部边界Face。
    const std::vector<Topology_Face>& faces() const;

    /// 局部空间数据与查询

    // 返回当前拓扑Shape在自身局部坐标系中的轴对齐包围盒。
    const Bounds3& bounds() const;
    // 判断指定局部坐标点是否位于拓扑Shape内部或边界上。
    bool containsPoint(const MyMath::Vector3& point) const;
    // 返回指定局部轴对齐包围盒与拓扑Shape之间的保守空间关系。
    ShapeRelation classifyBounds(const Bounds3& bounds) const;
    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

private:
    Foundation::RefPtr<const Geometry_Shape> m_geometry; // 当前拓扑Shape共享的不可变实体几何资源。
    std::vector<Topology_Face> m_faces; // 当前实体边界的局部拓扑Face集合，兼容旧构造时允许为空。
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_SHAPE_H
