#ifndef MYVOXEL_TOPOLOGY_TOPOLOGY_FACE_H
#define MYVOXEL_TOPOLOGY_TOPOLOGY_FACE_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Shape/ShapeRelation.h"
#include "MyVoxel/Geometry/Surface/Geometry_Surface.h"
#include "Topology_Wire.h"

namespace MyVoxel
{

// 表示连续曲面上由一个外边界和零个或多个孔洞边界裁剪得到的局部拓扑Face。
// Face中的Topology_Wire位于曲面UV参数平面，Wire点(x,y,0)分别表示参数(u,v)。
// 当前版本验证曲面资源及Wire结构，不执行Wire自相交、边界互交或孔洞嵌套检查。
class Topology_Face
{
public:
    // 构造不持有曲面和边界的空拓扑Face。
    Topology_Face();
    // 使用曲面和一个闭合非零面积外边界构造不含孔洞的拓扑Face。
    Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& surface, const Topology_Wire& outerWire);
    // 使用曲面、一个外边界和多个孔洞边界构造拓扑Face。
    Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& surface, const Topology_Wire& outerWire, const std::vector<Topology_Wire>& innerWires);
    Topology_Face(const Topology_Face&) = default;
    Topology_Face& operator=(const Topology_Face&) = default;

    /// 状态判断

    // 判断当前Face是否持有有效曲面、闭合外边界及全部闭合孔洞边界。
    bool isValid() const;
    // 判断当前Face是否未持有曲面资源。
    bool isNull() const;
    // 判断当前Face是否持有完整有效数据。
    explicit operator bool() const;
    // 判断当前Face是否与另一个Face共享同一份曲面几何资源。
    bool sharesSurfaceWith(const Topology_Face& other) const;
    // 判断当前Face是否相对曲面正向发生反转。
    bool isReversed() const;

    /// 曲面几何

    // 返回当前Face引用的不可变曲面几何，空对象调用属于调用错误。
    const Geometry_Surface& surface() const;
    // 返回当前Face引用的曲面几何普通指针，空对象返回空指针。
    const Geometry_Surface* surfacePointer() const;
    // 返回当前Face持有的曲面几何引用计数指针。
    const Foundation::RefPtr<const Geometry_Surface>& surfaceResource() const;
    // 返回当前Face引用的曲面标准类型。
    SurfaceKind surfaceKind() const;

    /// 边界拓扑

    // 返回Face外边界Wire。
    const Topology_Wire& outerWire() const;
    // 返回孔洞边界数量。
    std::size_t innerWireCount() const;
    // 返回指定编号的孔洞边界Wire。
    const Topology_Wire& innerWire(std::size_t index) const;
    // 返回全部孔洞边界Wire。
    const std::vector<Topology_Wire>& innerWires() const;

    /// 参数域与局部空间

    // 返回外边界覆盖的局部UV轴对齐参数范围。
    const Bounds3& parameterBounds() const;
    // 返回Face映射到曲面后的保守局部三维轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回参数点对应的局部三维曲面点，参数点必须位于局部UV平面。
    MyMath::Vector3 pointAt(const MyMath::Vector3& parameterPoint) const;
    // 返回参数点对应并考虑Face反向状态的局部单位法线。
    MyMath::Vector3 normalAt(const MyMath::Vector3& parameterPoint) const;

    /// 参数域查询

    // 判断指定UV参数点是否位于Face区域内部或边界上，孔洞内部返回false，孔洞边界返回true。
    bool containsParameterPoint(const MyMath::Vector3& parameterPoint, double tolerance = 0.0) const;
    // 返回指定UV轴对齐参数范围与Face区域之间的保守关系。
    ShapeRelation classifyParameterBounds(const Bounds3& bounds, double tolerance = 0.0) const;

    /// 拓扑创建

    // 返回曲面相同、全部边界方向相反且Face法线方向反转的新拓扑Face。
    Topology_Face reversed() const;

private:
    // 验证输入数据、统一边界方向并建立参数域和局部空间缓存。
    void rebuild();
    // 返回指定Wire与退化参数点包围盒之间的区域关系。
    static ShapeRelation classifyWirePoint(const Topology_Wire& wire, const MyMath::Vector3& point, double tolerance);

private:
    Foundation::RefPtr<const Geometry_Surface> m_surface; // 当前Face共享的不可变曲面几何资源。
    Topology_Wire m_outerWire; // 曲面UV参数平面中的外边界，正向Face使用逆时针方向。
    std::vector<Topology_Wire> m_innerWires; // 曲面UV参数平面中的孔洞边界，正向Face使用顺时针方向。
    Bounds3 m_parameterBounds; // 外边界覆盖的局部UV轴对齐参数范围。
    Bounds3 m_localBounds; // Face映射到曲面后的保守局部三维轴对齐包围盒。
    bool m_reversed; // 当前Face是否相对曲面正向发生反转。
    bool m_valid; // 当前Face是否包含完整有效数据。
};

}

#endif // MYVOXEL_TOPOLOGY_TOPOLOGY_FACE_H
