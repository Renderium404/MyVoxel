#ifndef MYVOXEL_TOPOLOGY_FACE_H
#define MYVOXEL_TOPOLOGY_FACE_H

#include <cstddef>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "Topology_Face.h"

namespace MyVoxel
{

// 表示Topology_Face在世界空间中的一次不可变放置，缓存正反变换、法线变换和保守世界包围盒。
// Face边界Wire位于曲面UV参数域，不能仅通过实例矩阵转换为普通三维Wire。
class Face
{
public:
    // 构造不包含局部拓扑Face的空Face实例。
    Face();
    // 使用单位变换放置局部拓扑Face。
    explicit Face(const Topology_Face& topology);
    // 使用可逆仿射变换放置局部拓扑Face。
    Face(const Topology_Face& topology, const MyMath::Matrix4& localToWorld);
    Face(const Face&) = default;
    Face& operator=(const Face&) = default;

    /// 状态判断

    // 判断当前Face是否包含有效局部拓扑Face和可逆仿射变换。
    bool isValid() const;
    // 判断当前Face是否未包含曲面资源。
    bool isNull() const;
    // 判断当前Face是否包含完整有效数据。
    explicit operator bool() const;
    // 判断当前Face是否与另一个Face共享同一份曲面几何资源。
    bool sharesSurfaceWith(const Face& other) const;
    // 判断当前Face是否相对曲面正向发生反转。
    bool isReversed() const;

    /// 局部拓扑与曲面资源

    // 返回当前Face持有的局部拓扑Face，空对象调用属于调用错误。
    const Topology_Face& topology() const;
    // 返回当前Face引用的不可变曲面几何，空对象调用属于调用错误。
    const Geometry_Surface& surface() const;
    // 返回当前Face引用的不可变曲面几何普通指针，空对象返回空指针。
    const Geometry_Surface* surfacePointer() const;
    // 返回当前Face引用的曲面标准类型。
    SurfaceKind surfaceKind() const;

    /// 参数域边界

    // 返回曲面UV参数域中的外边界Wire。
    const Topology_Wire& outerParameterWire() const;
    // 返回曲面UV参数域中的孔洞边界数量。
    std::size_t innerParameterWireCount() const;
    // 返回曲面UV参数域中的指定孔洞边界Wire。
    const Topology_Wire& innerParameterWire(std::size_t index) const;
    // 返回外边界覆盖的UV轴对齐参数范围。
    const Bounds3& parameterBounds() const;

    /// 空间数据

    // 返回当前Face映射到曲面后的局部三维轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回局部坐标到世界坐标的变换矩阵。
    const MyMath::Matrix4& localToWorld() const;
    // 返回世界坐标到局部坐标的变换矩阵。
    const MyMath::Matrix4& worldToLocal() const;
    // 返回局部法线到世界法线的逆转置变换矩阵。
    const MyMath::Matrix4& normalToWorld() const;
    // 返回当前Face在世界坐标系中的保守轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 参数曲面查询

    // 返回指定UV参数点对应的局部三维曲面点。
    MyMath::Vector3 localPointAt(const MyMath::Vector3& parameterPoint) const;
    // 返回指定UV参数点对应的世界三维曲面点。
    MyMath::Vector3 worldPointAt(const MyMath::Vector3& parameterPoint) const;
    // 返回指定UV参数点对应并考虑Face方向的局部单位法线。
    MyMath::Vector3 localNormalAt(const MyMath::Vector3& parameterPoint) const;
    // 返回指定UV参数点对应并考虑Face方向的世界单位法线。
    MyMath::Vector3 worldNormalAt(const MyMath::Vector3& parameterPoint) const;
    // 返回指定UV参数点处的局部u偏导向量。
    MyMath::Vector3 localDerivativeUAt(const MyMath::Vector3& parameterPoint) const;
    // 返回指定UV参数点处的局部v偏导向量。
    MyMath::Vector3 localDerivativeVAt(const MyMath::Vector3& parameterPoint) const;
    // 返回指定UV参数点处的世界u偏导向量。
    MyMath::Vector3 worldDerivativeUAt(const MyMath::Vector3& parameterPoint) const;
    // 返回指定UV参数点处的世界v偏导向量。
    MyMath::Vector3 worldDerivativeVAt(const MyMath::Vector3& parameterPoint) const;

    /// 参数域查询

    // 判断指定UV参数点是否位于Face区域内部或边界上。
    bool containsParameterPoint(const MyMath::Vector3& parameterPoint, double tolerance = 0.0) const;
    // 返回指定UV参数范围与Face区域之间的保守关系。
    ShapeRelation classifyParameterBounds(const Bounds3& bounds, double tolerance = 0.0) const;

    /// Face创建

    // 返回空间位置相同、全部边界方向相反且法线方向反转的新Face实例。
    Face reversed() const;

private:
    // 使用局部拓扑Face和局部到世界变换初始化完整Face实例数据。
    void initialize(const Topology_Face& topology, const MyMath::Matrix4& localToWorld);

private:
    Topology_Face m_topology; // 当前Face实例持有的局部拓扑Face。
    MyMath::Matrix4 m_localToWorld; // 局部坐标到世界坐标的变换矩阵。
    MyMath::Matrix4 m_worldToLocal; // 世界坐标到局部坐标的变换矩阵。
    MyMath::Matrix4 m_normalToWorld; // 局部法线到世界法线的逆转置变换矩阵。
    Bounds3 m_worldBounds; // 当前Face实例在世界坐标系中的保守轴对齐包围盒。
    bool m_valid; // 当前Face实例是否包含完整有效数据。
};

}

#endif // MYVOXEL_TOPOLOGY_FACE_H