#ifndef MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_SHAPE_H
#define MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_SHAPE_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
#include "ShapeKind.h"
#include "ShapeRelation.h"

namespace MyVoxel
{

// 定义局部坐标系中有限、封闭并且可执行空间查询的连续几何体。
//
// 每个连续几何体都由基类统一保存稳定的局部轴对齐包围盒。
// 派生类应在构造或内部缓存重建完成时初始化该包围盒，不再重复定义localBounds接口和成员。
class Geometry_Shape : public Foundation::ReferenceCounted
{
public:
    /// 几何属性

    // 返回当前连续几何体的标准类型。
    virtual ShapeKind kind() const = 0;
    // 返回当前连续几何体在局部坐标系中的稳定轴对齐包围盒。
    const Bounds3& localBounds() const;

    /// 标准空间查询

    // 判断指定局部坐标点是否位于几何体内部或边界上。
    virtual bool containsLocalPoint(const MyMath::Vector3& point) const = 0;
    // 判断当前几何体是否提供精确局部有符号距离查询。
    virtual bool supportsSignedDistance() const;
    // 返回指定局部点到实体边界的有符号距离，内部为负、外部为正、边界为零；调用前必须保证supportsSignedDistance()为true。
    virtual double signedDistanceLocalPoint(const MyMath::Vector3& point) const;
    // 返回指定局部轴对齐包围盒与几何体之间的保守空间关系。
    virtual ShapeRelation classifyLocalBounds(const Bounds3& bounds) const = 0;

    /// 快速空间查询

    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    // 默认实现构造Bounds3并回退到标准路径，因此现有及自定义几何不必提供快速实现。
    virtual ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
    {
        return classifyLocalBounds(Bounds3(center - extent, center + extent));
    }

protected:
    // 构造尚未建立局部包围盒的连续几何体，供需要先建立其他缓存的派生类使用。
    Geometry_Shape();
    // 使用指定稳定局部包围盒构造连续几何体。
    explicit Geometry_Shape(const Bounds3& localBounds);

    // 设置当前几何体的稳定局部包围盒，供构造或缓存重建阶段使用。
    void setLocalBounds(const Bounds3& localBounds);
    // 清除当前局部包围盒，供缓存重建开始阶段恢复未初始化状态。
    void clearLocalBounds();

    // 通过侵入式引用计数管理连续几何体生命周期。
    ~Geometry_Shape() override = default;

private:
    Bounds3 m_localBounds; // 当前连续几何体在局部坐标系中的稳定轴对齐包围盒。
};

}

#endif // MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_SHAPE_H
