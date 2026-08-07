#ifndef MYVOXEL_MESH_GEOMETRY_MESH_H
#define MYVOXEL_MESH_GEOMETRY_MESH_H

#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"
#include "MyVoxel/Mesh/Mesh.h"
#include "MyVoxel/Mesh/MeshQuery.h"

namespace MyVoxel
{

// 将有效、非空且由调用者确认封闭的三角网格作为不可变连续几何体。
//
// Geometry_Mesh只执行几何有效性和退化三角形基础检查，不在构造时重复执行高成本拓扑验证。
// 调用者应在导入或建模完成阶段使用MeshValidation确认边界边、非流形边和非流形顶点。
// Display_Normal和Display_Color不参与内外判断，自相交检查仍由更高层验证流程负责。
class Geometry_Mesh : public Geometry_Shape
{
public:
    // 复制指定内部三角网格并建立空间查询数据。
    explicit Geometry_Mesh(const Mesh& mesh);
    // 移动指定内部三角网格并建立空间查询数据。
    explicit Geometry_Mesh(Mesh&& mesh);
    /// 几何属性
    // 返回封闭三角网格类型。
    ShapeKind kind() const override;
    // 返回当前几何持有的只读内部三角网格。
    const Mesh& mesh() const;

    /// 标准空间查询

    // 判断指定局部点是否位于封闭网格内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    // 返回指定局部轴对齐包围盒与封闭网格实体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;
    /// 快速空间查询
    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const override;

protected:
    ~Geometry_Mesh() override = default;

private:
    Mesh m_mesh; // 当前几何独占保存的不可变内部三角网格。
    MeshQuery m_query; // 根据m_mesh建立的连续几何查询数据。
};

}

#endif // MYVOXEL_MESH_GEOMETRY_MESH_H
