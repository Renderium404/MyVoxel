#ifndef MYVOXEL_CORE_VOXELSHAPE_H
#define MYVOXEL_CORE_VOXELSHAPE_H

#include <cstddef>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

// 表示由体素网格、稀疏体素森林和实例空间变换组成的离散体积。
//
// 复制VoxelShape时共享体素网格和森林数据，创建VoxelShapeSession时执行Shape级写时复制。
// 空间变换由每个VoxelShape独立保存，修改变换不会复制体素数据。
class VoxelShape
{
public:
    // 使用原点位于世界原点、基础边长为1且最高层级为第0层的体素网格创建空体积。
    VoxelShape();
    // 使用指定体素网格创建空体积。
    explicit VoxelShape(const VoxelGrid& grid);
    // 使用指定基础体素边长和最高层级创建原点位于世界原点的空体积。
    VoxelShape(double baseCellEdgeLength, VoxelLevel maximumLevel);
    // 使用指定网格原点、基础体素边长和最高层级创建空体积。
    VoxelShape(const MyMath::Vector3& origin,
               double baseCellEdgeLength,
               VoxelLevel maximumLevel);

    // 复制体素体并共享体素网格和森林数据，空间变换独立复制。
    VoxelShape(const VoxelShape& other) = default;
    // 复制体素体并共享体素网格和森林数据，空间变换独立复制。
    VoxelShape& operator=(const VoxelShape& other) = default;
    ~VoxelShape() = default;

    /// 状态判断
    // 检查共享数据、体素网格和空间变换是否有效。
    bool isValid() const;
    // 判断当前体积是否不包含任何显式根树。
    bool isEmpty() const;
    // 判断当前体积数据是否还被其他VoxelShape共享。
    bool isDataShared() const;
    // 判断当前体积是否与另一个体积共享同一份体素网格和森林数据。
    bool sharesDataWith(const VoxelShape& other) const;

    /// 体素空间
    // 返回当前体积使用的只读体素网格。
    const VoxelGrid& grid() const;
    // 判断指定体素地址的层级是否位于当前网格允许范围内。
    bool supportsAddress(const VoxelCellAddress& address) const;

    /// 体素状态
    // 返回指定地址对应的逻辑体素状态。
    VoxelState state(const VoxelCellAddress& address) const;
    // 检查指定地址是否具有独立的显式节点表示。
    bool hasNode(const VoxelCellAddress& address) const;

    /// 修改入口
    // 创建当前体积的修改会话，并使用指定层级跟踪材料变化区域。
    VoxelShapeSession session(VoxelLevel changeTrackingLevel);

    /// 体素数据
    // 返回当前体积的只读稀疏体素森林。
    const VoxelForest& forest() const;
    // 返回当前体积实际保存的第0层根树数量。
    std::size_t rootCount() const;

    /// 空间变换
    // 返回当前体积从局部坐标到外部坐标的变换矩阵。
    const MyMath::Matrix4& transform() const;
    // 设置当前体积从局部坐标到外部坐标的仿射变换。
    void setTransform(const MyMath::Matrix4& transform);
    // 将当前体积的空间变换恢复为单位矩阵。
    void resetTransform();

private:
    // 保存可以在多个VoxelShape之间共享的体素网格和森林数据。
    class SharedData : public Foundation::ReferenceCounted
    {
    public:
        // 使用指定体素网格创建空共享数据。
        explicit SharedData(const VoxelGrid& gridValue);

        // 复制体素网格和森林根映射，森林内部根树继续共享节点池。
        SharedData(const SharedData& other);

        SharedData& operator=(const SharedData& other) = delete;

        VoxelGrid grid; // 当前体积使用的固定空间映射。
        VoxelForest forest; // 当前体积保存的稀疏体素材料和显式节点结构。

    protected:
        ~SharedData() override = default;
    };

    // 当共享数据被多个体积引用时创建当前体积的独立数据副本。
    void detach();

private:
    friend class VoxelShapeSession;

private:
    Foundation::RefPtr<SharedData> m_data; // 支持写时复制的体素网格和森林数据。
    MyMath::Matrix4 m_transform; // 当前体积独立保存的局部到外部空间变换。
};

}

#endif // MYVOXEL_CORE_VOXELSHAPE_H