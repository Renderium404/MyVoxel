#ifndef MYVOXEL_CORE_VOXELSHAPE_H
#define MYVOXEL_CORE_VOXELSHAPE_H

#include <cstddef>
#include <memory>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
#include "MyVoxel/Foundation/RefPtr.h"

namespace MyVoxel
{

// 标识当前VoxelShape显式表面特征是否已知完整。
enum class VoxelFeatureState
{
    Complete, // 当前FeatureSet完整表达已知显式锐特征，空集合也可能是正确结果。
    Unavailable // 当前TSDF有效，但显式特征尚未建立或已因直接场修改而失效。
};

// 表示由固定体素网格、固定截断距离、稀疏TSDF森林、显式表面特征和实例空间变换组成的离散体积。
//
// VoxelForest是当前材料场真值；VoxelFeatureSet保存零等值面上需要显式保留的高精度拓扑特征。
// 复制VoxelShape时共享网格、森林和FeatureSet，创建VoxelShapeSession时统一执行Shape级写时复制。
class VoxelShape
{
public:
    // 使用标准体素网格创建空体积，默认backgroundDistance等于最高层体素边长。
    VoxelShape();
    // 使用指定体素网格创建空体积，默认backgroundDistance等于最高层体素边长。
    explicit VoxelShape(const VoxelGrid& grid);
    // 使用指定体素网格和固定截断背景距离创建空体积。
    VoxelShape(const VoxelGrid& grid, float backgroundDistance);
    // 使用指定基础边长和最高层级创建空体积，默认backgroundDistance等于最高层体素边长。
    VoxelShape(double baseCellEdgeLength, VoxelLevel maximumLevel);
    // 使用指定基础边长、最高层级和固定截断背景距离创建空体积。
    VoxelShape(double baseCellEdgeLength, VoxelLevel maximumLevel, float backgroundDistance);
    // 使用指定原点、基础边长和最高层级创建空体积，默认backgroundDistance等于最高层体素边长。
    VoxelShape(const MyMath::Vector3& origin, double baseCellEdgeLength, VoxelLevel maximumLevel);
    // 使用指定原点、基础边长、最高层级和固定截断背景距离创建空体积。
    VoxelShape(const MyMath::Vector3& origin, double baseCellEdgeLength, VoxelLevel maximumLevel, float backgroundDistance);

    // 复制体素体并共享网格、TSDF森林和显式FeatureSet，空间变换独立复制。
    VoxelShape(const VoxelShape& other) = default;
    // 复制体素体并共享网格、TSDF森林和显式FeatureSet，空间变换独立复制。
    VoxelShape& operator=(const VoxelShape& other) = default;
    ~VoxelShape() = default;

    /// 状态判断

    // 检查共享数据、网格、TSDF森林、FeatureSet和空间变换是否有效。
    bool isValid() const;
    // 判断当前TSDF是否不包含任何显式根树；FeatureSet不参与材料场空状态判断。
    bool isEmpty() const;
    // 判断当前共享数据是否还被其他VoxelShape引用。
    bool isDataShared() const;
    // 判断当前体积是否与另一个体积共享同一份网格、TSDF森林和FeatureSet数据。
    bool sharesDataWith(const VoxelShape& other) const;

    /// 体素空间

    // 返回当前体积使用的只读体素网格。
    const VoxelGrid& grid() const;
    // 返回当前体积固定使用的正截断背景距离B。
    float backgroundDistance() const;
    // 判断指定体素地址的层级是否位于当前网格允许范围内。
    bool supportsAddress(const VoxelCellAddress& address) const;

    /// 体素与距离状态

    // 返回指定地址对应的逻辑体素状态。
    VoxelState state(const VoxelCellAddress& address) const;
    // 检查指定地址是否具有精确的逻辑节点表示。
    bool hasNode(const VoxelCellAddress& address) const;
    // 返回最高采样层级指定地址的TSDF距离；隐式Empty/Material分别返回+B/-B。
    float distance(const VoxelCellAddress& address) const;

    /// 修改入口

    // 创建当前体积的修改会话，并使用指定层级跟踪距离场变化区域。
    VoxelShapeSession session(VoxelLevel changeTrackingLevel);

    /// 体素数据

    // 返回当前体积的只读稀疏TSDF森林。
    const VoxelForest& forest() const;
    // 返回当前体积实际保存的第0层根树数量。
    std::size_t rootCount() const;

    /// 显式表面特征

    // 返回当前VoxelShape局部空间中的只读显式表面FeatureSet。
    const VoxelFeatureSet& features() const;
    // 返回当前显式表面特征的完整性状态。
    VoxelFeatureState featureState() const;
    // 判断当前FeatureSet是否已知完整；Complete且空集合可以表示Sphere等确认无锐特征的实体。
    bool hasCompleteFeatures() const;

    /// 空间变换

    // 返回当前体积从局部坐标到外部坐标的变换矩阵。
    const MyMath::Matrix4& transform() const;
    // 设置当前体积从局部坐标到外部坐标的仿射变换。
    void setTransform(const MyMath::Matrix4& transform);
    // 将当前体积的空间变换恢复为单位矩阵。
    void resetTransform();

private:
    // 保存可在多个VoxelShape之间共享的固定网格、TSDF森林和显式表面特征。
    class SharedData : public Foundation::ReferenceCounted
    {
    public:
        // 使用指定网格和固定截断距离创建空TSDF森林与空FeatureSet。
        SharedData(const VoxelGrid& gridValue, float backgroundDistance);
        // 复制网格、TSDF森林和FeatureSet；森林内部继续共享BlockPool，Topology特征身份保持共享不可变语义。
        SharedData(const SharedData& other);

        SharedData& operator=(const SharedData& other) = delete;

        VoxelGrid grid; // 当前体积使用的固定空间映射。
        VoxelForest forest; // 当前体积真正保存的稀疏截断有符号距离场。
        VoxelFeatureSet features; // 当前零等值面上需要显式保留的局部拓扑特征。
        VoxelFeatureState featureState; // 当前FeatureSet是否与TSDF保持完整一致。

    protected:
        ~SharedData() override = default;
    };

    // 当共享数据被多个体积引用时复制网格、森林根映射和FeatureSet，建立当前体积的独占状态。
    void detach();

private:
    friend class VoxelShapeSession;

private:
    Foundation::RefPtr<SharedData> m_data; // 支持写时复制的网格、TSDF森林和FeatureSet共享数据。
    MyMath::Matrix4 m_transform; // 当前体积独立保存的局部到外部空间变换。
};

}

#endif // MYVOXEL_CORE_VOXELSHAPE_H
