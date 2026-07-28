#ifndef MYVOXEL_VOXELSHAPE_H
#define MYVOXEL_VOXELSHAPE_H

#include <memory>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "VoxelNodeForest.h"

namespace MyVoxel
{

// 表示由体素节点森林和空间变换描述的形体。
class VoxelShape
{
public:
    VoxelShape();
    VoxelShape(const VoxelShape&) = default;
    VoxelShape& operator=(const VoxelShape&) = default;
    virtual ~VoxelShape() = default;

    /// 体素配置

    // 返回第0层默认体素边长。
    double baseVoxelEdgeLength() const;

    // 设置第0层默认体素边长，并清除当前形体的节点森林。
    void setBaseVoxelEdgeLength(double edgeLength);

    // 返回形体允许使用的最高细分层级。
    VoxelLevel maximumLevel() const;

    // 设置形体允许使用的最高细分层级，并清除当前形体的节点森林。
    void setMaximumLevel(VoxelLevel level);

    // 返回指定层级的体素边长。
    double voxelEdgeLength(VoxelLevel level) const;

    /// 节点森林

    // 返回可修改的节点森林，必要时执行写时复制。
    VoxelNodeForest& forest();

    // 返回只读节点森林，不触发写时复制。
    const VoxelNodeForest& forest() const;

    // 清空当前形体的节点森林。
    void clear();

    // 检查当前形体是否与另一个形体共享体素数据。
    bool sharesDataWith(const VoxelShape& other) const;

    /// 连续空间查询

    // 返回指定局部坐标点在给定层级对应的体素地址。
    VoxelCellAddress localAddressAt(const MyMath::Vector3& point, VoxelLevel level) const;

    // 返回指定局部坐标点对应的实际节点状态。
    VoxelState stateAtLocalPoint(const MyMath::Vector3& point) const;

    // 检查指定局部坐标点是否位于材料区域。
    bool containsLocalPoint(const MyMath::Vector3& point) const;

    /// 空间变换

    // 返回当前形体的空间变换。
    const MyMath::Matrix4& transform() const;

    // 设置当前形体的空间变换。
    void setTransform(const MyMath::Matrix4& transform);

    // 将当前形体的空间变换恢复为单位矩阵。
    void resetTransform();

private:
    struct SharedData
    {
        VoxelNodeForest forest; // 当前形体的节点森林。
        double baseVoxelEdgeLength = 1.0; // 第0层默认体素边长。
        VoxelLevel maximumLevel = BaseVoxelLevel; // 允许使用的最高细分层级。
    };

    // 当体素数据被多个形体共享时创建当前形体的独立副本。
    void detach();

    std::shared_ptr<SharedData> m_data; // 支持写时复制的共享体素数据。
    MyMath::Matrix4 m_transform; // 当前形体独立保存的空间变换。
};

}

#endif // MYVOXEL_VOXELSHAPE_H