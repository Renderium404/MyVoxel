#ifndef MYVOXEL_MODELING_FEATURE_FEATURETRACE_H
#define MYVOXEL_MODELING_FEATURE_FEATURETRACE_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"

namespace MyVoxel
{
namespace Modeling
{

// 保存一条尚未拓扑化的显式特征采样轨迹，轨迹点仅描述曲线几何，不代表Topology_Vertex。
class FeatureTrace
{
public:
    // 创建空特征轨迹。
    FeatureTrace();

    /// 状态判断

    // 判断当前轨迹是否没有任何特征点。
    bool isEmpty() const;
    // 判断当前轨迹是否至少包含一条由两个不同相邻点定义的线段。
    bool hasSegment() const;
    // 判断当前轨迹是否通过重复首点作为末点形成显式闭合序列。
    bool isClosed() const;

    /// 特征点访问

    // 返回当前轨迹中的特征点数量。
    std::size_t pointCount() const;
    // 返回指定编号的特征点。
    const MyMath::Vector3& point(std::size_t index) const;
    // 返回完整有序特征点序列。
    const std::vector<MyMath::Vector3>& points() const;
    // 返回轨迹第一个特征点。
    const MyMath::Vector3& startPoint() const;
    // 返回轨迹最后一个特征点。
    const MyMath::Vector3& endPoint() const;

    /// 特征点编辑

    // 将有限特征点追加到轨迹末端；与当前末点精确重复时不添加并返回false。
    bool addPoint(const MyMath::Vector3& point);
    // 删除轨迹最后一个特征点；空轨迹不修改并返回false。
    bool removeLastPoint();
    // 清空全部特征点。
    void clear();

private:
    std::vector<MyMath::Vector3> m_points; // 按特征曲线前进方向保存的几何采样点。
};

}
}

#endif // MYVOXEL_MODELING_FEATURE_FEATURETRACE_H