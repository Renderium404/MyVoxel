#ifndef MYVOXEL_MESHING_FEATUREPOINTSOLVER_H
#define MYVOXEL_MESHING_FEATUREPOINTSOLVER_H

#include <array>
#include <cstddef>

#include "MyMath/Vector3.h"

namespace MyVoxel
{
namespace Meshing
{

// 根据等值面交点及其法线求解特征敏感顶点。
//
// 求解器使用固定容量保存一个网格单元最多十二条边产生的切平面采样，
// 通过中心化QEF和对称矩阵伪逆计算平面、棱边或尖角的稳定交点。
class FeaturePointSolver
{
public:
    enum
    {
        MaximumSampleCount = 12
    };

public:
    // 构造不包含切平面采样的求解器。
    FeaturePointSolver();

    /// 采样管理

    // 删除当前全部切平面采样。
    void clear();

    // 添加一个等值面交点及其法线，法线无法归一化时返回false。
    bool addSample(const MyMath::Vector3& point, const MyMath::Vector3& normal);

    // 返回当前有效切平面采样数量。
    std::size_t sampleCount() const;

    /// 顶点求解

    // 返回全部交点的算术平均值，没有采样时返回零向量。
    MyMath::Vector3 averagePoint() const;

    // 使用中心化QEF求解特征点，至少保留一个有效特征方向时返回true。
    //
    // 当矩阵退化或结果无效时，将result设置为交点平均值并返回false。
    bool solve(MyMath::Vector3& result) const;

private:
    std::array<MyMath::Vector3, MaximumSampleCount> m_points; // 当前单元内的等值面交点。
    std::array<MyMath::Vector3, MaximumSampleCount> m_normals; // 与交点一一对应的单位法线。
    std::size_t m_sampleCount; // 当前有效采样数量。
};

}
}

#endif // MYVOXEL_MESHING_FEATUREPOINTSOLVER_H