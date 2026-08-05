#ifndef MYVOXEL_BASE_BOUNDS3_H
#define MYVOXEL_BASE_BOUNDS3_H

#include <cstddef>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

namespace MyVoxel
{

// 表示与当前坐标系三条坐标轴平行的有限三维包围盒。
class Bounds3
{
public:
    enum
    {
        CornerCount = 8
    };

public:
    // 构造无有效范围的包围盒。
    Bounds3();
    // 使用最小角点和最大角点构造包围盒。
    Bounds3(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum);

    /// 包围盒创建
    // 使用中心点和各轴完整尺寸创建包围盒。
    static Bounds3 fromCenterAndSize(const MyMath::Vector3& center, const MyMath::Vector3& size);

    /// 状态判断
    // 判断当前包围盒是否具有有限且有序的范围。
    bool isValid() const;
    // 判断当前包围盒的三个方向是否都具有大于指定误差的长度。
    bool hasVolume(double epsilon = 0.0) const;
    // 判断两个包围盒是否具有相同有效状态和近似相同的角点。
    bool isEqualTo(const Bounds3& other, double epsilon = MyMath::Vector3::DefaultEpsilon) const;

    /// 范围访问
    // 返回包围盒最小角点。
    const MyMath::Vector3& minimum() const;
    // 返回包围盒最大角点。
    const MyMath::Vector3& maximum() const;
    // 返回包围盒中心点。
    MyMath::Vector3 center() const;
    // 返回包围盒在三个方向上的完整尺寸。
    MyMath::Vector3 size() const;
    // 返回包围盒在三个方向上的半尺寸。
    MyMath::Vector3 extent() const;

    // 返回包围盒体积，退化包围盒返回零。
    double volume() const;
    // 返回指定编号的包围盒角点，index范围为[0,7]，二进制第0、1、2位分别控制X、Y、Z坐标。
    MyMath::Vector3 corner(std::size_t index) const;

    /// 空间关系
    // 判断指定点是否位于包围盒内部或边界上。
    bool contains(const MyMath::Vector3& point, double tolerance = 0.0) const;
    // 判断指定包围盒是否完整位于当前包围盒内部或边界上。
    bool contains(const Bounds3& bounds, double tolerance = 0.0) const;
    // 判断两个包围盒是否相交或接触。
    bool intersects(const Bounds3& bounds, double tolerance = 0.0) const;

    /// 范围修改
    // 清除当前包围盒范围。
    void clear();
    // 将指定点包含到当前包围盒中。
    void include(const MyMath::Vector3& point);
    // 将指定包围盒包含到当前包围盒中，无效包围盒不产生影响。
    void include(const Bounds3& bounds);

    /// 范围变换
    // 返回沿三个方向向外扩展指定距离的包围盒。
    Bounds3 expanded(double distance) const;
    // 返回平移指定偏移后的包围盒。
    Bounds3 translated(const MyMath::Vector3& offset) const;
    // 返回经过仿射变换后重新计算的轴对齐包围盒。
    Bounds3 transformed(const MyMath::Matrix4& transform) const;

private:
    MyMath::Vector3 m_minimum; // 包围盒最小角点。
    MyMath::Vector3 m_maximum; // 包围盒最大角点。
    bool m_valid; // 当前包围盒是否包含有效范围。
};

}

#endif // MYVOXEL_BASE_BOUNDS3_H