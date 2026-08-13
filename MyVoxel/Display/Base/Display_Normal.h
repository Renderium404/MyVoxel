#ifndef MYVOXEL_DISPLAY_BASE_DISPLAY_NORMAL_H
#define MYVOXEL_DISPLAY_BASE_DISPLAY_NORMAL_H

#include "MyMath/Vector3.h"

namespace MyVoxel
{

// 表示可选的局部显示法线，零向量明确表示当前顶点没有有效显示法线。
class Display_Normal
{
public:
    // 构造零法线。
    Display_Normal();
    // 使用指定有限分量构造显示法线。
    Display_Normal(double x, double y, double z);
    // 使用指定有限向量构造显示法线。
    explicit Display_Normal(const MyMath::Vector3& vector);

    /// 状态判断

    // 判断全部法线分量是否有限。
    bool isFinite() const;
    // 判断当前法线是否严格为零向量。
    bool isZero() const;
    // 判断当前法线是否为长度大于指定阈值的有效方向。
    bool isDirection(double epsilon = 1.0e-12) const;
    // 判断当前法线是否在指定误差内为单位向量。
    bool isUnit(double epsilon = 1.0e-5) const;

    /// 法线分量

    float x() const;
    float y() const;
    float z() const;
    MyMath::Vector3 vector() const;

    /// 法线创建

    // 返回单位化后的显示法线，当前法线必须为有效方向。
    Display_Normal normalized(double epsilon = 1.0e-12) const;
    // 返回方向相反的新显示法线。
    Display_Normal reversed() const;

    /// 比较

    bool operator==(const Display_Normal& other) const;
    bool operator!=(const Display_Normal& other) const;

private:
    float m_x; // 法线X分量。
    float m_y; // 法线Y分量。
    float m_z; // 法线Z分量。
};

}

#endif // MYVOXEL_DISPLAY_BASE_DISPLAY_NORMAL_H
