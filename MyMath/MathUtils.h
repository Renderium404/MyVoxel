#ifndef MYMATH_MATH_H
#define MYMATH_MATH_H

namespace MyMath
{

/// 数学常量

// 圆周率。
extern const double Pi;

// 二分之一圆周率。
extern const double HalfPi;

// 两倍圆周率。
extern const double TwoPi;

/// 数值状态

// 判断数值是否为有限值，不包含NaN和正负无穷。
bool isFinite(double value);

// 判断数值是否为NaN。
bool isNaN(double value);

// 返回双精度静默NaN。
double quietNaN();

/// 标量计算

// 将数值限制在闭区间[minimum, maximum]内，调用者必须保证minimum不大于maximum。
double clamp(double value, double minimum, double maximum);

// 返回两个数值绝对值中的最大值。
double maximumAbsolute(double first, double second);

// 返回三个数值绝对值中的最大值。
double maximumAbsolute(double first, double second, double third);

// 返回四个数值绝对值中的最大值。
double maximumAbsolute(double first, double second, double third, double fourth);

/// 稳定模长

// 返回两个分量按统一尺度缩放后的模长，调用者必须保证scale为非负有限值。
double scaledNorm(double first, double second, double scale);

// 返回三个分量按统一尺度缩放后的模长，调用者必须保证scale为非负有限值。
double scaledNorm(double first, double second, double third, double scale);

// 返回四个分量按统一尺度缩放后的模长，调用者必须保证scale为非负有限值。
double scaledNorm(double first, double second, double third, double fourth, double scale);

// 返回两个分量组成数据的稳定模长。
double norm(double first, double second);

// 返回三个分量组成数据的稳定模长。
double norm(double first, double second, double third);

// 返回四个分量组成数据的稳定模长。
double norm(double first, double second, double third, double fourth);

/// 角度转换

// 将角度转换为弧度。
double degreesToRadians(double degrees);

// 将弧度转换为角度。
double radiansToDegrees(double radians);

}

#endif // MYMATH_MATH_H