#ifndef MYVOXEL_DISPLAY_BASE_DISPLAY_COLOR_H
#define MYVOXEL_DISPLAY_BASE_DISPLAY_COLOR_H

namespace MyVoxel
{

// 表示与具体显示后端无关的线性RGBA颜色，各分量使用[0,1]范围的单精度浮点数。
class Display_Color
{
public:
    // 构造不透明白色。
    Display_Color();
    // 使用指定线性RGBA分量构造颜色，各分量必须有限且位于[0,1]。
    Display_Color(double red, double green, double blue, double alpha = 1.0);

    /// 状态判断

    // 判断全部颜色分量是否有限且位于[0,1]。
    bool isValid() const;

    /// 颜色分量

    // 返回红色分量。
    float red() const;
    // 返回绿色分量。
    float green() const;
    // 返回蓝色分量。
    float blue() const;
    // 返回透明度分量。
    float alpha() const;

    /// 颜色创建

    // 返回仅替换透明度分量的新颜色。
    Display_Color withAlpha(double alpha) const;
    // 返回当前颜色到目标颜色按指定比例线性插值得到的新颜色，factor必须位于[0,1]。
    Display_Color interpolated(const Display_Color& target, double factor) const;
    // 返回RGB分量取反且透明度保持不变的新颜色。
    Display_Color inverted() const;

    /// 比较

    bool operator==(const Display_Color& other) const;
    bool operator!=(const Display_Color& other) const;

    /// 标准颜色

    static Display_Color black();
    static Display_Color white();
    static Display_Color gray();
    static Display_Color redColor();
    static Display_Color greenColor();
    static Display_Color blueColor();
    static Display_Color cyan();
    static Display_Color magenta();
    static Display_Color yellow();
    static Display_Color transparent();

private:
    float m_red; // 线性红色分量。
    float m_green; // 线性绿色分量。
    float m_blue; // 线性蓝色分量。
    float m_alpha; // 透明度分量。
};

}

#endif // MYVOXEL_DISPLAY_BASE_DISPLAY_COLOR_H
