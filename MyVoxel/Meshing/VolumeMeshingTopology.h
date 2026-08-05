#ifndef MYVOXEL_MESHING_VOLUMEMESHINGTOPOLOGY_H
#define MYVOXEL_MESHING_VOLUMEMESHINGTOPOLOGY_H

#include <array>
#include <cstdint>

namespace MyVoxel
{
namespace Meshing
{

// 保存一个网格单元十二条边的局部表面连通分组。
struct VolumeCellTopology
{
    VolumeCellTopology();

    std::uint8_t signMask; // 当前拓扑对应的八位内部符号。
    std::uint8_t edgeGroupCount; // 当前单元需要生成的独立局部顶点数量。
    std::array<std::uint8_t, 12> edgeGroups; // 十二条边所属的顶点组，零表示不与等值面相交。
};

// 提供OpenVDB兼容的单元角点、边和边组拓扑计算。
class VolumeMeshingTopology
{
public:
    enum
    {
        CornerCount = 8,
        EdgeCount = 12,
        FaceCount = 6,
        SignConfigurationCount = 256,
        MaximumEdgeGroupCount = 4
    };

    /// 角点与边定义

    // 返回指定OpenVDB角点的X方向整数偏移。
    static int cornerOffsetX(unsigned int cornerIndex);

    // 返回指定OpenVDB角点的Y方向整数偏移。
    static int cornerOffsetY(unsigned int cornerIndex);

    // 返回指定OpenVDB角点的Z方向整数偏移。
    static int cornerOffsetZ(unsigned int cornerIndex);

    // 返回指定边的第一个角点编号。
    static unsigned int edgeCorner0(unsigned int edgeIndex);

    // 返回指定边的第二个角点编号。
    static unsigned int edgeCorner1(unsigned int edgeIndex);

    /// 符号与拓扑

    // 根据八个角点值生成内部符号掩码，值小于isoValue表示内部。
    static std::uint8_t computeSignMask(const std::array<float, CornerCount>& values, double isoValue);

    // 返回OpenVDB二义面编号，零表示当前符号构型不需要相邻单元修正。
    static std::uint8_t ambiguousFace(std::uint8_t signMask);

    // 使用面连通分析动态计算边组，作为独立标准路径和查找表生成基准。
    static VolumeCellTopology build(std::uint8_t signMask);

    // 通过预先生成的256项固定查找表返回边组拓扑。
    static const VolumeCellTopology& lookup(std::uint8_t signMask);

private:
    VolumeMeshingTopology() = delete;
};

}
}

#endif // MYVOXEL_MESHING_VOLUMEMESHINGTOPOLOGY_H