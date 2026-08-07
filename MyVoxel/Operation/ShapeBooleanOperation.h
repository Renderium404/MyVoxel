#ifndef MYVOXEL_OPERATION_SHAPEBOOLEANOPERATION_H
#define MYVOXEL_OPERATION_SHAPEBOOLEANOPERATION_H

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Operation/VoxelBooleanMask.h"
#include "MyVoxel/Topology/Shape.h"

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"
#include "MyVoxel/Operation/Algorithm/ShapeCutAlgorithm.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#endif

namespace MyVoxel
{
namespace Operation
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 保存一次连续Shape与VoxelShape布尔运算的高层执行统计。
struct ShapeBooleanStatistics
{
    ShapeBooleanStatistics();

    // 清空全部统计并恢复默认运算类型。
    void reset();

    VoxelBooleanType type; // 当前统计对应的布尔运算类型。
    bool usedDirectShapeCut; // 是否使用连续Shape直接差集快速路径。
    double totalMilliseconds; // 完整Shape布尔运算总墙钟耗时。
    double voxelizationMilliseconds; // 非差集操作中Shape对齐体素化耗时。
    double booleanMilliseconds; // 对齐VoxelShape离散布尔运算耗时。
    double shapeCutMilliseconds; // 连续Shape直接差集耗时。
    Modeling::VoxelizationStatistics voxelizationStatistics; // Shape对齐体素化统计。
    BooleanOperationStatistics voxelBooleanStatistics; // 离散体素布尔统计。
    Algorithm::ShapeCutStatistics shapeCutStatistics; // 连续Shape直接差集统计。
};

#endif

// 对VoxelShape和空间Shape执行统一布尔运算。
//
// Difference使用ShapeCutAlgorithm直接切削，不构造工具VoxelShape。
// Union、Intersection和ExclusiveOr先将Shape对齐体素化到object地址空间，再复用BooleanOperation。
// 因此四种操作最终都遵循object的VoxelGrid、transform和最高层采样精度。
class ShapeBooleanOperation
{
public:
    /// 通用布尔运算

    // 返回object与tool执行指定布尔运算后的VoxelShape。
    static VoxelShape apply(const VoxelShape& object, const Shape& tool, VoxelBooleanType type, VoxelChangeSet* changes = nullptr);
    // 将object与tool执行指定布尔运算后的结果写回object，返回是否发生实际变化。
    static bool applyInPlace(VoxelShape& object, const Shape& tool, VoxelBooleanType type, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回object与tool执行指定布尔运算后的VoxelShape并输出高层统计。
    static VoxelShape apply(const VoxelShape& object, const Shape& tool, VoxelBooleanType type,
                            VoxelChangeSet* changes, ShapeBooleanStatistics& statistics);
    // 原地执行指定Shape布尔运算并输出高层统计。
    static bool applyInPlace(VoxelShape& object, const Shape& tool, VoxelBooleanType type,
                             VoxelChangeSet* changes, ShapeBooleanStatistics& statistics);

#endif

    /// 并集

    // 返回object与tool的材料并集。
    static VoxelShape unite(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);
    // 将object与tool的材料并集写回object。
    static bool uniteInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);

    /// 交集

    // 返回object与tool的材料交集。
    static VoxelShape intersect(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);
    // 将object与tool的材料交集写回object。
    static bool intersectInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);

    /// 差集

    // 返回从object中直接减去连续tool后的体素体。
    static VoxelShape subtract(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);
    // 从object中直接减去连续tool并写回object。
    static bool subtractInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回连续Shape直接差集结果并输出切削统计。
    static VoxelShape subtract(const VoxelShape& object, const Shape& tool,
                               VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics);
    // 原地执行连续Shape直接差集并输出切削统计。
    static bool subtractInPlace(VoxelShape& object, const Shape& tool,
                                VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics);

#endif

    /// 异或

    // 返回object与tool的材料异或结果。
    static VoxelShape exclusiveOr(const VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);
    // 将object与tool的材料异或结果写回object。
    static bool exclusiveOrInPlace(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);

private:
    ShapeBooleanOperation() = delete;
};

}
}

#endif // MYVOXEL_OPERATION_SHAPEBOOLEANOPERATION_H
