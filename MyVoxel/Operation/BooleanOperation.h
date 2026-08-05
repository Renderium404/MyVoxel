#ifndef MYVOXEL_OPERATION_BOOLEANOPERATION_H
#define MYVOXEL_OPERATION_BOOLEANOPERATION_H

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Operation/VoxelBooleanMask.h"
#include "MyVoxel/Geometry/ShapeInstance.h"
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
#include "MyVoxel/Operation/Algorithm/VoxelForestBooleanAlgorithm.h"
#endif

namespace MyVoxel
{
namespace Operation
{

namespace Algorithm
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
struct ShapeCutStatistics;
#endif

}

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 保存一次或多次体素体布尔运算的执行统计。
struct BooleanOperationStatistics
{
    // 清空全部统计数据。
    void reset();

    // 累加另一次布尔运算统计。
    void accumulate(const BooleanOperationStatistics& other);

    double totalMilliseconds = 0.0; // 体素体布尔运算总耗时。
    double detachMilliseconds = 0.0; // 分离或准备可写共享数据的耗时。
    double forestMilliseconds = 0.0; // 执行森林级布尔运算的耗时。

    std::size_t operationCount = 0; // 累计布尔运算次数。
    std::size_t returnedShapeOperationCount = 0; // 返回新VoxelShape的运算次数。
    std::size_t inPlaceOperationCount = 0; // 原地修改VoxelShape的运算次数。
    std::size_t directResultCount = 0; // 通过确定性快速路径完成的次数。
    std::size_t sharedTargetCopyCount = 0; // 原地操作因目标数据共享而使用临时副本的次数。

    std::size_t inputLeftRootCount = 0; // 左操作数累计根树数量。
    std::size_t inputRightRootCount = 0; // 右操作数累计根树数量。
    std::size_t outputRootCount = 0; // 结果累计根树数量。

    Algorithm::VoxelForestBooleanStatistics forestStatistics; // 森林级和树级累计统计。
};

#endif

// 对完全对齐的VoxelShape执行快速离散布尔运算。
//
// 两个操作数必须使用完全一致的VoxelGrid和空间变换。
// 返回值接口使用写时复制生成结果；原地接口优先直接修改独占目标数据。
class BooleanOperation
{
public:
    /// 空间兼容性

    // 判断两个体素体是否能够直接按相同体素地址执行快速布尔运算。
    static bool isAligned(const VoxelShape& left, const VoxelShape& right);

    /// 通用布尔运算

    // 返回指定布尔运算结果。
    static VoxelShape apply(const VoxelShape& left, const VoxelShape& right, VoxelBooleanType type, VoxelChangeSet* changes = nullptr);

    // 将指定布尔运算结果写回left，返回left是否发生实际变化。
    static bool applyInPlace(VoxelShape& left, const VoxelShape& right, VoxelBooleanType type, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回指定布尔运算结果并输出执行统计。
    static VoxelShape apply(const VoxelShape& left, const VoxelShape& right, VoxelBooleanType type,
                            VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

    // 将指定布尔运算结果写回left并输出执行统计，返回left是否发生实际变化。
    static bool applyInPlace(VoxelShape& left, const VoxelShape& right, VoxelBooleanType type,
                             VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

#endif

    /// 并集

    // 返回左右体素材料并集。
    static VoxelShape unite(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

    // 将左右体素材料并集写回left，返回left是否发生实际变化。
    static bool uniteInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回左右体素材料并集并输出执行统计。
    static VoxelShape unite(const VoxelShape& left, const VoxelShape& right,
                            VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

    // 将左右体素材料并集写回left并输出执行统计。
    static bool uniteInPlace(VoxelShape& left, const VoxelShape& right,
                             VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

#endif

    /// 交集

    // 返回左右体素材料交集。
    static VoxelShape intersect(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

    // 将左右体素材料交集写回left，返回left是否发生实际变化。
    static bool intersectInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回左右体素材料交集并输出执行统计。
    static VoxelShape intersect(const VoxelShape& left, const VoxelShape& right,
                                VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

    // 将左右体素材料交集写回left并输出执行统计。
    static bool intersectInPlace(VoxelShape& left, const VoxelShape& right,
                                 VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

#endif

    /// 差集

    // 返回从left中减去right后的体素材料。
    static VoxelShape subtract(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

    // 从left中原地减去right，返回left是否发生实际变化。
    static bool subtractInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回从left中减去right后的体素材料并输出执行统计。
    static VoxelShape subtract(const VoxelShape& left, const VoxelShape& right,
                               VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

    // 从left中原地减去right并输出执行统计。
    static bool subtractInPlace(VoxelShape& left, const VoxelShape& right,
                                VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

#endif

    /// 异或

    // 返回左右体素材料异或结果。
    static VoxelShape exclusiveOr(const VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

    // 将左右体素材料异或结果写回left，返回left是否发生实际变化。
    static bool exclusiveOrInPlace(VoxelShape& left, const VoxelShape& right, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 返回左右体素材料异或结果并输出执行统计。
    static VoxelShape exclusiveOr(const VoxelShape& left, const VoxelShape& right,
                                  VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

    // 将左右体素材料异或结果写回left并输出执行统计。
    static bool exclusiveOrInPlace(VoxelShape& left, const VoxelShape& right,
                                   VoxelChangeSet* changes, BooleanOperationStatistics& statistics);

#endif
/// 连续几何差集

// 返回从object中减去连续几何实例tool后的体素体。
static VoxelShape subtract(const VoxelShape& object, const Geometry::ShapeInstance& tool,
                           VoxelChangeSet* changes = nullptr);

// 从object中原地减去连续几何实例tool，返回object是否发生实际变化。
static bool subtractInPlace(VoxelShape& object, const Geometry::ShapeInstance& tool,
                            VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 返回连续几何差集结果并输出几何切削统计。
static VoxelShape subtract(const VoxelShape& object, const Geometry::ShapeInstance& tool,
                           VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics);

// 原地执行连续几何差集并输出几何切削统计。
static bool subtractInPlace(VoxelShape& object, const Geometry::ShapeInstance& tool,
                            VoxelChangeSet* changes, Algorithm::ShapeCutStatistics& statistics);

#endif
private:
    BooleanOperation() = delete;
};

}
}

#endif // MYVOXEL_OPERATION_BOOLEANOPERATION_H