#ifndef MYVOXEL_OPERATION_ALGORITHM_SHAPECUTPLAN_H
#define MYVOXEL_OPERATION_ALGORITHM_SHAPECUTPLAN_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "ShapeCutContext.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 表示一个切削计划节点需要执行的操作。
enum class CutPlanAction : std::uint8_t
{
    Keep,
    Remove,
    Subdivide
};

// 表示一个连续Shape切削计划节点。
struct CutPlanNode
{
    CutPlanNode();
    CutPlanNode(CutPlanAction actionValue, VoxelState resultStateValue, std::uint32_t childGroupIndexValue);

    CutPlanAction action; // 当前计划节点的处理动作。
    VoxelState resultState; // 应用当前计划后该节点的最终状态。
    std::uint32_t childGroupIndex; // Subdivide动作连接的八子节点组索引。
};

// 表示一个切削计划八子节点组。
struct CutPlanNodeGroup
{
    std::array<CutPlanNode, VoxelCornerCount> children; // 八个角点对应的切削计划节点。
};

// 保存一个第0层根节点的完整切削计划。
struct RootCutPlan
{
    RootCutPlan();

    // 分配一个切削计划子节点组并返回其索引。
    std::uint32_t allocateNodeGroup();

    CutPlanNode root; // 当前第0层根节点的切削计划。
    std::vector<CutPlanNodeGroup> nodeGroups; // 当前根计划使用的全部八子节点组。
};

// 保存一个已经在主线程完成根级写时复制的计划应用任务。
struct RootPlanApplication
{
    RootPlanApplication(VoxelForest& forest, const VoxelCellAddress& rootAddress, const RootCutPlan& planValue);

    VoxelTreeEditor editor; // 当前任务独占修改的根树编辑器。
    const RootCutPlan* plan; // 当前根树对应的只读切削计划。
};

// 返回指定数量独立任务实际使用的工作线程数量。
unsigned int resolveParallelWorkerCount(const BooleanOperationOptions& options, std::size_t taskCount);

// 为全部候选根节点并行生成切削计划。
void buildShapeCutPlans(const VoxelShape& object,
                        const Geometry::ShapeInstance& tool,
                        const ShapeToolContext& context,
                        const std::vector<VoxelCellAddress>& rootCells,
                        unsigned int workerCount,
                        std::vector<RootCutPlan>& plans);

// 检查切削计划集合是否包含实际修改。
bool hasShapeCutPlanModification(const std::vector<RootCutPlan>& plans);

// 并行应用全部已经完成根级写时复制的切削计划。
void applyShapeCutPlans(std::vector<RootPlanApplication>& applications, unsigned int workerCount);

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_SHAPECUTPLAN_H