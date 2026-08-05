#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/Tree/VoxelForest.h"
#include "MyVoxel/Core/Tree/VoxelTree.h"
#include "MyVoxel/Core/Tree/VoxelTreeCursor.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#ifndef MYVOXEL_BENCHMARK_LEGACY_CORE_API
#include "MyVoxel/Core/VoxelShapeSession.h"
#endif
#include "MyVoxel/Operation/Algorithm/VoxelTreeBooleanAlgorithm.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "MyVoxel/Operation/VoxelBooleanMask.h"

namespace
{

using Clock = std::chrono::steady_clock;

const std::size_t DefaultTimingSampleCount = 9; // 每项默认采集九次正式计时并使用中位数。
const std::size_t DefaultWarmupSampleCount = 2; // 每项正式计时前执行两次完整预热。
const double DefaultMaximumSlowdownPercent = 10.0; // 当前版本相对基线默认最多允许10%的中位数回退。
const double DefaultAbsoluteToleranceNanoseconds = 2.0; // 极短基准额外允许2ns绝对计时波动。

volatile std::uint64_t ResultSink = 0; // 防止编译器删除基准中的可观察结果。

struct BenchmarkConfiguration
{
    BenchmarkConfiguration()
        : quick(false)
        , timingSampleCount(DefaultTimingSampleCount)
        , warmupSampleCount(DefaultWarmupSampleCount)
        , maximumSlowdownPercent(DefaultMaximumSlowdownPercent)
        , absoluteToleranceNanoseconds(DefaultAbsoluteToleranceNanoseconds)
    {
    }

    bool quick;
    std::size_t timingSampleCount;
    std::size_t warmupSampleCount;
    double maximumSlowdownPercent;
    double absoluteToleranceNanoseconds;
    std::string writeBaselinePath;
    std::string compareBaselinePath;
};

struct BenchmarkResult
{
    BenchmarkResult()
        : nanosecondsPerOperation(0.0)
        , operationCount(0)
    {
    }

    BenchmarkResult(const std::string& nameValue,
                    double nanosecondsPerOperationValue,
                    std::uint64_t operationCountValue)
        : name(nameValue)
        , nanosecondsPerOperation(nanosecondsPerOperationValue)
        , operationCount(operationCountValue)
    {
    }

    std::string name;
    double nanosecondsPerOperation;
    std::uint64_t operationCount;
};

struct BaselineEntry
{
    BaselineEntry()
        : nanosecondsPerOperation(0.0)
        , operationCount(0)
    {
    }

    BaselineEntry(double nanosecondsPerOperationValue,
                  std::uint64_t operationCountValue)
        : nanosecondsPerOperation(nanosecondsPerOperationValue)
        , operationCount(operationCountValue)
    {
    }

    double nanosecondsPerOperation;
    std::uint64_t operationCount;
};

struct BenchmarkScale
{
    BenchmarkScale(std::size_t quickCycleCountValue,
                   std::size_t normalCycleCountValue)
        : quickCycleCount(quickCycleCountValue)
        , normalCycleCount(normalCycleCountValue)
    {
    }

    std::size_t cycles(const BenchmarkConfiguration& configuration) const
    {
        return configuration.quick ? quickCycleCount : normalCycleCount;
    }

    std::size_t quickCycleCount;
    std::size_t normalCycleCount;
};

// 返回两个时刻之间经过的纳秒数。
double elapsedNanoseconds(const Clock::time_point& start,
                          const Clock::time_point& end)
{
    return std::chrono::duration_cast<std::chrono::duration<double, std::nano> >(end - start).count();
}

// 返回数值数组的中位数。
double median(std::vector<double> values)
{
    if (values.empty())
    {
        return 0.0;
    }

    std::sort(values.begin(), values.end());

    const std::size_t middle = values.size() / 2;

    if ((values.size() & 1U) != 0)
    {
        return values[middle];
    }

    return (values[middle - 1] + values[middle]) * 0.5;
}

// 使用固定整数混合函数生成可重复的非规则64位模式。
std::uint64_t mixedBits(std::uint64_t value)
{
    value += static_cast<std::uint64_t>(0x9E3779B97F4A7C15ULL);
    value = (value ^ (value >> 30U)) * static_cast<std::uint64_t>(0xBF58476D1CE4E5B9ULL);
    value = (value ^ (value >> 27U)) * static_cast<std::uint64_t>(0x94D049BB133111EBULL);
    return value ^ (value >> 31U);
}

// 返回既非全空也非全材料的稳定叶掩码。
std::uint64_t materialMask(std::uint64_t seed)
{
    std::uint64_t mask = mixedBits(seed);

    // 固定保留一个材料位和一个空位，避免随机模式退化为终止体素。
    mask |= static_cast<std::uint64_t>(1ULL);
    mask &= ~static_cast<std::uint64_t>(1ULL << 63U);
    return mask;
}

// 判断当前编辑器是否支持覆盖后两层的64位快速材料写入。
bool canSetEditorMaterialMask(const MyVoxel::VoxelTreeEditor& editor)
{
#ifdef MYVOXEL_BENCHMARK_LEGACY_CORE_API
    return editor.canStoreDirectMaskLeaf();
#else
    return editor.canSetMaterialMask();
#endif
}

// 使用当前Core版本对应的接口写入64位材料掩码。
bool setEditorMaterialMask(MyVoxel::VoxelTreeEditor& editor,
                           std::uint64_t mask)
{
#ifdef MYVOXEL_BENCHMARK_LEGACY_CORE_API
    return editor.setMaskLeafMaterialMask(mask);
#else
    return editor.setMaterialMask(mask);
#endif
}

// 调用当前编译配置下的树级布尔算法。
bool applyTreeBoolean(MyVoxel::VoxelTree& left,
                      const MyVoxel::VoxelTree& right,
                      MyVoxel::Operation::VoxelBooleanType type)
{
#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    MyVoxel::Operation::Algorithm::VoxelTreeBooleanStatistics statistics;
    return MyVoxel::Operation::Algorithm::VoxelTreeBooleanAlgorithm::apply(
        left,
        right,
        type,
        statistics);
#else
    return MyVoxel::Operation::Algorithm::VoxelTreeBooleanAlgorithm::apply(
        left,
        right,
        type);
#endif
}

// 递归构造具有普通分支和MaskLeaf的确定性测试子树。
void buildPatternSubtree(MyVoxel::VoxelTreeEditor editor,
                         unsigned int remainingLevelCount,
                         std::uint64_t seed)
{
    if (remainingLevelCount == 0)
    {
        if ((mixedBits(seed) & static_cast<std::uint64_t>(1ULL)) != 0)
        {
            editor.setMaterial();
        }
        else
        {
            editor.setEmpty();
        }

        return;
    }

    if (remainingLevelCount == 2 && canSetEditorMaterialMask(editor))
    {
        setEditorMaterialMask(editor, materialMask(seed));
        return;
    }

    editor.subdivide();

    for (unsigned int cornerIndex = 0;
         cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount);
         ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner =
            static_cast<MyVoxel::VoxelCorner>(cornerIndex);

        buildPatternSubtree(
            editor.child(corner),
            remainingLevelCount - 1,
            mixedBits(seed + static_cast<std::uint64_t>(cornerIndex + 1)));
    }
}

// 创建包含指定逻辑深度和稳定材料模式的体素树。
MyVoxel::VoxelTree makePatternTree(unsigned int depth,
                                   std::uint64_t seed)
{
    MyVoxel::VoxelTree tree(MyVoxel::VoxelState::Empty);
    buildPatternSubtree(tree.editor(), depth, seed);
    return tree;
}

// 递归构造同时包含终止体素、普通分支和MaskLeaf的混合测试子树。
void buildMixedPatternSubtree(MyVoxel::VoxelTreeEditor editor,
                              unsigned int remainingLevelCount,
                              unsigned int initialLevelCount,
                              std::uint64_t seed)
{
    if (remainingLevelCount == 0)
    {
        if ((mixedBits(seed) & static_cast<std::uint64_t>(1ULL)) != 0)
        {
            editor.setMaterial();
        }
        else
        {
            editor.setEmpty();
        }

        return;
    }

    // 根始终保持细分；中间层约四分之一位置提前终止，用于触发复制、反转和直接裁决动作。
    if (remainingLevelCount < initialLevelCount &&
        remainingLevelCount > 2)
    {
        const std::uint64_t selector = mixedBits(seed) & static_cast<std::uint64_t>(7ULL);

        if (selector == 0)
        {
            editor.setEmpty();
            return;
        }

        if (selector == 1)
        {
            editor.setMaterial();
            return;
        }
    }

    if (remainingLevelCount == 2 && canSetEditorMaterialMask(editor))
    {
        setEditorMaterialMask(editor, materialMask(seed));
        return;
    }

    editor.subdivide();

    for (unsigned int cornerIndex = 0;
         cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount);
         ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner =
            static_cast<MyVoxel::VoxelCorner>(cornerIndex);

        buildMixedPatternSubtree(
            editor.child(corner),
            remainingLevelCount - 1,
            initialLevelCount,
            mixedBits(seed + static_cast<std::uint64_t>(cornerIndex + 1)));
    }
}

// 创建用于覆盖七动作混合递归路径的确定性体素树。
MyVoxel::VoxelTree makeMixedPatternTree(unsigned int depth,
                                        std::uint64_t seed)
{
    MyVoxel::VoxelTree tree(MyVoxel::VoxelState::Empty);
    buildMixedPatternSubtree(tree.editor(), depth, depth, seed);
    return tree;
}

// 递归比较两个逻辑体素子树的材料结果。
bool sameSubtree(const MyVoxel::VoxelTreeCursor& first,
                 const MyVoxel::VoxelTreeCursor& second)
{
    if (first.state() != second.state())
    {
        return false;
    }

    if (first.isTerminal())
    {
        return true;
    }

    for (unsigned int cornerIndex = 0;
         cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount);
         ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner =
            static_cast<MyVoxel::VoxelCorner>(cornerIndex);

        if (!sameSubtree(first.child(corner), second.child(corner)))
        {
            return false;
        }
    }

    return true;
}

// 比较两棵体素树的完整逻辑材料结果。
bool sameTree(const MyVoxel::VoxelTree& first,
              const MyVoxel::VoxelTree& second)
{
    return sameSubtree(first.cursor(), second.cursor());
}

// 将线性根序号转换为稳定三维根索引。
MyVoxel::VoxelCellIndex rootIndex(std::size_t position)
{
    const MyVoxel::VoxelIndex x =
        static_cast<MyVoxel::VoxelIndex>(position & 15U);

    const MyVoxel::VoxelIndex y =
        static_cast<MyVoxel::VoxelIndex>((position >> 4U) & 15U);

    const MyVoxel::VoxelIndex z =
        static_cast<MyVoxel::VoxelIndex>(position >> 8U);

    return MyVoxel::VoxelCellIndex(x, y, z);
}

// 创建包含指定数量独立根树的体素体。
MyVoxel::VoxelShape makePatternShape(const MyVoxel::VoxelGrid& grid,
                                     std::size_t rootCount,
                                     unsigned int treeDepth,
                                     std::uint64_t seed)
{
    MyVoxel::VoxelShape shape(grid);

#ifdef MYVOXEL_BENCHMARK_LEGACY_CORE_API
    for (std::size_t rootPosition = 0;
         rootPosition < rootCount;
         ++rootPosition)
    {
        shape.editForest().setTree(
            rootIndex(rootPosition),
            makePatternTree(
                treeDepth,
                mixedBits(seed + static_cast<std::uint64_t>(rootPosition))));
    }
#else
    {
        MyVoxel::VoxelShapeSession session(
            shape,
            MyVoxel::BaseVoxelLevel);

        for (std::size_t rootPosition = 0;
             rootPosition < rootCount;
             ++rootPosition)
        {
            session.setTree(
                rootIndex(rootPosition),
                makePatternTree(
                    treeDepth,
                    mixedBits(seed + static_cast<std::uint64_t>(rootPosition))));
        }
    }
#endif

    return shape;
}

// 比较两个体素体的全部根和逻辑树结果。
bool sameShape(const MyVoxel::VoxelShape& first,
               const MyVoxel::VoxelShape& second)
{
    if (!MyVoxel::Operation::BooleanOperation::isAligned(first, second) ||
        first.rootCount() != second.rootCount())
    {
        return false;
    }

    bool equal = true;

    first.forest().forEachRootCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            if (!equal)
            {
                return;
            }

            const MyVoxel::VoxelTree* firstTree =
                first.forest().getTree(address.index);

            const MyVoxel::VoxelTree* secondTree =
                second.forest().getTree(address.index);

            equal = firstTree && secondTree &&
                    sameTree(*firstTree, *secondTree);
        });

    return equal;
}

// 通过空会话取得调用者可持有的指定层级变化集合。
MyVoxel::VoxelChangeSet makeEmptyChangeSet(const MyVoxel::VoxelGrid& grid,
                                           MyVoxel::VoxelLevel trackingLevel)
{
#ifdef MYVOXEL_BENCHMARK_LEGACY_CORE_API
    static_cast<void>(grid);
    static_cast<void>(trackingLevel);
    return MyVoxel::VoxelChangeSet();
#else
    MyVoxel::VoxelShape shape(grid);
    MyVoxel::VoxelShapeSession session(shape, trackingLevel);
    return session.takeChanges();
#endif
}

// 执行预热和多轮计时，并返回每个逻辑布尔操作的中位耗时。
template<typename BatchFunction>
BenchmarkResult benchmark(const std::string& name,
                          std::size_t operationCountPerSample,
                          const BenchmarkConfiguration& configuration,
                          BatchFunction batchFunction)
{
    for (std::size_t warmupIndex = 0;
         warmupIndex < configuration.warmupSampleCount;
         ++warmupIndex)
    {
        ResultSink += batchFunction();
    }

    std::vector<double> samples;
    samples.reserve(configuration.timingSampleCount);

    for (std::size_t sampleIndex = 0;
         sampleIndex < configuration.timingSampleCount;
         ++sampleIndex)
    {
        const Clock::time_point start = Clock::now();
        const std::uint64_t checksum = batchFunction();
        const Clock::time_point end = Clock::now();

        ResultSink += checksum;

        samples.push_back(
            elapsedNanoseconds(start, end) /
            static_cast<double>(operationCountPerSample));
    }

    return BenchmarkResult(
        name,
        median(samples),
        static_cast<std::uint64_t>(operationCountPerSample) *
            static_cast<std::uint64_t>(configuration.timingSampleCount));
}

// 测量根终止状态之间的O(1)直接裁决路径。
BenchmarkResult benchmarkTreeRootToggle(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t cycleCount =
        BenchmarkScale(100000, 1000000).cycles(configuration);

    MyVoxel::VoxelTree left(MyVoxel::VoxelState::Empty);
    const MyVoxel::VoxelTree right(MyVoxel::VoxelState::Material);

    return benchmark(
        "tree_root_direct_toggle",
        cycleCount * 2,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t changedCount = 0;

            for (std::size_t cycleIndex = 0;
                 cycleIndex < cycleCount;
                 ++cycleIndex)
            {
                changedCount +=
                    applyTreeBoolean(
                        left,
                        right,
                        MyVoxel::Operation::VoxelBooleanType::Union)
                        ? 1U
                        : 0U;

                changedCount +=
                    applyTreeBoolean(
                        left,
                        right,
                        MyVoxel::Operation::VoxelBooleanType::Difference)
                        ? 1U
                        : 0U;
            }

            return changedCount +
                   static_cast<std::uint64_t>(left.state());
        });
}

// 测量单根内八个MaskLeaf参与64位异或的热点路径。
BenchmarkResult benchmarkTreeMaskLeafXor(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t cycleCount =
        BenchmarkScale(5000, 50000).cycles(configuration);

    MyVoxel::VoxelTree left = makePatternTree(3, 11);
    const MyVoxel::VoxelTree original = left;
    const MyVoxel::VoxelTree right = makePatternTree(3, 29);

    const BenchmarkResult result = benchmark(
        "tree_mask_leaf_xor",
        cycleCount * 2,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t changedCount = 0;

            for (std::size_t cycleIndex = 0;
                 cycleIndex < cycleCount;
                 ++cycleIndex)
            {
                changedCount +=
                    applyTreeBoolean(
                        left,
                        right,
                        MyVoxel::Operation::VoxelBooleanType::ExclusiveOr)
                        ? 1U
                        : 0U;

                changedCount +=
                    applyTreeBoolean(
                        left,
                        right,
                        MyVoxel::Operation::VoxelBooleanType::ExclusiveOr)
                        ? 1U
                        : 0U;
            }

            return changedCount +
                   static_cast<std::uint64_t>(left.allocatedGroupCount());
        });

    if (!sameTree(left, original))
    {
        std::cerr << "MaskLeaf XOR round trip changed the original tree." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return result;
}

// 测量深层普通分支和大量MaskLeaf混合递归路径。
BenchmarkResult benchmarkTreeDeepXor(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t cycleCount =
        BenchmarkScale(50, 500).cycles(configuration);

    MyVoxel::VoxelTree left = makeMixedPatternTree(5, 101);
    const MyVoxel::VoxelTree original = left;
    const MyVoxel::VoxelTree right = makeMixedPatternTree(5, 307);

    const BenchmarkResult result = benchmark(
        "tree_deep_recursive_xor",
        cycleCount * 2,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t changedCount = 0;

            for (std::size_t cycleIndex = 0;
                 cycleIndex < cycleCount;
                 ++cycleIndex)
            {
                changedCount +=
                    applyTreeBoolean(
                        left,
                        right,
                        MyVoxel::Operation::VoxelBooleanType::ExclusiveOr)
                        ? 1U
                        : 0U;

                changedCount +=
                    applyTreeBoolean(
                        left,
                        right,
                        MyVoxel::Operation::VoxelBooleanType::ExclusiveOr)
                        ? 1U
                        : 0U;
            }

            return changedCount +
                   static_cast<std::uint64_t>(left.allocatedGroupCount());
        });

    if (!sameTree(left, original))
    {
        std::cerr << "Deep XOR round trip changed the original tree." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return result;
}

// 测量独占VoxelShape通过Session提交多个配对根的公共原地路径。
BenchmarkResult benchmarkShapeUniqueXor(
    const BenchmarkConfiguration& configuration,
    std::size_t rootCount,
    const std::string& name,
    const BenchmarkScale& scale)
{
    const std::size_t cycleCount = scale.cycles(configuration);
    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(5));

    MyVoxel::VoxelShape left =
        makePatternShape(grid, rootCount, 3, 1001);

    const MyVoxel::VoxelShape original =
        makePatternShape(grid, rootCount, 3, 1001);

    const MyVoxel::VoxelShape right =
        makePatternShape(grid, rootCount, 3, 2003);

    const BenchmarkResult result = benchmark(
        name,
        cycleCount * 2,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t changedCount = 0;

            for (std::size_t cycleIndex = 0;
                 cycleIndex < cycleCount;
                 ++cycleIndex)
            {
                changedCount +=
                    MyVoxel::Operation::BooleanOperation::exclusiveOrInPlace(
                        left,
                        right)
                        ? 1U
                        : 0U;

                changedCount +=
                    MyVoxel::Operation::BooleanOperation::exclusiveOrInPlace(
                        left,
                        right)
                        ? 1U
                        : 0U;
            }

            return changedCount +
                   static_cast<std::uint64_t>(left.rootCount());
        });

    if (!sameShape(left, original))
    {
        std::cerr << name
                  << " round trip changed the original shape."
                  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return result;
}

// 测量目标数据被快照共享时的Shape级分离和根级写时复制路径。
BenchmarkResult benchmarkShapeSharedTarget(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t operationCount =
        BenchmarkScale(20, 200).cycles(configuration);

    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(5));
    const MyVoxel::VoxelShape leftTemplate =
        makePatternShape(grid, 64, 3, 4001);

    const MyVoxel::VoxelShape right =
        makePatternShape(grid, 64, 3, 5003);

    {
        MyVoxel::VoxelShape verificationTarget = leftTemplate;
        const MyVoxel::VoxelShape snapshot = verificationTarget;

        if (!MyVoxel::Operation::BooleanOperation::exclusiveOrInPlace(
                verificationTarget,
                right) ||
            !sameShape(snapshot, leftTemplate))
        {
            std::cerr << "Shared-target boolean operation violated snapshot semantics."
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    return benchmark(
        "shape_shared_target_64_roots",
        operationCount,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t checksum = 0;

            for (std::size_t operationIndex = 0;
                 operationIndex < operationCount;
                 ++operationIndex)
            {
                MyVoxel::VoxelShape left = leftTemplate;
                const MyVoxel::VoxelShape snapshot = left;

                const bool changed =
                    MyVoxel::Operation::BooleanOperation::exclusiveOrInPlace(
                        left,
                        right);

                checksum += changed ? static_cast<std::uint64_t>(left.rootCount()) : 0U;
                checksum += static_cast<std::uint64_t>(snapshot.rootCount());
            }

            return checksum;
        });
}

// 测量返回值接口创建结果并保持两个输入不变的路径。
BenchmarkResult benchmarkShapeReturned(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t operationCount =
        BenchmarkScale(20, 200).cycles(configuration);

    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(5));
    const MyVoxel::VoxelShape left =
        makePatternShape(grid, 64, 3, 6007);

    const MyVoxel::VoxelShape right =
        makePatternShape(grid, 64, 3, 7001);

    {
        const MyVoxel::VoxelShape result =
            MyVoxel::Operation::BooleanOperation::exclusiveOr(left, right);

        if (result.isEmpty() ||
            !sameShape(left, makePatternShape(grid, 64, 3, 6007)) ||
            !sameShape(right, makePatternShape(grid, 64, 3, 7001)))
        {
            std::cerr << "Returned boolean operation changed an input or produced an empty result."
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    return benchmark(
        "shape_returned_64_roots",
        operationCount,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t checksum = 0;

            for (std::size_t operationIndex = 0;
                 operationIndex < operationCount;
                 ++operationIndex)
            {
                const MyVoxel::VoxelShape result =
                    MyVoxel::Operation::BooleanOperation::exclusiveOr(
                        left,
                        right);

                checksum += static_cast<std::uint64_t>(result.rootCount());
            }

            return checksum;
        });
}

// 测量请求VoxelChangeSet时的完整根变化记录成本。
BenchmarkResult benchmarkShapeTrackedChanges(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t cycleCount =
        BenchmarkScale(20, 200).cycles(configuration);

    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(5));

    MyVoxel::VoxelShape left =
        makePatternShape(grid, 64, 3, 8009);

    const MyVoxel::VoxelShape original =
        makePatternShape(grid, 64, 3, 8009);

    const MyVoxel::VoxelShape right =
        makePatternShape(grid, 64, 3, 9001);

    MyVoxel::VoxelChangeSet changes =
        makeEmptyChangeSet(
            grid,
            static_cast<MyVoxel::VoxelLevel>(3));

    {
        MyVoxel::VoxelShape verificationTarget =
            makePatternShape(grid, 64, 3, 8009);

        if (!MyVoxel::Operation::BooleanOperation::exclusiveOrInPlace(
                verificationTarget,
                right,
                &changes) ||
            !changes.hasChanges() ||
            changes.modifiedRootCount() != 64)
        {
            std::cerr << "Tracked boolean operation produced an invalid change set."
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    const BenchmarkResult result = benchmark(
        "shape_tracked_changes_64_roots",
        cycleCount * 2,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t checksum = 0;

            for (std::size_t cycleIndex = 0;
                 cycleIndex < cycleCount;
                 ++cycleIndex)
            {
                const bool firstChanged =
                    MyVoxel::Operation::BooleanOperation::exclusiveOrInPlace(
                        left,
                        right,
                        &changes);

                const bool secondChanged =
                    MyVoxel::Operation::BooleanOperation::exclusiveOrInPlace(
                        left,
                        right,
                        &changes);

                checksum += firstChanged ? 1U : 0U;
                checksum += secondChanged ? 1U : 0U;
            }

            return checksum;
        });

    if (!sameShape(left, original))
    {
        std::cerr << "Tracked XOR round trip changed the original shape."
                  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return result;
}

// 测量同数据并集直接返回不变的Shape级O(1)路径。
BenchmarkResult benchmarkShapeAliasedNoOp(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t operationCount =
        BenchmarkScale(100000, 1000000).cycles(configuration);

    MyVoxel::VoxelShape shape =
        makePatternShape(
            MyVoxel::VoxelGrid(
                1.0,
                static_cast<MyVoxel::VoxelLevel>(5)),
            64,
            3,
            10007);

    return benchmark(
        "shape_aliased_union_noop",
        operationCount,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t unchangedCount = 0;

            for (std::size_t operationIndex = 0;
                 operationIndex < operationCount;
                 ++operationIndex)
            {
                unchangedCount +=
                    MyVoxel::Operation::BooleanOperation::uniteInPlace(
                        shape,
                        shape)
                        ? 0U
                        : 1U;
            }

            return unchangedCount +
                   static_cast<std::uint64_t>(shape.rootCount());
        });
}

// 测量内容相同但不共享数据的两个Shape执行并集时的无变化遍历路径。
BenchmarkResult benchmarkShapeDistinctEqualNoOp(
    const BenchmarkConfiguration& configuration)
{
    const std::size_t operationCount =
        BenchmarkScale(10, 100).cycles(configuration);

    const MyVoxel::VoxelGrid grid(1.0, static_cast<MyVoxel::VoxelLevel>(5));
    MyVoxel::VoxelShape left =
        makePatternShape(grid, 64, 3, 12007);

    const MyVoxel::VoxelShape right =
        makePatternShape(grid, 64, 3, 12007);

    if (left.sharesDataWith(right))
    {
        std::cerr << "Distinct no-op benchmark requires independent Shape data."
                  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return benchmark(
        "shape_distinct_equal_union_noop_64_roots",
        operationCount,
        configuration,
        [&]() -> std::uint64_t
        {
            std::uint64_t unchangedCount = 0;

            for (std::size_t operationIndex = 0;
                 operationIndex < operationCount;
                 ++operationIndex)
            {
                unchangedCount +=
                    MyVoxel::Operation::BooleanOperation::uniteInPlace(
                        left,
                        right)
                        ? 0U
                        : 1U;
            }

            return unchangedCount +
                   static_cast<std::uint64_t>(left.rootCount());
        });
}

// 执行全部正确性检查和性能基准。
std::vector<BenchmarkResult> runBenchmarks(
    const BenchmarkConfiguration& configuration)
{
    std::vector<BenchmarkResult> results;
    results.reserve(11);

    results.push_back(benchmarkTreeRootToggle(configuration));
    results.push_back(benchmarkTreeMaskLeafXor(configuration));
    results.push_back(benchmarkTreeDeepXor(configuration));

    results.push_back(
        benchmarkShapeUniqueXor(
            configuration,
            1,
            "shape_unique_1_root",
            BenchmarkScale(200, 2000)));

    results.push_back(
        benchmarkShapeUniqueXor(
            configuration,
            64,
            "shape_unique_64_roots",
            BenchmarkScale(20, 200)));

    results.push_back(
        benchmarkShapeUniqueXor(
            configuration,
            512,
            "shape_unique_512_roots",
            BenchmarkScale(2, 20)));

    results.push_back(benchmarkShapeSharedTarget(configuration));
    results.push_back(benchmarkShapeReturned(configuration));
    results.push_back(benchmarkShapeTrackedChanges(configuration));
    results.push_back(benchmarkShapeAliasedNoOp(configuration));
    results.push_back(benchmarkShapeDistinctEqualNoOp(configuration));

    return results;
}

// 输出当前编译和运行环境摘要。
void printEnvironment(const BenchmarkConfiguration& configuration)
{
    std::cout << "MyVoxel boolean performance regression test" << std::endl;

#ifdef _MSC_VER
    std::cout << "Compiler: MSVC " << _MSC_VER << std::endl;
#else
    std::cout << "Compiler: non-MSVC" << std::endl;
#endif

#ifdef NDEBUG
    std::cout << "Build: Release-compatible (NDEBUG defined)" << std::endl;
#else
    std::cout << "Build: Debug (performance result is not accepted)" << std::endl;
#endif

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS
    std::cout << "Warning: MYVOXEL_ENABLE_OPERATION_STATISTICS is enabled; "
              << "this is not the production fast path." << std::endl;
#endif

#ifdef MYVOXEL_BENCHMARK_LEGACY_CORE_API
    std::cout << "Core API mode: legacy editForest/MaskLeaf interface" << std::endl;
#else
    std::cout << "Core API mode: VoxelShapeSession/current MaskLeaf interface" << std::endl;
#endif

    std::cout << "Hardware threads: "
              << std::thread::hardware_concurrency()
              << std::endl;

    std::cout << "Mode: "
              << (configuration.quick ? "quick" : "full")
              << ", samples: "
              << configuration.timingSampleCount
              << ", warmups: "
              << configuration.warmupSampleCount
              << std::endl;
}

// 以固定表格输出全部性能结果。
void printResults(const std::vector<BenchmarkResult>& results)
{
    std::cout << std::left
              << std::setw(38) << "Benchmark"
              << std::right
              << std::setw(18) << "ns/operation"
              << std::setw(18) << "timed operations"
              << std::endl;

    std::cout << std::string(74, '-') << std::endl;

    for (std::size_t resultIndex = 0;
         resultIndex < results.size();
         ++resultIndex)
    {
        const BenchmarkResult& result = results[resultIndex];

        std::cout << std::left
                  << std::setw(38) << result.name
                  << std::right
                  << std::setw(18) << std::fixed << std::setprecision(3)
                  << result.nanosecondsPerOperation
                  << std::setw(18) << result.operationCount
                  << std::endl;
    }
}

// 将当前结果写入可供后续版本比较的CSV基线文件。
bool writeBaseline(const std::string& path,
                   const std::vector<BenchmarkResult>& results)
{
    std::ofstream stream(path.c_str(), std::ios::out | std::ios::trunc);

    if (!stream)
    {
        std::cerr << "Cannot write baseline file: " << path << std::endl;
        return false;
    }

    stream << "benchmark,ns_per_operation,operation_count\n";
    stream << std::setprecision(17);

    for (std::size_t resultIndex = 0;
         resultIndex < results.size();
         ++resultIndex)
    {
        const BenchmarkResult& result = results[resultIndex];

        stream << result.name << ','
               << result.nanosecondsPerOperation << ','
               << result.operationCount << '\n';
    }

    return static_cast<bool>(stream);
}

// 读取由当前测试程序写出的CSV基线。
bool readBaseline(const std::string& path,
                  std::map<std::string, BaselineEntry>& baseline)
{
    baseline.clear();

    std::ifstream stream(path.c_str());

    if (!stream)
    {
        std::cerr << "Cannot read baseline file: " << path << std::endl;
        return false;
    }

    std::string line;
    std::getline(stream, line);

    while (std::getline(stream, line))
    {
        if (line.empty())
        {
            continue;
        }

        const std::size_t firstComma = line.find(',');
        const std::size_t secondComma =
            firstComma == std::string::npos
                ? std::string::npos
                : line.find(',', firstComma + 1);

        if (firstComma == std::string::npos ||
            secondComma == std::string::npos)
        {
            std::cerr << "Invalid baseline row: " << line << std::endl;
            return false;
        }

        const std::string name = line.substr(0, firstComma);
        const std::string valueText =
            line.substr(firstComma + 1, secondComma - firstComma - 1);
        const std::string operationCountText =
            line.substr(secondComma + 1);

        std::istringstream valueStream(valueText);
        double value = 0.0;
        valueStream >> value;

        std::istringstream operationCountStream(operationCountText);
        unsigned long long operationCount = 0;
        operationCountStream >> operationCount;

        if (!valueStream || value <= 0.0 ||
            !operationCountStream || operationCount == 0)
        {
            std::cerr << "Invalid baseline value: " << line << std::endl;
            return false;
        }

        baseline[name] =
            BaselineEntry(
                value,
                static_cast<std::uint64_t>(operationCount));
    }

    return !baseline.empty();
}

// 将当前结果与旧稳定版本基线比较并返回是否全部通过。
bool compareWithBaseline(
    const std::vector<BenchmarkResult>& results,
    const std::map<std::string, BaselineEntry>& baseline,
    const BenchmarkConfiguration& configuration)
{
    bool passed = true;

    std::cout << std::endl;
    std::cout << std::left
              << std::setw(38) << "Benchmark"
              << std::right
              << std::setw(14) << "baseline ns"
              << std::setw(14) << "current ns"
              << std::setw(12) << "slowdown"
              << std::setw(10) << "result"
              << std::endl;

    std::cout << std::string(88, '-') << std::endl;

    for (std::size_t resultIndex = 0;
         resultIndex < results.size();
         ++resultIndex)
    {
        const BenchmarkResult& result = results[resultIndex];
        const std::map<std::string, BaselineEntry>::const_iterator iterator =
            baseline.find(result.name);

        if (iterator == baseline.end())
        {
            std::cout << std::left
                      << std::setw(38) << result.name
                      << std::right
                      << std::setw(50) << "MISSING BASELINE"
                      << std::endl;

            passed = false;
            continue;
        }

        if (iterator->second.operationCount != result.operationCount)
        {
            std::cout << std::left
                      << std::setw(38) << result.name
                      << std::right
                      << std::setw(50) << "CONFIGURATION MISMATCH"
                      << std::endl;

            passed = false;
            continue;
        }

        const double baselineNanoseconds =
            iterator->second.nanosecondsPerOperation;
        const double slowdownPercent =
            (result.nanosecondsPerOperation / baselineNanoseconds - 1.0) * 100.0;

        const double permittedNanoseconds =
            baselineNanoseconds *
                (1.0 + configuration.maximumSlowdownPercent / 100.0) +
            configuration.absoluteToleranceNanoseconds;

        const bool currentPassed =
            result.nanosecondsPerOperation <= permittedNanoseconds;

        passed = currentPassed && passed;

        std::cout << std::left
                  << std::setw(38) << result.name
                  << std::right
                  << std::setw(14) << std::fixed << std::setprecision(3)
                  << baselineNanoseconds
                  << std::setw(14)
                  << result.nanosecondsPerOperation
                  << std::setw(11) << std::setprecision(2)
                  << slowdownPercent << '%'
                  << std::setw(10)
                  << (currentPassed ? "PASS" : "FAIL")
                  << std::endl;
    }

    return passed;
}

// 解析无符号整数参数。
bool parseSize(const char* text, std::size_t& value)
{
    if (!text || *text == '\0')
    {
        return false;
    }

    char* end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);

    if (!end || *end != '\0' || parsed == 0)
    {
        return false;
    }

    value = static_cast<std::size_t>(parsed);
    return true;
}

// 解析有限非负浮点参数。
bool parseDouble(const char* text, double& value)
{
    if (!text || *text == '\0')
    {
        return false;
    }

    char* end = nullptr;
    const double parsed = std::strtod(text, &end);

    const double infinity =
        (std::numeric_limits<double>::infinity)();

    if (!end || *end != '\0' ||
        parsed != parsed ||
        parsed == infinity ||
        parsed == -infinity ||
        parsed < 0.0)
    {
        return false;
    }

    value = parsed;
    return true;
}

// 输出命令行参数说明。
void printUsage(const char* executable)
{
    std::cout
        << "Usage: " << executable << " [options]\n"
        << "  --quick                         Run reduced iteration counts.\n"
        << "  --samples N                     Set formal timing sample count.\n"
        << "  --warmups N                     Set warmup sample count.\n"
        << "  --write-baseline FILE           Write current medians to CSV.\n"
        << "  --compare FILE                  Compare current medians with CSV.\n"
        << "  --max-slowdown PERCENT          Allowed relative slowdown, default 10.\n"
        << "  --absolute-tolerance-ns VALUE   Extra tolerance for tiny benchmarks.\n"
        << "  --help                          Show this message.\n";
}

// 解析测试配置，返回false表示参数无效或已输出帮助。
bool parseArguments(int argc,
                    char* argv[],
                    BenchmarkConfiguration& configuration,
                    bool& shouldRun)
{
    shouldRun = true;

    for (int argumentIndex = 1;
         argumentIndex < argc;
         ++argumentIndex)
    {
        const std::string argument(argv[argumentIndex]);

        if (argument == "--quick")
        {
            configuration.quick = true;
            continue;
        }

        if (argument == "--help")
        {
            printUsage(argv[0]);
            shouldRun = false;
            return true;
        }

        if (argumentIndex + 1 >= argc)
        {
            std::cerr << "Missing value for argument: " << argument << std::endl;
            return false;
        }

        const char* value = argv[++argumentIndex];

        if (argument == "--samples")
        {
            if (!parseSize(value, configuration.timingSampleCount))
            {
                return false;
            }

            continue;
        }

        if (argument == "--warmups")
        {
            if (!parseSize(value, configuration.warmupSampleCount))
            {
                return false;
            }

            continue;
        }

        if (argument == "--write-baseline")
        {
            configuration.writeBaselinePath = value;
            continue;
        }

        if (argument == "--compare")
        {
            configuration.compareBaselinePath = value;
            continue;
        }

        if (argument == "--max-slowdown")
        {
            if (!parseDouble(value, configuration.maximumSlowdownPercent))
            {
                return false;
            }

            continue;
        }

        if (argument == "--absolute-tolerance-ns")
        {
            if (!parseDouble(value, configuration.absoluteToleranceNanoseconds))
            {
                return false;
            }

            continue;
        }

        std::cerr << "Unknown argument: " << argument << std::endl;
        return false;
    }

    if (!configuration.writeBaselinePath.empty() &&
        !configuration.compareBaselinePath.empty())
    {
        std::cerr << "--write-baseline and --compare cannot be used together."
                  << std::endl;
        return false;
    }

    return true;
}

}

int main(int argc, char* argv[])
{
    BenchmarkConfiguration configuration;
    bool shouldRun = true;

    if (!parseArguments(argc, argv, configuration, shouldRun))
    {
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }

    if (!shouldRun)
    {
        return EXIT_SUCCESS;
    }

#ifndef NDEBUG
    std::cerr << "This benchmark must be executed from a Release build."
              << std::endl;
    return EXIT_FAILURE;
#endif

    printEnvironment(configuration);

    const std::vector<BenchmarkResult> results =
        runBenchmarks(configuration);

    std::cout << std::endl;
    printResults(results);

    if (!configuration.writeBaselinePath.empty())
    {
        if (!writeBaseline(configuration.writeBaselinePath, results))
        {
            return EXIT_FAILURE;
        }

        std::cout << std::endl
                  << "Baseline written: "
                  << configuration.writeBaselinePath
                  << std::endl;
    }

    if (!configuration.compareBaselinePath.empty())
    {
        std::map<std::string, BaselineEntry> baseline;

        if (!readBaseline(configuration.compareBaselinePath, baseline))
        {
            return EXIT_FAILURE;
        }

        const bool passed =
            compareWithBaseline(results, baseline, configuration);

        std::cout << std::endl
                  << (passed
                          ? "Boolean performance regression check PASSED."
                          : "Boolean performance regression check FAILED.")
                  << std::endl;

        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    std::cout << std::endl
              << "Performance run completed. Result sink: "
              << ResultSink
              << std::endl;

    return EXIT_SUCCESS;
}