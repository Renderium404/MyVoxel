#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <io.h>
#include <string>
#include <thread>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Builder/ShapeBuilder.h"
#include "MyVoxel/Builder/ShapeVoxelizer.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "MyVoxel/Operation/ShapeTransform.h"
#include "MyVoxel/Shape/Shape.h"

namespace
{

using Clock = std::chrono::steady_clock;

const double Pi = 3.14159265358979323846; // 圆周率，用于构造砂轮旋转角度。

// 设置Windows控制台使用宽字符输出，避免中文乱码。
void initializeConsole()
{
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_U16TEXT);
#endif
}

// 返回当前构建模式名称。
const wchar_t* buildMode()
{
#ifdef NDEBUG
    return L"Release";
#else
    return L"Debug";
#endif
}

// 返回两个时间点之间的毫秒数。
double elapsedMilliseconds(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// 检查两个浮点数是否近似相等。
bool nearlyEqual(double left, double right, double epsilon = 1.0e-10)
{
    return std::fabs(left - right) <= epsilon;
}

// 检查两个矩阵是否近似相等。
bool matrixNearlyEqual(const MyMath::Matrix4& left, const MyMath::Matrix4& right, double epsilon = 1.0e-10)
{
    for (int row = 0; row < 4; ++row)
    {
        for (int column = 0; column < 4; ++column)
        {
            if (!nearlyEqual(left(row, column), right(row, column), epsilon))
            {
                return false;
            }
        }
    }

    return true;
}

// 返回形体当前根节点数量，使用只读森林接口避免触发写时复制。
std::size_t rootCount(const MyVoxel::VoxelShape& shape)
{
    return shape.forest().rootCount();
}

// 返回材料节点展开到指定层级后对应的体素数量。
std::uint64_t materialCellCountAtLevel(const MyVoxel::VoxelShape& shape, MyVoxel::VoxelLevel level)
{
    std::uint64_t count = 0;

    shape.forest().forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            std::uint64_t expandedCount = 1;

            for (MyVoxel::VoxelLevel currentLevel = address.level; currentLevel < level; ++currentLevel)
            {
                expandedCount *= static_cast<std::uint64_t>(MyVoxel::VoxelCornerCount);
            }

            count += expandedCount;
        });

    return count;
}

// 创建连续Shape砂轮轨迹。
std::vector<MyVoxel::Shape> createShapeToolPath(const MyVoxel::Shape& tool, int cutCount)
{
    std::vector<MyVoxel::Shape> tools;
    tools.reserve(static_cast<std::size_t>(cutCount));

    for (int i = 0; i < cutCount; ++i)
    {
        const double ratio = cutCount > 1 ? static_cast<double>(i) / static_cast<double>(cutCount - 1) : 0.0;
        const double positionX = -2.0 + ratio * 4.0; // 砂轮中心沿X方向从-2毫米移动到2毫米。
        tools.push_back(MyVoxel::translate(tool, MyMath::Vector3(positionX, 0.0, 0.0)));
    }

    return tools;
}

// 输出布尔运算统计。
void printStatistics(const MyVoxel::BooleanOperationStatistics& statistics, int operationCount)
{
    const double divisor = operationCount > 0 ? static_cast<double>(operationCount) : 1.0;
    const std::uint64_t integerDivisor = operationCount > 0 ? static_cast<std::uint64_t>(operationCount) : 1;

    std::wcout << L"运算次数                : " << operationCount << std::endl;
    std::wcout << L"内部总耗时平均          : " << statistics.totalMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"查询上下文平均          : " << statistics.toolIndexMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"入口收集平均            : " << statistics.materialCollectionMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"写时复制平均            : " << statistics.detachMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"计划生成平均            : " << statistics.planningMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"根树准备平均            : " << statistics.rootPreparationMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"计划应用平均            : " << statistics.planApplicationMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"递归切削平均            : " << statistics.cuttingMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"计划生成线程平均        : " << statistics.planningWorkerCount / integerDivisor << std::endl;
    std::wcout << L"计划应用线程平均        : " << statistics.planApplicationWorkerCount / integerDivisor << std::endl;
    std::wcout << L"切削计划节点平均        : " << statistics.cutPlanNodeCount / integerDivisor << std::endl;
    std::wcout << L"切削计划组平均          : " << statistics.cutPlanGroupCount / integerDivisor << std::endl;
    std::wcout << L"准备根树平均            : " << statistics.preparedRootTreeCount / integerDivisor << std::endl;
    std::wcout << L"直接删除根树平均        : " << statistics.removedRootTreeCount / integerDivisor << std::endl;
    std::wcout << L"宽相候选根节点平均      : " << statistics.broadPhaseRootCandidateCount / integerDivisor << std::endl;
    std::wcout << L"宽相实际根节点平均      : " << statistics.broadPhaseExistingRootCount / integerDivisor << std::endl;
    std::wcout << L"递归入口节点平均        : " << statistics.inputMaterialCellCount / integerDivisor << std::endl;
    std::wcout << L"访问单元平均            : " << statistics.visitedCellCount / integerDivisor << std::endl;
    std::wcout << L"空单元跳过平均          : " << statistics.emptySkippedCellCount / integerDivisor << std::endl;
    std::wcout << L"区域分类平均            : " << statistics.classifiedCellCount / integerDivisor << std::endl;
    std::wcout << L"完全外部平均            : " << statistics.outsideCellCount / integerDivisor << std::endl;
    std::wcout << L"完全内部平均            : " << statistics.insideCellCount / integerDivisor << std::endl;
    std::wcout << L"边界相交平均            : " << statistics.intersectingCellCount / integerDivisor << std::endl;
    std::wcout << L"中心采样平均            : " << statistics.centerSampleCount / integerDivisor << std::endl;
    std::wcout << L"中心删除平均            : " << statistics.centerRemovedCellCount / integerDivisor << std::endl;
    std::wcout << L"整分支删除平均          : " << statistics.removedBranchCount / integerDivisor << std::endl;
    std::wcout << L"合并尝试平均            : " << statistics.mergeAttemptCount / integerDivisor << std::endl;
    std::wcout << L"合并成功平均            : " << statistics.mergeSuccessCount / integerDivisor << std::endl;
    std::wcout << L"体素区域查询节点平均    : " << statistics.queryVisitedNodeCount / integerDivisor << std::endl;
    std::wcout << L"体素点查询节点平均      : " << statistics.accessorNodeVisitCount / integerDivisor << std::endl;
}

// 保存指定线程数下的单次和连续轨迹测试结果。
struct ParallelBenchmarkResult
{
    unsigned int requestedWorkerCount = 0; // 调用方请求的工作线程数量。
    std::uint64_t actualSinglePlanningWorkerCount = 0; // 单次计划生成实际使用的工作线程数量。
    std::uint64_t actualSingleApplicationWorkerCount = 0; // 单次计划应用实际使用的工作线程数量。
    double singleMilliseconds = 0.0; // 单次切削外部总耗时。
    std::uint64_t singleCellCount = 0; // 单次切削结果体素数量。
    MyVoxel::BooleanOperationStatistics singleStatistics; // 单次切削内部统计。
    double sequentialMilliseconds = 0.0; // 完整连续轨迹外部总耗时。
    std::uint64_t sequentialCellCount = 0; // 连续轨迹最终体素数量。
    MyVoxel::BooleanOperationStatistics sequentialStatistics; // 连续轨迹累计内部统计。
};

// 使用指定线程数执行单次切削和连续轨迹切削。
ParallelBenchmarkResult runParallelBenchmark(const MyVoxel::VoxelShape& workpiece, const MyVoxel::Shape& singleCutTool, const std::vector<MyVoxel::Shape>& toolPath, MyVoxel::VoxelLevel maximumLevel, unsigned int workerCount, volatile std::uint64_t& benchmarkChecksum)
{
    ParallelBenchmarkResult benchmark;
    benchmark.requestedWorkerCount = workerCount;

    MyVoxel::BooleanOperationOptions options;
    options.workerCount = workerCount;
    options.minimumParallelRootCount = 1; // 固定线程对比时允许所有非空任务使用指定线程数。

    /// 单次切削

    const Clock::time_point singleStart = Clock::now();
    const MyVoxel::VoxelShape singleResult = MyVoxel::cut(workpiece, singleCutTool, options, benchmark.singleStatistics);
    const Clock::time_point singleEnd = Clock::now();

    benchmark.singleMilliseconds = elapsedMilliseconds(singleStart, singleEnd);
    benchmark.singleCellCount = materialCellCountAtLevel(singleResult, maximumLevel);
    benchmark.actualSinglePlanningWorkerCount = benchmark.singleStatistics.planningWorkerCount;
    benchmark.actualSingleApplicationWorkerCount = benchmark.singleStatistics.planApplicationWorkerCount;
    benchmarkChecksum += static_cast<std::uint64_t>(rootCount(singleResult));

    /// 连续轨迹切削

    MyVoxel::VoxelShape sequentialResult = workpiece;

    const Clock::time_point sequentialStart = Clock::now();

    for (std::size_t i = 0; i < toolPath.size(); ++i)
    {
        MyVoxel::BooleanOperationStatistics currentStatistics;

        sequentialResult = MyVoxel::cut(sequentialResult, toolPath[i], options, currentStatistics);
        benchmark.sequentialStatistics.accumulate(currentStatistics);
        benchmarkChecksum += static_cast<std::uint64_t>(rootCount(sequentialResult));
    }

    const Clock::time_point sequentialEnd = Clock::now();

    benchmark.sequentialMilliseconds = elapsedMilliseconds(sequentialStart, sequentialEnd);
    benchmark.sequentialCellCount = materialCellCountAtLevel(sequentialResult, maximumLevel);
    return benchmark;
}

// 输出不同线程数的性能对比。
void printParallelBenchmarkResults(const std::vector<ParallelBenchmarkResult>& benchmarks, int sequentialCutCount)
{
    const double sequentialDivisor = sequentialCutCount > 0 ? static_cast<double>(sequentialCutCount) : 1.0;

    std::wcout << L"请求  生成  应用  单次外部  单次生成  单次准备  单次应用  轨迹外部  轨迹生成  轨迹准备  轨迹应用" << std::endl;
    std::wcout << L"线程  线程  线程    ms/次     ms/次     ms/次     ms/次     ms/次     ms/次     ms/次     ms/次" << std::endl;

    for (std::size_t i = 0; i < benchmarks.size(); ++i)
    {
        const ParallelBenchmarkResult& benchmark = benchmarks[i];

        std::wcout << std::setw(4) << benchmark.requestedWorkerCount
                   << std::setw(6) << benchmark.actualSinglePlanningWorkerCount
                   << std::setw(6) << benchmark.actualSingleApplicationWorkerCount
                   << std::setw(11) << benchmark.singleMilliseconds
                   << std::setw(11) << benchmark.singleStatistics.planningMilliseconds
                   << std::setw(11) << benchmark.singleStatistics.rootPreparationMilliseconds
                   << std::setw(11) << benchmark.singleStatistics.planApplicationMilliseconds
                   << std::setw(11) << benchmark.sequentialMilliseconds / sequentialDivisor
                   << std::setw(11) << benchmark.sequentialStatistics.planningMilliseconds / sequentialDivisor
                   << std::setw(11) << benchmark.sequentialStatistics.rootPreparationMilliseconds / sequentialDivisor
                   << std::setw(11) << benchmark.sequentialStatistics.planApplicationMilliseconds / sequentialDivisor
                   << std::endl;
    }
}

// 保存测试结果并统一输出。
class TestContext
{
public:
    // 记录一个测试条件。
    void check(bool condition, const wchar_t* name)
    {
        if (condition)
        {
            ++m_passedCount;
            std::wcout << L"[通过] " << name << std::endl;
        }
        else
        {
            ++m_failedCount;
            std::wcout << L"[失败] " << name << std::endl;
        }
    }

    // 返回通过数量。
    int passedCount() const
    {
        return m_passedCount;
    }

    // 返回失败数量。
    int failedCount() const
    {
        return m_failedCount;
    }

    // 返回测试程序退出码。
    int exitCode() const
    {
        return m_failedCount == 0 ? 0 : 1;
    }

private:
    int m_passedCount = 0; // 已通过测试数量。
    int m_failedCount = 0; // 已失败测试数量。
};

}

int main()
{
    initializeConsole();

    const double baseVoxelEdgeLength = 1.0; // 第0层体素边长，单位为毫米。
    const MyVoxel::VoxelLevel maximumLevel = 6; // 体素化和切削使用的最高细分层级。
    const double toolRotation = Pi / 6.0; // 砂轮绕Y轴旋转30度。
    const int independentRepeatCount = 10; // 相同输入独立重复执行次数。
    const int sequentialCutCount = 20; // 连续轨迹切削次数。
    const std::array<unsigned int, 4> parallelWorkerCounts = {{1, 2, 4, 8}}; // 并行性能对比使用的工作线程数量。

    TestContext tests;

    /// 标准Shape创建

    const MyVoxel::Shape workpieceShape = MyVoxel::ShapeBuilder::makeBox(-3.0, -2.0, -1.5, 3.0, 2.0, 1.5);
    const MyVoxel::Shape sourceToolShape = MyVoxel::ShapeBuilder::makeCylinder(0.0, 0.0, -3.0, 3.0, 1.2);
    const MyVoxel::Shape rotatedToolShape = MyVoxel::rotate(sourceToolShape, MyMath::Vector3::unitY(), toolRotation);
    const MyVoxel::Shape singleCutToolShape = MyVoxel::translate(rotatedToolShape, MyMath::Vector3(0.5, 0.0, 0.0));
    const std::vector<MyVoxel::Shape> shapeToolPath = createShapeToolPath(rotatedToolShape, sequentialCutCount);

    tests.check(workpieceShape.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), L"标准盒体包含局部中心点");
    tests.check(!workpieceShape.containsLocalPoint(MyMath::Vector3(4.0, 0.0, 0.0)), L"标准盒体排除外部点");
    tests.check(sourceToolShape.containsLocalPoint(MyMath::Vector3(0.0, 0.0, 0.0)), L"标准圆柱包含局部中心点");
    tests.check(!sourceToolShape.containsLocalPoint(MyMath::Vector3(2.0, 0.0, 0.0)), L"标准圆柱排除径向外部点");
    tests.check(rotatedToolShape.sharesGeometryWith(sourceToolShape), L"Shape变换后共享原始连续几何");
    tests.check(singleCutToolShape.sharesGeometryWith(sourceToolShape), L"Shape平移后共享原始连续几何");
    tests.check(!matrixNearlyEqual(singleCutToolShape.transform(), sourceToolShape.transform()), L"Shape变换矩阵发生变化");

    /// 工件体素化

    const MyVoxel::VoxelShape workpiece = MyVoxel::voxelize(workpieceShape, baseVoxelEdgeLength, maximumLevel);
    const std::uint64_t workpieceCellCount = materialCellCountAtLevel(workpiece, maximumLevel);
    const std::size_t workpieceRootCount = rootCount(workpiece);

    tests.check(workpieceRootCount > 0, L"工件体素化生成根节点");
    tests.check(workpieceCellCount > 0, L"工件体素化生成材料体素");
    tests.check(workpiece.maximumLevel() == maximumLevel, L"体素化保留最高层级");
    tests.check(nearlyEqual(workpiece.baseVoxelEdgeLength(), baseVoxelEdgeLength), L"体素化保留第0层体素边长");
    tests.check(matrixNearlyEqual(workpiece.transform(), workpieceShape.transform()), L"体素化保留标准Shape姿态");

    /// 默认连续Shape单次切削

    volatile std::uint64_t benchmarkChecksum = 0;

    {
        const MyVoxel::VoxelShape warmupResult = MyVoxel::cut(workpiece, singleCutToolShape);
        benchmarkChecksum += static_cast<std::uint64_t>(rootCount(warmupResult));
    }

    MyVoxel::BooleanOperationStatistics continuousSingleStatistics;

    const Clock::time_point continuousSingleStart = Clock::now();
    const MyVoxel::VoxelShape continuousSingleResult = MyVoxel::cut(workpiece, singleCutToolShape, continuousSingleStatistics);
    const Clock::time_point continuousSingleEnd = Clock::now();

    benchmarkChecksum += static_cast<std::uint64_t>(rootCount(continuousSingleResult));

    const double continuousSingleMilliseconds = elapsedMilliseconds(continuousSingleStart, continuousSingleEnd);
    const std::uint64_t continuousSingleCellCount = materialCellCountAtLevel(continuousSingleResult, maximumLevel);
    const MyMath::Vector3 removedTestPoint(0.5, 0.0, 0.0);

    tests.check(continuousSingleCellCount < workpieceCellCount, L"连续Shape切削删除工件材料");
    tests.check(continuousSingleCellCount > 0, L"连续Shape切削后仍保留工件材料");
    tests.check(workpiece.containsLocalPoint(removedTestPoint), L"原始工件保留测试点");
    tests.check(!continuousSingleResult.containsLocalPoint(removedTestPoint), L"连续Shape切削删除测试点");
    tests.check(!continuousSingleResult.sharesDataWith(workpiece), L"切削结果与原始工件解除体素数据共享");
    tests.check(materialCellCountAtLevel(workpiece, maximumLevel) == workpieceCellCount, L"连续Shape切削不修改原始工件");
    tests.check(matrixNearlyEqual(continuousSingleResult.transform(), workpiece.transform()), L"连续Shape切削保留工件姿态");
    tests.check(continuousSingleStatistics.classifiedCellCount == continuousSingleStatistics.outsideCellCount + continuousSingleStatistics.insideCellCount + continuousSingleStatistics.intersectingCellCount, L"连续Shape区域分类统计一致");
    tests.check(continuousSingleStatistics.pointQueryCount == continuousSingleStatistics.centerSampleCount, L"连续Shape中心采样与点查询数量一致");
    tests.check(continuousSingleStatistics.queryVisitedNodeCount == 0, L"连续Shape切削不访问砂轮体素查询树");
    tests.check(continuousSingleStatistics.accessorNodeVisitCount == 0, L"连续Shape切削不访问砂轮体素点查询器");
    tests.check(continuousSingleStatistics.planningWorkerCount > 0, L"连续Shape切削生成并行计划");
    tests.check(continuousSingleStatistics.cutPlanNodeCount > 0, L"连续Shape切削生成计划节点");
    tests.check(continuousSingleStatistics.preparedRootTreeCount > 0, L"连续Shape切削预分离待修改根树");
    tests.check(continuousSingleStatistics.planApplicationWorkerCount > 0, L"连续Shape切削并行应用根计划");

    /// 默认连续Shape重复稳定性

    MyVoxel::BooleanOperationStatistics independentStatistics;
    std::uint64_t independentResultCellCount = 0;
    bool independentStable = true;

    const Clock::time_point independentStart = Clock::now();

    for (int i = 0; i < independentRepeatCount; ++i)
    {
        MyVoxel::BooleanOperationStatistics currentStatistics;
        const MyVoxel::VoxelShape currentResult = MyVoxel::cut(workpiece, singleCutToolShape, currentStatistics);
        const std::uint64_t currentCellCount = materialCellCountAtLevel(currentResult, maximumLevel);

        if (i == 0)
        {
            independentResultCellCount = currentCellCount;
        }
        else if (currentCellCount != independentResultCellCount)
        {
            independentStable = false;
        }

        independentStatistics.accumulate(currentStatistics);
        benchmarkChecksum += static_cast<std::uint64_t>(rootCount(currentResult));
    }

    const Clock::time_point independentEnd = Clock::now();
    const double independentMilliseconds = elapsedMilliseconds(independentStart, independentEnd);

    tests.check(independentStable, L"连续Shape相同输入重复结果稳定");
    tests.check(independentResultCellCount == continuousSingleCellCount, L"普通重复切削与单次统计切削结果一致");

    /// 默认连续轨迹切削

    MyVoxel::BooleanOperationStatistics sequentialStatistics;
    MyVoxel::VoxelShape sequentialResult = workpiece;

    const Clock::time_point sequentialStart = Clock::now();

    for (int i = 0; i < sequentialCutCount; ++i)
    {
        MyVoxel::BooleanOperationStatistics currentStatistics;

        sequentialResult = MyVoxel::cut(sequentialResult, shapeToolPath[static_cast<std::size_t>(i)], currentStatistics);
        sequentialStatistics.accumulate(currentStatistics);
        benchmarkChecksum += static_cast<std::uint64_t>(rootCount(sequentialResult));
    }

    const Clock::time_point sequentialEnd = Clock::now();
    const double sequentialMilliseconds = elapsedMilliseconds(sequentialStart, sequentialEnd);
    const std::uint64_t sequentialResultCellCount = materialCellCountAtLevel(sequentialResult, maximumLevel);

    tests.check(sequentialResultCellCount < continuousSingleCellCount, L"连续轨迹切削删除更多材料");
    tests.check(sequentialResultCellCount > 0, L"连续轨迹切削后仍保留材料");
    tests.check(materialCellCountAtLevel(workpiece, maximumLevel) == workpieceCellCount, L"连续轨迹切削不修改原始工件");

    /// 固定线程数并行对比

    std::vector<ParallelBenchmarkResult> parallelBenchmarks;
    parallelBenchmarks.reserve(parallelWorkerCounts.size());

    bool parallelSingleResultStable = true;
    bool parallelSequentialResultStable = true;
    bool requestedWorkerCountApplied = true;
    bool parallelStatisticsConsistent = true;

    for (std::size_t i = 0; i < parallelWorkerCounts.size(); ++i)
    {
        const ParallelBenchmarkResult benchmark = runParallelBenchmark(workpiece, singleCutToolShape, shapeToolPath, maximumLevel, parallelWorkerCounts[i], benchmarkChecksum);

        if (benchmark.singleCellCount != continuousSingleCellCount)
        {
            parallelSingleResultStable = false;
        }

        if (benchmark.sequentialCellCount != sequentialResultCellCount)
        {
            parallelSequentialResultStable = false;
        }

        if (benchmark.actualSinglePlanningWorkerCount != static_cast<std::uint64_t>(parallelWorkerCounts[i]) ||
            benchmark.actualSingleApplicationWorkerCount != static_cast<std::uint64_t>(parallelWorkerCounts[i]))
        {
            requestedWorkerCountApplied = false;
        }

        if (benchmark.singleStatistics.classifiedCellCount != benchmark.singleStatistics.outsideCellCount + benchmark.singleStatistics.insideCellCount + benchmark.singleStatistics.intersectingCellCount ||
            benchmark.singleStatistics.pointQueryCount != benchmark.singleStatistics.centerSampleCount ||
            benchmark.sequentialStatistics.classifiedCellCount != benchmark.sequentialStatistics.outsideCellCount + benchmark.sequentialStatistics.insideCellCount + benchmark.sequentialStatistics.intersectingCellCount ||
            benchmark.sequentialStatistics.pointQueryCount != benchmark.sequentialStatistics.centerSampleCount)
        {
            parallelStatisticsConsistent = false;
        }

        parallelBenchmarks.push_back(benchmark);
    }

    tests.check(parallelSingleResultStable, L"不同线程数连续Shape单次结果一致");
    tests.check(parallelSequentialResultStable, L"不同线程数连续轨迹结果一致");
    tests.check(requestedWorkerCountApplied, L"指定并行线程数量实际生效");
    tests.check(parallelStatisticsConsistent, L"不同线程数布尔运算统计一致");

    /// 体素砂轮对照路径

    const MyVoxel::VoxelShape voxelTool = MyVoxel::voxelize(singleCutToolShape, baseVoxelEdgeLength, maximumLevel);
    MyVoxel::BooleanOperationStatistics voxelToolStatistics;

    const Clock::time_point voxelToolStart = Clock::now();
    const MyVoxel::VoxelShape voxelToolResult = MyVoxel::cut(workpiece, voxelTool, voxelToolStatistics);
    const Clock::time_point voxelToolEnd = Clock::now();

    benchmarkChecksum += static_cast<std::uint64_t>(rootCount(voxelToolResult));

    const double voxelToolMilliseconds = elapsedMilliseconds(voxelToolStart, voxelToolEnd);
    const std::uint64_t voxelToolResultCellCount = materialCellCountAtLevel(voxelToolResult, maximumLevel);

    tests.check(voxelToolResultCellCount < workpieceCellCount, L"体素砂轮对照路径删除工件材料");
    tests.check(matrixNearlyEqual(voxelTool.transform(), singleCutToolShape.transform()), L"砂轮体素化保留连续Shape姿态");
    tests.check(voxelToolStatistics.queryVisitedNodeCount > 0, L"体素砂轮路径执行体素区域查询");
    tests.check(voxelToolStatistics.accessorNodeVisitCount > 0, L"体素砂轮路径执行体素点查询");

    /// 输出结果

    std::wcout << std::fixed << std::setprecision(3);

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"标准Shape与体素配置" << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"构建模式                : " << buildMode() << std::endl;
    std::wcout << L"硬件并发线程数量        : " << std::thread::hardware_concurrency() << std::endl;
    std::wcout << L"请求最高层级            : " << static_cast<int>(maximumLevel) << std::endl;
    std::wcout << L"工件最高层级            : " << static_cast<int>(workpiece.maximumLevel()) << std::endl;
    std::wcout << L"最高层体素边长          : " << workpiece.voxelEdgeLength(maximumLevel) << std::endl;
    std::wcout << L"工件根节点数量          : " << workpieceRootCount << std::endl;
    std::wcout << L"工件材料体素数量        : " << workpieceCellCount << std::endl;

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"切削结果" << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"连续Shape单次结果       : " << continuousSingleCellCount << std::endl;
    std::wcout << L"连续Shape单次删除       : " << workpieceCellCount - continuousSingleCellCount << std::endl;
    std::wcout << L"连续轨迹最终结果        : " << sequentialResultCellCount << std::endl;
    std::wcout << L"连续轨迹累计删除        : " << workpieceCellCount - sequentialResultCellCount << std::endl;
    std::wcout << L"体素砂轮单次结果        : " << voxelToolResultCellCount << std::endl;
    std::wcout << L"体素砂轮单次删除        : " << workpieceCellCount - voxelToolResultCellCount << std::endl;
    std::wcout << L"两种砂轮结果差值        : " << static_cast<std::int64_t>(continuousSingleCellCount) - static_cast<std::int64_t>(voxelToolResultCellCount) << std::endl;

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"默认并行外部计时" << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"连续Shape单次切削       : " << continuousSingleMilliseconds << L" ms" << std::endl;
    std::wcout << L"连续Shape独立平均       : " << independentMilliseconds / static_cast<double>(independentRepeatCount) << L" ms/次" << std::endl;
    std::wcout << L"连续轨迹平均            : " << sequentialMilliseconds / static_cast<double>(sequentialCutCount) << L" ms/次" << std::endl;
    std::wcout << L"体素砂轮单次切削        : " << voxelToolMilliseconds << L" ms" << std::endl;

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"固定线程数性能对比" << std::endl;
    std::wcout << L"========================================" << std::endl;
    printParallelBenchmarkResults(parallelBenchmarks, sequentialCutCount);

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"默认连续Shape单次统计" << std::endl;
    std::wcout << L"========================================" << std::endl;
    printStatistics(continuousSingleStatistics, 1);

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"默认连续轨迹累计统计" << std::endl;
    std::wcout << L"========================================" << std::endl;
    printStatistics(sequentialStatistics, sequentialCutCount);

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"体素砂轮对照统计" << std::endl;
    std::wcout << L"========================================" << std::endl;
    printStatistics(voxelToolStatistics, 1);

    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"测试汇总" << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << L"通过                    : " << tests.passedCount() << std::endl;
    std::wcout << L"失败                    : " << tests.failedCount() << std::endl;
    std::wcout << L"基准校验值              : " << benchmarkChecksum << std::endl;

    return tests.exitCode();
}