#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <io.h>
#include <numeric>
#include <thread>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelTypes.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"
#include "MyVoxel/Modeling/PrimitiveModeling.h"
#include "MyVoxel/Modeling/ShapeVoxelization.h"
#include "MyVoxel/Operation/BooleanOperation.h"

#if !defined(MYVOXEL_TEST)
#error voxel_cut_performance_test requires MYVOXEL_TEST.
#endif

#if !MYVOXEL_BOOLEAN_STATISTICS_ENABLED
#error voxel_cut_performance_test requires boolean operation statistics.
#endif

namespace
{

using Clock = std::chrono::steady_clock;

const double Pi = 3.14159265358979323846; // 圆周率，用于构造刀具旋转角度。

// 保存正式性能测试使用的全部输入参数。
// 保存正式性能测试使用的全部输入参数。
struct PerformanceParameters
{
    PerformanceParameters()
        : baseVoxelEdgeLength(1.0)
        , maximumLevel(6)
        , workpieceRootCountX(6)
        , workpieceRootCountY(4)
        , workpieceRootCountZ(4)
        , workpieceGridOrigin(-3.0, -2.0, -2.0)
        , toolLengthX(2.4)
        , toolLengthY(2.4)
        , toolLengthZ(6.0)
        , toolRotationY(Pi / 6.0)
        , singleCutPositionX(0.5)
        , trajectoryStartX(-2.0)
        , trajectoryEndX(2.0)
        , trajectoryCutCount(20)
        , singleWarmupCount(2)
        , singleSampleCount(10)
        , trajectoryWarmupCount(1)
        , trajectorySampleCount(4)
        , voxelToolWarmupCount(1)
        , voxelToolSampleCount(3)
    {
        workerCounts[0] = 1;
        workerCounts[1] = 2;
        workerCounts[2] = 4;
        workerCounts[3] = 8;
    }

    double baseVoxelEdgeLength; // 第0层体素边长，单位为毫米。
    MyVoxel::VoxelLevel maximumLevel; // 体素切削使用的最高细分层级。

    MyVoxel::VoxelIndex workpieceRootCountX; // 工件X方向完整材料根节点数量。
    MyVoxel::VoxelIndex workpieceRootCountY; // 工件Y方向完整材料根节点数量。
    MyVoxel::VoxelIndex workpieceRootCountZ; // 工件Z方向完整材料根节点数量。
    MyMath::Vector3 workpieceGridOrigin; // 工件第0层网格原点。

    double toolLengthX; // 连续刀具局部X方向长度，单位为毫米。
    double toolLengthY; // 连续刀具局部Y方向长度，单位为毫米。
    double toolLengthZ; // 连续刀具局部Z方向长度，单位为毫米。
    double toolRotationY; // 连续刀具绕Y轴旋转角度。
    double singleCutPositionX; // 单次切削刀具中心X坐标，单位为毫米。

    double trajectoryStartX; // 连续轨迹起点X坐标，单位为毫米。
    double trajectoryEndX; // 连续轨迹终点X坐标，单位为毫米。
    int trajectoryCutCount; // 连续轨迹包含的切削刀位数量。

    int singleWarmupCount; // 单次连续刀具正式测试前的预热次数。
    int singleSampleCount; // 单次连续刀具正式采样次数。
    int trajectoryWarmupCount; // 连续轨迹正式测试前的预热次数。
    int trajectorySampleCount; // 完整连续轨迹正式采样次数。
    int voxelToolWarmupCount; // 体素刀具正式测试前的预热次数。
    int voxelToolSampleCount; // 体素刀具正式采样次数。

    std::array<unsigned int, 4> workerCounts; // 并行伸缩测试使用的线程数量。
};

// 保存一组外部计时样本。
struct TimingSummary
{
    // 添加一个毫秒计时样本。
    void add(double milliseconds)
    {
        samples.push_back(milliseconds);
    }

    // 返回是否包含有效样本。
    bool isValid() const
    {
        return !samples.empty();
    }

    // 返回全部样本的算术平均值。
    double average() const
    {
        if (samples.empty())
        {
            return 0.0;
        }

        return std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(samples.size());
    }

    // 返回全部样本的中位数。
    double median() const
    {
        if (samples.empty())
        {
            return 0.0;
        }

        std::vector<double> sortedSamples = samples;
        std::sort(sortedSamples.begin(), sortedSamples.end());

        const std::size_t middleIndex = sortedSamples.size() / 2;

        if ((sortedSamples.size() & 1U) != 0U)
        {
            return sortedSamples[middleIndex];
        }

        return (sortedSamples[middleIndex - 1] + sortedSamples[middleIndex]) * 0.5;
    }

    // 返回最小计时样本。
    double minimum() const
    {
        return samples.empty() ? 0.0 : *std::min_element(samples.begin(), samples.end());
    }

    // 返回最大计时样本。
    double maximum() const
    {
        return samples.empty() ? 0.0 : *std::max_element(samples.begin(), samples.end());
    }

    std::vector<double> samples; // 全部外部毫秒计时样本。
};

// 保存一次或多次同类切削的性能测试结果。
struct CutBenchmarkResult
{
    TimingSummary timing; // 外部总耗时样本。
    MyVoxel::Operation::BooleanOperationStatistics statistics; // 全部正式样本累计统计。
    std::uint64_t resultMaterialVoxelCount = 0; // 代表性结果展开到最高层后的材料体素数量。
    std::uint64_t modifiedRootCount = 0; // 代表性切削记录的修改根节点数量。
    bool resultStable = true; // 全部正式采样是否得到相同材料体素数量。
};

// 保存指定线程数量下的单次和连续轨迹性能。
struct ParallelBenchmarkResult
{
    unsigned int requestedWorkerCount = 0; // 请求的线程数量。
    std::uint64_t actualPlanningWorkerCount = 0; // 统计路径计划生成实际线程数量。
    std::uint64_t actualApplicationWorkerCount = 0; // 统计路径计划应用实际线程数量。
    CutBenchmarkResult single; // 无统计单次连续刀具性能。
    CutBenchmarkResult trajectory; // 无统计连续轨迹性能。
    MyVoxel::Operation::BooleanOperationStatistics diagnosticStatistics; // 一次带统计单次切削的内部统计。
};

// 设置Windows控制台使用宽字符输出。
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

// 返回当前编译器名称。
const wchar_t* compilerName()
{
#ifdef _MSC_VER
    return L"Microsoft Visual C++";
#elif defined(__clang__)
    return L"Clang";
#elif defined(__GNUC__)
    return L"GNU C++";
#else
    return L"Unknown";
#endif
}

// 返回当前目标平台名称。
const wchar_t* platformName()
{
#ifdef _M_X64
    return L"x64";
#elif defined(_M_IX86)
    return L"Win32";
#else
    return L"Unknown";
#endif
}

// 返回两个时间点之间的毫秒数。
double elapsedMilliseconds(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// 返回指定层级相对父层级的单轴细分数量。
std::uint64_t levelScale(MyVoxel::VoxelLevel sourceLevel, MyVoxel::VoxelLevel targetLevel)
{
    assert(sourceLevel <= targetLevel);

    std::uint64_t scale = 1;

    for (MyVoxel::VoxelLevel level = sourceLevel; level < targetLevel; ++level)
    {
        scale *= 2;
    }

    return scale;
}

// 返回材料叶节点展开到指定层级后的等效体素数量。
std::uint64_t materialVoxelCountAtLevel(const MyVoxel::VoxelShape& shape, MyVoxel::VoxelLevel targetLevel)
{
    std::uint64_t count = 0;

    shape.forest().forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            const std::uint64_t scale = levelScale(address.level, targetLevel);
            count += scale * scale * scale;
        });

    return count;
}

// 返回形体当前根节点数量。
std::size_t rootCount(const MyVoxel::VoxelShape& shape)
{
    return shape.forest().rootCount();
}
// 返回形体当前实际保存的材料叶节点数量。
std::uint64_t materialLeafCellCount(const MyVoxel::VoxelShape& shape)
{
    std::uint64_t count = 0;

    shape.forest().forEachMaterialCell(
        [&count](const MyVoxel::VoxelCellAddress&)
        {
            ++count;
        });

    return count;
}
// 返回指定层级的体素边长。
double voxelEdgeLength(double baseVoxelEdgeLength, MyVoxel::VoxelLevel level)
{
    double edgeLength = baseVoxelEdgeLength;

    for (MyVoxel::VoxelLevel currentLevel = MyVoxel::BaseVoxelLevel; currentLevel < level; ++currentLevel)
    {
        edgeLength *= 0.5;
    }

    return edgeLength;
}

// 创建绕Y轴旋转的刚体变换矩阵。
MyMath::Matrix4 makeRotationY(double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    MyMath::Matrix4 transform = MyMath::Matrix4::identity();

    transform(0, 0) = cosine;
    transform(0, 2) = sine;
    transform(2, 0) = -sine;
    transform(2, 2) = cosine;
    return transform;
}

// 返回指定包围盒的中心点。
MyMath::Vector3 boundsCenter(const MyVoxel::Bounds3& bounds)
{
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();
    return MyMath::Vector3((minimum.x() + maximum.x()) * 0.5, (minimum.y() + maximum.y()) * 0.5, (minimum.z() + maximum.z()) * 0.5);
}

// 创建固定数量完整材料根节点组成的工件。
MyVoxel::VoxelShape makeWorkpiece(const PerformanceParameters& parameters)
{
    const MyVoxel::VoxelGrid grid(parameters.workpieceGridOrigin, parameters.baseVoxelEdgeLength, parameters.maximumLevel);
    MyVoxel::VoxelShape workpiece(grid);
    MyVoxel::VoxelForest& forest = workpiece.editForest();

    for (MyVoxel::VoxelIndex z = 0; z < parameters.workpieceRootCountZ; ++z)
    {
        for (MyVoxel::VoxelIndex y = 0; y < parameters.workpieceRootCountY; ++y)
        {
            for (MyVoxel::VoxelIndex x = 0; x < parameters.workpieceRootCountX; ++x)
            {
                const MyVoxel::VoxelCellAddress address(MyVoxel::VoxelCellIndex(x, y, z), MyVoxel::BaseVoxelLevel);
                forest.setState(address, MyVoxel::VoxelState::Material);
            }
        }
    }

    return workpiece;
}

// 创建连续刀具局部几何。
MyVoxel::Geometry::Shape makeToolShape(const PerformanceParameters& parameters)
{
    return MyVoxel::Modeling::makeBox(parameters.toolLengthX, parameters.toolLengthY, parameters.toolLengthZ);
}

// 创建指定X位置的旋转连续刀具实例。
MyVoxel::Geometry::ShapeInstance makeToolInstance(const MyVoxel::Geometry::Shape& shape, const PerformanceParameters& parameters, double positionX)
{
    assert(shape.isValid());

    const MyMath::Vector3 localCenter = boundsCenter(shape.localBounds());
    const MyMath::Matrix4 moveLocalCenterToOrigin = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-localCenter.x(), -localCenter.y(), -localCenter.z()));
    const MyMath::Matrix4 rotation = makeRotationY(parameters.toolRotationY);
    const MyMath::Matrix4 translation = MyMath::Matrix4::fromTranslation(MyMath::Vector3(positionX, 0.0, 0.0));
    const MyMath::Matrix4 localToWorld = translation * rotation * moveLocalCenterToOrigin;

    assert(localToWorld.isRigidTransform());
    return MyVoxel::Geometry::ShapeInstance(shape, localToWorld);
}

// 创建连续刀具轨迹。
std::vector<MyVoxel::Geometry::ShapeInstance> makeToolPath(const MyVoxel::Geometry::Shape& shape, const PerformanceParameters& parameters)
{
    std::vector<MyVoxel::Geometry::ShapeInstance> path;
    path.reserve(static_cast<std::size_t>(parameters.trajectoryCutCount));

    for (int cutIndex = 0; cutIndex < parameters.trajectoryCutCount; ++cutIndex)
    {
        const double ratio = parameters.trajectoryCutCount > 1 ? static_cast<double>(cutIndex) / static_cast<double>(parameters.trajectoryCutCount - 1) : 0.0;
        const double positionX = parameters.trajectoryStartX + (parameters.trajectoryEndX - parameters.trajectoryStartX) * ratio;
        path.push_back(makeToolInstance(shape, parameters, positionX));
    }

    return path;
}

// 使用默认或指定线程选项执行一次不带统计的连续刀具切削。
MyVoxel::VoxelShape cutContinuousWithoutStatistics(const MyVoxel::VoxelShape& object, const MyVoxel::Geometry::ShapeInstance& tool, const MyVoxel::Operation::BooleanOperationOptions& options, MyVoxel::VoxelChangeSet& changes)
{
    return MyVoxel::Operation::cut(object, tool, options, changes);
}

// 使用默认或指定线程选项执行一次带统计的连续刀具切削。
MyVoxel::VoxelShape cutContinuousWithStatistics(const MyVoxel::VoxelShape& object, const MyVoxel::Geometry::ShapeInstance& tool, const MyVoxel::Operation::BooleanOperationOptions& options, MyVoxel::VoxelChangeSet& changes, MyVoxel::Operation::BooleanOperationStatistics& statistics)
{
    return MyVoxel::Operation::cut(object, tool, options, changes, statistics);
}

// 测量不带统计的连续刀具单次切削。
CutBenchmarkResult benchmarkContinuousSingleWithoutStatistics(const MyVoxel::VoxelShape& workpiece, const MyVoxel::Geometry::ShapeInstance& tool, const MyVoxel::Operation::BooleanOperationOptions& options, int warmupCount, int sampleCount, volatile std::uint64_t& checksum)
{
    CutBenchmarkResult benchmark;

    for (int warmupIndex = 0; warmupIndex < warmupCount; ++warmupIndex)
    {
        MyVoxel::VoxelChangeSet changes;
        const MyVoxel::VoxelShape result = cutContinuousWithoutStatistics(workpiece, tool, options, changes);

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += static_cast<std::uint64_t>(changes.modifiedRootCount());
    }

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        MyVoxel::VoxelChangeSet changes;

        const Clock::time_point start = Clock::now();
        const MyVoxel::VoxelShape result = cutContinuousWithoutStatistics(workpiece, tool, options, changes);
        const Clock::time_point end = Clock::now();

        benchmark.timing.add(elapsedMilliseconds(start, end));

        const std::uint64_t resultVoxelCount = materialVoxelCountAtLevel(result, workpiece.grid().maximumLevel());

        if (sampleIndex == 0)
        {
            benchmark.resultMaterialVoxelCount = resultVoxelCount;
            benchmark.modifiedRootCount = static_cast<std::uint64_t>(changes.modifiedRootCount());
        }
        else if (resultVoxelCount != benchmark.resultMaterialVoxelCount)
        {
            benchmark.resultStable = false;
        }

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += static_cast<std::uint64_t>(changes.modifiedRootCount());
    }

    return benchmark;
}

// 测量带统计的连续刀具单次切削。
CutBenchmarkResult benchmarkContinuousSingleWithStatistics(const MyVoxel::VoxelShape& workpiece, const MyVoxel::Geometry::ShapeInstance& tool, const MyVoxel::Operation::BooleanOperationOptions& options, int warmupCount, int sampleCount, volatile std::uint64_t& checksum)
{
    CutBenchmarkResult benchmark;

    for (int warmupIndex = 0; warmupIndex < warmupCount; ++warmupIndex)
    {
        MyVoxel::VoxelChangeSet changes;
        MyVoxel::Operation::BooleanOperationStatistics statistics;
        const MyVoxel::VoxelShape result = cutContinuousWithStatistics(workpiece, tool, options, changes, statistics);

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += statistics.visitedCellCount;
    }

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        MyVoxel::VoxelChangeSet changes;
        MyVoxel::Operation::BooleanOperationStatistics statistics;

        const Clock::time_point start = Clock::now();
        const MyVoxel::VoxelShape result = cutContinuousWithStatistics(workpiece, tool, options, changes, statistics);
        const Clock::time_point end = Clock::now();

        benchmark.timing.add(elapsedMilliseconds(start, end));
        benchmark.statistics.accumulate(statistics);

        const std::uint64_t resultVoxelCount = materialVoxelCountAtLevel(result, workpiece.grid().maximumLevel());

        if (sampleIndex == 0)
        {
            benchmark.resultMaterialVoxelCount = resultVoxelCount;
            benchmark.modifiedRootCount = static_cast<std::uint64_t>(changes.modifiedRootCount());
        }
        else if (resultVoxelCount != benchmark.resultMaterialVoxelCount)
        {
            benchmark.resultStable = false;
        }

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += statistics.visitedCellCount;
    }

    return benchmark;
}

// 测量不带统计的完整连续轨迹。
CutBenchmarkResult benchmarkTrajectoryWithoutStatistics(const MyVoxel::VoxelShape& workpiece, const std::vector<MyVoxel::Geometry::ShapeInstance>& path, const MyVoxel::Operation::BooleanOperationOptions& options, int warmupCount, int sampleCount, volatile std::uint64_t& checksum)
{
    CutBenchmarkResult benchmark;

    for (int warmupIndex = 0; warmupIndex < warmupCount; ++warmupIndex)
    {
        MyVoxel::VoxelShape result = workpiece;

        for (std::size_t toolIndex = 0; toolIndex < path.size(); ++toolIndex)
        {
            MyVoxel::VoxelChangeSet changes;
            result = cutContinuousWithoutStatistics(result, path[toolIndex], options, changes);
            checksum += static_cast<std::uint64_t>(changes.modifiedRootCount());
        }

        checksum += static_cast<std::uint64_t>(rootCount(result));
    }

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        MyVoxel::VoxelShape result = workpiece;
        std::uint64_t modifiedRootCount = 0;

        const Clock::time_point start = Clock::now();

        for (std::size_t toolIndex = 0; toolIndex < path.size(); ++toolIndex)
        {
            MyVoxel::VoxelChangeSet changes;
            result = cutContinuousWithoutStatistics(result, path[toolIndex], options, changes);
            modifiedRootCount += static_cast<std::uint64_t>(changes.modifiedRootCount());
        }

        const Clock::time_point end = Clock::now();

        benchmark.timing.add(elapsedMilliseconds(start, end));

        const std::uint64_t resultVoxelCount = materialVoxelCountAtLevel(result, workpiece.grid().maximumLevel());

        if (sampleIndex == 0)
        {
            benchmark.resultMaterialVoxelCount = resultVoxelCount;
            benchmark.modifiedRootCount = modifiedRootCount;
        }
        else if (resultVoxelCount != benchmark.resultMaterialVoxelCount)
        {
            benchmark.resultStable = false;
        }

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += modifiedRootCount;
    }

    return benchmark;
}

// 执行一次指定线程数量的带统计连续刀具切削。
MyVoxel::Operation::BooleanOperationStatistics runContinuousDiagnostic(const MyVoxel::VoxelShape& workpiece, const MyVoxel::Geometry::ShapeInstance& tool, const MyVoxel::Operation::BooleanOperationOptions& options, std::uint64_t& resultVoxelCount, std::uint64_t& modifiedRootCount, volatile std::uint64_t& checksum)
{
    MyVoxel::VoxelChangeSet changes;
    MyVoxel::Operation::BooleanOperationStatistics statistics;
    const MyVoxel::VoxelShape result = cutContinuousWithStatistics(workpiece, tool, options, changes, statistics);

    resultVoxelCount = materialVoxelCountAtLevel(result, workpiece.grid().maximumLevel());
    modifiedRootCount = static_cast<std::uint64_t>(changes.modifiedRootCount());
    checksum += static_cast<std::uint64_t>(rootCount(result));
    checksum += statistics.visitedCellCount;
    return statistics;
}

// 测量指定线程数下的单次和轨迹性能。
ParallelBenchmarkResult benchmarkParallel(const MyVoxel::VoxelShape& workpiece, const MyVoxel::Geometry::ShapeInstance& singleTool, const std::vector<MyVoxel::Geometry::ShapeInstance>& path, const PerformanceParameters& parameters, unsigned int workerCount, volatile std::uint64_t& checksum)
{
    ParallelBenchmarkResult benchmark;
    benchmark.requestedWorkerCount = workerCount;

    MyVoxel::Operation::BooleanOperationOptions options;
    options.workerCount = workerCount;
    options.minimumParallelRootCount = 1;

    benchmark.single = benchmarkContinuousSingleWithoutStatistics(workpiece, singleTool, options, parameters.singleWarmupCount, parameters.singleSampleCount, checksum);
    benchmark.trajectory = benchmarkTrajectoryWithoutStatistics(workpiece, path, options, parameters.trajectoryWarmupCount, parameters.trajectorySampleCount, checksum);

    std::uint64_t diagnosticResultVoxelCount = 0;
    std::uint64_t diagnosticModifiedRootCount = 0;

    benchmark.diagnosticStatistics = runContinuousDiagnostic(workpiece, singleTool, options, diagnosticResultVoxelCount, diagnosticModifiedRootCount, checksum);
    benchmark.actualPlanningWorkerCount = benchmark.diagnosticStatistics.planningWorkerCount;
    benchmark.actualApplicationWorkerCount = benchmark.diagnosticStatistics.planApplicationWorkerCount;

    if (diagnosticResultVoxelCount != benchmark.single.resultMaterialVoxelCount || diagnosticModifiedRootCount != benchmark.single.modifiedRootCount)
    {
        benchmark.single.resultStable = false;
    }

    return benchmark;
}

// 测量不带统计的体素刀具切削。
CutBenchmarkResult benchmarkVoxelToolWithoutStatistics(const MyVoxel::VoxelShape& workpiece, const MyVoxel::VoxelShape& tool, int warmupCount, int sampleCount, volatile std::uint64_t& checksum)
{
    CutBenchmarkResult benchmark;

    for (int warmupIndex = 0; warmupIndex < warmupCount; ++warmupIndex)
    {
        MyVoxel::VoxelChangeSet changes;
        const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes);

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += static_cast<std::uint64_t>(changes.modifiedRootCount());
    }

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        MyVoxel::VoxelChangeSet changes;

        const Clock::time_point start = Clock::now();
        const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes);
        const Clock::time_point end = Clock::now();

        benchmark.timing.add(elapsedMilliseconds(start, end));

        const std::uint64_t resultVoxelCount = materialVoxelCountAtLevel(result, workpiece.grid().maximumLevel());

        if (sampleIndex == 0)
        {
            benchmark.resultMaterialVoxelCount = resultVoxelCount;
            benchmark.modifiedRootCount = static_cast<std::uint64_t>(changes.modifiedRootCount());
        }
        else if (resultVoxelCount != benchmark.resultMaterialVoxelCount)
        {
            benchmark.resultStable = false;
        }

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += static_cast<std::uint64_t>(changes.modifiedRootCount());
    }

    return benchmark;
}

// 测量带统计的体素刀具切削。
CutBenchmarkResult benchmarkVoxelToolWithStatistics(const MyVoxel::VoxelShape& workpiece, const MyVoxel::VoxelShape& tool, int warmupCount, int sampleCount, volatile std::uint64_t& checksum)
{
    CutBenchmarkResult benchmark;

    for (int warmupIndex = 0; warmupIndex < warmupCount; ++warmupIndex)
    {
        MyVoxel::VoxelChangeSet changes;
        MyVoxel::Operation::BooleanOperationStatistics statistics;
        const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes, statistics);

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += statistics.visitedCellCount;
    }

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        MyVoxel::VoxelChangeSet changes;
        MyVoxel::Operation::BooleanOperationStatistics statistics;

        const Clock::time_point start = Clock::now();
        const MyVoxel::VoxelShape result = MyVoxel::Operation::cut(workpiece, tool, changes, statistics);
        const Clock::time_point end = Clock::now();

        benchmark.timing.add(elapsedMilliseconds(start, end));
        benchmark.statistics.accumulate(statistics);

        const std::uint64_t resultVoxelCount = materialVoxelCountAtLevel(result, workpiece.grid().maximumLevel());

        if (sampleIndex == 0)
        {
            benchmark.resultMaterialVoxelCount = resultVoxelCount;
            benchmark.modifiedRootCount = static_cast<std::uint64_t>(changes.modifiedRootCount());
        }
        else if (resultVoxelCount != benchmark.resultMaterialVoxelCount)
        {
            benchmark.resultStable = false;
        }

        checksum += static_cast<std::uint64_t>(rootCount(result));
        checksum += statistics.visitedCellCount;
    }

    return benchmark;
}

// 输出分隔标题。
void printSection(const wchar_t* title)
{
    std::wcout << std::endl;
    std::wcout << L"========================================" << std::endl;
    std::wcout << title << std::endl;
    std::wcout << L"========================================" << std::endl;
}

// 输出一组外部计时摘要。
void printTimingSummary(const wchar_t* name, const TimingSummary& timing, const wchar_t* suffix)
{
    std::wcout << name << L"平均 : " << timing.average() << suffix << std::endl;
    std::wcout << name << L"中位 : " << timing.median() << suffix << std::endl;
    std::wcout << name << L"最小 : " << timing.minimum() << suffix << std::endl;
    std::wcout << name << L"最大 : " << timing.maximum() << suffix << std::endl;
}

// 输出布尔运算累计统计的单次平均值。
void printStatistics(const MyVoxel::Operation::BooleanOperationStatistics& statistics, std::uint64_t operationCount)
{
    const double divisor = operationCount > 0 ? static_cast<double>(operationCount) : 1.0;
    const std::uint64_t integerDivisor = operationCount > 0 ? operationCount : 1;

    std::wcout << std::fixed << std::setprecision(6);
    std::wcout << L"内部总耗时平均          : " << statistics.totalMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"查询上下文平均          : " << statistics.toolIndexMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"入口收集平均            : " << statistics.materialCollectionMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"写时复制平均            : " << statistics.detachMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"计划生成平均            : " << statistics.planningMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"根树准备平均            : " << statistics.rootPreparationMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"计划应用平均            : " << statistics.planApplicationMilliseconds / divisor << L" ms/次" << std::endl;
    std::wcout << L"递归切削平均            : " << statistics.cuttingMilliseconds / divisor << L" ms/次" << std::endl;

    std::wcout << std::setprecision(3);
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
    std::wcout << L"体素区域查询候选平均    : " << statistics.queryRootCandidateCount / integerDivisor << std::endl;
    std::wcout << L"体素区域查询存在根平均  : " << statistics.queryExistingRootCount / integerDivisor << std::endl;
    std::wcout << L"体素区域包围盒测试平均  : " << statistics.queryNodeBoundsTestCount / integerDivisor << std::endl;
    std::wcout << L"体素区域访问节点平均    : " << statistics.queryVisitedNodeCount / integerDivisor << std::endl;
    std::wcout << L"访问器根缓存命中平均    : " << statistics.accessorRootCacheHitCount / integerDivisor << std::endl;
    std::wcout << L"访问器根缓存未命中平均  : " << statistics.accessorRootCacheMissCount / integerDivisor << std::endl;
    std::wcout << L"访问器路径复用平均      : " << statistics.accessorPathReuseCount / integerDivisor << std::endl;
    std::wcout << L"访问器复用层级平均      : " << statistics.accessorReusedPathLevelCount / integerDivisor << std::endl;
    std::wcout << L"体素点查询节点平均      : " << statistics.accessorNodeVisitCount / integerDivisor << std::endl;
}

// 输出全部正式测试参数。
void printParameters(const PerformanceParameters& parameters, const MyVoxel::Geometry::Shape& toolShape)
{
    const MyVoxel::Bounds3 toolBounds = toolShape.localBounds();

    printSection(L"正式性能测试参数");

    std::wcout << L"构建模式                : " << buildMode() << std::endl;
    std::wcout << L"编译器                  : " << compilerName();
#ifdef _MSC_VER
    std::wcout << L" " << _MSC_VER;
#endif
    std::wcout << std::endl;
    std::wcout << L"目标平台                : " << platformName() << std::endl;
    std::wcout << L"硬件并发线程数量        : " << std::thread::hardware_concurrency() << std::endl;
    std::wcout << L"测试宏MYVOXEL_TEST      : 已定义" << std::endl;
    std::wcout << L"性能统计编译            : 已启用" << std::endl;
    std::wcout << L"正式计时路径            : 不带性能统计" << std::endl;
    std::wcout << L"变更集记录              : 启用" << std::endl;

    std::wcout << std::endl;
    std::wcout << L"第0层体素边长           : " << parameters.baseVoxelEdgeLength << L" mm" << std::endl;
    std::wcout << L"最高细分层级            : " << static_cast<unsigned int>(parameters.maximumLevel) << std::endl;
    std::wcout << L"最高层体素边长          : " << voxelEdgeLength(parameters.baseVoxelEdgeLength, parameters.maximumLevel) << L" mm" << std::endl;
    std::wcout << L"工件网格原点            : (" << parameters.workpieceGridOrigin.x() << L", " << parameters.workpieceGridOrigin.y() << L", " << parameters.workpieceGridOrigin.z() << L")" << std::endl;
    std::wcout << L"工件根节点排列          : " << parameters.workpieceRootCountX << L" × " << parameters.workpieceRootCountY << L" × " << parameters.workpieceRootCountZ << std::endl;
    std::wcout << L"工件根节点总数          : " << parameters.workpieceRootCountX * parameters.workpieceRootCountY * parameters.workpieceRootCountZ << std::endl;
    std::wcout << L"工件物理尺寸            : " << parameters.workpieceRootCountX * parameters.baseVoxelEdgeLength << L" × " << parameters.workpieceRootCountY * parameters.baseVoxelEdgeLength << L" × " << parameters.workpieceRootCountZ * parameters.baseVoxelEdgeLength << L" mm" << std::endl;

    std::wcout << std::endl;
    std::wcout << L"连续刀具类型            : 旋转盒体" << std::endl;
    std::wcout << L"刀具输入尺寸            : " << parameters.toolLengthX << L" × " << parameters.toolLengthY << L" × " << parameters.toolLengthZ << L" mm" << std::endl;
    std::wcout << L"刀具局部最小点          : (" << toolBounds.minimum().x() << L", " << toolBounds.minimum().y() << L", " << toolBounds.minimum().z() << L")" << std::endl;
    std::wcout << L"刀具局部最大点          : (" << toolBounds.maximum().x() << L", " << toolBounds.maximum().y() << L", " << toolBounds.maximum().z() << L")" << std::endl;
    std::wcout << L"刀具绕Y轴角度           : " << parameters.toolRotationY * 180.0 / Pi << L"°" << std::endl;
    std::wcout << L"单次切削X位置           : " << parameters.singleCutPositionX << L" mm" << std::endl;
    std::wcout << L"轨迹起止X位置           : " << parameters.trajectoryStartX << L" → " << parameters.trajectoryEndX << L" mm" << std::endl;
    std::wcout << L"连续轨迹刀位数量        : " << parameters.trajectoryCutCount << std::endl;

    std::wcout << std::endl;
    std::wcout << L"单次切削预热/采样       : " << parameters.singleWarmupCount << L" / " << parameters.singleSampleCount << std::endl;
    std::wcout << L"连续轨迹预热/采样       : " << parameters.trajectoryWarmupCount << L" / " << parameters.trajectorySampleCount << std::endl;
    std::wcout << L"体素刀具预热/采样       : " << parameters.voxelToolWarmupCount << L" / " << parameters.voxelToolSampleCount << std::endl;
    std::wcout << L"固定线程数量            : ";

    for (std::size_t index = 0; index < parameters.workerCounts.size(); ++index)
    {
        if (index > 0)
        {
            std::wcout << L", ";
        }

        std::wcout << parameters.workerCounts[index];
    }

    std::wcout << std::endl;
}

// 输出固定线程数量性能表。
void printParallelResults(const std::vector<ParallelBenchmarkResult>& benchmarks, int trajectoryCutCount)
{
    printSection(L"固定线程数无统计性能对比");

    std::wcout << L"请求  生成  应用  单次平均  单次中位  单次加速  轨迹平均  每刀平均  计划生成  根树准备  计划应用" << std::endl;
    std::wcout << L"线程  线程  线程      ms        ms         倍        ms        ms        ms        ms        ms" << std::endl;

    const double singleThreadAverage = benchmarks.empty() ? 0.0 : benchmarks.front().single.timing.average();

    for (std::size_t index = 0; index < benchmarks.size(); ++index)
    {
        const ParallelBenchmarkResult& benchmark = benchmarks[index];
        const double speedup = benchmark.single.timing.average() > 0.0 ? singleThreadAverage / benchmark.single.timing.average() : 0.0;
        const double perCutMilliseconds = trajectoryCutCount > 0 ? benchmark.trajectory.timing.average() / static_cast<double>(trajectoryCutCount) : 0.0;

        std::wcout << std::setw(4) << benchmark.requestedWorkerCount
                   << std::setw(6) << benchmark.actualPlanningWorkerCount
                   << std::setw(6) << benchmark.actualApplicationWorkerCount
                   << std::setw(12) << benchmark.single.timing.average()
                   << std::setw(12) << benchmark.single.timing.median()
                   << std::setw(11) << speedup
                   << std::setw(12) << benchmark.trajectory.timing.average()
                   << std::setw(11) << perCutMilliseconds
                   << std::setw(12) << benchmark.diagnosticStatistics.planningMilliseconds
                   << std::setw(12) << benchmark.diagnosticStatistics.rootPreparationMilliseconds
                   << std::setw(12) << benchmark.diagnosticStatistics.planApplicationMilliseconds
                   << std::endl;
    }
}

}

int main()
{
    initializeConsole();
    std::wcout << std::fixed << std::setprecision(3);

    const PerformanceParameters parameters;
    volatile std::uint64_t benchmarkChecksum = 0;

    /// 创建测试输入

    const MyVoxel::VoxelShape workpiece = makeWorkpiece(parameters);
    const MyVoxel::Geometry::Shape toolShape = makeToolShape(parameters);
    const MyVoxel::Geometry::ShapeInstance singleTool = makeToolInstance(toolShape, parameters, parameters.singleCutPositionX);
    const std::vector<MyVoxel::Geometry::ShapeInstance> toolPath = makeToolPath(toolShape, parameters);

    printParameters(parameters, toolShape);

    const Clock::time_point voxelToolStart = Clock::now();
    const MyVoxel::VoxelShape voxelTool = MyVoxel::Modeling::voxelize(singleTool, workpiece.grid());
    const Clock::time_point voxelToolEnd = Clock::now();

    const double voxelToolVoxelizationMilliseconds = elapsedMilliseconds(voxelToolStart, voxelToolEnd);
    const std::uint64_t workpieceMaterialVoxelCount = materialVoxelCountAtLevel(workpiece, parameters.maximumLevel);
    const std::uint64_t voxelToolMaterialVoxelCount = materialVoxelCountAtLevel(voxelTool, parameters.maximumLevel);

    printSection(L"离散输入规模");

    std::wcout << L"工件根节点数量          : " << rootCount(workpiece) << std::endl;
    std::wcout << L"工件最高层等效体素      : " << workpieceMaterialVoxelCount << std::endl;
    std::wcout << L"体素刀具根节点数量      : " << rootCount(voxelTool) << std::endl;
    std::wcout << L"工件材料叶节点数量      : " << materialLeafCellCount(workpiece) << std::endl;
    std::wcout << L"体素刀具材料叶节点      : " << materialLeafCellCount(voxelTool) << std::endl;
    std::wcout << L"体素刀具最高层等效体素  : " << voxelToolMaterialVoxelCount << std::endl;
    std::wcout << L"体素刀具体素化耗时      : " << voxelToolVoxelizationMilliseconds << L" ms" << std::endl;

    /// 默认自动线程性能

    MyVoxel::Operation::BooleanOperationOptions automaticOptions;
    automaticOptions.workerCount = 0;
    automaticOptions.minimumParallelRootCount = 8;

    const CutBenchmarkResult continuousWithoutStatistics =
        benchmarkContinuousSingleWithoutStatistics(
            workpiece,
            singleTool,
            automaticOptions,
            parameters.singleWarmupCount,
            parameters.singleSampleCount,
            benchmarkChecksum);

    const CutBenchmarkResult continuousWithStatistics =
        benchmarkContinuousSingleWithStatistics(
            workpiece,
            singleTool,
            automaticOptions,
            parameters.singleWarmupCount,
            parameters.singleSampleCount,
            benchmarkChecksum);

    const CutBenchmarkResult trajectoryWithoutStatistics =
        benchmarkTrajectoryWithoutStatistics(
            workpiece,
            toolPath,
            automaticOptions,
            parameters.trajectoryWarmupCount,
            parameters.trajectorySampleCount,
            benchmarkChecksum);

    /// 固定线程数量性能

    std::vector<ParallelBenchmarkResult> parallelBenchmarks;
    parallelBenchmarks.reserve(parameters.workerCounts.size());

    for (std::size_t index = 0; index < parameters.workerCounts.size(); ++index)
    {
        parallelBenchmarks.push_back(
            benchmarkParallel(
                workpiece,
                singleTool,
                toolPath,
                parameters,
                parameters.workerCounts[index],
                benchmarkChecksum));
    }

    /// 体素刀具性能

    const CutBenchmarkResult voxelWithoutStatistics =
        benchmarkVoxelToolWithoutStatistics(
            workpiece,
            voxelTool,
            parameters.voxelToolWarmupCount,
            parameters.voxelToolSampleCount,
            benchmarkChecksum);

    const CutBenchmarkResult voxelWithStatistics =
        benchmarkVoxelToolWithStatistics(
            workpiece,
            voxelTool,
            parameters.voxelToolWarmupCount,
            parameters.voxelToolSampleCount,
            benchmarkChecksum);

    /// 正确性和稳定性

    const bool continuousRemovedMaterial =
        continuousWithoutStatistics.resultMaterialVoxelCount < workpieceMaterialVoxelCount &&
        continuousWithoutStatistics.resultMaterialVoxelCount > 0;

    const bool voxelRemovedMaterial =
        voxelWithoutStatistics.resultMaterialVoxelCount < workpieceMaterialVoxelCount &&
        voxelWithoutStatistics.resultMaterialVoxelCount > 0;

    const bool continuousPathsMatch =
        continuousWithoutStatistics.resultMaterialVoxelCount ==
        continuousWithStatistics.resultMaterialVoxelCount;

    const bool voxelPathsMatch =
        voxelWithoutStatistics.resultMaterialVoxelCount ==
        voxelWithStatistics.resultMaterialVoxelCount;

    const bool trajectoryRemovedMoreMaterial =
        trajectoryWithoutStatistics.resultMaterialVoxelCount <
        continuousWithoutStatistics.resultMaterialVoxelCount;

    bool parallelResultsStable = true;
    bool requestedWorkerCountsApplied = true;

    for (std::size_t index = 0; index < parallelBenchmarks.size(); ++index)
    {
        const ParallelBenchmarkResult& benchmark = parallelBenchmarks[index];

        if (!benchmark.single.resultStable ||
            !benchmark.trajectory.resultStable ||
            benchmark.single.resultMaterialVoxelCount != continuousWithoutStatistics.resultMaterialVoxelCount ||
            benchmark.trajectory.resultMaterialVoxelCount != trajectoryWithoutStatistics.resultMaterialVoxelCount)
        {
            parallelResultsStable = false;
        }

        if (benchmark.actualPlanningWorkerCount != benchmark.requestedWorkerCount)
        {
            requestedWorkerCountsApplied = false;
        }

        if (benchmark.actualApplicationWorkerCount == 0 ||
            benchmark.actualApplicationWorkerCount > benchmark.requestedWorkerCount)
        {
            requestedWorkerCountsApplied = false;
        }
    }

    const bool statisticsConsistent =
        continuousWithStatistics.statistics.classifiedCellCount ==
            continuousWithStatistics.statistics.outsideCellCount +
            continuousWithStatistics.statistics.insideCellCount +
            continuousWithStatistics.statistics.intersectingCellCount &&
        continuousWithStatistics.statistics.pointQueryCount ==
            continuousWithStatistics.statistics.centerSampleCount &&
        voxelWithStatistics.statistics.classifiedCellCount ==
            voxelWithStatistics.statistics.outsideCellCount +
            voxelWithStatistics.statistics.insideCellCount +
            voxelWithStatistics.statistics.intersectingCellCount &&
        voxelWithStatistics.statistics.pointQueryCount ==
            voxelWithStatistics.statistics.centerSampleCount;

    printSection(L"结果与稳定性");

    std::wcout << L"工件初始材料体素        : " << workpieceMaterialVoxelCount << std::endl;
    std::wcout << L"连续单次结果体素        : " << continuousWithoutStatistics.resultMaterialVoxelCount << std::endl;
    std::wcout << L"连续单次删除体素        : " << workpieceMaterialVoxelCount - continuousWithoutStatistics.resultMaterialVoxelCount << std::endl;
    std::wcout << L"连续单次修改根数        : " << continuousWithoutStatistics.modifiedRootCount << std::endl;
    std::wcout << L"连续轨迹最终体素        : " << trajectoryWithoutStatistics.resultMaterialVoxelCount << std::endl;
    std::wcout << L"连续轨迹累计删除        : " << workpieceMaterialVoxelCount - trajectoryWithoutStatistics.resultMaterialVoxelCount << std::endl;
    std::wcout << L"体素刀具结果体素        : " << voxelWithoutStatistics.resultMaterialVoxelCount << std::endl;
    std::wcout << L"体素刀具删除体素        : " << workpieceMaterialVoxelCount - voxelWithoutStatistics.resultMaterialVoxelCount << std::endl;
    std::wcout << L"两种刀具结果差值        : " << static_cast<std::int64_t>(voxelWithoutStatistics.resultMaterialVoxelCount) - static_cast<std::int64_t>(continuousWithoutStatistics.resultMaterialVoxelCount) << std::endl;

    std::wcout << std::endl;
    std::wcout << L"连续刀具确实删除材料    : " << (continuousRemovedMaterial ? L"是" : L"否") << std::endl;
    std::wcout << L"体素刀具确实删除材料    : " << (voxelRemovedMaterial ? L"是" : L"否") << std::endl;
    std::wcout << L"连续无统计/统计结果一致 : " << (continuousPathsMatch ? L"是" : L"否") << std::endl;
    std::wcout << L"体素无统计/统计结果一致 : " << (voxelPathsMatch ? L"是" : L"否") << std::endl;
    std::wcout << L"连续轨迹删除更多材料    : " << (trajectoryRemovedMoreMaterial ? L"是" : L"否") << std::endl;
    std::wcout << L"不同线程结果稳定        : " << (parallelResultsStable ? L"是" : L"否") << std::endl;
    std::wcout << L"指定线程数量实际生效    : " << (requestedWorkerCountsApplied ? L"是" : L"否") << std::endl;
    std::wcout << L"统计内部关系一致        : " << (statisticsConsistent ? L"是" : L"否") << std::endl;

    printSection(L"默认自动线程正式性能");

    std::wcout << L"正式路径                : 无性能统计、记录变更集" << std::endl;
    printTimingSummary(L"连续单次", continuousWithoutStatistics.timing, L" ms");
    std::wcout << std::endl;
    printTimingSummary(L"连续轨迹", trajectoryWithoutStatistics.timing, L" ms");
    std::wcout << L"连续轨迹每刀平均        : " << trajectoryWithoutStatistics.timing.average() / static_cast<double>(parameters.trajectoryCutCount) << L" ms/刀" << std::endl;

    printSection(L"性能统计开销对比");

    printTimingSummary(L"无统计连续单次", continuousWithoutStatistics.timing, L" ms");
    std::wcout << std::endl;
    printTimingSummary(L"带统计连续单次", continuousWithStatistics.timing, L" ms");

    const double continuousStatisticsOverhead =
        continuousWithoutStatistics.timing.average() > 0.0
            ? continuousWithStatistics.timing.average() / continuousWithoutStatistics.timing.average()
            : 0.0;

    std::wcout << L"连续统计版/正式版       : " << continuousStatisticsOverhead << L" 倍" << std::endl;

    printParallelResults(parallelBenchmarks, parameters.trajectoryCutCount);

    printSection(L"连续刀具带统计内部明细");
    printStatistics(continuousWithStatistics.statistics, static_cast<std::uint64_t>(parameters.singleSampleCount));

    printSection(L"体素刀具正式性能");

    printTimingSummary(L"无统计体素刀具", voxelWithoutStatistics.timing, L" ms");
    std::wcout << std::endl;
    printTimingSummary(L"带统计体素刀具", voxelWithStatistics.timing, L" ms");

    const double voxelStatisticsOverhead =
        voxelWithoutStatistics.timing.average() > 0.0
            ? voxelWithStatistics.timing.average() / voxelWithoutStatistics.timing.average()
            : 0.0;

    const double voxelToContinuousRatio =
        continuousWithoutStatistics.timing.average() > 0.0
            ? voxelWithoutStatistics.timing.average() / continuousWithoutStatistics.timing.average()
            : 0.0;

    std::wcout << L"体素统计版/正式版       : " << voxelStatisticsOverhead << L" 倍" << std::endl;
    std::wcout << L"体素刀具/连续刀具       : " << voxelToContinuousRatio << L" 倍" << std::endl;

    printSection(L"体素刀具带统计内部明细");
    printStatistics(voxelWithStatistics.statistics, static_cast<std::uint64_t>(parameters.voxelToolSampleCount));

    const bool passed =
        continuousRemovedMaterial &&
        voxelRemovedMaterial &&
        continuousPathsMatch &&
        voxelPathsMatch &&
        trajectoryRemovedMoreMaterial &&
        continuousWithoutStatistics.resultStable &&
        continuousWithStatistics.resultStable &&
        trajectoryWithoutStatistics.resultStable &&
        voxelWithoutStatistics.resultStable &&
        voxelWithStatistics.resultStable &&
        parallelResultsStable &&
        requestedWorkerCountsApplied &&
        statisticsConsistent;

    printSection(L"正式性能测试汇总");

    std::wcout << L"总体结果                : " << (passed ? L"通过" : L"失败") << std::endl;
    std::wcout << L"防优化校验值            : " << benchmarkChecksum << std::endl;

    return passed ? 0 : 1;
}