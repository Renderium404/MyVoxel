#include <cstdlib>
#include <iostream>

#include "MyMath/Vector3.h"
#include "MyVoxel/Modeling/Feature/FeatureTrace.h"

namespace
{

std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果。
void check(bool condition, const char* name)
{
    if (condition)
    {
        ++g_passedCount;
        std::cout << "[PASS] " << name << std::endl;
    }
    else
    {
        ++g_failedCount;
        std::cout << "[FAIL] " << name << std::endl;
    }
}

}

int main()
{
    std::cout << "MyVoxel FeatureTrace F1 test" << std::endl << std::endl;

    /// 空轨迹

    MyVoxel::Modeling::FeatureTrace trace;

    check(trace.isEmpty(), "Default FeatureTrace is empty");
    check(trace.pointCount() == 0, "Empty FeatureTrace point count is zero");
    check(!trace.hasSegment(), "Empty FeatureTrace has no segment");
    check(!trace.isClosed(), "Empty FeatureTrace is not closed");
    check(!trace.removeLastPoint(), "Removing point from empty FeatureTrace returns false");

    /// 开放轨迹

    const MyMath::Vector3 p0(0.0, 0.0, 0.0);
    const MyMath::Vector3 p1(1.0, 2.0, 3.0);
    const MyMath::Vector3 p2(4.0, 5.0, 6.0);

    check(trace.addPoint(p0), "First feature point added");
    check(trace.pointCount() == 1, "Single-point FeatureTrace point count correct");
    check(!trace.hasSegment(), "Single-point FeatureTrace has no segment");
    check(trace.startPoint().isEqualTo(p0, 0.0), "Single-point FeatureTrace start point correct");
    check(trace.endPoint().isEqualTo(p0, 0.0), "Single-point FeatureTrace end point correct");

    check(trace.addPoint(p1), "Second feature point added");
    check(trace.hasSegment(), "Two-point FeatureTrace has segment");
    check(!trace.isClosed(), "Two-point FeatureTrace is open");
    check(trace.startPoint().isEqualTo(p0, 0.0), "Open FeatureTrace start point correct");
    check(trace.endPoint().isEqualTo(p1, 0.0), "Open FeatureTrace end point correct");

    /// 相邻重复点

    check(!trace.addPoint(p1), "Adjacent duplicate feature point rejected");
    check(trace.pointCount() == 2, "Rejected duplicate does not change FeatureTrace");

    /// 三维点序列

    check(trace.addPoint(p2), "Third three-dimensional feature point added");
    check(trace.pointCount() == 3, "Three-point FeatureTrace point count correct");
    check(trace.point(0).isEqualTo(p0, 0.0), "FeatureTrace first stored point correct");
    check(trace.point(1).isEqualTo(p1, 0.0), "FeatureTrace second stored point correct");
    check(trace.point(2).isEqualTo(p2, 0.0), "FeatureTrace third stored point correct");

    /// 删除

    check(trace.removeLastPoint(), "Removing last feature point succeeds");
    check(trace.pointCount() == 2, "Removing last feature point updates count");
    check(trace.endPoint().isEqualTo(p1, 0.0), "Removing last feature point updates endpoint");

    /// 闭合轨迹

    trace.clear();

    check(trace.addPoint(p0), "Closed trace first point added");
    check(trace.addPoint(p1), "Closed trace second point added");
    check(trace.addPoint(p2), "Closed trace third point added");
    check(trace.addPoint(p0), "Repeated start point accepted as closing point");

    check(trace.pointCount() == 4, "Closed FeatureTrace keeps explicit closing point");
    check(trace.isClosed(), "Repeated start point closes FeatureTrace");
    check(trace.startPoint().isEqualTo(trace.endPoint(), 0.0), "Closed FeatureTrace endpoints coincide");

    check(!trace.addPoint(p0), "Adjacent duplicate after closure rejected");
    check(trace.pointCount() == 4, "Rejected closing duplicate does not change count");

    check(trace.removeLastPoint(), "Removing closing point succeeds");
    check(!trace.isClosed(), "Removing closing point reopens FeatureTrace");
    check(trace.endPoint().isEqualTo(p2, 0.0), "Reopened FeatureTrace endpoint correct");

    /// 值语义

    MyVoxel::Modeling::FeatureTrace copied(trace);

    bool copyCorrect = copied.pointCount() == trace.pointCount();

    for (std::size_t index = 0; index < trace.pointCount() && copyCorrect; ++index)
    {
        copyCorrect = copied.point(index).isEqualTo(trace.point(index), 0.0);
    }

    check(copyCorrect, "FeatureTrace copy preserves ordered point sequence");

    copied.clear();
    check(copied.isEmpty() && !trace.isEmpty(), "FeatureTrace copy owns independent point container");

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;

    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}