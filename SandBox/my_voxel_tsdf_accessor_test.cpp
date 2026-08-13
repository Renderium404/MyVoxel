#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Meshing/VoxelTsdfAccessor.h"
#include "MyVoxel/Modeling/Shape/PrimitiveModeling.h"
#include "MyVoxel/Modeling/Shape/ShapeVoxelization.h"

namespace
{

const double ValueTolerance = 1.0e-6; // TSDF内部保存float，距离读取比较使用1e-6绝对容差。
const double PositionTolerance = 1.0e-12; // VoxelGrid中心位置由同一路径计算，使用严格双精度容差。
std::size_t g_passedCount = 0;
std::size_t g_failedCount = 0;

// 输出单项测试结果并累计统计。
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

// 判断两个标量是否在指定容差内一致。
bool nearValue(double first, double second, double tolerance = ValueTolerance)
{
    return std::fabs(first - second) <= tolerance;
}

// 判断两个三维向量是否在指定容差内一致。
bool nearVector(const MyMath::Vector3& first, const MyMath::Vector3& second, double tolerance)
{
    return first.isEqualTo(second, tolerance);
}

/// 基础绑定与直接读取

void testBasicAccess()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4)); // 最高层采样间距固定为0.25。
    const float backgroundDistance = 1.0f; // 使用四个最高层采样间距作为窄带，保证梯度测试具有完整邻域。
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    const MyVoxel::Shape sphere = MyVoxel::Modeling::makeSphere(1.5);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(sphere, reference);
    const MyVoxel::Meshing::VoxelTsdfAccessor accessor(voxels);

    check(accessor.isValid(), "VoxelTsdfAccessor binds valid TSDF shape");
    check(accessor.sampleLevel() == grid.maximumLevel(), "VoxelTsdfAccessor sample level equals grid maximum level");
    check(nearValue(accessor.sampleSpacing(), grid.minimumCellEdgeLength()), "VoxelTsdfAccessor sample spacing equals highest-level edge length");
    check(nearValue(accessor.backgroundDistance(), backgroundDistance), "VoxelTsdfAccessor exposes shape background distance");

    const MyVoxel::VoxelCellIndex indices[] =
    {
        MyVoxel::VoxelCellIndex(16, 16, 16),
        MyVoxel::VoxelCellIndex(21, 16, 16),
        MyVoxel::VoxelCellIndex(24, 16, 16),
        MyVoxel::VoxelCellIndex(40, -12, 7)
    };

    bool valuesMatch = true;
    bool positionsMatch = true;

    for (unsigned int index = 0; index < sizeof(indices) / sizeof(indices[0]); ++index)
    {
        const MyVoxel::VoxelCellAddress address(indices[index], grid.maximumLevel());
        valuesMatch = valuesMatch && nearValue(accessor.value(indices[index]), voxels.distance(address));
        positionsMatch = positionsMatch && nearVector(accessor.samplePosition(indices[index]), grid.cellCenter(address), PositionTolerance);
    }

    check(valuesMatch, "VoxelTsdfAccessor values exactly follow VoxelShape TSDF");
    check(positionsMatch, "VoxelTsdfAccessor sample positions equal highest-level cell centers");
    check(!accessor.supportsSampleAddress(MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(0, 0, 0), static_cast<MyVoxel::VoxelLevel>(3))),
          "VoxelTsdfAccessor rejects non-highest sample level");
}

/// Box中心差分尺度

void testBoxGradientScale()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4)); // h=0.25。
    const float backgroundDistance = 1.0f;
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    const MyVoxel::Shape box = MyVoxel::Modeling::makeBox(2.0, 2.0, 2.0);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(box, reference);
    const MyVoxel::Meshing::VoxelTsdfAccessor accessor(voxels);

    const MyVoxel::VoxelCellIndex sample = grid.cellIndex(MyMath::Vector3(0.625, 0.125, 0.125), grid.maximumLevel());
    const MyMath::Vector3 position = accessor.samplePosition(sample);
    const MyMath::Vector3 gradient = accessor.gradient(sample);

    check(position.x() > 0.0 && std::fabs(position.y()) < 0.3 && std::fabs(position.z()) < 0.3, "Box gradient sample lies nearest +X face");
    check(nearValue(gradient.x(), 1.0, 1.0e-5) && nearValue(gradient.y(), 0.0, 1.0e-5) && nearValue(gradient.z(), 0.0, 1.0e-5),
          "VoxelTsdfAccessor center difference includes physical 1/(2h) scale");
}

/// Sphere梯度方向

void testSphereGradientDirection()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(5)); // h=0.125，提高曲面梯度方向验证精度。
    const float backgroundDistance = 0.75f;
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    const MyVoxel::Shape sphere = MyVoxel::Modeling::makeSphere(1.5);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(sphere, reference);
    const MyVoxel::Meshing::VoxelTsdfAccessor accessor(voxels);

    const MyVoxel::VoxelCellIndex sample = grid.cellIndex(MyMath::Vector3(1.1, 0.7, 0.4), grid.maximumLevel());
    const MyMath::Vector3 position = accessor.samplePosition(sample);
    MyMath::Vector3 expected = position;
    MyMath::Vector3 actual = accessor.gradient(sample);
    const bool expectedNormalized = expected.normalize();
    const bool actualNormalized = actual.normalize();
    const double alignment = expectedNormalized && actualNormalized ? MyMath::Vector3::dot(expected, actual) : -1.0;

    check(expectedNormalized && actualNormalized, "Sphere surface-band gradient is non-zero");
    check(alignment > 0.995, "Sphere TSDF gradient points along analytic outward normal");
}

/// 饱和区域梯度

void testSaturatedGradient()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-4.0, -4.0, -4.0), 4.0, static_cast<MyVoxel::VoxelLevel>(4));
    const float backgroundDistance = 0.5f;
    const MyVoxel::VoxelShape reference(grid, backgroundDistance);
    const MyVoxel::Shape sphere = MyVoxel::Modeling::makeSphere(1.0);
    const MyVoxel::VoxelShape voxels = MyVoxel::Modeling::voxelizeAligned(sphere, reference);
    const MyVoxel::Meshing::VoxelTsdfAccessor accessor(voxels);

    const MyVoxel::VoxelCellIndex farOutside = grid.cellIndex(MyMath::Vector3(3.0, 3.0, 3.0), grid.maximumLevel());
    const MyMath::Vector3 gradient = accessor.gradient(farOutside);

    check(nearValue(accessor.value(farOutside), backgroundDistance), "Far exterior TSDF sample resolves to +B");
    check(nearVector(gradient, MyMath::Vector3(0.0, 0.0, 0.0), ValueTolerance), "Constant +B region has zero center-difference gradient");
}

}

int main()
{
    std::cout << "MyVoxel direct TSDF accessor regression test" << std::endl << std::endl;

    testBasicAccess();
    testBoxGradientScale();
    testSphereGradientDirection();
    testSaturatedGradient();

    std::cout << std::endl;
    std::cout << "Passed: " << g_passedCount << std::endl;
    std::cout << "Failed: " << g_failedCount << std::endl;
    return g_failedCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}