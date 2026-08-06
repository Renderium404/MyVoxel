#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Volume/VolumeField.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Volume/VolumeFieldBuilder.h"
#include "MyVoxel/Volume/VolumeFieldView.h"

namespace
{

int g_passed = 0;
int g_failed = 0;

// 记录一项测试结果。
void check(bool condition, const std::string& name)
{
    if (condition)
    {
        ++g_passed;
        std::cout << "[通过] " << name << std::endl;
    }
    else
    {
        ++g_failed;
        std::cout << "[失败] " << name << std::endl;
    }
}

// 判断单精度距离是否近似相等。
bool nearValue(float first, float second, float epsilon = 1.0e-5f)
{
    return std::fabs(static_cast<double>(first - second)) <= static_cast<double>(epsilon);
}

// 判断三维位置是否近似相等。
bool nearPoint(const MyMath::Vector3& first, const MyMath::Vector3& second, double epsilon = 1.0e-9)
{
    return first.isEqualTo(second, epsilon);
}

}

int main()
{
    using namespace MyVoxel;

    std::cout << "开始VolumeFieldView统一距离采样测试" << std::endl << std::endl;

    VolumeFieldBuildOptions options;
    options.exteriorBandWidth = 2.0;
    options.interiorBandWidth = 2.0;

    VoxelShape emptyShape(4.0, 2);
    VolumeFieldBuilder::build(emptyShape, options);
    VolumeFieldView emptyView(emptyShape);
    const VoxelCellAddress emptyAddress(VoxelCellIndex(20, -7, 3), 2);
    check(emptyView.isValid() && emptyView.isCurrent(), "空Shape距离视图有效且同步");
    check(!emptyView.hasStoredSample(emptyAddress), "空Shape远处样本不分配距离块");
    check(emptyView.isSampleReadable(emptyAddress), "空Shape窄带外样本允许读取");
    check(nearValue(emptyView.value(emptyAddress), 2.0f), "空Shape缺失块返回外部背景距离");
    check(nearPoint(emptyView.samplePosition(emptyAddress), emptyShape.grid().cellCenter(emptyAddress)), "距离视图采样位置与VoxelGrid一致");

    VoxelShape singleShape(4.0, 2);
    const VoxelCellAddress materialAddress(VoxelCellIndex(0, 0, 0), 2);
    const VoxelCellAddress adjacentAddress(VoxelCellIndex(1, 0, 0), 2);
    {
        VoxelShapeSession session = singleShape.session(2);
        check(session.setState(materialAddress, VoxelState::Material), "创建单个最高层材料体素");
    }
    VolumeFieldBuilder::build(singleShape, options);
    VolumeFieldView singleView(singleShape);
    check(singleView.hasStoredSample(materialAddress), "材料表面附近样本由VolumeBlock保存");
    check(nearValue(singleView.value(materialAddress), -0.5f), "视图读取材料体素中心负距离");
    check(nearValue(singleView.value(adjacentAddress), 0.5f), "视图读取相邻空体素中心正距离");
    check(nearValue(singleView.value(materialAddress.index), -0.5f), "视图支持按最高层索引读取距离");

    VoxelShape solidShape(16.0, 4);
    const VoxelCellAddress solidRoot(VoxelCellIndex(0, 0, 0), 0);
    {
        VoxelShapeSession session = solidShape.session(4);
        check(session.setState(solidRoot, VoxelState::Material), "创建完整第0层材料根体素");
    }
    VolumeFieldBuilder::build(solidShape, options);
    VolumeFieldView solidView(solidShape);
    const VoxelCellAddress deepInteriorAddress(VoxelCellIndex(7, 7, 7), 4);
    const VoxelCellAddress farExteriorAddress(VoxelCellIndex(40, 40, 40), 4);
    check(!solidView.hasStoredSample(deepInteriorAddress), "深层材料样本位于稀疏窄带外");
    check(nearValue(solidView.value(deepInteriorAddress), -2.0f), "缺失材料块返回内部背景距离");
    check(!solidView.hasStoredSample(farExteriorAddress), "远离材料的空样本不分配距离块");
    check(nearValue(solidView.value(farExteriorAddress), 2.0f), "缺失空块返回外部背景距离");

    const MyMath::Vector3 positionBeforeTransform = solidView.samplePosition(deepInteriorAddress);
    solidShape.setTransform(MyMath::Matrix4::fromTranslation(MyMath::Vector3(5.0, -2.0, 8.0)));
    check(solidView.isCurrent(), "刚体变换不影响距离视图同步状态");
    check(nearPoint(solidView.samplePosition(deepInteriorAddress), positionBeforeTransform), "距离视图始终返回Shape局部采样位置");

    {
        VoxelShapeSession session = singleShape.session(2);
        check(session.setState(materialAddress, VoxelState::Empty), "修改材料使既有距离场失效");
    }
    check(!singleView.isCurrent(), "材料修改后既有视图识别距离场失效");
    check(!singleView.isSampleReadable(materialAddress), "完全失效状态禁止读取任意距离样本");

    std::cout << std::endl << "Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}