#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/Storage/VolumeBlock.h"
#include "MyVoxel/Core/Volume/VolumeField.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Volume/VolumeFieldBuilder.h"

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

}

int main()
{
    using namespace MyVoxel;

    std::cout << "开始VoxelShape全量VolumeField构建测试" << std::endl << std::endl;

    VolumeFieldBuildOptions options;
    options.exteriorBandWidth = 2.0;
    options.interiorBandWidth = 2.0;

    VoxelShape emptyShape(4.0, 2);
    VolumeFieldBuilder::build(emptyShape, options);
    const VolumeField* emptyField = emptyShape.volumeField();
    check(emptyField && emptyField->isCurrent() && emptyField->isEmpty(), "空Shape构建为空且有效的距离场");
    check(nearValue(emptyField->interiorBackgroundDistance(), -2.0f) &&
          nearValue(emptyField->exteriorBackgroundDistance(), 2.0f), "空Shape背景距离正确");

    VoxelShape shape(4.0, 2);
    const VoxelCellAddress materialAddress(VoxelCellIndex(0, 0, 0), 2);
    {
        VoxelShapeSession session = shape.session(2);
        check(session.setState(materialAddress, VoxelState::Material), "创建单个最高层材料体素");
    }

    VolumeFieldBuildStatistics statistics;
    VolumeFieldBuilder::build(shape, options, &statistics);
    const VolumeField* field = shape.volumeField();
    check(field && field->isCurrent() && field->isValid(), "单体素距离场构建后有效");
    check(field->blockCount() > 0 && statistics.allocatedBlockCount == field->blockCount(), "构建生成稀疏距离块");
    check(statistics.surfaceTriangleCount == 12, "单体素边界网格包含十二个三角形");

    const VoxelCellAddress blockAddress = field->blockAddress(materialAddress);
    const VolumeBlock* block = field->findBlock(blockAddress);
    check(block != nullptr, "材料体素所属距离块存在");

    const unsigned int materialSample = field->sampleIndex(materialAddress);
    const VoxelCellAddress outsideAddress(VoxelCellIndex(1, 0, 0), 2);
    const unsigned int outsideSample = field->sampleIndex(outsideAddress);
    check(nearValue(volumeDistance(*block, materialSample), -0.5f), "材料体素中心距离为负半个体素");
    check(nearValue(volumeDistance(*block, outsideSample), 0.5f), "相邻空体素中心距离为正半个体素");

    const std::size_t blockCountBeforeTransform = field->blockCount();
    shape.setTransform(MyMath::Matrix4::fromTranslation(MyMath::Vector3(7.0, -3.0, 2.0)));
    check(shape.volumeField()->isCurrent() && shape.volumeField()->blockCount() == blockCountBeforeTransform,
          "刚体变换不影响已经构建的局部距离场");

    VoxelShape copy = shape;
    VolumeFieldBuilder::build(shape, options);
    check(!shape.sharesDataWith(copy), "共享Shape重建距离场时执行Shape级COW");
    check(shape.volumeField()->isCurrent(), "当前Shape重建后距离场有效");
    check(copy.volumeField()->isCurrent(), "原共享Shape保留原有效距离场");

    {
        VoxelShapeSession session = shape.session(2);
        check(session.setState(materialAddress, VoxelState::Empty), "删除单个材料体素");
    }
    check(shape.volumeField()->isCompletelyDirty(), "材料修改后距离场立即失效");
    VolumeFieldBuilder::build(shape, options);
    check(shape.volumeField()->isCurrent() && shape.volumeField()->isEmpty(), "删除材料后全量重建为空距离场");

    std::cout << std::endl << "Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}