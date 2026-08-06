#include <cstdlib>
#include <iostream>
#include <string>

#include "MyVoxel/Core/Volume/VolumeField.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelShapeSession.h"

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

// 将支持距离场的Shape距离资源确认到当前状态。
void markVolumeFieldCurrent(MyVoxel::VoxelShape& shape)
{
    MyVoxel::VolumeField* field = shape.editVolumeField();

    if (field)
    {
        field->markAllValid();
    }
}

}

int main()
{
    using namespace MyVoxel;

    std::cout << "开始VoxelShape距离场失效时机测试" << std::endl << std::endl;

    VoxelShape lowLevelShape(1.0, 1);
    check(!lowLevelShape.supportsVolumeField() && lowLevelShape.volumeField() == nullptr,
          "低层级Shape不创建距离场");

    VoxelShape shape(1.0, 4);
    check(shape.supportsVolumeField() && shape.volumeField() != nullptr,
          "高层级Shape持有距离场");
    check(shape.volumeField()->isCompletelyDirty(), "初始距离场完全失效");

    markVolumeFieldCurrent(shape);
    check(shape.volumeField()->isCurrent(), "完整构建后距离场有效");

    shape.setTransform(MyMath::Matrix4::fromTranslation(MyMath::Vector3(1.0, 2.0, 3.0)));
    check(shape.volumeField()->isCurrent(), "刚体变换不使距离场失效");

    const VoxelCellAddress rootAddress(VoxelCellIndex(0, 0, 0), BaseVoxelLevel);

    {
        VoxelShapeSession session = shape.session(2);
        check(session.setState(rootAddress, VoxelState::Material), "材料写入实际发生变化");
        check(shape.volumeField()->isCompletelyDirty(), "材料变化立即使距离场失效");
    }

    markVolumeFieldCurrent(shape);

    {
        VoxelShapeSession session = shape.session(2);
        check(!session.setState(rootAddress, VoxelState::Material), "重复材料写入不产生变化");
        check(shape.volumeField()->isCurrent(), "无效材料写入不使距离场失效");
    }

    {
        VoxelShapeSession session = shape.session(2);
        check(session.split(rootAddress), "材料节点结构细分成功");
        check(shape.volumeField()->isCurrent(), "纯split不使距离场失效");
        check(session.merge(rootAddress), "一致子节点结构合并成功");
        check(shape.volumeField()->isCurrent(), "纯merge不使距离场失效");
    }

    {
        VoxelShapeSession session = shape.session(2);
        const VoxelChangeSet changes = session.takeChanges();
        check(!changes.hasChanges() && shape.volumeField()->isCurrent(),
              "取走空变化记录不影响距离场");
    }

    {
        VoxelShapeSession session = shape.session(2);
        check(session.clear(), "清空材料成功");
        check(shape.volumeField()->isCompletelyDirty(), "清空材料立即使距离场失效");
    }

    VoxelShape sharedSource(1.0, 4);
    markVolumeFieldCurrent(sharedSource);
    VoxelShape sharedCopy = sharedSource;
    check(sharedSource.sharesDataWith(sharedCopy), "复制Shape共享材料和距离资源");

    {
        VoxelShapeSession session = sharedCopy.session(2);
        check(!sharedCopy.sharesDataWith(sharedSource), "创建Session触发Shape级COW");
        check(sharedSource.volumeField()->isCurrent(), "原Shape保留有效距离场");
        check(sharedCopy.volumeField()->isCompletelyDirty(), "当前保守COW策略使新副本距离场完全失效");
    }

    std::cout << std::endl << "Passed " << g_passed << " / Failed " << g_failed << std::endl;
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}