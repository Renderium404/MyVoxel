#include <cassert>
#include <cmath>
#include <iostream>

#include "MyVoxel/Core/VoxelShape.h"

namespace
{

bool nearValue(float first, float second)
{
    return std::fabs(first - second) <= 1.0e-6f; // 测试仅比较本轮直接写入的简单float常量，固定使用1e-6容差。
}

}

int main()
{
    using namespace MyVoxel;

    const float backgroundDistance = 1.0f; // 本测试固定使用[-1,+1]截断范围验证终止背景、显式叶和裁剪。
    const VoxelLevel maximumLevel = static_cast<VoxelLevel>(3); // MaskLeaf作为第1层子节点时覆盖第2、3层，最高样本层级固定为3。
    const VoxelGrid grid(8.0, maximumLevel);
    const VoxelCellIndex rootIndex(0, 0, 0);
    const VoxelCellAddress rootAddress(rootIndex, BaseVoxelLevel);
    const VoxelCellAddress sampleA(VoxelCellIndex(0, 0, 0), maximumLevel);
    const VoxelCellAddress sampleB(VoxelCellIndex(1, 0, 0), maximumLevel);

    VoxelShape shape(grid, backgroundDistance);
    assert(shape.isValid());
    assert(shape.isEmpty());
    assert(nearValue(shape.backgroundDistance(), backgroundDistance));
    assert(nearValue(shape.distance(sampleA), backgroundDistance));

    {
        VoxelShapeSession session = shape.session(static_cast<VoxelLevel>(1));
        assert(session.setDistance(sampleA, 0.25f));
        assert(session.hasChanges());
        assert(session.changes().containsDirtyRegion(rootIndex));
        assert(nearValue(session.distance(sampleA), 0.25f));
        assert(session.state(sampleA) == VoxelState::Empty);
    }

    assert(shape.rootCount() == 1);
    assert(nearValue(shape.distance(sampleA), 0.25f));

    // 同号距离变化也必须记录FieldChange，不能只依赖材料符号变化。
    {
        VoxelShapeSession session = shape.session(static_cast<VoxelLevel>(1));
        assert(session.setDistance(sampleA, 0.50f));
        assert(session.changes().containsDirtyRegion(rootIndex));
        assert(session.state(sampleA) == VoxelState::Empty);
    }

    // 负距离同步更新MaskBlock材料符号。
    {
        VoxelShapeSession session = shape.session(static_cast<VoxelLevel>(1));
        assert(session.setDistance(sampleB, -0.40f));
        assert(session.state(sampleB) == VoxelState::Material);
        assert(nearValue(session.distance(sampleB), -0.40f));
    }

    // Shape级COW必须同时分离Node和MaskLeaf距离数据。
    VoxelShape copy = shape;
    assert(copy.sharesDataWith(shape));
    {
        VoxelShapeSession session = copy.session(static_cast<VoxelLevel>(1));
        assert(session.setDistance(sampleB, 0.60f));
    }
    assert(!copy.sharesDataWith(shape));
    assert(nearValue(shape.distance(sampleB), -0.40f));
    assert(nearValue(copy.distance(sampleB), 0.60f));

    // 将显式样本恢复到+B后，prune只改变结构并最终删除Empty根。
    {
        VoxelShapeSession session = copy.session(static_cast<VoxelLevel>(1));
        assert(session.setDistance(sampleA, backgroundDistance));
        assert(session.setDistance(sampleB, backgroundDistance));
        const VoxelChangeSet fieldChanges = session.takeChanges();
        assert(fieldChanges.containsDirtyRegion(rootIndex));
        assert(session.pruneTree(rootIndex));
        assert(session.changes().containsModifiedRoot(rootIndex));
        assert(!session.changes().containsDirtyRegion(rootIndex));
    }
    assert(copy.isEmpty());
    assert(nearValue(copy.distance(sampleA), backgroundDistance));

    // Material终止根表示-B，写入有限负距离时按需显式化MaskLeaf。
    {
        VoxelShapeSession session = copy.session(static_cast<VoxelLevel>(1));
        assert(session.setState(rootAddress, VoxelState::Material));
        assert(nearValue(session.distance(sampleA), -backgroundDistance));
        session.takeChanges();
        assert(session.setDistance(sampleA, -0.20f));
        assert(nearValue(session.distance(sampleA), -0.20f));
        assert(session.state(sampleA) == VoxelState::Material);
    }

    assert(copy.isValid());
    std::cout << "TSDF tree stage2 test passed.\n";
    return 0;
}
