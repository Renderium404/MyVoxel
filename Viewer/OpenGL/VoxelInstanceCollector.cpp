#include "VoxelInstanceCollector.h"

#include "MyMath/Matrix4.h"

namespace
{

// 创建指定体素在形体局部坐标中的模型矩阵。
MyMath::Matrix4 createCellTransform(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellAddress& address)
{
    const double edgeLength = shape.voxelEdgeLength(address.level);
    const double centerOffset = edgeLength * 0.5; // 体素中心距离最小角为半个体素边长。
    const double centerX = static_cast<double>(address.index.x) * edgeLength + centerOffset;
    const double centerY = static_cast<double>(address.index.y) * edgeLength + centerOffset;
    const double centerZ = static_cast<double>(address.index.z) * edgeLength + centerOffset;

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 0) = edgeLength;
    transform(1, 1) = edgeLength;
    transform(2, 2) = edgeLength;
    transform(0, 3) = centerX;
    transform(1, 3) = centerY;
    transform(2, 3) = centerZ;
    return transform;
}

// 创建指定材料体素的最终显示实例。
MyVoxelViewer::VoxelDisplayInstance createDisplayInstance(const MyVoxel::VoxelShape& shape, const MyVoxel::VoxelCellAddress& address, const MyVoxelViewer::DisplayColor& color)
{
    const MyMath::Matrix4 cellTransform = createCellTransform(shape, address);
    return MyVoxelViewer::VoxelDisplayInstance(shape.transform() * cellTransform, color);
}

}

namespace MyVoxelViewer
{

void VoxelInstanceCollector::append(std::vector<VoxelDisplayInstance>& instances, const MyVoxel::VoxelShape& shape, const DisplayColor& color)
{
    shape.forest().forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            instances.push_back(createDisplayInstance(shape, address, color));
        });
}

std::vector<VoxelDisplayInstance> VoxelInstanceCollector::collect(const MyVoxel::VoxelShape& shape, const DisplayColor& color)
{
    std::vector<VoxelDisplayInstance> instances;
    append(instances, shape, color);
    return instances;
}

}