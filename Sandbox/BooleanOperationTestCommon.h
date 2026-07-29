#ifndef MYVOXEL_SANDBOX_BOOLEANOPERATIONTESTCOMMON_H
#define MYVOXEL_SANDBOX_BOOLEANOPERATIONTESTCOMMON_H

#include <cmath>
#include <cstdint>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelTypes.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"
#include "MyVoxel/Modeling/PrimitiveModeling.h"
#include "MyVoxel/Modeling/ShapeVoxelization.h"

namespace BooleanOperationTest
{

const double Pi = 3.14159265358979323846; // 圆周率，用于构造测试刀具旋转角度。
const double BaseVoxelEdgeLength = 1.0; // 测试网格第0层体素边长。
const MyVoxel::VoxelLevel MaximumLevel = 4; // 测试网格最高细分层级。

// 保存与八叉树压缩方式无关的最高层材料占用签名。
struct OccupancySignature
{
    OccupancySignature()
        : voxelCount(0)
        , xorHash(0)
        , sumHash(0)
    {
    }

    // 检查两个占用签名是否完全相同。
    bool isEqualTo(const OccupancySignature& other) const
    {
        return voxelCount == other.voxelCount && xorHash == other.xorHash && sumHash == other.sumHash;
    }

    std::uint64_t voxelCount; // 展开到最高层后的材料体素数量。
    std::uint64_t xorHash; // 全部最高层材料体素哈希的异或值。
    std::uint64_t sumHash; // 全部最高层材料体素哈希的累加值。
};

// 创建指定索引和层级的体素地址。
inline MyVoxel::VoxelCellAddress makeAddress(MyVoxel::VoxelIndex x, MyVoxel::VoxelIndex y, MyVoxel::VoxelIndex z, MyVoxel::VoxelLevel level)
{
    return MyVoxel::VoxelCellAddress(MyVoxel::VoxelCellIndex(x, y, z), level);
}

// 创建绕Z轴旋转的刚体变换矩阵。
// 创建绕Z轴旋转的刚体变换矩阵。
inline MyMath::Matrix4 makeRotationZ(double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    MyMath::Matrix4 transform = MyMath::Matrix4::identity();

    transform(0, 0) = cosine;
    transform(0, 1) = -sine;
    transform(1, 0) = sine;
    transform(1, 1) = cosine;
    return transform;
}

// 对64位整数执行稳定混合。
inline std::uint64_t mixHash(std::uint64_t value)
{
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
}

// 返回一个最高层体素索引的稳定哈希。
inline std::uint64_t voxelHash(std::int64_t x, std::int64_t y, std::int64_t z)
{
    const std::uint64_t hashX = mixHash(static_cast<std::uint64_t>(x));
    const std::uint64_t hashY = mixHash(static_cast<std::uint64_t>(y) ^ 0x632be59bd9b4e019ULL);
    const std::uint64_t hashZ = mixHash(static_cast<std::uint64_t>(z) ^ 0x8cb92baa3f3d8dd7ULL);
    return mixHash(hashX ^ hashY ^ hashZ);
}

// 返回指定材料叶节点映射到最高层后的单轴体素数量。
inline std::int64_t maximumLevelScale(const MyVoxel::VoxelCellAddress& address, MyVoxel::VoxelLevel maximumLevel)
{
    std::int64_t scale = 1;

    for (MyVoxel::VoxelLevel level = address.level; level < maximumLevel; ++level)
    {
        scale *= 2;
    }

    return scale;
}

// 返回VoxelShape展开到最高层后的材料占用签名。
inline OccupancySignature occupancySignature(const MyVoxel::VoxelShape& shape)
{
    OccupancySignature signature;
    const MyVoxel::VoxelLevel maximumLevel = shape.grid().maximumLevel();

    shape.forest().forEachMaterialCell(
        [&signature, maximumLevel](const MyVoxel::VoxelCellAddress& address)
        {
            const std::int64_t scale = maximumLevelScale(address, maximumLevel);
            const std::int64_t startX = static_cast<std::int64_t>(address.index.x) * scale;
            const std::int64_t startY = static_cast<std::int64_t>(address.index.y) * scale;
            const std::int64_t startZ = static_cast<std::int64_t>(address.index.z) * scale;
            const std::int64_t endX = startX + scale;
            const std::int64_t endY = startY + scale;
            const std::int64_t endZ = startZ + scale;

            for (std::int64_t z = startZ; z < endZ; ++z)
            {
                for (std::int64_t y = startY; y < endY; ++y)
                {
                    for (std::int64_t x = startX; x < endX; ++x)
                    {
                        const std::uint64_t hash = voxelHash(x, y, z);

                        ++signature.voxelCount;
                        signature.xorHash ^= hash;
                        signature.sumHash += hash;
                    }
                }
            }
        });

    return signature;
}

// 创建包含32个完整材料根节点的测试工件。
inline MyVoxel::VoxelShape makeWorkpiece()
{
    const MyVoxel::VoxelGrid grid(MyMath::Vector3(-2.0, -2.0, -1.0), BaseVoxelEdgeLength, MaximumLevel);
    MyVoxel::VoxelShape workpiece(grid);
    MyVoxel::VoxelForest& forest = workpiece.editForest();

    for (MyVoxel::VoxelIndex z = 0; z < 2; ++z)
    {
        for (MyVoxel::VoxelIndex y = 0; y < 4; ++y)
        {
            for (MyVoxel::VoxelIndex x = 0; x < 4; ++x)
            {
                forest.setState(makeAddress(x, y, z, MyVoxel::BaseVoxelLevel), MyVoxel::VoxelState::Material);
            }
        }
    }

    return workpiece;
}

// 创建旋转30度的连续盒体刀具。
inline MyVoxel::Geometry::ShapeInstance makeContinuousTool()
{
    const MyVoxel::Geometry::Shape shape = MyVoxel::Modeling::makeBox(2.4, 1.4, 1.4);
    return MyVoxel::Geometry::ShapeInstance(shape, makeRotationZ(Pi / 6.0));
}

// 将连续刀具离散为带相同姿态的体素刀具。
inline MyVoxel::VoxelShape makeVoxelTool(const MyVoxel::VoxelShape& workpiece)
{
    return MyVoxel::Modeling::voxelize(makeContinuousTool(), workpiece.grid());
}

// 创建完全位于工件外部的连续刀具。
inline MyVoxel::Geometry::ShapeInstance makeOutsideTool()
{
    const MyVoxel::Geometry::Shape shape = MyVoxel::Modeling::makeBox(1.0, 1.0, 1.0);
    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(20.0, 0.0, 0.0));
    return MyVoxel::Geometry::ShapeInstance(shape, transform);
}

}

#endif // MYVOXEL_SANDBOX_BOOLEANOPERATIONTESTCOMMON_H