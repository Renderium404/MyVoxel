#ifndef VOXELMESHADAPTER_H
#define VOXELMESHADAPTER_H

#include "MyVoxel/Geometry/Mesh/Mesh.h"

// 定义建模库三角网格与MyVoxel三角网格之间的双向转换接口。
template<typename TriangleMeshType>
class VoxelMeshAdapter
{
public:
    virtual ~VoxelMeshAdapter(){}
    // 将建模库三角网格转换为MyVoxel三角网格。
    virtual MyVoxel::Geometry::Mesh toMyVoxelMesh(const TriangleMeshType& mesh) const = 0;
    // 将MyVoxel三角网格转换为建模库三角网格。
    virtual TriangleMeshType fromMyVoxelMesh(const MyVoxel::Geometry::Mesh& mesh) const = 0;
};

#endif