#ifndef VOXELMESHDISPLAYADAPTER_H
#define VOXELMESHDISPLAYADAPTER_H

#include <cstdint>

#include "MyMath/Matrix4.h"

typedef std::uint64_t VoxelMeshPartId;

// 定义建模库对体素表面分片的增量显示接口。
template<typename TriangleMeshType>
class VoxelMeshDisplayAdapter
{
public:
    virtual ~VoxelMeshDisplayAdapter()
    {

    }
    // 添加新分片或替换已有分片，但不立即刷新画面。
    virtual void setMeshPart(VoxelMeshPartId partId,const TriangleMeshType& mesh) = 0;
    
    // 删除指定分片，但不立即刷新画面。
    virtual void removeMeshPart(VoxelMeshPartId partId) = 0;

    // 设置整个体素对象的模型矩阵，不重新上传网格。
    virtual void setModelMatrix(const MyMath::Matrix4& matrix) = 0;

    // 一次性提交累计变化并刷新画面。
    virtual void commit() = 0;
};

#endif