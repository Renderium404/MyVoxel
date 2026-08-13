#ifndef MYVOXEL_CORE_STORAGE_BLOCKPOOL_H
#define MYVOXEL_CORE_STORAGE_BLOCKPOOL_H

#include <cassert>
#include <cstddef>

#include "LeafPool.h"
#include "NodePool.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"

namespace MyVoxel
{

// 保存一个MaskLeaf对应的可写材料掩码和距离块引用。
struct LeafData
{
    MaskBlock* mask;  // 当前MaskLeaf的材料二值掩码。
    LeafBlock* block; // 当前MaskLeaf的64个有符号距离样本。
};

// 保存一个MaskLeaf对应的只读材料掩码和距离块引用。
struct ConstLeafData
{
    const MaskBlock* mask;  // 当前MaskLeaf的只读材料二值掩码。
    const LeafBlock* block; // 当前MaskLeaf的只读有符号距离样本。
};

// 统一管理一棵VoxelTree的全部节点和MaskLeaf物理存储。
//
// NodePool和LeafPool不向上层暴露。
// Empty和Material状态槽通过value()/setValue()访问非分化节点数值。
// Node状态槽通过node()访问NodeBlock。
// MaskLeaf状态槽通过leaf()自动读取IndexBlock::leafIndex并定位对应MaskBlock和LeafBlock。
class BlockPool : public Foundation::ReferenceCounted
{
public:
    // 构造空统一块池。
    BlockPool();
    // 禁止复制统一块池，VoxelTree通过引用计数共享并在写入时执行显式深复制。
    BlockPool(const BlockPool& other) = delete;
    // 禁止复制赋值统一块池。
    BlockPool& operator=(const BlockPool& other) = delete;
    // 释放全部节点和叶数据存储。
    ~BlockPool() override;

    /// 节点组分配与回收

    // 分配一个八槽节点组，并将八个非分化物理槽统一初始化为指定数值。
    VoxelIndex allocateGroup(float value = 0.0f);
    // 将已经分配的八槽节点组统一重新初始化为指定非分化数值。
    void initGroup(VoxelIndex firstSlotIndex, float value);
    // 回收一个八槽节点组，不递归释放其中记录的后代。
    void releaseGroup(VoxelIndex firstSlotIndex);

    /// 非分化节点

    // 将指定物理槽完整初始化为保存指定数值的非分化NodeBlock。
    void initializeValue(VoxelIndex index, float value);
    // 返回指定非分化物理槽保存的数值，调用者必须保证父节点将该槽记录为Empty或Material。
    inline float value(VoxelIndex index) const;
    // 修改指定非分化物理槽保存的数值，调用者必须保证父节点将该槽记录为Empty或Material。
    inline void setValue(VoxelIndex index, float value);

    /// 普通节点初始化与访问

    // 将指定物理槽初始化为普通分化NodeBlock，尚不分配该节点自己的八槽子表。
    NodeBlock& initializeNode(VoxelIndex index, VoxelNodeState childState = VoxelNodeState::Empty);
    // 返回指定物理槽中的可写NodeBlock，调用者必须保证父节点将该槽记录为VoxelNodeState::Node。
    inline NodeBlock& node(VoxelIndex index);
    // 返回指定物理槽中的只读NodeBlock，调用者必须保证父节点将该槽记录为VoxelNodeState::Node。
    inline const NodeBlock& node(VoxelIndex index) const;

    /// MaskLeaf初始化与释放

    // 为指定物理槽分配统一叶数据并初始化IndexBlock，返回创建后的可写MaskLeaf数据。
    LeafData initializeLeaf(VoxelIndex index, float distance = 0.0f);
    // 将另一个BlockPool中的指定MaskLeaf完整复制到当前指定物理槽。
    LeafData copyLeaf(VoxelIndex destinationIndex, const BlockPool& sourcePool, VoxelIndex sourceIndex);
    // 释放指定MaskLeaf槽引用的统一叶数据，并将IndexBlock恢复为无效索引。
    void releaseLeaf(VoxelIndex index);

    /// MaskLeaf访问

    // 返回指定物理槽引用的可写MaskBlock和LeafBlock，调用者必须保证父节点将该槽记录为MaskLeaf。
    inline LeafData leaf(VoxelIndex index);
    // 返回指定物理槽引用的只读MaskBlock和LeafBlock，调用者必须保证父节点将该槽记录为MaskLeaf。
    inline ConstLeafData leaf(VoxelIndex index) const;

    /// 存储检查

    // 检查指定节点组当前是否有效。
    bool containsGroup(VoxelIndex firstSlotIndex) const;
    // 检查指定VoxelBlock物理槽当前是否有效。
    bool containsNodeSlot(VoxelIndex index) const;
    // 检查指定MaskLeaf物理槽记录的leafIndex是否引用有效叶数据。
    bool containsLeaf(VoxelIndex index) const;

    /// 资源管理

    // 回卷节点池和叶池全部分配状态并保留Chunk。
    void rewind();
    // 清空全部节点和叶数据Chunk。
    void clear();

    /// 节点存储统计

    std::size_t allocatedGroupCount() const;
    std::size_t highWaterGroupCount() const;
    std::size_t nodeChunkCount() const;
    std::size_t nodeStorageCapacityBytes() const;

    /// 叶存储统计

    std::size_t allocatedLeafCount() const;
    std::size_t highWaterLeafCount() const;
    std::size_t leafChunkCount() const;
    std::size_t leafStorageCapacityBytes() const;

    /// 总存储统计

    // 返回节点和叶数据Chunk的总存储容量。
    std::size_t storageCapacityBytes() const;

private:
    // 返回指定MaskLeaf槽保存的IndexBlock。
    inline IndexBlock& leafIndexBlock(VoxelIndex index);
    inline const IndexBlock& leafIndexBlock(VoxelIndex index) const;

private:
    NodePool m_nodePool; // 管理8字节VoxelBlock节点槽和64字节八槽节点组。
    LeafPool m_leafPool; // 管理同一leafIndex对应的MaskBlock和LeafBlock。
};

inline float BlockPool::value(VoxelIndex index) const
{
    return nodeValue(m_nodePool.node(index));
}

inline void BlockPool::setValue(VoxelIndex index, float value)
{
    setNodeValue(m_nodePool.node(index), value);
}

inline NodeBlock& BlockPool::node(VoxelIndex index)
{
    return m_nodePool.node(index);
}

inline const NodeBlock& BlockPool::node(VoxelIndex index) const
{
    return m_nodePool.node(index);
}

inline LeafData BlockPool::leaf(VoxelIndex index)
{
    IndexBlock& indexBlock = leafIndexBlock(index);
    assert(indexBlock.reserved == 0);
    assert(m_leafPool.containsBlock(indexBlock.leafIndex));

    LeafData result;
    result.mask = &m_leafPool.mask(indexBlock.leafIndex);
    result.block = &m_leafPool.leaf(indexBlock.leafIndex);
    return result;
}

inline ConstLeafData BlockPool::leaf(VoxelIndex index) const
{
    const IndexBlock& indexBlock = leafIndexBlock(index);
    assert(indexBlock.reserved == 0);
    assert(m_leafPool.containsBlock(indexBlock.leafIndex));

    ConstLeafData result;
    result.mask = &m_leafPool.mask(indexBlock.leafIndex);
    result.block = &m_leafPool.leaf(indexBlock.leafIndex);
    return result;
}

inline IndexBlock& BlockPool::leafIndexBlock(VoxelIndex index)
{
    return m_nodePool.index(index);
}

inline const IndexBlock& BlockPool::leafIndexBlock(VoxelIndex index) const
{
    return m_nodePool.index(index);
}

}

#endif // MYVOXEL_CORE_STORAGE_BLOCKPOOL_H