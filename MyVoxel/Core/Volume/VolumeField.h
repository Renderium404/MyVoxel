#ifndef MYVOXEL_CORE_VOLUME_VOLUMEFIELD_H
#define MYVOXEL_CORE_VOLUME_VOLUMEFIELD_H

#include <cstddef>
#include <map>
#include <set>

#include "MyVoxel/Core/Storage/VolumeBlockPool.h"
#include "MyVoxel/Core/VoxelAddress.h"

namespace MyVoxel
{

// 以逻辑掩码叶区为单位稀疏保存最高层体素中心的有符号距离。
//
// 每个VolumeBlock对应blockLevel层的一个逻辑体素，并保存其两级后代中
// 4×4×4个sampleLevel层体素中心的64个距离样本。VolumeField只管理距离
// 块、地址映射和有效状态，不负责从材料体素生成距离或执行网格化。
class VolumeField
{
public:
    using BlockIndexMap = std::map<VoxelCellIndex, VoxelIndex>;
    using DirtyBlockSet = std::set<VoxelCellIndex>;

public:
    // 使用指定最高采样层级创建空距离场，采样层级必须至少为第2层。
    explicit VolumeField(VoxelLevel sampleLevel);

    // 距离场持有独立块池，禁止直接复制。
    VolumeField(const VolumeField& other) = delete;

    // 距离场持有独立块池，禁止直接复制赋值。
    VolumeField& operator=(const VolumeField& other) = delete;

    ~VolumeField() = default;

    /// 状态判断

    // 检查层级关系、块映射、块池分配状态和距离样本是否一致。
    bool isValid() const;

    // 判断当前距离场是否没有任何已分配距离块。
    bool isEmpty() const;

    // 判断当前距离场是否不存在完全失效或局部失效区域。
    bool isCurrent() const;

    // 判断当前距离场是否整体失效，必须执行一次完整重建。
    bool isCompletelyDirty() const;

    // 判断指定逻辑叶区是否处于失效状态。
    bool isBlockDirty(const VoxelCellAddress& blockAddress) const;

    /// 距离范围

    // 返回窄带之外材料内部使用的固定负距离。
    float interiorBackgroundDistance() const;

    // 返回窄带之外材料外部使用的固定正距离。
    float exteriorBackgroundDistance() const;

    // 设置窄带之外使用的内部负距离和外部正距离，并将整个距离场标记为失效。
    void setBackgroundDistances(float interiorDistance, float exteriorDistance);

    /// 距离场层级

    // 返回距离样本所在的最高体素层级。
    VoxelLevel sampleLevel() const;

    // 返回一个VolumeBlock对应的逻辑叶区层级，固定比sampleLevel低两层。
    VoxelLevel blockLevel() const;

    // 判断指定地址是否是当前距离场支持的逻辑叶区地址。
    bool supportsBlockAddress(const VoxelCellAddress& blockAddress) const;

    // 判断指定地址是否是当前距离场支持的距离样本地址。
    bool supportsSampleAddress(const VoxelCellAddress& sampleAddress) const;

    /// 地址转换

    // 返回指定最高层采样地址所属的逻辑叶区地址。
    VoxelCellAddress blockAddress(const VoxelCellAddress& sampleAddress) const;

    // 返回指定最高层采样地址在所属VolumeBlock中的样本索引。
    unsigned int sampleIndex(const VoxelCellAddress& sampleAddress) const;

    // 返回指定逻辑叶区内样本索引对应的最高层采样地址。
    VoxelCellAddress sampleAddress(const VoxelCellAddress& blockAddress, unsigned int sampleIndex) const;

    // 返回指定逻辑叶区内局部坐标对应的最高层采样地址，局部坐标范围均为[0, 3]。
    VoxelCellAddress sampleAddress(const VoxelCellAddress& blockAddress, unsigned int x, unsigned int y, unsigned int z) const;

    /// 距离块访问

    // 判断指定逻辑叶区是否持有实际距离块，不考虑块是否失效。
    bool containsBlock(const VoxelCellAddress& blockAddress) const;

    // 返回指定逻辑叶区的只读距离块，不存在时返回空指针。
    const VolumeBlock* findBlock(const VoxelCellAddress& blockAddress) const;

    // 返回指定逻辑叶区的可写距离块，不存在时返回空指针；成功取得后自动标记该块失效。
    VolumeBlock* editBlock(const VoxelCellAddress& blockAddress);

    // 返回指定逻辑叶区的可写距离块，不存在时分配并统一初始化；返回前自动标记该块失效。
    VolumeBlock& ensureBlock(const VoxelCellAddress& blockAddress, float initialDistance = 0.0f);

    // 删除指定逻辑叶区的距离块并清除该地址的局部失效记录，返回是否实际删除。
    bool eraseBlock(const VoxelCellAddress& blockAddress);

    /// 有效状态管理

    // 将指定逻辑叶区标记为失效；完全失效状态下无需重复记录。
    void markBlockDirty(const VoxelCellAddress& blockAddress);

    // 将指定逻辑叶区标记为已经更新；完全失效状态下禁止局部确认。
    void markBlockValid(const VoxelCellAddress& blockAddress);

    // 将整个距离场标记为失效并清空局部失效集合，不立即释放距离块。
    void markAllDirty();

    // 确认一次完整重建已经结束，将全部距离块和缺失块状态视为有效。
    void markAllValid();

    /// 资源管理

    // 清空全部逻辑距离块并回卷块池，保留已经分配的Chunk，结果保持完全失效。
    void resetPreservingStorage();

    // 清空全部逻辑距离块并释放块池存储，结果保持完全失效。
    void releaseStorage();

    // 与采样层级相同的另一个距离场交换全部距离块、背景距离和有效状态。
    void swap(VolumeField& other);

    /// 存储统计与遍历

    // 返回当前稀疏距离块数量。
    std::size_t blockCount() const;

    // 返回当前显式记录的局部失效块数量，整体失效时该值为零。
    std::size_t dirtyBlockCount() const;

    // 返回当前距离块实际占用的样本存储字节数，不包含索引容器开销。
    std::size_t allocatedSampleBytes() const;

    // 返回块池全部Chunk的存储容量，单位为字节。
    std::size_t storageCapacityBytes() const;

    // 返回逻辑叶区索引到物理距离块索引的只读映射。
    const BlockIndexMap& blockIndices() const;

    // 返回局部失效逻辑叶区集合；整体失效时该集合为空。
    const DirtyBlockSet& dirtyBlocks() const;

private:
    // 返回指定逻辑叶区的物理块映射迭代器。
    BlockIndexMap::iterator findBlockIterator(const VoxelCellAddress& blockAddress);
    BlockIndexMap::const_iterator findBlockIterator(const VoxelCellAddress& blockAddress) const;

private:
    VoxelLevel m_sampleLevel; // 距离样本所在的最高体素层级。
    VoxelLevel m_blockLevel; // 逻辑叶区所在层级，固定为sampleLevel-2。
    VolumeBlockPool m_blockPool; // 保存全部实际分配距离块的独立物理块池。
    BlockIndexMap m_blockIndices; // 逻辑叶区索引到物理距离块索引的稀疏映射。
    DirtyBlockSet m_dirtyBlocks; // 当前需要局部重建的逻辑叶区索引。
    float m_interiorBackgroundDistance; // 窄带之外材料内部使用的固定负距离。
    float m_exteriorBackgroundDistance; // 窄带之外材料外部使用的固定正距离。
    bool m_completelyDirty; // 整个距离场是否失效并需要完整重建。
};

}

#endif // MYVOXEL_CORE_VOLUME_VOLUMEFIELD_H