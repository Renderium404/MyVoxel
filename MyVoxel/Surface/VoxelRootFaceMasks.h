#ifndef MYVOXEL_SURFACE_VOXELROOTFACEMASKS_H
#define MYVOXEL_SURFACE_VOXELROOTFACEMASKS_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "MyVoxel/Core/VoxelAddress.h"

#include "VoxelFaceAddress.h"
#include "VoxelFaceSet.h"

namespace MyVoxel
{

// 使用按方向和切片组织的二维位平面保存一个第0层根拥有的全部最高层单位外表面。
//
// 每个方向包含axisCellCount个切片，每个切片包含axisCellCount行；
// 每行使用一个或多个std::uint64_t保存平面U方向的连续单位面。
// X方向平面坐标为Y、Z，Y方向平面坐标为X、Z，Z方向平面坐标为X、Y。
class VoxelRootFaceMasks
{
public:
    VoxelRootFaceMasks();

    // 为指定第0层根创建空面掩码。
    VoxelRootFaceMasks(const VoxelCellIndex& rootIndex, std::size_t axisCellCount);

    VoxelRootFaceMasks(const VoxelRootFaceMasks&) = default;
    VoxelRootFaceMasks& operator=(const VoxelRootFaceMasks&) = default;

    // 移动构造和移动赋值手动实现，以兼容Visual Studio 2013。
    VoxelRootFaceMasks(VoxelRootFaceMasks&& other);
    VoxelRootFaceMasks& operator=(VoxelRootFaceMasks&& other);

    /// 掩码状态

    // 判断当前对象是否已经使用根索引和单轴分辨率初始化。
    bool isInitialized() const;

    // 判断当前根是否不包含任何单位外表面。
    bool isEmpty() const;

    // 返回当前掩码所属的第0层根索引。
    const VoxelCellIndex& rootIndex() const;

    // 返回当前根单轴最高层体素数量。
    std::size_t axisCellCount() const;

    // 返回一个二维平面行包含的64位字数量。
    std::size_t wordsPerPlaneRow() const;

    // 返回当前根包含的单位外表面数量。
    std::size_t faceCount() const;

    /// 掩码读取

    // 返回指定方向、切片、平面行和字索引对应的单位面掩码。
    std::uint64_t planeWord(VoxelFaceDirection direction,
                            std::size_t slice,
                            std::size_t v,
                            std::size_t wordIndex) const;

    // 判断当前根是否包含指定全局单位面地址。
    bool contains(const VoxelFaceAddress& face) const;

    // 判断指定方向是否不包含任何单位外表面。
    bool directionIsEmpty(VoxelFaceDirection direction) const;

    // 判断当前根与other在指定方向上的全部单位面掩码是否相同。
    bool directionEquals(const VoxelRootFaceMasks& other,
                         VoxelFaceDirection direction) const;

    /// 掩码构建

    // 设置指定方向切片中的一个完整平面行字，目标字必须尚未写入。
    void setPlaneWord(VoxelFaceDirection direction,
                      std::size_t slice,
                      std::size_t v,
                      std::size_t wordIndex,
                      std::uint64_t word);

    // 设置指定根内最高层体素的一个方向单位面。
    void setFace(std::size_t localX,
                 std::size_t localY,
                 std::size_t localZ,
                 VoxelFaceDirection direction);

    // 使用replaceMask覆盖指定平面行字中的局部单位面状态，并同步单位面数量。
    void replacePlaneWordMasked(VoxelFaceDirection direction,
                                std::size_t slice,
                                std::size_t v,
                                std::size_t wordIndex,
                                std::uint64_t calculatedWord,
                                std::uint64_t replaceMask);

    // 当前Root已经不包含单位面时释放连续位图存储。
    void releaseEmptyStorage();

    /// 面地址转换

    // 将当前掩码转换为按照VoxelFaceAddress升序排列的兼容面集合。
    VoxelFaceSet toFaceSet() const;

    // 将当前掩码中的全部单位面地址追加到目标数组，追加顺序不作保证。
    void appendFaces(VoxelFaceSet::Container& faces) const;

    // 将当前掩码存在而other不存在的单位面地址追加到目标数组。
    void appendDifferenceFaces(const VoxelRootFaceMasks& other,
                               VoxelFaceSet::Container& faces) const;

    bool operator==(const VoxelRootFaceMasks& other) const;
    bool operator!=(const VoxelRootFaceMasks& other) const;

private:
    // 返回指定方向切片、平面行和字索引的线性存储位置。
    std::size_t planeWordIndex(VoxelFaceDirection direction,
                               std::size_t slice,
                               std::size_t v,
                               std::size_t wordIndex) const;

    // 返回最后一个平面行字中属于当前分辨率的有效位。
    std::uint64_t validWordMask(std::size_t wordIndex) const;

    // 返回完整六方向面掩码需要的64位字数量。
    std::size_t storageWordCount() const;

    // 在第一次写入非零面时建立完整连续位图存储。
    void ensureStorage();

    // 将根内XYZ坐标转换为指定方向的slice、u和v。
    static void planeCoordinates(VoxelFaceDirection direction,
                                 std::size_t localX,
                                 std::size_t localY,
                                 std::size_t localZ,
                                 std::size_t& slice,
                                 std::size_t& u,
                                 std::size_t& v);

    // 将指定方向平面坐标转换为全局单位面地址。
    VoxelFaceAddress faceAddress(VoxelFaceDirection direction,
                                 std::size_t slice,
                                 std::size_t u,
                                 std::size_t v) const;

private:
    VoxelCellIndex m_rootIndex; // 当前面掩码所属的第0层根索引。
    std::size_t m_axisCellCount; // 当前根单轴最高层体素数量。
    std::size_t m_wordsPerPlaneRow; // 一个二维平面行包含的64位字数量。
    std::size_t m_faceCount; // 当前根全部方向单位外表面数量。
    std::vector<std::uint64_t> m_words; // 非空Root按方向、切片、V行和U方向字连续保存的单位面位图。
};

}

#endif // MYVOXEL_SURFACE_VOXELROOTFACEMASKS_H
