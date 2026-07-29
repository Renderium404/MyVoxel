#include "VoxelTreeOperation.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "../../Foundation/Diagnostic.h"
#include "VoxelTreeAccessor.h"
#include "VoxelTreeEditor.h"

namespace
{

// 返回指定地址所属的第0层根地址。
MyVoxel::VoxelCellAddress rootAddressOf(MyVoxel::VoxelCellAddress address)
{
    while (MyVoxel::hasParentCell(address))
    {
        address = MyVoxel::parentCellAddress(address);
    }

    return address;
}

// 构造从第0层根节点到目标地址的角点路径。
void buildCornerPath(const MyVoxel::VoxelCellAddress& address, std::vector<MyVoxel::VoxelCorner>& path)
{
    path.clear();
    path.reserve(static_cast<std::size_t>(address.level));

    MyVoxel::VoxelCellAddress currentAddress = address;

    while (MyVoxel::hasParentCell(currentAddress))
    {
        path.push_back(MyVoxel::childCornerInParent(currentAddress));
        currentAddress = MyVoxel::parentCellAddress(currentAddress);
    }

    std::reverse(path.begin(), path.end());
}

// 执行无符号64位饱和加法。
std::uint64_t saturatedAdd(std::uint64_t first, std::uint64_t second, bool& saturated)
{
    const std::uint64_t maximum = (std::numeric_limits<std::uint64_t>::max)();

    if (first > maximum - second)
    {
        saturated = true;
        return maximum;
    }

    return first + second;
}

// 返回指定压缩材料节点对应的最高层材料体素数量。
std::uint64_t finestVoxelCount(MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel, bool& saturated)
{
    MYVOXEL_ASSERT(level <= maximumLevel);

    const std::uint64_t maximum = (std::numeric_limits<std::uint64_t>::max)();
    std::uint64_t count = 1;

    for (MyVoxel::VoxelLevel currentLevel = level; currentLevel < maximumLevel; ++currentLevel)
    {
        if (count > maximum / static_cast<std::uint64_t>(MyVoxel::VoxelCornerCount))
        {
            saturated = true;
            return maximum;
        }

        count *= static_cast<std::uint64_t>(MyVoxel::VoxelCornerCount);
    }

    return count;
}

// 递归遍历全部压缩材料节点。
void visitMaterialCells(const MyVoxel::VoxelTreeCursor& cursor, const MyVoxel::VoxelCellAddress& address,
                        MyVoxel::VoxelLevel maximumLevel, const MyVoxel::VoxelTreeOperation::MaterialCellVisitor& visitor)
{
    if (cursor.isEmpty())
    {
        return;
    }

    if (cursor.isMaterial())
    {
        visitor(address);
        return;
    }

    MYVOXEL_REQUIRE_MESSAGE(address.level < maximumLevel, "Subdivided voxel exceeds the configured maximum level.");

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        visitMaterialCells(cursor.child(corner), MyVoxel::childCellAddress(address, corner), maximumLevel, visitor);
    }
}

// 递归收集树统计。
void collectStatistics(const MyVoxel::VoxelTreeCursor& cursor, const MyVoxel::VoxelCellAddress& address,
                       MyVoxel::VoxelLevel maximumLevel, MyVoxel::VoxelTreeStatistics& result)
{
    ++result.logicalNodeCount;

    if (address.level > result.deepestVisitedLevel)
    {
        result.deepestVisitedLevel = address.level;
    }

    if (cursor.isEmpty())
    {
        ++result.emptyNodeCount;
        return;
    }

    if (cursor.isMaterial())
    {
        ++result.materialNodeCount;

        bool saturated = false;
        const std::uint64_t representedCount = finestVoxelCount(address.level, maximumLevel, saturated);

        result.finestMaterialVoxelCount =
            saturatedAdd(result.finestMaterialVoxelCount, representedCount, result.finestMaterialVoxelCountSaturated);

        if (saturated)
        {
            result.finestMaterialVoxelCountSaturated = true;
        }

        return;
    }

    ++result.subdividedNodeCount;

    MYVOXEL_REQUIRE_MESSAGE(address.level < maximumLevel, "Subdivided voxel exceeds the configured maximum level.");

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        collectStatistics(cursor.child(corner), MyVoxel::childCellAddress(address, corner), maximumLevel, result);
    }
}

// 递归合并状态完全相同的八个子节点。
std::size_t normalizeEditor(MyVoxel::VoxelTreeEditor& editor, MyVoxel::VoxelLevel level, MyVoxel::VoxelLevel maximumLevel)
{
    if (!editor.isSubdivided())
    {
        return 0;
    }

    MYVOXEL_REQUIRE_MESSAGE(level < maximumLevel, "Subdivided voxel exceeds the configured maximum level.");

    std::size_t mergeCount = 0;

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        MyVoxel::VoxelTreeEditor childEditor = editor.child(static_cast<MyVoxel::VoxelCorner>(cornerIndex));
        mergeCount += normalizeEditor(childEditor, static_cast<MyVoxel::VoxelLevel>(level + 1), maximumLevel);
    }

    MyVoxel::VoxelTreeEditor firstChild = editor.child(MyVoxel::VoxelCorner::Minimum);

    if (firstChild.isSubdivided())
    {
        return mergeCount;
    }

    const bool material = firstChild.isMaterial();

    for (unsigned int cornerIndex = 1; cornerIndex < static_cast<unsigned int>(MyVoxel::VoxelCornerCount); ++cornerIndex)
    {
        const MyVoxel::VoxelTreeEditor childEditor = editor.child(static_cast<MyVoxel::VoxelCorner>(cornerIndex));

        if (childEditor.isSubdivided() || childEditor.isMaterial() != material)
        {
            return mergeCount;
        }
    }

    if (material)
    {
        editor.setMaterial();
    }
    else
    {
        editor.setEmpty();
    }

    return mergeCount + 1;
}

}

namespace MyVoxel
{

/// 整树状态

void VoxelTreeOperation::clear(VoxelTree& tree)
{
    VoxelTreeEditor editor(tree);
    editor.setEmpty();
}

void VoxelTreeOperation::fill(VoxelTree& tree)
{
    VoxelTreeEditor editor(tree);
    editor.setMaterial();
}

/// 地址访问

VoxelCursorState VoxelTreeOperation::stateAt(const VoxelTree& tree, const VoxelCellAddress& rootAddress,
                                             VoxelLevel maximumLevel, const VoxelCellAddress& address)
{
    VoxelTreeAccessor accessor(tree, rootAddress, maximumLevel);
    return accessor.seek(address);
}

void VoxelTreeOperation::setCellState(VoxelTree& tree, const VoxelCellAddress& rootAddress,
                                      VoxelLevel maximumLevel, const VoxelCellAddress& address, VoxelState state)
{
    MYVOXEL_REQUIRE_MESSAGE(rootAddress.level == BaseVoxelLevel, "Root address must be at BaseVoxelLevel.");
    MYVOXEL_REQUIRE_MESSAGE(address.level <= maximumLevel, "Target voxel address exceeds the configured maximum level.");
    MYVOXEL_REQUIRE_MESSAGE(rootAddressOf(address) == rootAddress, "Target voxel address does not belong to this VoxelTree root.");
    MYVOXEL_REQUIRE_MESSAGE(state == VoxelState::Empty || state == VoxelState::Material,
                            "VoxelTreeOperation::setCellState only accepts Empty or Material.");

    std::vector<VoxelCorner> path;
    buildCornerPath(address, path);

    VoxelTreeEditor editor(tree);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        if (editor.isTerminal())
        {
            editor.subdivide();
        }

        editor = editor.child(path[pathIndex]);
    }

    if (state == VoxelState::Material)
    {
        editor.setMaterial();
    }
    else
    {
        editor.setEmpty();
    }
}

/// 树规范化

std::size_t VoxelTreeOperation::normalize(VoxelTree& tree, VoxelLevel maximumLevel)
{
    MYVOXEL_REQUIRE_MESSAGE(tree.isValid(), "Cannot normalize an invalid VoxelTree.");

    VoxelTreeEditor editor(tree);
    return normalizeEditor(editor, BaseVoxelLevel, maximumLevel);
}

/// 树遍历

void VoxelTreeOperation::forEachMaterialCell(const VoxelTree& tree, const VoxelCellAddress& rootAddress,
                                             VoxelLevel maximumLevel, const MaterialCellVisitor& visitor)
{
    MYVOXEL_REQUIRE_MESSAGE(rootAddress.level == BaseVoxelLevel, "Root address must be at BaseVoxelLevel.");
    MYVOXEL_REQUIRE_MESSAGE(visitor, "Material cell visitor must not be empty.");
    MYVOXEL_REQUIRE_MESSAGE(tree.isValid(), "Cannot traverse an invalid VoxelTree.");

    visitMaterialCells(VoxelTreeCursor(tree), rootAddress, maximumLevel, visitor);
}

/// 树统计

VoxelTreeStatistics VoxelTreeOperation::statistics(const VoxelTree& tree, const VoxelCellAddress& rootAddress,
                                                   VoxelLevel maximumLevel)
{
    MYVOXEL_REQUIRE_MESSAGE(rootAddress.level == BaseVoxelLevel, "Root address must be at BaseVoxelLevel.");
    MYVOXEL_REQUIRE_MESSAGE(tree.isValid(), "Cannot collect statistics from an invalid VoxelTree.");

    VoxelTreeStatistics result;

    collectStatistics(VoxelTreeCursor(tree), rootAddress, maximumLevel, result);

    result.allocatedGroupCount = tree.blockPool->allocatedGroupCount();
    result.storageGroupCount = tree.blockPool->storageGroupCount();
    result.chunkCount = tree.blockPool->chunkCount();

    return result;
}

}