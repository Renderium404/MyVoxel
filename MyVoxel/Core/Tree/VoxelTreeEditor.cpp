#include "VoxelTreeEditor.h"

#include <algorithm>
#include <cassert>
#include <vector>

namespace
{

// 返回指定地址所属的第0层根节点地址。
MyVoxel::VoxelCellAddress rootCellAddress(const MyVoxel::VoxelCellAddress& address)
{
    MyVoxel::VoxelCellAddress rootAddress = address;

    while (MyVoxel::hasParentCell(rootAddress))
    {
        rootAddress = MyVoxel::parentCellAddress(rootAddress);
    }

    return rootAddress;
}

// 构造从第0层根节点到目标节点的角点路径。
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

}

namespace MyVoxel
{

VoxelTreeEditor::VoxelTreeEditor(VoxelForest& forest, const VoxelCellAddress& address)
    : m_editor(createEditor(forest, address))
{
}

VoxelTreeEditor::VoxelTreeEditor(VoxelTree& tree)
    : m_editor(tree)
{
}

VoxelTreeEditor::VoxelTreeEditor(const VoxelPackedTreeEditor& editor)
    : m_editor(editor)
{
}

VoxelState VoxelTreeEditor::state() const
{
    return m_editor.state();
}

void VoxelTreeEditor::setState(VoxelState stateValue)
{
    m_editor.setState(stateValue);
}

bool VoxelTreeEditor::split()
{
   return m_editor.split();
}
bool VoxelTreeEditor::makeMaskLeaf()
{
    return m_editor.makeMaskLeaf();
}

bool VoxelTreeEditor::isMaskLeaf() const
{
    return m_editor.isMaskLeaf();
}

bool VoxelTreeEditor::canAccessChildren() const
{
    return m_editor.canAccessChildren();
}
VoxelTreeEditor VoxelTreeEditor::child(VoxelCorner corner) const
{
    return VoxelTreeEditor(m_editor.child(corner));
}

VoxelState VoxelTreeEditor::merge()
{
    return m_editor.merge();
}

VoxelPackedTreeEditor VoxelTreeEditor::createEditor(VoxelForest& forest, const VoxelCellAddress& address)
{
    const VoxelCellAddress rootAddress = rootCellAddress(address);
    VoxelTree* tree = forest.detachRootTree(rootAddress.index);

    assert(tree);

    VoxelPackedTreeEditor editor(*tree);
    std::vector<VoxelCorner> path;

    buildCornerPath(address, path);

    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
    {
        assert(editor.canAccessChildren());
        editor = editor.child(path[pathIndex]);
    }

    return editor;
}

}