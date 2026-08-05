#include <cassert>

#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"

int main()
{
    using namespace MyVoxel;

    const VoxelChildStateMasks emptyStates =
        uniformChildStateMasks(VoxelState::Empty);

    assert(emptyStates.isValid());
    assert(emptyStates.isCollapsible());
    assert(emptyStates.collapsedState() == VoxelState::Empty);
    assert(emptyStates.empty == FullVoxelChildMask);
    assert(emptyStates.terminalMask() == FullVoxelChildMask);

    const VoxelChildStateMasks materialStates =
        uniformChildStateMasks(VoxelState::Material);

    assert(materialStates.isValid());
    assert(materialStates.isCollapsible());
    assert(materialStates.collapsedState() == VoxelState::Material);
    assert(materialStates.material == FullVoxelChildMask);

    VoxelChildStateMasks mixedStates;
    mixedStates.empty = static_cast<std::uint8_t>(0x0FU);
    mixedStates.material = static_cast<std::uint8_t>(0xF0U);

    assert(mixedStates.isValid());
    assert(!mixedStates.isCollapsible());

    VoxelChildStateMasks subdividedStates;
    subdividedStates.subdivided = FullVoxelChildMask;

    assert(subdividedStates.isValid());
    assert(!subdividedStates.isCollapsible());

    return 0;
}
