#include "VoxelDisplayInstance.h"

#include <cassert>

namespace MyVoxelViewer
{

VoxelDisplayInstance::VoxelDisplayInstance()
{
    for (int i = 0; i < 16; ++i)
    {
        transform[i] = 0.0f;
    }

    transform[0] = 1.0f;
    transform[5] = 1.0f;
    transform[10] = 1.0f;
    transform[15] = 1.0f;
}

VoxelDisplayInstance::VoxelDisplayInstance(const MyMath::Matrix4& transformValue, const DisplayColor& colorValue)
    : color(colorValue)
{
    assert(transformValue.isAffine());

    for (int column = 0; column < 4; ++column)
    {
        for (int row = 0; row < 4; ++row)
        {
            transform[column * 4 + row] = static_cast<float>(transformValue(row, column));
        }
    }
}

}