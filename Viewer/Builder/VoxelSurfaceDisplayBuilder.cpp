#include "VoxelSurfaceDisplayBuilder.h"

namespace MyVoxelViewer
{

DisplayMesh VoxelSurfaceDisplayBuilder::build(const MyVoxel::VoxelSurfaceMesh& mesh)
{
    DisplayMesh result;
    result.reserve(mesh.vertexCount(), mesh.indexCount());

    for (const MyVoxel::VoxelSurfaceVertex& vertex : mesh.vertices())
    {
        const DisplayColor color(vertex.color.red, vertex.color.green, vertex.color.blue, vertex.color.alpha);
        result.vertices().push_back(DisplayVertex(vertex.x, vertex.y, vertex.z, vertex.nx, vertex.ny, vertex.nz, color));
    }

    result.indices() = mesh.indices();
    return result;
}

}