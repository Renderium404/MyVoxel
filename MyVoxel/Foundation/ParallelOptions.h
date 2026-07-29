#ifndef MYVOXEL_FOUNDATION_PARALLELOPTIONS_H
#define MYVOXEL_FOUNDATION_PARALLELOPTIONS_H

#include <cstddef>

namespace MyVoxel
{
namespace Foundation
{

// 配置一次并行循环的执行方式。
struct ParallelForOptions
{
    std::size_t workerCount = 0; // 使用的工作线程数量，0表示使用执行器允许的最大线程数量。
    std::size_t minimumParallelTaskCount = 2; // 任务数量至少达到2个时才允许进入并行执行。
};

}
}

#endif // MYVOXEL_FOUNDATION_PARALLELOPTIONS_H