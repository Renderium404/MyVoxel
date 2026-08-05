#ifndef MYVOXEL_FOUNDATION_CANCELLATION_H
#define MYVOXEL_FOUNDATION_CANCELLATION_H

#include <atomic>
#include <memory>

namespace MyVoxel
{
namespace Foundation
{

class CancellationSource;

// 提供只读的线程安全协作取消状态。
class CancellationToken
{
public:
    // 构造不连接任何取消状态的空令牌。
    CancellationToken()
    {
    }

    // 判断当前令牌是否连接到取消状态。
    bool canBeCancelled() const
    {
        return static_cast<bool>(m_state);
    }

    // 判断当前令牌是否已经收到取消请求。
    bool isCancellationRequested() const
    {
        return m_state && m_state->requested.load(std::memory_order_acquire);
    }

private:
    friend class CancellationSource;

    // 保存一个取消源及其全部令牌共享的状态。
    struct State
    {
        State()
            : requested(false)
        {
        }

        std::atomic<bool> requested; // 当前共享状态是否已经收到取消请求。
    };

    // 使用指定共享状态构造取消令牌。
    explicit CancellationToken(const std::shared_ptr<State>& state)
        : m_state(state)
    {
    }

private:
    std::shared_ptr<State> m_state; // 当前令牌观察的共享取消状态。
};

// 创建并控制一组共享取消令牌。
class CancellationSource
{
public:
    CancellationSource()
        : m_state(std::make_shared<CancellationToken::State>())
    {
    }

    // 返回观察当前共享状态的取消令牌。
    CancellationToken token() const
    {
        return CancellationToken(m_state);
    }

    // 发出取消请求，首次请求返回true，重复请求返回false。
    bool requestCancellation() const
    {
        return !m_state->requested.exchange(true, std::memory_order_acq_rel);
    }

private:
    std::shared_ptr<CancellationToken::State> m_state; // 当前取消源控制的共享取消状态。
};

}
}

#endif // MYVOXEL_FOUNDATION_CANCELLATION_H