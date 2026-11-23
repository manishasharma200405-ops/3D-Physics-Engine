#include "Pipeline/ThreadPool.h"

#include <algorithm>
#include <atomic>

namespace eng {

ThreadPool::ThreadPool(unsigned workers) {
    for (unsigned i = 0; i < workers; i++) workers_.emplace_back([this] { workerLoop(); });
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& t : workers_) t.join();
}

void ThreadPool::workerLoop() {
    for (;;) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
            if (stop_ && tasks_.empty()) return;
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        task();
    }
}

void ThreadPool::parallelFor(size_t count, size_t grain, const std::function<void(size_t, size_t)>& fn) {
    if (count == 0) return;
    grain = std::max<size_t>(grain, 1);
    if (workers_.empty() || count <= grain) {
        fn(0, count);
        return;
    }

    size_t maxChunks = workers_.size() * 4;
    size_t chunks = std::min((count + grain - 1) / grain, maxChunks);
    size_t per = (count + chunks - 1) / chunks;
    chunks = (count + per - 1) / per;  // every chunk non-empty

    std::atomic<size_t> remaining(chunks);
    std::mutex doneMutex;
    std::condition_variable doneCv;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t c = 0; c < chunks; c++) {
            size_t begin = c * per;
            size_t end = std::min(count, begin + per);
            tasks_.push([&, begin, end] {
                fn(begin, end);
                std::lock_guard<std::mutex> dl(doneMutex);
                if (--remaining == 0) doneCv.notify_all();
            });
        }
    }
    cv_.notify_all();

    std::unique_lock<std::mutex> lock(doneMutex);
    doneCv.wait(lock, [&] { return remaining.load() == 0; });
}

}  // namespace eng
