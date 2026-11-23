#pragma once
// Minimal fixed-size thread pool with a blocking parallelFor.
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace eng {

class ThreadPool {
public:
    // workers == 0 runs everything on the calling thread.
    explicit ThreadPool(unsigned workers);
    ~ThreadPool();
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    unsigned size() const { return (unsigned)workers_.size(); }

    // Splits [0, count) into chunks of at least `grain` items and calls fn(begin, end)
    // on each, returning once all chunks are done.  fn must be safe to run concurrently
    // on disjoint ranges.
    void parallelFor(size_t count, size_t grain, const std::function<void(size_t, size_t)>& fn);

private:
    void workerLoop();

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
};

}  // namespace eng
