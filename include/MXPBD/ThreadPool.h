#pragma once

namespace MXPBD {
    class TBB_CoreMasking : public tbb::task_scheduler_observer {
        DWORD_PTR mask;

    public:
        TBB_CoreMasking(tbb::task_arena& arena, DWORD_PTR m)
            : tbb::task_scheduler_observer(arena), mask(m) {
            observe(true);
        }

        void on_scheduler_entry(bool worker) override {
            if (worker) {
                static thread_local bool pinned = false;
                if (!pinned) {
                    SetThreadAffinityMask(GetCurrentThread(), mask);
                    pinned = true;
                }
            }
        }
    };

    class TBB_ThreadPool {
    public:
        TBB_ThreadPool() = delete;
        TBB_ThreadPool(std::uint32_t a_threadSize, std::uint64_t a_coreMask)
            : workers(std::make_unique<tbb::task_arena>(a_threadSize)) {
            if (a_coreMask != 0)
                observer = std::make_unique<TBB_CoreMasking>(*workers, a_coreMask);
        }

        template <typename F>
        void Execute(F&& f) {
            workers->execute(std::forward<F>(f));
        }

        template <typename F>
        void Enqueue(F&& f) {
            workers->enqueue(std::forward<F>(f));
        }

        std::int32_t GetThreadSize() const { return workers->max_concurrency(); }

    private:
        std::unique_ptr<tbb::task_arena> workers;
        std::unique_ptr<TBB_CoreMasking> observer;
    };

    void GetPCore(std::uint32_t& count, std::uint64_t& mask);
}
