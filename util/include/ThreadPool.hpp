// The copyright in this software is being made available under the BSD
// License, included below. This software may be subject to other third party
// and contributor rights, including patent rights, and no such rights are
// granted under this license.
//
// Copyright (c) 2022, ISO/IEC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  * Neither the name of the ISO/IEC nor the names of its contributors may
//    be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
//
// ThreadPool.hpp
//
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace lctm {

// Minimal persistent thread pool: tasks are submitted and executed by a fixed set of worker
// threads that live for the process, avoiding per-call thread creation in the hot loops.
// Nested parallel_for calls are supported: while a thread waits for its tasks it also runs
// queued tasks from the pool, so workers never deadlock on each other's subtasks.
//
class ThreadPool {
public:
	static ThreadPool &instance() {
		static ThreadPool pool;
		return pool;
	}

	// Number of worker threads
	unsigned threads() const { return (unsigned)workers_.size(); }

	// Run fn(i) for i in [0, count), split across the workers; blocks until complete.
	//
	void parallel_for(unsigned count, const std::function<void(unsigned)> &fn) {
		if (count == 0)
			return;

		if (count == 1) {
			fn(0);
			return;
		}

		std::atomic<unsigned> next(0);
		std::atomic<unsigned> remaining(count);
		std::atomic<unsigned> tasks_pending(0);

		// Grab one index and run it; returns false when the range is exhausted
		auto grab_one = [&]() -> bool {
			const unsigned i = next.fetch_add(1);
			if (i >= count)
				return false;
			fn(i);
			remaining.fetch_sub(1);
			return true;
		};

		// Each worker grabs indices until the range is exhausted; tasks_pending counts the
		// queued copies of this closure so the caller can wait until every one has run (they
		// reference locals of this call, so none may outlive it)
		auto worker_task = [&]() {
			while (grab_one())
				;
			tasks_pending.fetch_sub(1);
		};

		const unsigned n_workers = (unsigned)workers_.size();
		{
			std::lock_guard<std::mutex> lock(mutex_);
			tasks_pending.store(n_workers);
			for (unsigned w = 0; w < n_workers; ++w)
				tasks_.push(worker_task);
		}
		cv_.notify_all();

		// Help with the work from this thread too - and while waiting, keep draining queued
		// tasks (which may be nested parallel_for submissions) so nested calls cannot deadlock
		while (remaining.load() != 0 || tasks_pending.load() != 0) {
			grab_one();
			run_pending();
		}
	}

private:
	ThreadPool() {
		const unsigned cores = std::max(1u, std::thread::hardware_concurrency());
		const unsigned n_workers = std::max(1u, cores - 1); // leave a core for the caller
		for (unsigned w = 0; w < n_workers; ++w)
			workers_.emplace_back([this]() { worker_loop(); });
	}

	~ThreadPool() {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			stop_ = true;
		}
		cv_.notify_all();
		for (auto &t : workers_)
			t.join();
	}

	ThreadPool(const ThreadPool &) = delete;
	ThreadPool &operator=(const ThreadPool &) = delete;

	// Run one queued task if any, without blocking
	void run_pending() {
		std::function<void()> task;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (tasks_.empty())
				return;
			task = std::move(tasks_.front());
			tasks_.pop();
		}
		task();
	}

	void worker_loop() {
		for (;;) {
			std::function<void()> task;
			{
				std::unique_lock<std::mutex> lock(mutex_);
				cv_.wait(lock, [&]() { return stop_ || !tasks_.empty(); });
				if (stop_ && tasks_.empty())
					return;
				task = std::move(tasks_.front());
				tasks_.pop();
			}
			task();
		}
	}

	std::vector<std::thread> workers_;
	std::queue<std::function<void()>> tasks_;
	std::mutex mutex_;
	std::condition_variable cv_;
	bool stop_ = false;
};

} // namespace lctm