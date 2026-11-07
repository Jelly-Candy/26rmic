#pragma once
#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

// One pending owned image: a slow sender replaces stale images instead of
// delaying camera/recognition or building an unbounded latency queue.
template<class Frame> class LatestPreview {
  std::function<void(Frame&)> send_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::unique_ptr<Frame> pending_;
  bool stopping_=false;
  std::exception_ptr failure_;
  std::atomic<uint64_t> completed_{0};
  std::thread worker_;
  void run() noexcept {
    try {
      for(;;) {
        std::unique_ptr<Frame> frame;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          changed_.wait(lock,[&]{return stopping_ || pending_;});
          if(stopping_)return;
          frame=std::move(pending_);
        }
        send_(*frame);
        ++completed_;
      }
    } catch(...) {
      std::lock_guard<std::mutex> lock(mutex_);
      failure_=std::current_exception();pending_.reset();
    }
  }
public:
  explicit LatestPreview(std::function<void(Frame&)> send):send_(std::move(send)),worker_([this]{run();}){}
  ~LatestPreview(){stop();}
  LatestPreview(const LatestPreview&)=delete;
  LatestPreview& operator=(const LatestPreview&)=delete;
  void rethrowFailure() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(failure_)std::rethrow_exception(failure_);
  }
  void submit(std::unique_ptr<Frame> frame) {
    if(!frame)throw std::runtime_error("Preview image copy failed");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(failure_)std::rethrow_exception(failure_);
      if(stopping_)throw std::runtime_error("Preview sender stopped");
      pending_=std::move(frame);
    }
    changed_.notify_one();
  }
  uint64_t completed()const{return completed_.load();}
  void stop() noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_=true;pending_.reset();
    }
    changed_.notify_one();
    if(worker_.joinable())worker_.join();
  }
};
