#ifndef AS64_THREAD_LIB_SEMAPHORE_H
#define AS64_THREAD_LIB_SEMAPHORE_H

#include <cstdlib>
#include <mutex>
#include <condition_variable>


namespace as64_
{

namespace thr_
{

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  // unsigned long count_ = 0; // Initialized as locked.
  bool count_ = false;  // Initialized as locked.

public:

  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    // ++count_;
    count_ = true;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    // --count_;
    count_ = false;
  }

  bool wait_until(double time_ms)
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);

    int wait_time = time_ms * 1e6;

    while(!count_) // Handle spurious wake-ups.
    {
      if (condition_.wait_for(lock,std::chrono::nanoseconds(wait_time))==std::cv_status::timeout)
        return false;
    }
    // --count_;
    count_ = false;

    return true;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      // --count_;
      count_ = false;
      return true;
    }
    return false;
  }
};

} // namespace thr_

} // namespace as64_

#endif // AS64_THREAD_LIB_SEMAPHORE_H
