#ifndef GRAVITY_COMPENSATION_UTILS_H
#define GRAVITY_COMPENSATION_UTILS_H

#define PACKAGE_NAME "grav_comp"

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>
#include <mutex>
#include <condition_variable>

#include <armadillo>

#include <QMetaType>

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);

class ExecResultMsg
{
public:

  enum TYPE
  {
      INFO,
      QUESTION,
      WARNING,
      ERROR
  };

  ExecResultMsg() { type=ExecResultMsg::INFO; msg=""; }

  std::string getMsg() const { return msg; }
  void setMsg(const std::string &msg) { this->msg = msg; }
  void addToMsg(const std::string &msg) { this->msg += msg; }
  void setType(ExecResultMsg::TYPE type) { this->type = type; }
  ExecResultMsg::TYPE getType() const { return type; }
private:
  std::string msg;
  ExecResultMsg::TYPE type;
};

Q_DECLARE_METATYPE(ExecResultMsg);

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

template<typename T>
class MtxVar
{
public:
  MtxVar() { }
  MtxVar& operator=(const T &val) { set(val); return *this; }
  bool operator()() const { return get(); }
  T get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&var_mtx))); return var; }
  void set(const T &val) { std::unique_lock<std::mutex> lck(var_mtx); var=val; }
private:
  std::mutex var_mtx;
  T var;
};

#endif // GRAVITY_COMPENSATION_UTILS_H