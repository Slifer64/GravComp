#ifndef AS64_THREAD_LIB_MTX_VAR_H
#define AS64_THREAD_LIB_MTX_VAR_H

#include <mutex>

namespace as64_
{

namespace thr_
{

template<typename T>
class MtxVar
{
public:
  MtxVar() { }
  MtxVar& operator=(const T &val) { set(val); return *this; }
  T operator()() const { return get(); }
  T get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&var_mtx))); return var; }
  T read() const { return var; }
  void set(const T &val) { std::unique_lock<std::mutex> lck(var_mtx); var=val; }
private:
  std::mutex var_mtx;
  T var;
};

// specialization for bool
template<>
class MtxVar<bool>
{
public:
  MtxVar() { var = false; }
  MtxVar& operator=(const bool &val) { set(val); return *this; }
  bool operator()() const { return get(); }
  bool get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&var_mtx))); return var; }
  bool read() const { return var; }
  void set(const bool &val) { std::unique_lock<std::mutex> lck(var_mtx); var=val; }
private:
  std::mutex var_mtx;
  bool var;
};

} // namespace thr_

} // namespace as64_

#endif // AS64_THREAD_LIB_MTX_VAR_H