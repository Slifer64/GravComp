#include <thread_lib/thread_utils.h>

namespace as64_
{

namespace thr_
{

std::string setThreadPriorErrMsg(int error_code)
{
  if (error_code == 0) return "";

  switch (error_code)
  {
    case ESRCH:
      return "No thread with the ID thread could be found.";
    case EINVAL:
      return "Policy is not a recognized policy, or param does not make sense for the policy.";
    case EPERM:
      return "The caller does not have appropriate privileges to set the specified scheduling policy and parameters.";
    case ENOTSUP:
      return "Attempt was made to set the policy or scheduling parameters to an unsupported value.";
    default:
      return "Unknown error code.";
  }
}

int setThreadPriority(std::thread &thr, int policy, int priority)
{
  struct sched_param sch_param;
  int prev_policy;
  pthread_getschedparam(thr.native_handle(), &prev_policy, &sch_param);
  sch_param.sched_priority = priority;
  int ret_code = pthread_setschedparam(thr.native_handle(), policy, &sch_param);

  return ret_code;
}

int makeThreadRT(std::thread &thr)
{
  return setThreadPriority(thr, SCHED_FIFO, 99);
}

} // namespace thr_

} // namespace as64_
