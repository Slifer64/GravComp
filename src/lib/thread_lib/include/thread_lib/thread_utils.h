#ifndef AS64_THREAD_LIB_THREAD_UTILS_H
#define AS64_THREAD_LIB_THREAD_UTILS_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <pthread.h>

namespace as64_
{

namespace thr_
{

int makeThreadRT(std::thread &thr);
int setThreadPriority(std::thread &thr, int policy, int priority);
std::string setThreadPriorErrMsg(int error_code);


} // namespace thr_

} // namespace as64_

#endif // AS64_THREAD_LIB_THREAD_UTILS_H
