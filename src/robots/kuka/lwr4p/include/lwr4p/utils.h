#ifndef LWR_ROBOT_UTILS_H
#define LWR_ROBOT_UTILS_H

#include <iostream>
#include <fstream>

namespace lwr4p
{

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cerr);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cerr);


class Timer
{
public:
  Timer()
  {
    clock_gettime(CLOCK_REALTIME, &beg_);
  }

  long int elapsedNanoSec()
  {
    clock_gettime(CLOCK_REALTIME, &end_);
    return (end_.tv_sec - beg_.tv_sec)*1000000000 + (end_.tv_nsec - beg_.tv_nsec);
  }

  double elapsedMicroSec()
  {
    return elapsedNanoSec()/1000.0;
  }

  double elapsedMilliSec()
  {
    return elapsedNanoSec()/1000000.0;
  }

  double elapsedSec()
  {
    return elapsedNanoSec()/1000000000.0;
  }

  void start() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
  timespec beg_, end_;
};


}  // namespace lwr4p

#endif  // LWR_ROBOT_UTILS_H
