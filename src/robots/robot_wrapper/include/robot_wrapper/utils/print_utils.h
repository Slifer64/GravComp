#ifndef AS64_ROBOT_WRAPPER_PRINT_UTILS_H
#define AS64_ROBOT_WRAPPER_PRINT_UTILS_H

namespace rw_

{

inline void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

inline void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout)
{
  out << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

inline void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout)
{
  out << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

inline void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout)
{
  out << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_PRINT_UTILS_H