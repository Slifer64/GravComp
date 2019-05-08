#include <grav_comp/utils.h>

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}

FileFormat getFileFormat(const std::string &file_name, std::string *suffix_ptr)
{
  std::string suffix = "";
  int n = file_name.size();
  int i = n-1;
  for (; i>-1 && file_name[i]!='.'; i--);
  if (i<n-1) suffix = file_name.substr(i+1);

  FileFormat file_fmt;
  if (suffix.compare("bin")==0) file_fmt = FileFormat::BIN;
  else if (suffix.compare("txt")==0) file_fmt = FileFormat::TXT;
  else if (suffix.compare("yaml")==0 || suffix.compare("yml")==0) file_fmt = FileFormat::YAML;
  else file_fmt = FileFormat::UNKNOWN;

  if (suffix_ptr) *suffix_ptr = suffix;

  return file_fmt;
}
