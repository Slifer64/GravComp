#include <ur_modern_driver/socket_com.h>
#include <ur_modern_driver/do_output.h>

#include <iostream>
#include <exception>

namespace ur_
{

namespace com_
{

#define Ur_Com_fun_ std::string("[ur_::com_::") + __func__ + "]: "

std::string getErrMsg(int err_code)
{
  return std::strerror(err_code);
}

int openSocket(int domain_, int type_)
{
  int sock_fd = ::socket(domain_, type_, 0);
  if (sock_fd < 0)
    throw std::runtime_error(Ur_Com_fun_+ "Error on \"socket()\": " + std::strerror(errno));

  return sock_fd;
}

void closeSocket(int sock_fd)
{
  if (sock_fd < 0) return;
  ::close(sock_fd);
}

void bind(int sock_fd, int port, const std::string &listen_addr)
{
  struct sockaddr_in serv_addr;
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
  inet_pton(AF_INET, listen_addr.c_str(), (void *)&serv_addr.sin_addr);
	// serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);

	if (::bind(sock_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    throw std::runtime_error(Ur_Com_fun_ + "Error on \"bind()\": " + std::strerror(errno));
}

void listen(int sock_fd, int num_of_connections)
{
	if ( ::listen(sock_fd, num_of_connections) < 0 ) // listen(server_fid, SOMAXCONN)
		throw std::runtime_error(Ur_Com_fun_ + "Error on \"listen()\": " + std::strerror(errno));
}

int acceptClient(int sock_fd)
{
  struct sockaddr_in cli_addr;
	socklen_t clilen = sizeof(cli_addr);
	int new_sockfd_ = ::accept(sock_fd, (struct sockaddr *) &cli_addr, &clilen);
  if (new_sockfd_ < 0) throw std::runtime_error(Ur_Com_fun_ + "Error on \"accept()\": " + std::strerror(errno));

  return new_sockfd_;
}

int acceptClient(int sock_fd, const Timeout &tm_out)
{
	int err_code = com_::waitForRead(sock_fd, tm_out);

  if (err_code != 0) throw std::runtime_error(Ur_Com_fun_ + com_::getErrMsg(err_code));

  struct sockaddr_in cli_addr;
	socklen_t clilen = sizeof(cli_addr);
	int new_sockfd_ = ::accept(sock_fd, (struct sockaddr *) &cli_addr, &clilen);
  if (new_sockfd_ < 0) throw std::runtime_error(Ur_Com_fun_ + "Error on \"accept()\": " + std::strerror(errno));

  return new_sockfd_;
}

int waitForWrite(int sock_fd, const Timeout &tm_out)
{
  fd_set write_fds;
  FD_ZERO(&write_fds);
  FD_SET(sock_fd, &write_fds);

  struct timespec timeout_;
  timeout_.tv_sec = tm_out.getSec();
  timeout_.tv_nsec = tm_out.getMicroSec() * 1000;

  if (::pselect(sock_fd + 1, NULL, &write_fds, NULL, &timeout_, NULL) < 0) return errno;

  if (FD_ISSET(sock_fd, &write_fds)) return 0;
  else return ETIME;
}

int waitForRead(int sock_fd, const Timeout &tm_out)
{
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(sock_fd, &read_fds);

  struct timespec timeout_;
  timeout_.tv_sec = tm_out.getSec();
  timeout_.tv_nsec = tm_out.getMicroSec() * 1000;

  if (::pselect(sock_fd + 1, &read_fds, NULL, NULL, &timeout_, NULL) < 0) return errno;

  if (FD_ISSET(sock_fd, &read_fds)) return 0;
  else return ETIME;
}

void connectToServer(int sock_fd, const std::string &server_name, int port, const Timeout &tm_out)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "You have to call \"openSocket()\" first!");

  struct sockaddr_in serv_addr_;
	struct hostent *server_;
  bzero((char *) &serv_addr_, sizeof(serv_addr_));
	server_ = gethostbyname(server_name.c_str());
	if (server_ == NULL) throw std::runtime_error(Ur_Com_fun_ + "ERROR, no such host");
	serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&serv_addr_.sin_addr.s_addr, server_->h_length);
	// inet_pton(AF_INET, host_ip.c_str(), (void *)&server_addr.sin_addr.s_addr);
	serv_addr_.sin_port = htons(port);

	bool was_blocking = isBlocking(sock_fd);
	if (was_blocking) setNonBlocking(sock_fd, true); // make non blocking to use timeout

  if (::connect(sock_fd, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_)) < 0)
  {
    if (errno==EINPROGRESS) // && !tm.isNull())
    {
      int err_code = waitForWrite(sock_fd, tm_out);
      if ( err_code != 0)
          throw std::runtime_error(Ur_Com_fun_ + "Error while waiting to connect to port " + std::to_string(port) + ": " + com_::getErrMsg(err_code));
    }
    else throw std::runtime_error(Ur_Com_fun_ + "Error on \"connect()\": " + std::strerror(errno));
  }

  if (was_blocking) setNonBlocking(sock_fd, false); // restore previous setup
}

void connectToServer(int sock_fd, const std::string &server_name, int port)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "You have to call \"openSocket()\" first!");

  struct sockaddr_in serv_addr_;
	struct hostent *server_;
  bzero((char *) &serv_addr_, sizeof(serv_addr_));
	server_ = gethostbyname(server_name.c_str());
	if (server_ == NULL) throw std::runtime_error(Ur_Com_fun_ + "ERROR, no such host");
	serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&serv_addr_.sin_addr.s_addr, server_->h_length);
	// inet_pton(AF_INET, host_ip.c_str(), (void *)&server_addr.sin_addr.s_addr);
	serv_addr_.sin_port = htons(port);

  if (::connect(sock_fd, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_)) < 0)
    throw std::runtime_error(Ur_Com_fun_ + "Error on \"connect()\": " + std::strerror(errno));
}

int write(int sockfd_, const char *buffer, unsigned len, bool send_all)
{
  if (sockfd_ < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid file descriptor!");

  int remain = len;
  while (remain && send_all)
  {
    int len2 = ::write(sockfd_, buffer, len);
    if (len2 < 0) throw std::runtime_error(Ur_Com_fun_ + "Error: " + std::strerror(errno));
    remain -= len2;
  }
  return (len-remain);
}

int read(int sockfd_, char *buffer, unsigned max_len, int *err_code)
{
  if (sockfd_ < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid file descriptor!");
  int bytes_read = ::read(sockfd_, buffer, max_len);
  if (bytes_read < 0 && err_code) *err_code = errno;
  return bytes_read;
}

int read(int sockfd_, char *buffer, unsigned max_len, const Timeout &tm_out, int *err_code)
{
  if (sockfd_ < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid file descriptor!");

  int err_ = com_::waitForRead(sockfd_, tm_out);

  if (err_ != 0)
  {
    if (err_code) *err_code = err_;
    return -1;
  }

  return com_::read(sockfd_, buffer, max_len, err_code);
}

void setReuseAddr(int sock_fd, bool set)
{
  int flag = set;
  if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *) &flag, sizeof(flag)) < 0) // to bypass TIME_WAIT state on close
    throw std::runtime_error(Ur_Com_fun_ + "Error: " + std::strerror(errno));
}

void setNonBlocking(int sock_fd, bool set)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid socket file descriptor...\n");

  int flags = fcntl(sock_fd, F_GETFL);
  if (flags < 0) throw std::runtime_error(Ur_Com_fun_ + "Error on \"fcntl(fd, F_GETFL)\": " + std::strerror(errno));

  if (set) flags |= O_NONBLOCK;
  else flags &= ~O_NONBLOCK;

  if (fcntl(sock_fd, F_SETFL, flags) < 0)
    throw std::runtime_error(Ur_Com_fun_ + "Error on \"fcntl(fd, F_SETFL, O_NONBLOCK)\": " + std::strerror(errno));
}

bool isBlocking(int sock_fd)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid socket file descriptor...\n");

  int flags = fcntl(sock_fd, F_GETFL);
  if (flags < 0) throw std::runtime_error(Ur_Com_fun_ + "Error on \"fcntl(fd, F_GETFL)\": " + std::strerror(errno));

  return flags & O_NONBLOCK;
}

void setQuickAck(int sock_fd, bool set)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid socket file descriptor...\n");

  int flag = set;
  if (setsockopt(sock_fd, IPPROTO_TCP, TCP_QUICKACK, (char *)&flag, sizeof(flag)) < 0)
    throw std::runtime_error(Ur_Com_fun_ + "Error: " + std::strerror(errno));
}

void setNoDelay(int sock_fd, bool set)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid socket file descriptor...\n");

  int flag = set;
  if (setsockopt(sock_fd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag)) < 0)
    throw std::runtime_error(Ur_Com_fun_ + "Error: " + std::strerror(errno));
}

std::string getLocalIp(int sock_fd)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid socket file descriptor...\n");

  sockaddr_in addr_;
  socklen_t len = sizeof(addr_);
  if (::getsockname(sock_fd, (sockaddr*)&addr_, &len))
    throw std::runtime_error(Ur_Com_fun_ + "Error: " + std::strerror(errno));

  int ip_addr_len;
  int domain = addr_.sin_family;
  if (domain == AF_INET) ip_addr_len = INET_ADDRSTRLEN;
  else ip_addr_len = INET6_ADDRSTRLEN;

  char ip_[ip_addr_len];
  if ( inet_ntop(domain, (void *)&addr_.sin_addr, ip_, ip_addr_len) == NULL )
    throw std::runtime_error(Ur_Com_fun_ + "Error on \"inet_ntop()\": " + std::strerror(errno));

  return std::string(ip_);
}

int getLocalPort(int sock_fd)
{
  if (sock_fd < 0) throw std::runtime_error(Ur_Com_fun_ + "Invalid socket file descriptor...\n");

  sockaddr_in addr_;
  socklen_t len = sizeof(addr_);
  if (::getsockname(sock_fd, (sockaddr*)&addr_, &len))
    throw std::runtime_error(Ur_Com_fun_ + "Error: " + std::strerror(errno));

  return ntohs(addr_.sin_port);
}


} // namespace com_

} // namespace ur_
