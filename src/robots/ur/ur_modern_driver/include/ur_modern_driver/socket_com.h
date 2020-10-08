#ifndef UR_SOCKET_COM_H
#define UR_SOCKET_COM_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstring>
#include <string>

#include <unistd.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>

namespace ur_
{

namespace com_
{

enum WaitResult
{
  READY,
  TIMEOUT,
  ERROR
};

std::string getErrMsg(int err_code);

int write(int sockfd_, const char *buffer, unsigned len, bool send_all=true);
int read(int sockfd_, char *buffer, unsigned max_len);

WaitResult waitForWrite(int sock_fd, struct timeval timeout, int *err_code = NULL);
WaitResult waitForRead(int sock_fd, struct timeval timeout, int *err_code = NULL);

int openSocket(int domain_ = AF_INET, int type_ = SOCK_STREAM);
void closeSocket(int sock_fd);

void connectToServer(int sock_fd, const std::string &server_name, int port, struct timeval timeout);
void connectToServer(int sock_fd, const std::string &server_name, int port);

void setNoDelay(int sock_fd, bool set);
void setQuickAck(int sock_fd, bool set);
void setReuseAddr(int sock_fd, bool set);
void setNonBlocking(int sock_fd, bool set);

std::string getLocalIp(int sock_fd);
int getLocalPort(int sock_fd);

} // namespace com_

} // namespace ur_


#endif // UR_SOCKET_COM_H
