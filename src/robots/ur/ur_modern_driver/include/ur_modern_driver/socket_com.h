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

class Timeout
{
public:
  Timeout(unsigned sec, unsigned usec)
  {
    setSec(sec);
    setMicroSec(usec);
  }

  void setSec(unsigned sec) { timeout.tv_sec = sec; }
  void setMicroSec(unsigned usec) { timeout.tv_usec = usec; }

  unsigned getSec() const { return timeout.tv_sec; }
  unsigned getMicroSec() const { return timeout.tv_usec; }

private:
  struct timeval timeout;
};

std::string getErrMsg(int err_code);

int write(int sockfd_, const char *buffer, unsigned len, bool send_all=true);

int read(int sockfd_, char *buffer, unsigned max_len, int *err_code=NULL);
int read(int sockfd_, char *buffer, unsigned max_len, const Timeout &tm_out, int *err_code=NULL);

int waitForWrite(int sock_fd, const Timeout &tm_out);
int waitForRead(int sock_fd, const Timeout &tm_out);

int openSocket(int domain_ = AF_INET, int type_ = SOCK_STREAM);
void closeSocket(int sock_fd);

void bind(int sock_fd, int port, const std::string &listen_addr="0.0.0.0");

void listen(int sock_fd, int num_of_connections);

int acceptClient(int sock_fd);
int acceptClient(int sock_fd, const Timeout &tm_out);

void connectToServer(int sock_fd, const std::string &server_name, int port, const Timeout &tm_out);
void connectToServer(int sock_fd, const std::string &server_name, int port);

void setNoDelay(int sock_fd, bool set);
void setQuickAck(int sock_fd, bool set);
void setReuseAddr(int sock_fd, bool set);
void setNonBlocking(int sock_fd, bool set);

bool isBlocking(int sock_fd);

std::string getLocalIp(int sock_fd);
int getLocalPort(int sock_fd);

} // namespace com_

} // namespace ur_


#endif // UR_SOCKET_COM_H
