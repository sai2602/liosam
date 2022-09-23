// Definition of the Socket class

#ifndef Socket_class
#define Socket_class

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>

#include "SocketData.h"


const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
//const int MAXRECV = 500;
const int MAXRECV = 1000;


class Socket
{
 public:
  Socket();
  virtual ~Socket();

  // Server initialization
  bool create( bool setReusePortKeepAlive = false );
  bool bind ( const int port );
  bool listen() const;
  bool accept ( Socket& ) const;

  // Client initialization
  bool connect ( const std::string host, const int port );

  // Data Transimission
  int send ( const SocketData & ) const;
  int send_all ( const SocketData &, const int32_t ) const;
  int recv ( SocketData & ) const;
  int recv_all ( SocketData &, const int32_t ) const;


  void set_non_blocking ( const bool );

  bool is_valid() const { return m_sock != -1; };


 private:

  int m_sock;
  sockaddr_in m_addr;


};


#endif
