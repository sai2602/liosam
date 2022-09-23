// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.h"


class ClientSocket : private Socket
{
 public:

  ClientSocket ( std::string host, int port );
  virtual ~ClientSocket();

  const ClientSocket& operator << ( const SocketData &data ) const;
  const ClientSocket& send ( const SocketData &data ) const;

  const ClientSocket& operator >> ( SocketData &data ) const;
  const ClientSocket& recv ( SocketData &data ) const;

  void setRecvAll ( bool recvAll );


 private:

  const ClientSocket& recv_all ( SocketData &data ) const;

  bool recvAll;

};


#endif
