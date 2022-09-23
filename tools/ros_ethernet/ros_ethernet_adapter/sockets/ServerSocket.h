// Definition of the ServerSocket class

#ifndef ServerSocket_class
#define ServerSocket_class

#include "Socket.h"


class ServerSocket : private Socket
{
 public:

  ServerSocket ( int port );
  ServerSocket () : sendAll(false) {};
  virtual ~ServerSocket();

  const ServerSocket& operator << ( const SocketData &data ) const;
  const ServerSocket& send ( const SocketData &data ) const;

  const ServerSocket& operator >> ( SocketData &data ) const;
  const ServerSocket& recv ( SocketData &data ) const;

  void accept ( ServerSocket& );
  bool isValid() const;
  void setSendAll ( bool sendAll );


 private:

  const ServerSocket& send_all ( const SocketData &data ) const;

  bool sendAll;

};


#endif
