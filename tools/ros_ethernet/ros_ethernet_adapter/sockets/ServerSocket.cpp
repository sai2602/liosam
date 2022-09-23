#include "ServerSocket.h"
#include "SocketException.h"

#include <stdio.h>


ServerSocket::ServerSocket ( int port ) : sendAll(false)
{

  bool setReusePortKeepAlive = false;

  if ( ! Socket::create( setReusePortKeepAlive ) )
    {
      throw SocketException ( "Could not create server socket. \n" );
    }

  if ( ! Socket::bind ( port ) )
    {
      throw SocketException ( "Could not bind to port. \n" );
    }

  if ( ! Socket::listen() )
    {
      throw SocketException ( "Could not listen to socket. \n" );
    }
}


ServerSocket::~ServerSocket()
{
}


const ServerSocket& ServerSocket::send ( const SocketData &data ) const
{
  if (!sendAll)
  {
    if ( !(Socket::send ( data ) > 0 ) )
    {
      throw SocketException ( "Could not write to socket. \n" );
    }
  } else {
    if ( !(Socket::send_all ( data, data.getNumDataChar() ) > 0 ) )
    {
      throw SocketException ( "Could not write to socket. \n" );
    }
  }

  return *this;

}

const ServerSocket& ServerSocket::send_all ( const SocketData &data ) const
{
  if ( !(Socket::send_all ( data, data.getNumDataChar() ) > 0 ) )
    {
      throw SocketException ( "Could not write to socket. \n" );
    }

  return *this;

}

const ServerSocket& ServerSocket::operator << ( const SocketData &data ) const
{
  if (!sendAll)
  {
    if ( !(Socket::send ( data ) > 0) )
    {
      throw SocketException ( "Could not write to socket. \n" );
    }
  } else {
    if ( !(Socket::send_all ( data, data.getNumDataChar() ) > 0) )
    {
      throw SocketException ( "Could not write to socket. \n" );
    }
  }

  return *this;

}


const ServerSocket& ServerSocket::recv ( SocketData &data ) const
{
  if ( !(Socket::recv ( data ) > 0) )
    {
      throw SocketException ( "Could not read from socket. \n" );
    }

  return *this;
}

const ServerSocket& ServerSocket::operator >> ( SocketData &data ) const
{
  if ( !(Socket::recv ( data ) > 0) )
    {
      throw SocketException ( "Could not read from socket. \n" );
    }

  return *this;
}

void ServerSocket::accept ( ServerSocket& sock )
{
  if ( ! Socket::accept ( sock ) )
    {
      throw SocketException ( "Could not accept socket. \n" );
    }
}

void ServerSocket::setSendAll ( bool sendAll )
{

    this->sendAll = sendAll;

}
