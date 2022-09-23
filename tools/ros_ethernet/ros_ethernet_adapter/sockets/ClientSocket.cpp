// Implementation of the ClientSocket class

#include "ClientSocket.h"
#include "SocketException.h"


ClientSocket::ClientSocket ( std::string host, int port ) : recvAll(false)
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "Could not create client socket. \n" );
    }

  if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "Could not bind to port. \n" );
    }

}


ClientSocket::~ClientSocket()
{
}


const ClientSocket& ClientSocket::send ( const SocketData &data ) const
{
  if ( !(Socket::send ( data ) > 0) )
    {
      throw SocketException ( "Could not write to socket. \n" );
    }

  return *this;

}

const ClientSocket& ClientSocket::operator << ( const SocketData &data ) const
{
  if ( !(Socket::send ( data ) > 0) )
    {
      throw SocketException ( "Could not write to socket. \n" );
    }

  return *this;

}

const ClientSocket& ClientSocket::recv ( SocketData &data ) const
{
  if (!recvAll)
  {
    if ( !(Socket::recv ( data ) > 0) )
    {
      throw SocketException ( "Could not read from socket. \n" );
    }
  } else {
    if ( !(Socket::recv_all ( data, data.getNumDataCharRecv() ) > 0) )
    {
      throw SocketException ( "Could not read from socket. \n" );
    }
  }

  return *this;
}

const ClientSocket& ClientSocket::recv_all ( SocketData &data ) const
{
  if ( !(Socket::recv_all ( data, data.getNumDataCharRecv() ) > 0) )
    {
      throw SocketException ( "Could not read from socket. \n" );
    }

  return *this;
}

const ClientSocket& ClientSocket::operator >> ( SocketData &data ) const
{
  if (!recvAll)
  {
    if ( !(Socket::recv ( data ) > 0) )
    {
      throw SocketException ( "Could not read from socket. \n" );
    }
  } else {
    if ( !(Socket::recv_all ( data, data.getNumDataCharRecv() ) > 0) )
    {
      throw SocketException ( "Could not read from socket. \n" );
    }
  }

  return *this;
}

void ClientSocket::setRecvAll ( bool recvAll )
{

    this->recvAll = recvAll;

}
