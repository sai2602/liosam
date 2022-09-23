// Implementation of the Socket class.


#include "Socket.h"
#include "string.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/tcp.h>    // required for option "TCP_KEEPIDLE" etc.


Socket::Socket() :
  m_sock ( -1 )
{

  memset ( &m_addr,
	   0,
	   sizeof ( m_addr ) );

}

Socket::~Socket()
{

  if ( is_valid() )
    ::close ( m_sock );

}

bool Socket::create( bool setReusePortKeepAlive )
{
  m_sock = socket ( AF_INET,
		    SOCK_STREAM,
		    0 );

  if ( ! is_valid() )
    return false;

  // Set options
  int on = 1;
  if ( setsockopt ( m_sock, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 )
    return false;

  if (setReusePortKeepAlive) {

    on = 1;
    if ( setsockopt ( m_sock, SOL_SOCKET, SO_REUSEPORT, ( const char* ) &on, sizeof ( on ) ) == -1 )
    return false;

    on = 1;
    if ( setsockopt ( m_sock, SOL_SOCKET, SO_KEEPALIVE, ( const char* ) &on, sizeof ( on ) ) == -1 )
    return false;

    int keepalive_time = 60;
    int keepalive_count = 3;
    int keepalive_interval = 10;
    if (setsockopt( m_sock, IPPROTO_TCP, TCP_KEEPIDLE, ( const char* ) &keepalive_time, sizeof ( keepalive_time ) ) == -1 )
    return false;
    if (setsockopt( m_sock, IPPROTO_TCP, TCP_KEEPCNT, ( const char* ) &keepalive_count, sizeof ( keepalive_count ) ) == -1 )
    return false;
    if (setsockopt( m_sock, IPPROTO_TCP, TCP_KEEPINTVL, ( const char* ) &keepalive_interval, sizeof ( keepalive_interval ) ) == -1 )
    return false;

  }

    // Check option status
    /*
    unsigned int sizeon = sizeof(on);
    if(getsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, ( char* ) &on, &sizeon ) == -1)
    return false;
    std::cout << "SO_REUSEADDR is " << on << std::endl;

    if(getsockopt(m_sock, SOL_SOCKET, SO_REUSEPORT, ( char* ) &on, &sizeon ) == -1)
    return false;
    std::cout << "SO_REUSEPORT is " << on << std::endl;

    if(getsockopt(m_sock, SOL_SOCKET, SO_KEEPALIVE, ( char* ) &on, &sizeon ) == -1)
    return false;
    std::cout << "SO_KEEPALIVE is " << on << std::endl;
    */

  return true;

}



bool Socket::bind ( const int port )
{

  if ( ! is_valid() )
    {
      return false;
    }


  m_addr.sin_family = AF_INET;
  m_addr.sin_addr.s_addr = INADDR_ANY;
  m_addr.sin_port = htons ( port );

  int bind_return = ::bind ( m_sock,
			     ( struct sockaddr * ) &m_addr,
			     sizeof ( m_addr ) );


  if ( bind_return == -1 )
    {
      return false;
    }

  return true;
}


bool Socket::listen() const
{
  if ( ! is_valid() )
    {
      return false;
    }

  int listen_return = ::listen ( m_sock, MAXCONNECTIONS );

  if ( listen_return == -1 )
    {
      return false;
    }

  return true;
}


bool Socket::accept ( Socket& new_socket ) const
{
  int addr_length = sizeof ( m_addr );

  new_socket.m_sock = ::accept ( m_sock, ( sockaddr * ) &m_addr, ( socklen_t * ) &addr_length );

  if ( new_socket.m_sock <= 0 ) {
    return false;
  } else {

   // Check option status
   /*
   int on = 1;
   unsigned int sizeon = sizeof(on);
   if(getsockopt(new_socket.m_sock, SOL_SOCKET, SO_REUSEADDR, ( char* ) &on, &sizeon ) == -1)
    return false;
   std::cout << "SO_REUSEADDR is " << on << std::endl;

   if(getsockopt(new_socket.m_sock, SOL_SOCKET, SO_REUSEPORT, ( char* ) &on, &sizeon ) == -1)
    return false;
   std::cout << "SO_REUSEPORT is " << on << std::endl;

   if(getsockopt(new_socket.m_sock, SOL_SOCKET, SO_KEEPALIVE, ( char* ) &on, &sizeon ) == -1)
    return false;
   std::cout << "SO_KEEPALIVE is " << on << std::endl;
   */

    return true;
  }
}


int Socket::send ( const SocketData &data ) const
{
  int num_bytes_send = ::send ( m_sock, data.getPtrData(), data.getNumDataChar(), MSG_NOSIGNAL );

  return num_bytes_send;

}

int Socket::send_all ( const SocketData &data, const int32_t num_bytes_to_send ) const
{

    int total_num_bytes_send = 0;

    while (total_num_bytes_send < num_bytes_to_send) {

        int num_bytes_send = ::send ( m_sock, data.getPtrData(), data.getNumDataChar(), MSG_NOSIGNAL );

        if ( num_bytes_send == -1) {

//          std::cout << "num_bytes_send == -1   errno == " << errno << "  in Socket::send\n";
            return -1;

            /*
            // 1st retry
            int num_retry = 0;
            bool data_send = false;

            while (num_retry < 10 && data_send == false)
            {
                usleep(1000000);

                int num_bytes_send = ::send ( m_sock, data.getPtrData(), data.getNumDataChar(), MSG_NOSIGNAL );

                if ( num_bytes_send >= 0) {
                    total_num_bytes_send += num_bytes_send;
                    data_send = true;
                }

                num_retry++;
                std::cout << "send retry " << num_retry << std::endl;
            }

            if (data_send == false) {
                std::cout << "num_bytes_send == -1   errno == " << errno << "  in Socket::send\n";
                return -1;
            }
            */

        } else {
            total_num_bytes_send += num_bytes_send;
        }
    }

    return total_num_bytes_send;

}

int Socket::recv ( SocketData &data ) const
{
    char buf [ MAXRECV + 1 ];

    memset ( buf, 0, MAXRECV + 1 );

    int num_bytes_recvd = ::recv ( m_sock, buf, MAXRECV, 0 );

    if ( num_bytes_recvd == -1 ){
//        std::cout << "num_bytes_recvd == -1   errno == " << errno << "  in Socket::recv\n";
        return -1;
    } else if ( num_bytes_recvd == 0 ) {
        return 0;
    } else {

        data.setData(buf, num_bytes_recvd);

        return num_bytes_recvd;
    }
}


int Socket::recv_all ( SocketData &data, const int32_t num_bytes_to_read ) const
{

    data.enableDataPrealloc( num_bytes_to_read );

    int total_num_bytes_recvd = 0;

    while (total_num_bytes_recvd < num_bytes_to_read) {

        int num_bytes_recvd = Socket::recv( data );

        if ( num_bytes_recvd == -1) {
//            std::cout << "num_bytes_recvd == -1   errno == " << errno << "  in Socket::recv\n";
            return -1;
        } else {
            total_num_bytes_recvd += num_bytes_recvd;
        }

//        std::cout << "Socket::recv_all   total_num_bytes_recvd: " << total_num_bytes_recvd << std::endl;

    }

    data.disableDataPrealloc();

    return total_num_bytes_recvd;

}


bool Socket::connect ( const std::string host, const int port )
{
  if ( ! is_valid() ) return false;

  m_addr.sin_family = AF_INET;
  m_addr.sin_port = htons ( port );

  int status = inet_pton ( AF_INET, host.c_str(), &m_addr.sin_addr );

  if ( errno == EAFNOSUPPORT ) return false;

  status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );

  if ( status == 0 )
    return true;
  else
    return false;
}

void Socket::set_non_blocking ( const bool b )
{

  int opts;

  opts = fcntl ( m_sock,
		 F_GETFL );

  if ( opts < 0 )
    {
      return;
    }

  if ( b )
    opts = ( opts | O_NONBLOCK );
  else
    opts = ( opts & ~O_NONBLOCK );

  fcntl ( m_sock,
	  F_SETFL,opts );

}
