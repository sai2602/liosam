#ifndef SOCKET_API_H_INCLUDED
#define SOCKET_API_H_INCLUDED

#define SOCKETS_ON 1

#ifdef SOCKETS_ON
#include "../definition.h"
#include "ServerSocketWrapper.h"
#include "SocketDataWrapper.h"

#endif // SOCKETS_ON

void *serverSockInst; // socket instance
void *socketDatInst; // socket data instance

#endif // SOCKET_API_H_INCLUDED
