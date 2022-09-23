#include "ServerSocketWrapper.h"
#include "ServerSocket.h"
#include "SocketException.h"

#include <stdio.h>
#include <iostream>

//#define DEBUG_SERVER_SOCKET_WRAPPER 1

struct serverSocketInstance
{
    ServerSocket *ptr_server_socket;
};

CALL void *serverSocketInit( int port )
{

    serverSocketInstance *ptr_server_socket_instance    = new serverSocketInstance;

    try
    {
        ptr_server_socket_instance->ptr_server_socket       = new ServerSocket( port );
    }
    catch ( SocketException& e )
    {
        std::cout << e.what();
        std::terminate();
    }

    return (void *)ptr_server_socket_instance;
}


CALL void serverSocketAccept ( void *ptr_server_socket_instance )
{
    try
    {
        if (ptr_server_socket_instance) {
            ServerSocket* socket = ((serverSocketInstance *)ptr_server_socket_instance)->ptr_server_socket;
            if (socket) socket->accept( *socket );
        }
    }
    catch ( SocketException& e )
    {
        std::cout << e.what();
        std::terminate();
    }

    //std::cout << "new socket accept \n";
}

CALL void serverSocketSend( void *ptr_server_socket_instance, const void *ptr_data_instance )
{
    try
    {
        if (ptr_server_socket_instance) {
            ServerSocket* socket = ((serverSocketInstance *)ptr_server_socket_instance)->ptr_server_socket;
            if (socket) socket->send( *(((socketDataInstance *) ptr_data_instance)->ptr_socket_data) );
        }
    }
    catch ( SocketException& e )
    {
        //std::cout << e.what();
    }

#ifdef DEBUG_SERVER_SOCKET_WRAPPER
    std::cout << "ra: ServerSocketWrapper.cpp: sent something ... \n";
#endif // DEBUG_SERVER_SOCKET_WRAPPER
}

CALL void serverSocketRecv( void *ptr_server_socket_instance, void* ptr_data_instance )
{
    try
    {
        if (ptr_server_socket_instance) {
            ServerSocket* socket = ((serverSocketInstance *)ptr_server_socket_instance)->ptr_server_socket;
            if (socket) socket->recv( *(((socketDataInstance *) ptr_data_instance)->ptr_socket_data) );
        }
    }
    catch ( SocketException& e )
    {
        //std::cout << e.what();
    }

#ifdef DEBUG_SERVER_SOCKET_WRAPPER
    std::cout << "ra: ServerSocketWrapper.cpp: recvd something ... \n";
#endif // DEBUG_SERVER_SOCKET_WRAPPER
}

CALL void serverSocketDestroy(void *ptr_server_socket_instance)
{

    if (ptr_server_socket_instance) {

        ServerSocket* socket = ((serverSocketInstance *)ptr_server_socket_instance)->ptr_server_socket;

        if (socket)
        {
#ifdef DEBUG_SERVER_SOCKET_WRAPPER
            std::cout << "ra: ServerSocketWrapper.cpp: before deleting serverSocketInstance ... \n";
#endif // DEBUG_SERVER_SOCKET_WRAPPER

            delete (ServerSocket *)socket;
            delete (serverSocketInstance *)ptr_server_socket_instance;

#ifdef DEBUG_SERVER_SOCKET_WRAPPER
            std::cout << "ra: ServerSocketWrapper.cpp: after deleting serverSocketInstance ... \n";
#endif // DEBUG_SERVER_SOCKET_WRAPPER

        }

    }

}
