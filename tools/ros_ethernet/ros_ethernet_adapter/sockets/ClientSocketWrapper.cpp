#include "ClientSocketWrapper.h"
#include "ClientSocket.h"
#include "SocketException.h"

struct clientSocketInstance
{
    ClientSocket *ptr_client_socket;
};

CALL void *clientSocketInit( const std::string& ipaddr, int port )
{

    clientSocketInstance *ptr_client_socket_instance    = new clientSocketInstance;
    ptr_client_socket_instance->ptr_client_socket       = new ClientSocket( ipaddr, port );

    return (void *)ptr_client_socket_instance;
}

CALL void clientSocketSend( void *ptr_client_socket_instance, const void *ptr_data_instance )
{
    if (ptr_client_socket_instance) {
        ClientSocket* socket = ((clientSocketInstance *)ptr_client_socket_instance)->ptr_client_socket;
        if (socket) socket->send( *(((socketDataInstance *) ptr_data_instance)->ptr_socket_data) );
    }
}

CALL void clientSocketRecv( void *ptr_client_socket_instance, void* ptr_data_instance )
{
    if (ptr_client_socket_instance) {
        ClientSocket* socket = ((clientSocketInstance *)ptr_client_socket_instance)->ptr_client_socket;
        if (socket) socket->recv( *(((socketDataInstance *) ptr_data_instance)->ptr_socket_data) );
    }
}

CALL void clientSocketDestroy(void *ptr_client_socket_instance)
{

    if (ptr_client_socket_instance) {
        ClientSocket* socket = ((clientSocketInstance *)ptr_client_socket_instance)->ptr_client_socket;
        if (socket) delete (clientSocketInstance *)ptr_client_socket_instance;
    }
}
