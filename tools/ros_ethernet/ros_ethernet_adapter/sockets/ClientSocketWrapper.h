#ifndef __CLIENT_SOCKET_WRAPPER__
#define __CLIENT_SOCKET_WRAPPER__

#include <string>

#ifdef __cplusplus
#define CALL extern "C"
#else
#define CALL
#endif


CALL void *clientSocketInit    ( const std::string& ipaddr, int port );

CALL void clientSocketSend( void *ptr_client_socket_instance, const void *ptr_data_instance );
CALL void clientSocketRecv( void *ptr_client_socket_instance, void* ptr_data_instance );

CALL void clientSocketDestroy  (void *ptr_client_socket_instance);


#endif

