#ifndef __SERVER_SOCKET_WRAPPER__
#define __SERVER_SOCKET_WRAPPER__


#ifdef __cplusplus
#define CALL extern "C"
#else
#define CALL
#endif


CALL void *serverSocketInit    ( int port );

CALL void serverSocketAccept ( void *ptr_server_socket_instance );

CALL void serverSocketSend( void *ptr_server_socket_instance, const void *ptr_data_instance );
CALL void serverSocketRecv( void *ptr_server_socket_instance, void* ptr_data_instance );

CALL void serverSocketDestroy  (void *ptr_server_socket_instance);


#endif
