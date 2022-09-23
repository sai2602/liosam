
#ifndef __SOCKET_DATA_WRAPPER__
#define __SOCKET_DATA_WRAPPER__

#ifdef __cplusplus
#define CALL extern "C"
#else
#define CALL
#endif



CALL void *socketDataInit    ();

CALL void socketDataSetData( void *ptr_socket_data_instance, const void *in_ptr_data, const int in_num_data_char );
CALL void socketDataClearData( void *ptr_socket_data_instance );

CALL const char* socketDataGetDataPtr( void *ptr_socket_data_instance );
CALL int socketDataGetNumDataChar( void *ptr_socket_data_instance );

CALL void socketDataDestroy  (void *ptr_socket_data_instance);


#endif
