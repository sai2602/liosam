#include "SocketDataWrapper.h"
#include "SocketData.h"

#include <stdio.h>

CALL void *socketDataInit    ()
{
    socketDataInstance *ptr_socket_data_instance    = new socketDataInstance;
    ptr_socket_data_instance->ptr_socket_data       = new SocketData();

    return (void *)ptr_socket_data_instance;
}

CALL void socketDataSetData( void *ptr_socket_data_instance, const void *in_ptr_data, const int in_num_data_char )
{
    if (ptr_socket_data_instance) {
        SocketData* data = ((socketDataInstance *)ptr_socket_data_instance)->ptr_socket_data;
        if (data) data->setData( in_ptr_data, in_num_data_char );
    }
}

CALL void socketDataClearData( void *ptr_socket_data_instance )
{
    if (ptr_socket_data_instance) {
        SocketData* data = ((socketDataInstance *)ptr_socket_data_instance)->ptr_socket_data;
        if (data) data->clearData();
    }
}

CALL const char* socketDataGetDataPtr( void *ptr_socket_data_instance )
{
    if (ptr_socket_data_instance) {
        SocketData* data = ((socketDataInstance *)ptr_socket_data_instance)->ptr_socket_data;
        if (data)
        {
            return data->getPtrData();
        }
    }

    return (const char*)NULL;
}

CALL int socketDataGetNumDataChar( void *ptr_socket_data_instance )
{
    if (ptr_socket_data_instance) {
        SocketData* data = ((socketDataInstance *)ptr_socket_data_instance)->ptr_socket_data;
        if (data)
        {
            return data->getNumDataChar();
        }
    }

    return -1;
}

CALL void socketDataDestroy  (void *ptr_socket_data_instance)
{
    if (ptr_socket_data_instance) {
        SocketData* data = ((socketDataInstance *)ptr_socket_data_instance)->ptr_socket_data;

        if (data)
        {
            delete (SocketData*) data;
            delete (socketDataInstance *)ptr_socket_data_instance;
        }

    }

}
