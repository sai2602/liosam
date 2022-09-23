#include "SocketData.h"

#include "SocketException.h"

#include <iostream>

SocketData::SocketData(): ptr_data(NULL), num_data_char(0), num_data_char_recv(0), is_new_data(false), prealloc_data_mode(false)
{
    //ctor
}

SocketData::SocketData( const void *in_ptr_data, const uint32_t &in_num_data_char ) : ptr_data(NULL), num_data_char(0), num_data_char_recv(0), is_new_data(false), prealloc_data_mode(false)
{

    setData( in_ptr_data, in_num_data_char );

}

SocketData::~SocketData()
{
    //dtor
    clearData();
}

void SocketData::enableDataPrealloc( const uint32_t &total_num_data_char )
{
    clearData();

    ptr_data = new char[total_num_data_char];

    prealloc_data_mode = true;
}

void SocketData::disableDataPrealloc()
{
    //clearData();

    prealloc_data_mode = false;
}


// Previous version of setData without option for data preallocation
/*
void SocketData::setData( const void *in_ptr_data, const uint32_t &in_num_data_char )
{
    clearData();

    if ( in_ptr_data == NULL )
    {
        throw SocketException ( "NULL pointer exception for socket data" );
    }

    ptr_data = new char[in_num_data_char];
    char *tmp_in_ptr_data = (char *) in_ptr_data;

    for (uint32_t i = 0; i < in_num_data_char; i++) {
        ptr_data[i] = tmp_in_ptr_data[i];
    }

    num_data_char   = in_num_data_char;
    is_new_data     = true;
}
*/

void SocketData::setData( const void *in_ptr_data, const uint32_t &in_num_data_char )
{
    if (!prealloc_data_mode) {
        clearData();
    }

    if ( in_ptr_data == NULL )
    {
        throw SocketException ( "NULL pointer exception for socket data" );
    }

    if (!prealloc_data_mode) {
        ptr_data = new char[in_num_data_char];
    }
    char *tmp_in_ptr_data = (char *) in_ptr_data;

    for (uint32_t i = 0; i < in_num_data_char; i++) {
        ptr_data[num_data_char + i] = tmp_in_ptr_data[i];
    }

    if (!prealloc_data_mode) {
        num_data_char   = in_num_data_char;
    } else {
        num_data_char   += in_num_data_char;
    }
    is_new_data     = true;
}

void SocketData::clearData(  )
{
    if( ptr_data != NULL ) {
        delete[] ptr_data;
        ptr_data = NULL;
    }

    num_data_char   = 0;
    is_new_data     = false;
    prealloc_data_mode  = false;
}
