#ifndef SOCKETDATA_H
#define SOCKETDATA_H

#include <stdint.h>

class SocketData
{
    public:
        SocketData();
        SocketData( const void *in_ptr_data, const uint32_t &in_num_data_char );
        virtual ~SocketData();

        void setData( const void *in_ptr_data, const uint32_t &in_num_data_char );
        inline void setNumDataCharRecv( const uint32_t &in_num_data_char_recv ) { num_data_char_recv = in_num_data_char_recv; };
        void enableDataPrealloc( const uint32_t &total_num_data_char );
        void disableDataPrealloc();

        inline const char* getPtrData(  ) const { return ptr_data; }
        inline uint32_t getNumDataChar(  ) const { return num_data_char; }
        inline uint32_t getNumDataCharRecv(  ) const { return num_data_char_recv; }

        void clearData(  );

    protected:
    private:
        char        *ptr_data;
        uint32_t    num_data_char;
        uint32_t    num_data_char_recv;
        bool        is_new_data;
        bool        prealloc_data_mode;
};

struct socketDataInstance
{
    SocketData *ptr_socket_data;
};

#endif // SOCKETDATA_H

