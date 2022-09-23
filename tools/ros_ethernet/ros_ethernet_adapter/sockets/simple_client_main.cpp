#include <string.h>
#include <iostream>
#include <string>

#include "ClientSocket.h"
#include "SocketException.h"
#include "definition.h"

int main ( int argc, int argv[] )
{
    sGpsAttitude att_send;
    att_send.fHeading    = 21.7;
    att_send.fPitch      = 12.16;

    try
    {

        ClientSocket client_socket ( "localhost", 30000 );

        std::string reply;
        SocketData data_send, data_recvd;

        char buf [MAXRECV];
        memcpy((void *) buf, (void* ) &att_send, sizeof(att_send));
        data_send.setData(buf, sizeof(att_send));

        try
        {
            client_socket << data_send;
            client_socket >> data_recvd;
        }
        catch ( SocketException& ) {}

        const sGpsAttitude *ptr_att_recvd = (sGpsAttitude *) data_recvd.getPtrData();
        std::cout << "Received heading: " << ptr_att_recvd->fHeading << ", Received pitch: " << ptr_att_recvd->fPitch << "\n";;

    }
    catch ( SocketException& e )
    {
        std::cout << "Exception was caught:" << e.description() << "\n";
    }

    return 0;
}
