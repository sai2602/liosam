#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include "../definition.h"
#include "../ClientSocketWrapper.h"
#include "../SocketDataWrapper.h"

void *clientSockInst; // socket instance
void *socketDatInst; // socket data instance

/* Signal Handler for SIGINT */
void sigintHandler(int sig_num)
{
    socketDataDestroy  ( socketDatInst );
    clientSocketDestroy( clientSockInst );

    printf("\n \n************************************ \n");
    printf("Cleaning up and exiting program ... \n");
    printf("************************************ \n");

    fflush(stdout);
    exit(0);
}

int main()
{

    /* Set the SIGINT (Ctrl-C) signal handler to sigintHandler
    Refer http://en.cppreference.com/w/c/program/signal */
    signal(SIGINT, sigintHandler);

    printf("Client running ... \n");

    // TCP Socket initialization
    clientSockInst = clientSocketInit();
    socketDatInst  = socketDataInit();

    // send data out to the GUI
    sGpsAttitude att_send;
    att_send.fHeading    = 233.7;
    att_send.fPitch      = 121.16;

    while ( 1 )
    {
        att_send.fHeading    = att_send.fHeading + 0.1;
        att_send.fPitch      = att_send.fPitch - 0.1;

        char buf [500];
        memcpy((void *) buf, &att_send, sizeof( att_send ));
        socketDataSetData( socketDatInst, buf, sizeof( att_send ) );

        clientSocketSend( clientSockInst, socketDatInst );
        clientSocketRecv( clientSockInst, socketDatInst );

        const sGpsAttitude *ptr_att_recvd = (sGpsAttitude *) socketDataGetDataPtr( socketDatInst );
        printf("Received heading: %lf \n", ptr_att_recvd->fHeading);
        printf("Received pitch: %lf \n", ptr_att_recvd->fPitch);
    }

    return 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus
