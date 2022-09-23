#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include "../definition.h"
#include "../ServerSocketWrapper.h"
#include "../SocketDataWrapper.h"

//#define DEBUG_SERVER_MAIN 1

void *serverSockInst; // socket instance
void *socketDatInst; // socket data instance

/* Signal Handler for SIGINT */
void sigintHandler(int sig_num)
{
    socketDataDestroy  ( socketDatInst );
    serverSocketDestroy( serverSockInst );

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

    printf("Server running ... \n");

    //while ( 1 )
    //{
        // TCP Socket initialization
        serverSockInst = serverSocketInit();
        socketDatInst  = socketDataInit();
        serverSocketAccept( serverSockInst );

        //printf("new socket accept \n");

        while ( 1 )
        {
            serverSocketRecv( serverSockInst, socketDatInst );
            serverSocketSend( serverSockInst, socketDatInst );

            //break;
#ifdef DEBUG_SERVER_MAIN
            printf( "ra: C_simple_TCP_server/main.c: recvd and sent something ... \n" );
#endif // DEBUG_SERVER_MAIN
        }

    return 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus
