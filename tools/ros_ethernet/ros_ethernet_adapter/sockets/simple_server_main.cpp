#include <string>
#include <iostream>

#include "ServerSocket.h"
#include "SocketException.h"

int main ( int argc, int argv[] )
{
  std::cout << "running....\n";

  try
    {
      // Create the socket
      ServerSocket server ( 30000 );

        while ( true )
        {

          ServerSocket new_sock;
          server.accept ( new_sock );

          std::cout << "new socket accept \n";

          try
            {
                while ( true )
                {
                  SocketData data;
                  new_sock >> data;
                  new_sock << data;
                }
            }
          catch ( SocketException& ) {}

        }
    }
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

  return 0;
}
