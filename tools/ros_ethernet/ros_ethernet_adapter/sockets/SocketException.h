// SocketException class


#ifndef SocketException_class
#define SocketException_class

#include <iostream>
#include <stdexcept>
#include <string>

using namespace std;

class SocketException : public exception
{
 public:
  SocketException ( string message )throw() {
      //cerr << "constructed" << endl;
      this->message = message;
  };
  virtual ~SocketException ()throw() {
      //cerr << "deleted" << endl;
  };

  virtual const char* what() const throw(){ return message.c_str(); }

 private:

  string message;

};

#endif
