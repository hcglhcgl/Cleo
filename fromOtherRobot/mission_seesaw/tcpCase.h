/**
 * Code from http://www.the-tech-tutorial.com/category/code-snippets/
 * 
 * Imported by Christian Andersen jca@elektro.dtu.dk 
 * */


//<tcpCase.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <time.h>


#ifndef TCPCASE_H_
#define TCPCASE_H_

class tcpCase
{
public:
  tcpCase();

  virtual ~tcpCase();

protected:
  /**
   * convert address and port from string to addr info,
   * and creates the socket (but no connect) */
  void createSocket(const char * port, const char * addr);
  /**
   * Connect to the socket created by createSocket */
  void tryConnect();
  /**
   * send string data 
   * \returns number of bytes send */
  int sendData(const char *msg);
  /**
   * Receive 1 character from socket 
   * \returns the character or 0 if no data */
  int readChar(char * buf, bool * socError);
  
public:
  bool connected;
protected:
  const char *addrStr;
  const char *portStr;
private:
  addrinfo hints, *servinfo;
  int soc; //the socket descriptor
  /**
   * A number of bytes are received */
//   void manageRecv(int numbytes, char * buf);
};

#endif /* TCPCASE_H_ */

