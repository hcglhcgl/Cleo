//<tcpCase.cpp>
#include "tcpCase.h"


using namespace std;

#define MAXDATASIZE 100

tcpCase::tcpCase()
{
  portStr = "";
  addrStr = "";
  connected = false;
  servinfo = NULL;
}

void tcpCase::createSocket(const char * port, const char * addr)
{
  portStr = port;
  addrStr = addr;
    //Ensure that servinfo is clear
  memset(&hints, 0, sizeof hints); // make sure the struct is empty
  //setup hints
  hints.ai_family = AF_UNSPEC; // don't care IPv4 or IPv6
  hints.ai_socktype = SOCK_STREAM; // TCP stream sockets

  //Setup the structs if error print why
  int res;
  if ((res = getaddrinfo(addrStr,portStr,&hints,&servinfo)) != 0)
  {
    fprintf(stderr,"getaddrinfo: %s\n", gai_strerror(res));
  }
  //setup the socket
  if ((soc = socket(servinfo->ai_family,servinfo->ai_socktype,servinfo->ai_protocol)) == -1)
  {
    perror("client: socket");
  }
}

tcpCase::~tcpCase()
{
  if (connected)
  {
    connected = false;
    close(soc);
  }
  if (servinfo != NULL)
    freeaddrinfo(servinfo);  
  printf("Closed socket\n");
}

void tcpCase::tryConnect()
{//This goes into the send/rcv loop
  //Connect
  if (connect(soc,servinfo->ai_addr, servinfo->ai_addrlen) == -1)
  {
    close (soc);
    perror("Bridge connect failed");
    connected = false;
  }
  else
    connected = true;
}


int tcpCase::readChar(char * buf, bool * error)
{ // get one character into buffer or return 0 (or -1 on error)
  int numbytes;
  *error = false;
  numbytes = recv(soc, buf, 1, MSG_DONTWAIT);
  if (numbytes == -1)
  {
    if (errno != EAGAIN and errno != EWOULDBLOCK)
      *error = true;
    else
      numbytes = 0;
  }
//   else if (numbytes == 0)
//   { // connection closed from other end or no characters received
// //     printf("tcpCase::readChar: connection closed\n");
//   }
  return numbytes;
}


int tcpCase::sendData(const char *msg)
{//Send some data
  //Send some data
  int len = strlen(msg);
//   printf("tcpCase::sendData (len=%d): %s",len, msg); 
  int bytes_sent = send(soc,msg,len,0);
//   printf("tcpCase::sendData 9\n"); 
  return bytes_sent;
}

// void tcpCase::manageRecv(int numbytes, char * buf)
// {
// 
// 
// }

