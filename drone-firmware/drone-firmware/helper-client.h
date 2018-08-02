#pragma once

/**
   Includes socket example code from http://gnosis.cx/publish/programming/sockets.html
*/

#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <string>
#include <queue>
#include <algorithm>

#include "threading/thread.h"
#include "threading/mutex.h"

class HelperClient : public Thread
{

  public:
    HelperClient():
      _sock(-1),
      _connected(false),
      _ticks(0)
    {
      //stop broken connections from bringing the app down
      signal(SIGPIPE, SIG_IGN);
    }
    
    int main()
    {
        while(true)
        {
            usleep(1000ul * DRONE_CONNECTION_ATTEMPT_PERIOD_MS);  
            if(!_connected)
            {
                _mutex.lock();
                tryConnect();
                _mutex.unlock();
            }
        }
        return 0;
    }

    void createSocket()
    {
      if ((_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        Util::log("Helper", "Failed to create socket");
        _sock = -1;
        return;
      }

    }

    bool tryConnect()
    {
      //Util::log("Helper", "Attempting connection", Util::LS_INFO);
      if(_sock < 0)
        createSocket();

      if(_sock < 0)
        return _connected = false;
      
      struct sockaddr_in stationServer;
      memset(&stationServer, 0, sizeof(stationServer));

      stationServer.sin_family = AF_INET;                  /* Internet/IP */
      stationServer.sin_addr.s_addr = inet_addr(DRONE_HELPER_HOST);  /* IP address */
      stationServer.sin_port = htons(DRONE_HELPER_PORT);       /* server port */

      /* Establish connection */
      if (connect(_sock, (struct sockaddr *) &stationServer, sizeof(stationServer)) < 0)
      {
       //Util::log("Helper", "Failed to connect", Util::LS_INFO);
        _connected = false;
        return false;
      }
      
//      int flags = fcntl(_sock, F_GETFL, 0);
//      if(fcntl(_sock, F_SETFL, flags | O_NONBLOCK))
//        Util::log("Helper", "Could not make socket nonblocking");

      //Util::log("Helper", "Connected");
      return _connected = true;
    }

    bool connected() const {
      return _connected;
    }


    bool readLine(int fd, std::string* line)
    {

      static std::string buf;
      
      int avail = 0;
      ioctl(fd,FIONREAD,&avail);
      
      while(avail)
      {
        char inC;
        read(fd, &inC, 1);
        avail--;

        if(inC == '\n')
        {
          (*line) = buf;
          buf.clear();
          return true;
        }
        
        buf.push_back(inC);
      }

      return false;
      
    }

    bool write(const void * data, size_t s)
    {
      int ret = send(_sock, data, s, MSG_NOSIGNAL);
      if(ret == -1)
      {

          if (errno == EWOULDBLOCK){
              Util::log("Helper", "Send buffer too full");
          } else if (errno == EPIPE){
              Util::log("Helper", "Connection dropped");
          } else
              Util::log("Helper", "Failed to write to socket (connection drop?)");
          _connected = false;
          close(_sock);
          _sock = -1;
      }

      return _connected;
    }

    void tick()
    {  
        
      if(!_mutex.tryLock()) // maybe trying to connect
      {
          Util::log("Helper", "Connection attempt");
          return;
      }
        
      if(_connected)
      {
        
        std::string in;

        while(readLine(_sock, &in))
        {
            processLine(in);
        }
        
      }
      
      cleanup:
      _mutex.unlock();
    
    }


    typedef std::queue<std::string> MessageQueue;
    
    
    void processLine(std::string line)
    {
        stringvec_t tokens;
        Util::split(line, ' ', tokens);
        
        BB.nav.gpsHasFix = Util::parseInt(tokens[0]) != 0;
        BB.nav.gpsLon = Util::parseDouble(tokens[1]);
        BB.nav.gpsLat = Util::parseDouble(tokens[2]);
        
        double heading = -Util::parseDouble(tokens[3]);
        heading += 270;
        
        
        while(heading < 0)
            heading += 360.0f;
        
        while(heading > 360)
            heading -= 360.0f;
        
        BB.sensors.imuHeading = heading;
    }


  private:

    int _sock;
    volatile bool _connected;
    int _ticks;
    Mutex _mutex;
};

