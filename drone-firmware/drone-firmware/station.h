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


class Station : public Thread
{

    static const int STATION_PORT = 6666;
    static const int DRONE_STATION_BB_SEND_INTERVAL = 50;

  public:
    Station():
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
        Util::log("Station", "Failed to create socket");
        _sock = -1;
        return;
      }

    }

    bool tryConnect()
    {
      //Util::log("Station", "Attempting connection", Util::LS_INFO);
      if(_sock < 0)
        createSocket();

      if(_sock < 0)
        return _connected = false;
      
      struct sockaddr_in stationServer;
      memset(&stationServer, 0, sizeof(stationServer));

      stationServer.sin_family = AF_INET;                  /* Internet/IP */
      stationServer.sin_addr.s_addr = inet_addr(DRONE_STATION_HOST);  /* IP address */
      stationServer.sin_port = htons(STATION_PORT);       /* server port */

      /* Establish connection */
      if (connect(_sock, (struct sockaddr *) &stationServer, sizeof(stationServer)) < 0)
      {
        //Util::log("Station", "Failed to connect", Util::LS_FLOOD);
        _connected = false;
        return false;
      }
      
//      int flags = fcntl(_sock, F_GETFL, 0);
//      if(fcntl(_sock, F_SETFL, flags | O_NONBLOCK))
//        Util::log("Station", "Could not make socket nonblocking");

      //Util::log("Station", "Connected");
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
              Util::log("Station", "Send buffer too full");
          } else if (errno == EPIPE){
              Util::log("Station", "Connection dropped");
          } else
              Util::log("Station", "Failed to write to socket (connection drop?)");
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
          //Util::log("Station", "Connection attempt");
          return;
      }
        
      if(_connected)
      {
        char prepended[1 + sizeof(Blackboard)];
        prepended[0] = '@';
        memcpy(prepended+1, &BB, sizeof(Blackboard));
        if(!write(prepended, sizeof(Blackboard) + 1)) goto cleanup;

        std::string in;

        while(readLine(_sock, &in))
        {
          queueMessage(in);
        }

        while(_logs.size() > 0)
        {
          std::string str = _logs.front();
          _logs.pop();
          
          char c = '$';
          if(!write(&c, 1)) goto cleanup;
          if(!write(str.c_str(), str.size())) goto cleanup;
          
          c = '\n';
          if(str.find(c) == std::string::npos)
            if(!write(&c, 1)) goto cleanup;

        }
        
      }
      
      cleanup:
      _mutex.unlock();
    
    }


    typedef std::queue<std::string> MessageQueue;
    
    void queueMessage(std::string msg)
    {
      _msgs.push(msg);
    }
    
    std::string unqueueMessage()
    {
      std::string ret;
      
      if(_msgs.size())
      {
        ret = _msgs.front();
        _msgs.pop();
      }
      
      return ret;
    }
    
    bool hasMessages()
    {
      return _msgs.size() > 0;
    }

    void queueLog(std::string l)
    {
      _logs.push(l);
    }

  private:

    int _sock;
    volatile bool _connected;
    int _ticks;
    MessageQueue _msgs;
    MessageQueue _logs;
    Mutex _mutex;
};

