#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <vector>
#include <string>
#include <unistd.h>

/* Standard speeds from termios.h
#define B0      0
#define B50     50
#define B75     75
#define B110    110
#define B134    134
#define B150    150
#define B200    200
#define B300    300
#define B600    600
#define B1200   1200
#define B1800   1800
#define B2400   2400
#define B4800   4800
#define B9600   9600
#define B19200  19200
#define B38400  38400
#if !defined(_POSIX_C_SOURCE) || defined(_DARWIN_C_SOURCE)
#define B7200   7200
#define B14400  14400
#define B28800  28800
#define B57600  57600
#define B76800  76800
#define B115200 115200
#define B230400 230400
#define EXTA    19200
#define EXTB    38400
#endif
*/



namespace serial
{
  typedef unsigned char byte_t;

  inline double get_time()
  {
    static struct timeval tv;
    gettimeofday(&tv, 0x0);
    return tv.tv_sec + tv.tv_usec * 1e-6;
  }


  // exception
  class Error
  {
  public:
    Error() {}
    Error(const char* msg) : _msg(msg) {}
    Error(const std::string& msg) : _msg(msg) {}
    const std::string& msg() const { return _msg; }
  private:
    std::string _msg;
  };

  class Serial
  {
  public:
    Serial(const std::string& name, int baudrate=B115200);
    // general send
    void send(const std::string& str) const
    {
      //std::cout<<"sending :"<<str<<std::endl;
      write(_fd, (byte_t*)str.c_str(), str.size());
    }
    // general receive
    bool recv(std::string& str, int size=100, float timeout = 10);
    byte_t* recv_buffer() { return _recv_buffer; }
  private:
    int _fd;
    static const size_t _recv_buffer_size = 256;
    byte_t _recv_buffer[_recv_buffer_size];
  };
}

#endif
