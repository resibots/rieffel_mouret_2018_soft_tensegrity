#include <iostream>
#include <cstring>
#include <sstream>
#include <cstdlib>
#include <unistd.h>

#include "serial.hpp"

namespace serial
{
  Serial :: Serial(const std::string& name, int baudrate)
  {
   struct termios tio_serial;
   //std::cout<<"opening..."<<name<<std::endl;
    _fd = open(name.c_str(), O_RDWR | O_NOCTTY);
    if (_fd == -1)
      throw Error("error opening device:" + name + " ->" + std::string(strerror(errno)));

  //  std::cout << "serial port opened" << std::endl;

   // Serial port setting
   bzero(&tio_serial, sizeof(tio_serial));
   tio_serial.c_cflag = CS8 | CLOCAL | CREAD;
   tio_serial.c_iflag = IGNBRK | IGNPAR;
   tio_serial.c_oflag = 0;
   tio_serial.c_lflag = 0;

   // could be 1
   tio_serial.c_cc[VMIN] = 0;

   cfsetispeed(&tio_serial, baudrate);
   cfsetospeed(&tio_serial, baudrate);
   cfgetispeed(&tio_serial);
   tcflush(_fd, TCIFLUSH);
   tcsetattr(_fd, TCSANOW, &tio_serial);

  }

  bool Serial :: recv(std::string& str, int size, float timeout)
  {
    int p = 0;
    double time = get_time();
    bool done = false;
    memset(_recv_buffer, 0, _recv_buffer_size);
    do
      {
        double current_time = get_time();
        byte_t b;
        int res = read(_fd, &b, 1);
        if (res > 0)
          {
            _recv_buffer[p++] = b;
            time = current_time;
	          done = (p == size) || _recv_buffer[p - 1] == '\n';
          }
        if (current_time - time > timeout)
          done = true;
      }
    while (!done);
    str = std::string((char*)_recv_buffer);
    std::cout<<"recv:["<<str<<"]"<<std::endl;
    return true;
  }


}
