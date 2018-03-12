#ifndef TENSEGRITY_HPP_
#define TENSEGRITY_HPP_


#include <string>
#include <iostream>

#include "serial.hpp"

#define QIK_START 0xAA
#define QIK_DEVICE_ID0 0x00
#define QIK_DEVICE_ID1 0x01
#define QIK_M0_FWD 0x09
#define QIK_M1_FWD 0x0C
#define QIK_M0_REV 0x0A
#define QIK_M1_REV 0x0E

#define SPINTIME 200000

#define MAXSPEED 127

namespace tensegrity
{
  class Tensegrity_Controller
  {
  public:
    Tensegrity_Controller(const std::string& portname) :
    serial_port(portname,B19200)
    {}



    std::string build_motor_cmd(int motorNum, char speed) const
    {
        std::string cmd_str(5,0);
        cmd_str[0] = QIK_START;
        char deviceID = motorNum < 2 ? QIK_DEVICE_ID0 : QIK_DEVICE_ID1;
        cmd_str[1] = deviceID;

        if (speed >= 0)
        {
           // Assume that controller 0 has motors 0 and 1
           // and         controller 1 has motor 2 (and 3)
           // so    even numbered motors are always motor 0 on that controller
            cmd_str[2] = (motorNum % 2 == 0) ? QIK_M0_FWD : QIK_M1_FWD;
        }
        else
        {
          cmd_str[2] = (motorNum % 2 == 0) ? QIK_M0_REV : QIK_M1_REV;
        }

        cmd_str[3] = abs(speed);

        return cmd_str;

    }

    void stop_all_motors() const
    {

        set_motor_speed(0,0);
        set_motor_speed(1,0);
        set_motor_speed(2,0);
    }

    //need to "spin up" for low speeds


    void spinup_motor(int motorNum, char speed) const
    {
      // for non-zero spin values we want
      // to first "spin" the motors
      // to overcome static friction/momentum
      // manual testing indicates that speeds below 15 don't make the motor move
      // but we'll do 12 just to be safe.
      if (abs(speed) > 12)
      {
        char spinspeed = (speed > 0) ? MAXSPEED : -MAXSPEED;
        std::string spinString = build_motor_cmd(motorNum,spinspeed);
        serial_port.send(spinString);
      }

    }
    void set_all_motor_speeds(int spd1, int spd2, int spd3) const
    {
      std::cout << "speed:" << spd1 << " " << spd2 << " " << spd3 << std::endl;
        spinup_motor(0,spd1);
        spinup_motor(1,spd2);
        spinup_motor(2,spd3);
        usleep(SPINTIME);
        set_motor_speed(0,spd1);
        set_motor_speed(1,spd2);
        set_motor_speed(2,spd3);

    }



  private:
    serial::Serial serial_port;
  ///  char motor0speed, motor1speed, motor2speed;
  void set_motor_speed(int motorNum, char speed) const
    {
      std::string sendString = build_motor_cmd(motorNum,speed);
      serial_port.send(sendString);
    }

  };
}

#endif
