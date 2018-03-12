
#include <iostream>
#include <unistd.h>
#include <time.h>

#include "tensegrity-controller.hpp"
#include "Tracking.hpp"



int main(int argc, char** argv)
{

	srand(time(NULL));


	int opt;


	float inspd0 = 0.5;
	float inspd1 = 0.5;
	float inspd2 = 0.5;

	int spd0 = 0;
	int spd1 = 0;
	int spd2 = 0;

	int duration = 3; //seconds

	bool randomspeeds = 0;
	bool testmotors = 0;
	bool useTracking = 0;


	//std::cout << argv << std::endl;


	std::string portName("/dev/ttyUSB0");

  while ((opt = getopt(argc, argv, "p:1:2:3:d:rtT")) != -1) {
        switch (opt) {
					case 'p':
					      portName = std::string(optarg);
								break;
        case '1':
            inspd0= atof(optarg);
            break;
        case '2':
            inspd1 = atof(optarg);
            break;
        case '3':
            inspd2 = atof(optarg);
            break;
				case 'd':
						duration = atoi(optarg);
						break;
				case 'r':
						randomspeeds = 1;
						break;
				case 't':
						testmotors = 1;
						break;
				case 'T':
					useTracking = 1;
					//std::cout << "tracking" << std::endl;
					break;
        default: /* '?' */
            fprintf(stderr, "Usage: %s [-p serialPort] [-1 motor1speed] [-2 motor2speed] [-3 motor3speed] [-d duration] [-r]andom [-t]estmotors\n",
                    argv[0]);
            exit(EXIT_FAILURE);
        }
    }

	tensegrity::Tensegrity_Controller tens(portName);

	ros::init(argc, argv, "limbo_voltaire_test");	  
	Position pos;
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::AsyncSpinner spinner(4);
	
	if (useTracking)
	{
	  /// THIS
	  sub = n.subscribe("Voltaire/pose", 1000, &Position::callback, &pos);

	  spinner.start();	  
		//spinner.start();
	}
	
	if (testmotors)
	{

		tens.set_all_motor_speeds(40,0,0);
		sleep(2);
		tens.stop_all_motors();

		tens.set_all_motor_speeds(0,40,0);
		sleep(2);
		tens.stop_all_motors();

		tens.set_all_motor_speeds(0,0,40);
		sleep(2);
		tens.stop_all_motors();

	}
	else
	{
		if (!randomspeeds)
		{
			spd0 = 2*MAXSPEED*inspd0 - MAXSPEED;
			spd1 = 2*MAXSPEED*inspd1 - MAXSPEED;
			spd2 = 2*MAXSPEED*inspd2 - MAXSPEED;

			// spd0 = 254*inspd0 - 127;
			// spd1 = 254*inspd1 - 127;
			// spd2 = 254*inspd2 - 127;

			//std::cout << "***" << std::endl<< a << "," << spd0 <<","<<b<<","<<spd1<<","<<c<<","<<spd2<<std::endl;
		}
		else
		{
			spd0 = ((double)rand()/RAND_MAX)*2*MAXSPEED - MAXSPEED;
			spd1 = ((double)rand()/RAND_MAX)*2*MAXSPEED - MAXSPEED;
			spd2 = ((double)rand()/RAND_MAX)*2*MAXSPEED - MAXSPEED;
		}

		Eigen::Vector3d start, end;

		tf::Quaternion startQ, endQ;
		if (useTracking)
			{

				ros::topic::waitForMessage<geometry_msgs::PoseStamped>("Voltaire/pose", ros::Duration(1));
				//ros::spinOnce();

				// while(pos.once == 0)
				// {
				// 	start = pos.pos;// tracker::GetPosition();
				// 	std::cout << "startpos: " << start.transpose() << std::endl;
				// }
						start = pos.pos;// tracker::GetPosition();
						startQ = pos.q;
					std::cout << "startpos: " << start.transpose() << std::endl;

			}

		std::cout << spd0 << " " << spd1 << " "	 << spd2 << std::endl;

		tens.set_all_motor_speeds(spd0,spd1,spd2);
		// tens.set_motor_speed(0, spd0);
		// tens.set_motor_speed(1, spd1);
		// tens.set_motor_speed(2, spd2);
		sleep(duration);

		if (useTracking)
		{
			end = pos.pos;//tracker::GetPosition();
			endQ = pos.q;
		  double d = (end - start).norm();
        //std::cout<<"new position:"<<pos.pos.transpose()<<std::endl;A
			tf::Quaternion aInv = startQ.inverse();
			tf::Quaternion c = endQ * aInv;
			tf::Matrix3x3 m(c);
      double roll, pitch, yaw;
      m.getRPY(roll,pitch,yaw);
      std::cout << "Delta Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

      std::cout<< "start:" << start.transpose() << "end: " << end.transpose() << " distance:" << d << std::endl;

			//THIS
			 spinner.stop();
		}
}
/*
	for (int i = 0; i < 10; i++)
	{

		cout << spd0 << " " << spd1 << " " << spd2 ;
		tens.set_motor_speed(0, spd0);
		tens.set_motor_speed(1, spd0);
		tens.set_motor_speed(2, spd0);
		usleep(1000000);
	}
*/
		tens.stop_all_motors();

	//THIS
	     spinner.stop();


	/*
	std::cout << "hello world``!\n";
  serial::Serial sp("/dev/ttyUSB1",B19200);
  std::string str(5,0);
  str[0] = 0xAA;
  str[1] = 0x00;
  str[2] = 0x09;
  str[3] = 0x00;
  sp.send(str);
	return 0;
*/

}
