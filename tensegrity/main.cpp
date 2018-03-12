#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>

#include <limbo/limbo.hpp>

#include "tensegrity-controller.hpp"
#include "Tracking.hpp"

using namespace limbo;

//#define MAXSPEED 254

struct Params {
    // option for the Bayesian optimizer
    struct bayes_opt_boptimizer {
        BO_PARAM(double, noise, 0.5);
        BO_PARAM(int,hp_period,5);
    };

    struct stat_gp{
      BO_PARAM(int,bins,50);
    };
    // enable / disable the output
    struct bayes_opt_bobase {
      BO_PARAM(int, stats_enabled, true);
    };

    // options for the internal optimizer
    #ifdef USE_LIBCMAES
    struct opt_cmaes : public defaults::opt_cmaes { };
    #elif defined(USE_NLOPT)
    struct opt_nloptnograd : public defaults::opt_nloptnograd { };
    #else
    struct opt_gridsearch : public defaults::opt_gridsearch { };
    #endif

    // options for the initializer
    struct init_randomsampling {
        //BO_PARAM(int, samples, 10);
        BO_PARAM(int, samples, 10);// ogininal post-john:10
    };

    // options for the stopping criteria
    struct stop_maxiterations {
        BO_PARAM(int, iterations, 80);
    };
    struct kernel_exp
    {
      // how big the neighborhood is, from [0,1]
      BO_PARAM(double, sigma, 0.15);
    };

    struct mean_constant {
            ///@ingroup mean_defaults
            // the higher the value, relative to the
            // actual behavior, the more optimistic we are

            //JB says - keep it around average of random trials
            BO_PARAM(double, constant, 0.05);
        };

    struct acqui_ucb {
      /// @ingroup acqui_defaults
      // exploit --> smaller values
      // explore --> bigger values
      // (scaled relative to performance of Eval)
      BO_PARAM(double, alpha, 0.2);
        };

    // options for the hyper-parameter optimizer
    // (here we just take the default values)
    struct opt_rprop : public defaults::opt_rprop { };
    struct opt_parallelrepeater : public defaults::opt_parallelrepeater { };
};


namespace global
{
  Position pos;
  std::string serialPort("/dev/ttyUSB0");
}

/*
struct Position
{
  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  //    ROS_INFO("I heard (%f, %f, %f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) ;
//      std::cout<<"timestamp:"<<msg->header.stamp<<std::endl;
      pos(0) = msg->pose.position.x;
      pos(1) = msg->pose.position.y;
      pos(2) = msg->pose.position.z;
  }
  Eigen::Vector3d pos;
};
*/

struct Eval {
    static constexpr size_t dim_in = 3;
    static constexpr size_t dim_out = 1;

    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {


        //Create the tensegrity controller
        tensegrity::Tensegrity_Controller tens(global::serialPort);

        // Settup up the callback for the mocap streaming
      //  ros::NodeHandle n;
      //  Position pos;
      //  ros::Subscriber sub = n.subscribe("Voltaire/pose", 1000, &Position::callback, &pos);//chatterCallback);




        bool retry = 0;

        Eigen::VectorXd res(1);

        //std::string str;
        //std::cout<<"press any key then enter to start" << std::endl;
        //std::cin >> str;

        int MAX_TRIALS = 1;

        std::vector<double> distances;
        for (int trials = 0; trials < MAX_TRIALS; trials++)
        {

        do {
          retry = 0;
        std::cout << x.transpose() << std::endl;
          /* code */

        //Tracker t;

      //  ros::spinOnce();
      //Eigen::Vector3d start = tracker::GetPosition();
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>("Voltaire/pose", ros::Duration(1)) ;
      Eigen::Vector3d start = global::pos.pos;
      tf::Quaternion startQ = global::pos.q;

        // we have determined that negative motor values produce
        // distinct behaviors from postive ones in some cases
        // and so have expanded the range of values accordingly.

        tens.set_all_motor_speeds(x(0)*2*MAXSPEED-MAXSPEED,
                                  x(1)*2*MAXSPEED-MAXSPEED,
                                  x(2)*2*MAXSPEED-MAXSPEED);
        //
        // tens.set_motor_speed(0,x(0)*254-127);
        // tens.set_motor_speed(1,x(1)*254-127);
        // tens.set_motor_speed(2,x(2)*254-127);
        //
 //       std::cout<<" now we are waiting!"<<std::endl;
        int elapsed = 0;
        int sleeptime = 100000;
        int evaltime = 3000000;
        float yawmax = 1.0;


        int strikes = 0;
        int maxStrikes = 3;

        double roll = 0;
        double pitch = 0;
        double yaw = 0;

        while (elapsed < evaltime)
     {
          usleep(sleeptime);
          elapsed += sleeptime;
          tf::Quaternion endQ= global::pos.q;
          tf::Quaternion aInv = startQ.inverse();
          tf::Quaternion c = endQ * aInv;
          tf::Matrix3x3 m(c);
          m.getRPY(roll,pitch,yaw);
          if (fabs(yaw) > yawmax)
          {
            std::cout << yaw << "is over limit, strike #" <<strikes << std::endl;
            strikes++;
          }
          else{
            strikes--;
            strikes = std::max(strikes,0);
          }
          if (strikes > maxStrikes)
          {
            std::cout << yaw << " is past yaw limit, bailing after " << strikes << " strikes" << std::endl;
              break;
          }
        }
        std::cout << "Delta RPY: " << roll << " " << pitch << " " << yaw << std::endl;
        tens.stop_all_motors();


        // Eigen::Vector3d end = tracker::GetPosition();
         Eigen::Vector3d end = global::pos.pos;
      //  t.GetPosition();
        double d = (end - start).norm();
        //std::cout<<"new position:"<<pos.pos.transpose()<<std::endl;
         std::cout<<" distance:" << d << std::endl;

        res(0) = d;

        std::string str;
        std::cout<<"Press y to restart, x to zero value, any other key to continue" << std::endl;
        std::cin >> str;
        if (str == "y")
          retry = 1;
        else if (str == "x")
        {
          d = 0;
          res(0) = d;
        }

      }while (retry == 1);

      distances.push_back(res(0));

}
      double min_distance = *std::max_element(distances.begin(), distances.end());
      std::cout<<"Distances:";
      for (auto x : distances)
        std::cout<<x<< " ";
      std::cout<<std::endl;
      res(0) = min_distance;
        return res;
  }
};

#include <Eigen/Core>

namespace limbo {
    namespace mean {
        ///@ingroup mean
        ///Use the mean of the observation as a constant mean
        template <typename Params>
        struct Prior {
          typedef kernel::Exp<Params> kf_t;
          typedef mean::Constant<Params> mean_t;
          typedef model::GP<Params, kf_t, mean_t> model_t;

            Prior(int dim_out=1) : _gp(3, 1) {
              for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                  for (int k = 0; k < 2; ++k)
                  {
                    Eigen::VectorXd v(3);
                    v << i, j, k;
                    auto o = tools::make_vector(0.3);
                    _gp.add_sample(v, o, 0.05);
                  }
              Eigen::VectorXd v(3);
              v << 0.5, 0.5, 0.5;
              auto o = tools::make_vector(0);
              _gp.add_sample(v,o,0.05);
            }

            template <typename GP>
            Eigen::VectorXd operator()(const Eigen::VectorXd& v, const GP& gp) const
            {
                return _gp.mu(v);
            }
          protected:
            model_t _gp;
        };
    }
}


int main(int argc, char** argv) {
  srand(time(0));
  int opt;
  while ((opt = getopt(argc, argv, "p:")) != -1) {
    switch (opt) {
      case 'p':
      std::cout << "using port " << optarg;
      global::serialPort = std::string(optarg);
      break;
      default: /* '?' */
      fprintf(stderr, "Usage: %s [-p port]\n",
      argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  using stat_t =
  boost::fusion::vector<stat::ConsoleSummary<Params>,
  stat::Samples<Params>,
  stat::Observations<Params>,
  stat::GP<Params> >;

  typedef kernel::Exp<Params> kf_t;

  // CHANGE THIS
  // TOP IS NO PRIOR
  //typedef mean::Constant<Params> mean_t;
  typedef mean::Prior<Params> mean_t;


  typedef model::GP<Params, kf_t, mean_t> model_t;
  typedef acqui::UCB<Params, model_t> acqui_t;
  ros::init(argc, argv, "limbo_voltaire_main");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Voltaire/pose", 1000, &Position::callback, &global::pos);

  //CHANGE THIS
  //TOP IS NO PRIOR
  //bayes_opt::BOptimizer<Params, statsfun<stat_t>,modelfun<model_t>, acquifun<acqui_t>> boptimizer;
  bayes_opt::BOptimizer<Params, initfun<limbo::init::NoInit<Params>>, statsfun<stat_t>,modelfun<model_t>, acquifun<acqui_t>> boptimizer;
  boptimizer.optimize(Eval());
  std::cout << "Best sample: " << boptimizer.best_sample() << " - Best observation: " << boptimizer.best_observation()(0) << std::endl;
  return 0;

}
