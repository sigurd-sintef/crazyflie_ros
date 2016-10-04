#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/Crazyflie.h>
/******************

MUST RUN IN ROOT!!!!!!!!!!!!!!


*************************/
float pitch_sp=0;
float roll_sp=0;
float yaw_sp=0;
float thrust_sp=0;

std::string uri;
void sticksCallback(const sensor_msgs::Joy msg)
{
  pitch_sp = msg.axes[5];
  roll_sp = msg.axes[2];
  yaw_sp = msg.axes[0];
  thrust_sp = msg.axes[1];
}
int init_scan(int argc, char **argv)
{

  std::string addressStr;
  std::string defaultAddressStr("0xE7E7E7E7E7");

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("address", po::value<std::string>(&addressStr)->default_value(defaultAddressStr), "device address")
  ;

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  }
  catch(po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  try
  {
    uint64_t address;
    std::stringstream sstr;
    sstr << std::hex << addressStr;
    sstr >> address;
//    printf("%d\n", address);
    Crazyradio radio(0);
    radio.setAddress(address);

    for (uint8_t datarate = 0; datarate < 3; ++datarate) {
      radio.setDatarate((Crazyradio::Datarate)datarate);
      for (uint8_t channel = 0; channel <= 125; ++channel) {
        radio.setChannel(channel);

        uint8_t test[] = {0xFF};
        Crazyradio::Ack ack;
        radio.sendPacket(test, sizeof(test), ack);
        //if there are flies powered on, following will be displayed
        if (ack.ack) {
          std::cout << "radio://0/" << (uint32_t)channel << "/";
          switch(datarate) {
          case Crazyradio::Datarate_250KPS:
            std::cout << "250K";
            break;
          case Crazyradio::Datarate_1MPS:
            std::cout << "1M";
            break;
          case Crazyradio::Datarate_2MPS:
            std::cout << "2M";
            break;
          }

          if (defaultAddressStr != addressStr) {
            std::cout << "/" << addressStr.substr(2);
          }
          std::cout << std::endl;
        }
      }
    }
    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
int connect_get_param(int argc, char **argv)
{
//  std::string uri;
  std::string defaultUri("radio://0/80/250K/E7E7E7E7E7");//2M changed to 250K

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("uri", po::value<std::string>(&uri)->default_value(defaultUri), "unique ressource identifier")
  ;

  try//without which the uri will not be valid
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  }
  catch(po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  

  //   return 0;
  // }
  // catch(std::exception& e)
  // {
  //   std::cerr << e.what() << std::endl;
  //   return 1;
  // }
}
int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
  int ret2=connect_get_param(argc, argv);
  Crazyflie cf(uri);
  cf.requestParamToc();




  ros::init(argc, argv, "fly");
  ros::NodeHandle n;
  ros::Subscriber commands_sub = n.subscribe("/joy",5,sticksCallback);
  ros::Rate loop_rate(50);

  int count = 0;
  while (ros::ok()){
    printf("pitch: %f\n", pitch_sp);
    printf("roll: %f\n", roll_sp);
    printf("yaw: %f\n", yaw_sp);
    printf("thrust: %f\n", thrust_sp);

    cf.sendSetpoint(-roll_sp*20, -pitch_sp*20, -yaw_sp*50, thrust_sp*40000);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;


}