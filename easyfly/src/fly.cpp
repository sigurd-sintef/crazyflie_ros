#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/Crazyflie.h>

/******************

MUST RUN IN ROOT!!!!!!!!!!!!!!


*************************/
// template <unsigned int num_flies>
// class Crazyflie_multi : public Crazyflie
// {
// public:
//   Crazyflie cf[num_flies];
//   Crazyflie_multi(const std::string uri[num_flies])
//   {

//   }
//   ~Crazyflie_multi();
  
// };


std::string g_uri;
class fly212
{
public:
  fly212(int argc, char **argv);
  void run(double frequency);
  std::string uri;
private:
  void joyCallback0(const sensor_msgs::Joy::ConstPtr& joy0);
  void joyCallback1(const sensor_msgs::Joy::ConstPtr& joy1);
  ros::NodeHandle nh;
  ros::Subscriber joy_sub0;
  ros::Subscriber joy_sub1;
//  int init_scan(int argc, char **argv);
//  int connect_get_param(int argc, char **argv);
  void iteration(const ros::TimerEvent& e);
  
  float pitch_sp[2];
  float roll_sp[2];
  float yaw_sp[2];
  float thrust_sp[2];
  std::vector<Crazyflie> v_cf;//Crazyflie container

};

fly212::fly212(int argc, char **argv)

{
  Crazyflie cf(g_uri);
  v_cf.push_back(cf);
  uri=g_uri;
  v_cf[0].requestParamToc();
  joy_sub0 = nh.subscribe<sensor_msgs::Joy>("/joygroup0/joy",5,&fly212::joyCallback0,this);
  joy_sub1 = nh.subscribe<sensor_msgs::Joy>("/joygroup1/joy",5,&fly212::joyCallback1,this);
}
void fly212::run(double frequency)
{
  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &fly212::iteration, this);
  ros::spin();
}
void fly212::iteration(const ros::TimerEvent& e)
{
  float dt = e.current_real.toSec() - e.last_real.toSec();
  printf("pitch: %f\t", pitch_sp[0]);
  printf("pitch: %f\n", pitch_sp[1]);

  printf("roll: %f\t", roll_sp[0]);
  printf("roll: %f\n", roll_sp[1]);

  printf("yaw: %f\t", yaw_sp[0]);
  printf("yaw: %f\n", yaw_sp[1]);

  printf("thrust: %f\t", thrust_sp[0]);
  printf("thrust: %f\n", thrust_sp[1]);

  v_cf[0].sendSetpoint(-roll_sp[0]*20, -pitch_sp[0]*20, -yaw_sp[0]*50, thrust_sp[0]*40000);
}
void fly212::joyCallback0(const sensor_msgs::Joy::ConstPtr& joy)
{
  pitch_sp[0] = joy->axes[5];
  roll_sp[0] = joy->axes[2];
  yaw_sp[0] = joy->axes[0];
  thrust_sp[0] = joy->axes[1];
  if(thrust_sp[0]<0)
    thrust_sp[0]=0;
}
void fly212::joyCallback1(const sensor_msgs::Joy::ConstPtr& joy)
{
  pitch_sp[1] = joy->axes[5];
  roll_sp[1] = joy->axes[2];
  yaw_sp[1] = joy->axes[0];
  thrust_sp[1] = joy->axes[1];
  if(thrust_sp[1]<0)
    thrust_sp[1]=0;
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
    ("uri", po::value<std::string>(&g_uri)->default_value(defaultUri), "unique ressource identifier")
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
  ros::init(argc, argv, "fly");
  int ret2 = connect_get_param(argc, argv);
  fly212 Fly212(argc, argv);
  Fly212.run(50);


  return 0;


}