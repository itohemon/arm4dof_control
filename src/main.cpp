#include <signal.h>
#include <stdint.h>

#include <arm4dof_control/sts3215_config.hpp>

#define UPDATE_RATE_HZ (250)
#define STS3215_BAUDRATE (100000)

void SigintHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, ros::this_node::getName(), ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, SigintHandler);

  Sts3215Config config(nh);
  if (!config.load()) {
    ROS_ERROR("Config load failed.");

  //トルクOn
  //sts3215.torque_on();

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate r(UPDATE_RATE_HZ);
  while (ros::ok()) {
    //sts3215.update();
    r.sleep();
  }

  // 終了処理
  spinner.stop();
  ros::Duration(1).sleep();
  //sts3215.torque_free();

  return 0;
}
