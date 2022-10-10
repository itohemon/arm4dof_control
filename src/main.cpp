#include <signal.h>
#include <stdint.h>

#include <sts3215_hardware.hpp>
#include <sts3215_config.hpp>

#define UPDATE_RATE_HZ (250)
#define STS3215_BAUDRATE (100000)

void SigintHandler(int sig)
{
  ROS_INFO("Shutdown!");
  ros::shutdown();
}

int main(int argc, char* argv[])
{
  // ノードの初期化
  ros::init(argc, argv, "arm4dof_control", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, SigintHandler);

  // 設定読み込み処理
  Sts3215Config config(nh);
  if (!config.load()) {
    ROS_ERROR("Config load failed.");
    return -1;
  }


  // STS3215ハードウェアインタフェースの初期化
  Sts3215Hardware sts3215(config.get_portname(), config.get_baudrate(), nh);
  if( !sts3215.is_init() ){
    ROS_ERROR("Port init failed.");
    return -1;
  }

  // ジョイント登録
  std::vector<stSts3215JointConfig> joints = config.get_joint_config();
  for(std::vector<stSts3215JointConfig>::iterator itr=joints.begin() ; itr!=joints.end() ; ++itr){
    uint8_t         reg_id;
    enSts3215JointType  reg_type;
    std::string     reg_name;
    bool            reg_reverse;
    reg_id = itr->id_;
    switch( itr->type_ ){
      case enSts3215JointConfigType_Velocity:
        reg_type = enSts3215JointType_Velocity;
        break;
      case enSts3215JointConfigType_Effort:
        reg_type = enSts3215JointType_Effort;
        break;
      case enSts3215JointConfigType_Position:
      default:
        reg_type = enSts3215JointType_Position;
        break;
    }
    reg_name = itr->name_;
    reg_reverse= itr->reverse_;
    sts3215.regist_joint(reg_id,reg_type,reg_name,reg_reverse); // 登録
  }
  sts3215.regist_interface();
  // sts3215.reset();

  /*    
  //トルクOn
  sts3215.torque_on();
  */
    
  // spin開始
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  // メインループ処理
  ros::Rate rate(UPDATE_RATE_HZ);
  while (ros::ok()) {
    //sts3215.update();   // 更新処理
    rate.sleep();
  }
  
  // 終了処理
  spinner.stop();
  ros::Duration(1).sleep();
  //sts3215.torque_free();

  return 0;
}
