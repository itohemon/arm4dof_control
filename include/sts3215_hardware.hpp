#ifndef STS3215_HARDWARE_HPP_
#define STS3215_HARDWARE_HPP_

#include <stdint.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "sts3215_design.hpp"
#include "sts3215_joint.hpp"
#include "sts3215_port.hpp"

/**
 * STS3215ハードウェアインタフェースクラス
 */
class Sts3215Hardware : public hardware_interface::RobotHW
{
public:
  Sts3215Hardware( std::string dev_name, uint32_t baudrate, ros::NodeHandle handle );
  ~Sts3215Hardware();
  bool is_init(void);
  bool regist_joint( uint8_t id, enSts3215JointType type, std::string name, bool reverse );
  void regist_interface(void);
  void update(void);
  void torque_on(void);
  void torque_free(void);
  void set_torque(bool torque);
  // void reset(void);

private:
  Sts3215Port                                 port_;      // 通信ポート情報
  std::vector<Sts3215Joint>                   joint_;     // ジョイント情報

  ros::Time                                   updt_time_; // アップデート時刻
  ros::Duration                               updt_d_;    // アップデート間隔

  controller_manager::ControllerManager       manager_;   // コントローラマネージャ
  hardware_interface::JointStateInterface     stat_if_;   // ステータスインタフェース
  hardware_interface::PositionJointInterface  pos_if_;    // 位置指令インタフェース
  hardware_interface::VelocityJointInterface  vel_if_;    // 速度指令インタフェース
  hardware_interface::EffortJointInterface    eff_if_;    // トルク指令インタフェース
};

#endif  // STS3215_HARDWARE_HPP_
