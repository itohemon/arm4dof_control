#include <errno.h>

#include <ros/ros.h>

#include "arm4dof_control/sts3215_design.hpp"
#include "arm4dof_control/sts3215_port.hpp"
#include "arm4dof_control/sts3215_servo.hpp"

/**
 * STS3215通信ポート コンストラクタ
 * @brief STS3215通信ポートクラスを初期化する
 * @param dev_name：通信ポートのデバイスファイル名
 * @param baudrate：通信ポートのボーレート
 */
Sts3215Port::Sts3215Port(std::string dev_name, uint32_t baudrate)
: dev_name_(dev_name), baudrate_(baudrate)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(dev_name_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  init_result_ = initialize();
  if( !init_result_ ){
    // 初期化に失敗した場合
    printf("init NG\r\n");
  }
  return;
}

/**
 * STS3215通信ポート デストラクタ
 * @brief STS3215通信ポートクラスを解放する
 * @param なし
 */
Sts3215Port::~Sts3215Port()
{
  if( init_result_ ){
    // ポートクローズする
    portHandler_->closePort();
    init_result_ = false;
  }
  return;
}

/**
 * STS3215通信ポート 初期化状態確認処理
 * @brief STS3215通信ポートの初期化状態を返却する
 * @param なし
 * @return ポート初期化状態
 */
bool Sts3215Port::is_init(void)
{
  return init_result_;
}

/**
 * STS3215通信ポート 現在値読み込み処理
 * @brief STS3215へ現在値を要求する
 * @param servo：サーボデータ
 */
void Sts3215Port::readCurrent(Sts3215Servo &servo)
{
  uint8_t data[8];
  uint8_t err;
  int comm_result;

  comm_result = 
    packetHandler_->readTxRx(portHandler_,
			     servo.get_id(),
			     ADDR_SERVO_CURRENT_ALL,
			     SIZE_SERVO_CURRENT_ALL,
			     data,
			     &err
			     );

  if (comm_result != COMM_SUCCESS) {
    ROS_ERROR("read failed: %s", packetHandler_->getTxRxResult(comm_result));
    return;
  }
  else if (err != 0) {
    ROS_ERROR("get packet error: %s", packetHandler_->getRxPacketError(err));
    return;
  }
  
  servo.parse_current_all(data,SIZE_SERVO_CURRENT_ALL);
  
  return;
}

/**
 * STS3215通信ポート 指示値書き込み処理
 * @brief STS3215へ指示値を設定する
 * @param servo：サーボデータ
 * @note ControlTypeで書き込み先を変える
 */
void Sts3215Port::writeDesired(Sts3215Servo &servo)
{
  int16_t value = servo.get_desired();
  uint8_t address;
  uint8_t err = 0;
  int comm_result;
  uint16_t goalval;

  switch(servo.get_control_type()){
    case enOptions_ControlVelocity:     // 速度指令
      address = ADDR_GOAL_VELOCITY;
      break;
    // case enOptions_ControlTorque:       // トルク指令
      // address = ADDR_SERVO_DESIRED_TORQUE;
      // break;
    case enOptions_ControlPosition:     // 位置指令
    default:                            // それ以外の場合も位置
      address = ADDR_GOAL_POSITION;
      break;
  }

  // 負の値を設定するときは15ビット目を1にする（Feetech仕様)
  goalval = abs(value);
  if (value < 0) {
    goalval |= 1 << 15;
  }
  
  comm_result = 
    packetHandler_->write2ByteTxRx(portHandler_,
      servo.get_id(),
      address,
      goalval,
      &err);

  if (comm_result != COMM_SUCCESS) {
    ROS_ERROR("id : %d, Failed to write value(%d)", servo.get_id(), goalval);
  } else {
    ROS_DEBUG("id : %d, Succeed to write value(%d)", servo.get_id(), goalval);
  }

  return;
}

void Sts3215Port::writeRunMode(Sts3215Servo &servo)
{
  uint8_t err = 0;
  int comm_result;
  
  comm_result = 
    packetHandler_->write1ByteTxRx(portHandler_,
				   servo.get_id(),
				   ADDR_TORQUE_ENABLE,
				   servo.get_run_state(),
				   &err);

  if (comm_result != COMM_SUCCESS) {
    ROS_ERROR("id : %d, Failed to change torque change(%d)", servo.get_id(), servo.get_run_state());
  } else {
    ROS_DEBUG("id : %d, Succeed to change torque change(%d)", servo.get_id(), servo.get_run_state());
  }

  return;
}


/* Private */
/**
 * STS3215通信ポート 初期化処理
 * @brief STS3215通信ポートを初期化する
 * @param なし
 */
bool Sts3215Port::initialize(void)
{
  if (portHandler_->openPort()) {
    printf("Succeeded to open the port!\n");
  } else {
    printf("Failed to open the port!\n");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_)) {
    printf("Succeeded to change the baudrate!\n");
  } else {
    printf("Failed to change the baudrate!\n");
    return false;
  }

  return true;
}
