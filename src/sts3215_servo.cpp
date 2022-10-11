#include <stdio.h>
#include "arm4dof_control/sts3215_servo.hpp"

/**
 * STS3215サーボ コンストラクタ
 * @brief STS3215サーボクラスを初期化する
 * @param id：サーボID
 * @param control_type：サーボ制御種別
 */
Sts3215Servo::Sts3215Servo(uint8_t id, EN_OPTIONS control_type)
{
  id_             = id;
  run_state_      = enOptions_TorqueDisable;

  switch(control_type){
  case enOptions_ControlPosition: // 位置指令
  case enOptions_ControlVelocity: // 速度指令
    control_type_   = control_type;
    break;
    // case enOptions_ControlTorque:   // トルク指令
  default:
    control_type_   = enOptions_ControlPosition;
    break;
  }
}

/**
 * STS3215サーボ デストラクタ
 * @brief STS3215サーボクラスを解放する
 * @param なし
 */
Sts3215Servo::~Sts3215Servo()
{
}

/**
 * STS3215サーボ ID取得処理
 * @brief ID設定を返却する
 * @param なし
 * @return サーボID
 */
uint8_t Sts3215Servo::get_id(void)
{
  return id_;
}

/**
 * STS3215サーボ 動作状態取得処理
 * @brief 動作状態を返却する
 * @param なし
 * @return 動作状態
 */
EN_OPTIONS Sts3215Servo::get_run_state(void)
{
  return run_state_;
}

/**
 * STS3215サーボ 制御タイプ取得処理
 * @brief 制御タイプをを返却する
 * @param なし
 * @return 制御タイプ
 */
EN_OPTIONS Sts3215Servo::get_control_type(void)
{
  return control_type_;
}

/* Private */
/**
 * STS3215サーボ RUN状態設定
 * @brief STS3215サーボのRUN状態を設定する
 * @param state：RUN状態
 */
void Sts3215Servo::set_run_state(EN_OPTIONS state)
{
  run_state_ = state;
  return;
}

/**
 * STS3215サーボ サーボ制御種別設定
 * @brief STS3215サーボの制御種別を設定する
 * @param control_type：制御種別
 */
void Sts3215Servo::set_control_type(EN_OPTIONS control_type)
{
  control_type_ = control_type;
  return;
}

/**
 * STS3215サーボ システムエラー設定
 * @brief STS3215サーボのシステムエラーを設定する
 * @param error：エラー
 */
void Sts3215Servo::set_sys_error(EN_SYS_ERRORS error)
{
  sys_error_ = error;
  return;
}

/**
 * STS3215サーボ モータエラー設定
 * @brief STS3215サーボのモータエラーを設定する
 * @param error：エラー
 */
void Sts3215Servo::set_motor_error(EN_MOTOR_ERRORS error)
{
  motor_error_ = error;
  return;
}

/**
 * STS3215サーボ UARTエラー設定
 * @brief STS3215サーボのUARTエラーを設定する
 * @param error：エラー
 */
void Sts3215Servo::set_uart_error(EN_UART_ERRORS error)
{
  uart_error_ = error;
  return;
}

/**
 * STS3215サーボ コマンドエラー設定
 * @brief STS3215サーボのコマンドエラーを設定する
 * @param error：エラー
 */
void Sts3215Servo::set_cmd_error(EN_COMMAND_ERRORS error)
{
  cmd_error_ = error;
  return;
}

/**
 * STS3215サーボ 現在位置設定処理
 * @brief STS3215サーボから読みだした現在位置を格納する
 * @param value：位置値[100倍deg]
 */
void Sts3215Servo::set_current_pos( int16_t value )
{
  current_pos_ = value;
  return;
}

/**
 * STS3215サーボ 現在速度設定処理
 * @brief STS3215サーボから読みだした現在速度を格納する
 * @param value：速度値[100倍deg/sec]
 */
void Sts3215Servo::set_current_vel( int16_t value )
{
  current_vel_ = value;
  return;
}

/**
 * STS3215サーボ 現在トルク設定処理
 * @brief STS3215サーボから読みだした現在トルクを格納する
 * @param value：トルク値[1000倍Nm]
 */
// void Sts3215Servo::set_current_trq( int16_t value )
// {
//     current_trq_ = value;
//     return;
// }

/**
 * STS3215サーボ 現在値パース処理
 * @brief STS3215サーボから読みだした現在値のバイト列を解析し格納する
 * @param data：解析する受信データ先頭アドレス
 * @param length：データ長
 */
void Sts3215Servo::parse_current_all( uint8_t* data, uint8_t length )
{
  int16_t value;
  uint8_t index;

  if( length==SIZE_SERVO_CURRENT_ALL ){
    index = 0;
    value = static_cast<int16_t>( (uint16_t)(data[index+1])<<8 | data[index]&0xFF );
    set_current_pos(value);
    index = (ADDR_PRESENT_VELOCITY - ADDR_SERVO_CURRENT_ALL);
    value = static_cast<int16_t>( (uint16_t)(data[index+1])<<8 | data[index]&0xFF );
    set_current_vel(value);
  }
  return;
}

/**
 * STS3215サーボ 目標値格納処理
 * @brief STS3215サーボへ書き込む制御目標値を格納する
 * @param value：目標値
 * @note 目標値は単位変換してから入力すること
 */
void Sts3215Servo::set_desired( int16_t value )
{
  int32_t max, min;

  switch( control_type_ ){
  case enOptions_ControlPosition: // 位置指令の場合
    max = MAX_POSITION;
    min = MIN_POSITION;
    break;
  case enOptions_ControlVelocity: // 速度指令の場合
    max = MAX_VELOCITY;
    min = MIN_VELOCITY;
    break;
    // case enOptions_ControlTorque:   // トルク指令の場合
    //     max = MAX_TORQUE;
    //     min = MIN_TORQUE;
    //     break;
  default:                        // その他の場合
    max = MAX_UNDEFINED;
    min = MIN_UNDEFINED;
    break;
  }
  if( value > max ){
    value = static_cast<int16_t>(max);
  }
  if( value < min ){
    value = static_cast<int16_t>(min);
  }
  desired_ = value;
  return;
}

/**
 * STS3215サーボ 現在位置取得処理
 * @brief 格納されている現在位置を返却する
 * @param なし
 * @return 位置値[100倍deg]
 */
int16_t Sts3215Servo::get_current_pos( void )
{
  return current_pos_;
}

/**
 * STS3215サーボ 現在速度取得処理
 * @brief 格納されている現在速度を返却する
 * @param なし
 * @return 位置値[100倍deg/sec]
 */
int16_t Sts3215Servo::get_current_vel( void )
{
  return current_vel_;
}

/**
 * STS3215サーボ 目標値取得処理
 * @brief 格納されている目標値を返却する
 * @param なし
 * @return 目標値
 */
int16_t Sts3215Servo::get_desired( void )
{
  return desired_;
}
