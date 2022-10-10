#include "arm4dof_control/sts3215_hardware.hpp"

#define DEG2RAD(deg)    ((deg)*(M_PI/180.0))
#define RAD2DEG(rad)    ((rad)*(180.0/M_PI))

/**
 * STS3215ハードウェア コンストラクタ
 * @brief STS3215ハードウェアクラスを初期化する
 * @param id：サーボID
 * @param control_type：サーボ制御種別
 */
Sts3215Hardware::Sts3215Hardware(std::string dev_name, uint32_t baudrate, ros::NodeHandle handle)
: port_(dev_name, baudrate), manager_(this, handle)
{
  updt_time_  = ros::Time::now();
  updt_d_     = ros::Duration(0);
}

/**
 * STS3215ハードウェア デストラクタ
 * @brief STS3215ハードウェアクラスを解放する
 * @param なし
 */
Sts3215Hardware::~Sts3215Hardware()
{
}

/**
 * STS3215ハードウェア 初期化状態確認処理
 * @brief STS3215ハードウェアの初期化状態を返却する
 * @param なし
 * @return 初期化状態
 */
bool Sts3215Hardware::is_init(void)
{
  return port_.is_init(); // 通信ポート以外で失敗しないのでそのまま返却
}

/**
 * STS3215ハードウェア ジョイント情報登録処理
 * @brief 制御するジョイントを追加する
 * @param id：サーボID
 * @param type：ジョイント種別
 * @param name：ジョイント名称
 * @param reverse：回転方向の反転設定
 * @return 登録OK(true)
 * @return 登録NG(false)
 */
bool Sts3215Hardware::regist_joint( uint8_t id, enSts3215JointType type, std::string name, bool reverse )
{
  bool result = true;

  for(std::vector<Sts3215Joint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
    if( itr->get_id() == id ){
      result = false;         // IDの重複は認めない
    }
  }
  if( result ){
    joint_.push_back( Sts3215Joint( id, type, name, reverse ) );
    std::vector<Sts3215Joint>::iterator joint_itr = (joint_.end()-1); // 今登録したデータ

    joint_itr->set_pos( 0.0 );
    joint_itr->set_vel( 0.0 );
    joint_itr->set_eff( 0.0 );
  }
  return result;
}

/**
 * STS3215ハードウェア インタフェース登録処理
 * @brief インタフェース登録を実行し、ROSコントロール制御を開始する
 * @param なし
 * @note ジョイント登録が終わったら呼び出します
 */
void Sts3215Hardware::regist_interface(void)
{
  for(std::vector<Sts3215Joint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
    hardware_interface::JointStateHandle stat_handle(   itr->get_name(),
      itr->get_pos_addr(),
      itr->get_vel_addr(),
      itr->get_eff_addr() );
    stat_if_.registerHandle( stat_handle );
    hardware_interface::JointHandle joint_handle( stat_if_.getHandle(itr->get_name()), itr->get_cmd_addr() );
    switch( itr->get_type() ){
      case enSts3215JointType_Position:
        pos_if_.registerHandle( joint_handle );
        break;
      case enSts3215JointType_Velocity:
        vel_if_.registerHandle( joint_handle );
        break;
      case enSts3215JointType_Effort:
        eff_if_.registerHandle( joint_handle );
        break;
    }
  }
  registerInterface( &stat_if_ );
  registerInterface( &pos_if_ );
  registerInterface( &vel_if_ );
  registerInterface( &eff_if_ );
}

/**
 * STS3215ハードウェア コントロール更新処理
 * @brief サーボの読み書きを行いROSコントロール制御を更新する
 * @param なし
 */
void Sts3215Hardware::update( void )
{
  // Read Current
  for(std::vector<Sts3215Joint>::iterator ritr=joint_.begin() ; ritr!=joint_.end() ; ++ritr){
    port_.readCurrent(*ritr);
    ritr->set_pos( DEG2RAD(ritr->get_current_pos() / 100.0) );
    ritr->set_vel( DEG2RAD(ritr->get_current_vel() / 100.0) );
    ritr->set_eff( ritr->get_current_trq() / 1000.0 );
  }

  // Update
  ros::Time now = ros::Time::now();
  updt_d_ = (now - updt_time_);
  manager_.update(now,updt_d_);
  updt_time_ = now;

  // Write desired
  for(std::vector<Sts3215Joint>::iterator witr=joint_.begin() ; witr!=joint_.end() ; ++witr){
    int16_t desired;
    switch( witr->get_type() ){
      case enSts3215JointType_Position:
      case enSts3215JointType_Velocity:
        desired = static_cast<int16_t>(RAD2DEG(witr->get_cmd()) * 100.0); // rad->deg
        break;
      case enSts3215JointType_Effort:
        desired = static_cast<int16_t>(witr->get_cmd() * 1000.0);// Nm->mNm
        break;
    }
    witr->set_desired( desired );
    port_.writeDesired(*witr);
  }
  return;
}

/**
 * STS3215ハードウェア トルクON処理
 * @brief 登録されている全てのジョイントをトルクON状態にする
 * @param なし
 */
void Sts3215Hardware::torque_on(void)
{
  set_torque(true);
}

/**
 * STS3215ハードウェア トルクFREE処理
 * @brief 登録されている全てのジョイントのトルクをFREE状態にする
 * @param なし
 */
void Sts3215Hardware::torque_free(void)
{
  set_torque(false);
}

/**
 * STS3215ハードウェア ジョイントトルク設定処理
 * @brief 登録されている全てのジョイントのトルクを設定する
 * @param torque：設定状態
 */
void Sts3215Hardware::set_torque(bool torque)
{
  EN_OPTIONS value = torque?enOptions_TorqueEnable:enOptions_TorqueDisable;
  for(std::vector<Sts3215Joint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
    itr->set_run_state( value );
    port_.writeRunMode(*itr);
  }
}

/**
 * STS3215ハードウェア リセット処理
 * @brief 登録されている全てのジョイントをリセットする
 * @param なし
 */
// void Sts3215Hardware::reset(void)
// {
//   for(std::vector<Sts3215Joint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
//     port_.reset(*itr);
//   }
// }
