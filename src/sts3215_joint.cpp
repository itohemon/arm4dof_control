#include <cstdio>
#include <cstring>
#include "arm4dof_control/sts3215_joint.hpp"

/**
 * STS3215ジョイント コンストラクタ
 * @brief STS3215ジョイントクラスを初期化する
 * @param id：サーボID
 * @param type：ジョイントタイプ
 * @param name：ジョイント名
 * @param reverse：回転方向の反転設定
 */
Sts3215Joint::Sts3215Joint( uint8_t id, enSts3215JointType type, std::string name, bool reverse )
 : Sts3215Servo( id, conv_type(type) )
{
    type_   = type;
    if( name.length() == 0 ){
        name = std::string("joint") + std::to_string(id);
    }
    name_   = name;
    reverse_= reverse;
    pos_    = 0.0;
    vel_    = 0.0;
    eff_    = 0.0;
    cmd_    = 0.0;
}

/**
 * STS3215ジョイント デストラクタ
 * @brief STS3215ジョイントクラスを解放する
 * @param なし
 */
Sts3215Joint::~Sts3215Joint()
{
}

/**
 * STS3215ジョイント ジョイント種別変換処理
 * @brief ジョイント種別をサーボクラスの定義へ変換する
 * @param type：ジョイント種別
 * @return サーボ制御タイプ
 */
EN_OPTIONS Sts3215Joint::conv_type( enSts3215JointType type )
{
    EN_OPTIONS result;

    switch( type ){
    case enSts3215JointType_Position:
        result = enOptions_ControlPosition;
        break;
    case enSts3215JointType_Velocity:
    case enSts3215JointType_Effort:
        throw "Unsupported joint type\n";
        break;
    default:
        throw "Illegal joint type\n";
        break;
    }
    return result;
}

/**
 * STS3215ジョイント 位置設定処理
 * @brief ジョイント位置を格納する
 * @param value：位置値[rad]
 */
void Sts3215Joint::set_pos(double value)
{
    pos_ = reverse_?-value:value;
}

/**
 * STS3215ジョイント 速度設定処理
 * @brief ジョイント速度を格納する
 * @param value：速度値[rad/sec]
 */
void Sts3215Joint::set_vel(double value)
{
    vel_ = reverse_?-value:value;
}

/**
 * STS3215ジョイント トルク設定処理
 * @brief ジョイントトルクを格納する
 * @param value：トルク値[Nm]
 */
void Sts3215Joint::set_eff(double value)
{
    eff_ = reverse_?-value:value;
}

/**
 * STS3215ジョイント ジョイントタイプ取得処理
 * @brief ジョイントタイプを返却する
 * @param なし
 * @return ジョイントタイプ
 */
enSts3215JointType Sts3215Joint::get_type(void)
{
    return type_;
}

/**
 * STS3215ジョイント ジョイント名取得処理
 * @brief ジョイント名を返却する
 * @param なし
 * @return ジョイント名
 */
std::string Sts3215Joint::get_name(void)
{
    return name_;
}

/**
 * STS3215ジョイント 位置取得処理
 * @brief ジョイント位置を返却する
 * @param なし
 * @return 位置値[rad]
 */
double Sts3215Joint::get_pos(void)
{
    return pos_;
}

/**
 * STS3215ジョイント 速度取得処理
 * @brief ジョイント速度を返却する
 * @param なし
 * @return 速度値[rad/sec]
 */
double Sts3215Joint::get_vel(void)
{
    return vel_;
}

/**
 * STS3215ジョイント トルク取得処理
 * @brief ジョイントトルクを返却する
 * @param なし
 * @return トルク値[Nm]
 */
double Sts3215Joint::get_eff(void)
{
    return eff_;
}

/**
 * STS3215ジョイント HWIfコマンド値取得処理
 * @brief ジョイントHWIfコマンド値を返却する
 * @param なし
 * @return コマンド値
 */
double Sts3215Joint::get_cmd(void)
{
    return reverse_?-cmd_:cmd_;
}

/**
 * STS3215ジョイント 位置データアドレス取得処理
 * @brief ジョイント位置格納アドレスを返却する
 * @param なし
 * @return 位置値格納アドレス
 */
double* Sts3215Joint::get_pos_addr(void)
{
    return &pos_;
}

/**
 * STS3215ジョイント 速度データアドレス取得処理
 * @brief ジョイント速度格納アドレスを返却する
 * @param なし
 * @return 速度値格納アドレス
 */
double* Sts3215Joint::get_vel_addr(void)
{
    return &vel_;
}

/**
 * STS3215ジョイント トルクデータアドレス取得処理
 * @brief ジョイントトルク格納アドレスを返却する
 * @param なし
 * @return トルク値格納アドレス
 */
double* Sts3215Joint::get_eff_addr(void)
{
    return &eff_;
}

/**
 * STS3215ジョイント HWIfコマンドデータアドレス取得処理
 * @brief ジョイントHWIfコマンド格納アドレスを返却する
 * @param なし
 * @return コマンド値格納アドレス
 */
double* Sts3215Joint::get_cmd_addr(void)
{
    return &cmd_;
}
