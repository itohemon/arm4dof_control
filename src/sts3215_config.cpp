#include "arm4dof_control/sts3215_config.hpp"

/**
 * STS3215設定 コンストラクタ
 * @brief STS3215設定クラスを初期化する
 * @param handle：ノードハンドル
 */
Sts3215Config::Sts3215Config( ros::NodeHandle handle )
  : nh_(handle)
{
    joints_.clear();
}

/**
 * STS3215設定 デストラクタ
 * @brief STS3215設定クラスを解放する
 * @param なし
 */
Sts3215Config::~Sts3215Config()
{
    /* Nothing todo... */
}

/**
 * STS3215設定 ロード処理
 * @brief STS3215設定値を読み込む
 * @param なし
 * @return 読み込み成功(true)
 * @return 読み込み失敗(false)
 */
bool Sts3215Config::load( void )
{
    std::string port_name;
    uint32_t baudrate;
    bool result;

    result = true;

    /* 通信ポートの読み込み */
    std::string name = load_portname();
    if( name.length() > 0 ){
        port_ = name;
    }else{
        return false;
    }

    ROS_DEBUG("portname: %s", name.c_str());

    /* 通信速度の読み込み */
    baudrate = load_baudrate();
    if( baudrate > 0 ){
        baudrate_ = baudrate;
    }else{
        result = false;
    }
    ROS_DEBUG("baudrate: %d", baudrate_);

    /* ジョイント設定の読み込み */
    if( load_joints() ){
        load_param();
    }else{
        result = false;
    }

    ROS_DEBUG("Size of joints: %ld", joints_.size());

    return result;
}

/**
 * STS3215設定 通信ポート名ロード処理
 * @brief STS3215設定通信ポート名を読み込む
 * @param なし
 * @return 通信ポート名(成功)
 * @return 空文字(失敗)
 */
std::string Sts3215Config::load_portname( void )
{
  std::string key_port = KEY_STS3215_CONFIG;
  std::string result;

  key_port += KEY_PORTNAME;
  if( !nh_.getParam(key_port, result ) ){
    ROS_ERROR("Undefined key %s", key_port.c_str());
    result = "";
  }
  return result;
}

/**
 * STS3215設定 通信速度ロード処理
 * @brief STS3215設定通信速度を読み込む
 * @param なし
 * @return !=0(成功)
 * @return =0(失敗)
 */
uint32_t Sts3215Config::load_baudrate( void )
{
  std::string key_baudrate = KEY_STS3215_CONFIG;
  key_baudrate += KEY_BAUDRATE;
  int result;
  
  if( !nh_.getParam(key_baudrate, result ) ){
    ROS_ERROR("Undefined key %s", key_baudrate.c_str());
    result = 0;
  }
  return static_cast<uint32_t>(result);
}

/**
 * STS3215設定 ジョイント定義ロード処理
 * @brief STS3215設定ジョイント定義を読み込む
 * @param なし
 * @return 読み込み結果
 */
bool Sts3215Config::load_joints( void )
{
  std::string key_joints = KEY_STS3215_CONFIG;
  bool result = false;
  XmlRpc::XmlRpcValue load_joints;
  
  key_joints += KEY_JOINTS;
  if( !nh_.getParam( key_joints, load_joints ) ){
    ROS_ERROR("Undefined key %s", key_joints.c_str());
  }else{
    // jointsキーを見つけた
    if( load_joints.getType() != XmlRpc::XmlRpcValue::TypeArray ){
      ROS_ERROR("XmlRpc get type error! line%d", __LINE__);
    }else{
      // jointsリストを作成する
      for( uint32_t ii=0; ii < load_joints.size(); ++ii ){
        if( load_joints[ii].getType() != XmlRpc::XmlRpcValue::TypeString ){
          ROS_ERROR("XmlRpc get type[%d] error! line%d",ii,__LINE__);
        }else{
          XmlRpc::XmlRpcValue &joint_work = load_joints[ii];
          stSts3215JointConfig work;
          work.name_ = (std::string)joint_work;
          joints_.push_back( work );
        }
      }
      result = true;
    }
  }
  return result;
}

/**
 * STS3215設定 ジョイントパラメータロード処理
 * @brief STS3215設定ジョイントパラメータを読み込む
 * @param なし
 * @return 読み込み結果
 */
bool Sts3215Config::load_param( void )
{
  bool result = true;
    
  for(std::vector<stSts3215JointConfig>::iterator itr=joints_.begin() ; itr!=joints_.end() ; ++itr){
    std::string key_joint           = (std::string(KEY_STS3215_CONFIG) + "/" + itr->name_);
    std::string key_param_id        = (key_joint + KEY_PARAM_ID);
    std::string key_param_type      = (key_joint + KEY_PARAM_TYPE);
    std::string key_param_reverse   = (key_joint + KEY_PARAM_REVERSE);
    std::string key_param_pos_offset = (key_joint + KEY_PARAM_POS_OFFSET);
    int load_id = 0;
    std::string load_type = "";
    bool load_reverse = false;
    bool load_result = true;
    int load_pos_offset = 0;
    if( !nh_.getParam( key_param_id, load_id ) ){
      ROS_ERROR("Undefined key %s", key_param_id.c_str());
      load_result = false;
    }
    if( !nh_.getParam( key_param_type, load_type ) ){
      ROS_ERROR("Undefined key %s", key_param_type.c_str());
      load_result = false;
    }
    if( !nh_.getParam( key_param_reverse, load_reverse ) ){
      ROS_ERROR("Undefined key %s", key_param_reverse.c_str());
      load_result = false;
    }
    if( !nh_.getParam( key_param_pos_offset, load_pos_offset ) ){
      ROS_ERROR("Undefined key %s", key_param_pos_offset.c_str());
      load_result = false;
    }
    if( load_result ){
      itr->id_ = static_cast<uint8_t>(load_id);
      if(load_type == "Position"){
        itr->type_ = enSts3215JointConfigType_Position;
      }else if(load_type == "Velocity"){
        itr->type_ = enSts3215JointConfigType_Velocity;
      // }else if(load_type == "Effort"){
      //   itr->type_ = enSts3215JointConfigType_Effort;
      }else{
        ROS_ERROR("Undefined type %s", load_type.c_str());
        result = false;
        break;
      }
      itr->reverse_ = load_reverse;
      itr->pos_offset_ = load_pos_offset;
    }else{
      result = false;
      break;
    }
  }
  return result;
}
