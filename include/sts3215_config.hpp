#ifndef STS3215_CONFIG_HPP_
#define STS3215_CONFIG_HPP_

#include <ros/ros.h>

/* 定数・マクロ定義 */
#define     KEY_STS3215_CONFIG  ("sts3215_config")
#define     KEY_PORTNAME        ("/port")
#define     KEY_BAUDRATE        ("/baudrate")
#define     KEY_JOINTS          ("/joints")
#define     KEY_PARAM_ID        ("/id")
#define     KEY_PARAM_TYPE      ("/type")
#define     KEY_PARAM_REVERSE   ("/reverse")

#define     TYPE_POSITION       ("Position")
#define     TYPE_VELOCITY       ("Velocity")
#define     TYPE_EFFORT         ("Effort")

/**
 * STS3215ジョイントタイプ列挙型
 */
typedef enum {
    enSts3215JointConfigType_Position   = 0,
    enSts3215JointConfigType_Velocity   = 1,
    enSts3215JointConfigType_Effort     = 2
} enSts3215JointConfigType;

/**
 * STS3215ジョイント設定構造体
 */
typedef struct {
    std::string              name_;
    uint8_t                  id_;
    enSts3215JointConfigType type_;
    bool                     reverse_;
} stSts3215JointConfig;

/**
 * STS3215設定クラス
 */
class Sts3215Config
{
public:
    Sts3215Config( ros::NodeHandle handle );
    ~Sts3215Config();
    bool load( void );
    std::string get_portname( void ){ return port_; }
    uint32_t get_baudrate( void ){ return baudrate_; }
    std::vector<stSts3215JointConfig> get_joint_config( void ){ return joints_; }

private:
    ros::NodeHandle nh_;
    std::string port_;
    uint32_t baudrate_;
    std::vector<stSts3215JointConfig> joints_;

    bool load_joints( void );
    std::string load_portname( void );
    uint32_t load_baudrate( void );
    bool load_param( void );
};

#endif

