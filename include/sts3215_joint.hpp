#ifndef STS3215_JOINT_HPP_
#define STS3215_JOINT_HPP_

#include <cstdint>
#include <cstdio>
#include <string>
#include "sts3215_servo.hpp"

/**
 * STS3215Mジョイントタイプ列挙型
 */
typedef enum {
    enSts3215JointType_Position,
    enSts3215JointType_Velocity,
    enSts3215JointType_Effort
} enSts3215JointType;

/**
 * STS3215ジョイントクラス
 */
class Sts3215Joint : public Sts3215Servo
{
public:
    Sts3215Joint( uint8_t id, enSts3215JointType type, std::string name, bool reverse );
    ~Sts3215Joint();
    static EN_OPTIONS conv_type( enSts3215JointType type );

    void set_pos(double value);
    void set_vel(double value);
    void set_eff(double value);

    enSts3215JointType get_type(void);
    std::string get_name(void);
    double get_pos(void);
    double get_vel(void);
    double get_eff(void);
    double get_cmd(void);
    double* get_pos_addr(void);
    double* get_vel_addr(void);
    double* get_eff_addr(void);
    double* get_cmd_addr(void);

private:
    enSts3215JointType  type_;
    std::string     name_;
    bool            reverse_;
    double          pos_;   // [rad]
    double          vel_;   // [rad/sec]
    double          eff_;   // [Nm]
    double          cmd_;   // HardwareInterface command data
};

#endif  // STS3215_JOINT_HPP_
