#ifndef STS3215_DESIGN_HPP_
#define STS3215_DESIGN_HPP_

#define ADDR_FIRMWARE_MAIN_VERSION_NO      (0x00)
#define ADDR_FIRMWARE_SECONDARY_VERSION_NO (0x01)
#define ADDR_SERVO_MAIN_VERSION            (0x03)
#define ADDR_SERVO_SUB_VERSION             (0x04)
#define ADDR_ID                            (0x05)
#define ADDR_BAUD_RATE                     (0x06)
#define ADDR_RETURN_DELAY_TIME             (0x07)
#define ADDR_STATUS_RETURN_LEVEL           (0x08)
#define ADDR_MIN_POSITION_LIMIT            (0x09)
#define ADDR_MAX_POSITION_LIMIT            (0x0B)
#define ADDR_MAX_TEMPERATURE_LIMIT         (0x0D)
#define ADDR_MAX_INPUT_VOLTAGE             (0x0E)
#define ADDR_MIN_INPUT_VOLTAGE             (0x0F)
#define ADDR_MAX_TORQUE_LIMIT              (0x10)
#define ADDR_SETTING_BYTE                  (0x12)
#define ADDR_PROTECTION_SWITCH             (0x13)
#define ADDR_LED_ALARM_CONDITIONS          (0x14)
#define ADDR_POSITION_P_GAIN               (0x15)
#define ADDR_POSITION_D_GAIN               (0x16)
#define ADDR_POSITION_I_GAIN               (0x17)
#define ADDR_PUNCH                         (0x18)
#define ADDR_CW_DEAD_BAND                  (0x1A)
#define ADDR_CCW_DEAD_BAND                 (0x1B)
#define ADDR_OVERLOAD_CURRENT              (0x1C)
#define ADDR_ANGULAR_RESOLUTION            (0x1E)
#define ADDR_POSITION_OFFSET_VALUE         (0x1F)
#define ADDR_WORK_MODE                     (0x21)
#define ADDR_PROTECT_TORQUE                (0x22)
#define ADDR_OVERLOAD_PROTECTION_TIME      (0x23)
#define ADDR_OVERLOAD_TORQUE               (0x24)
#define ADDR_VELOCITY_P_GAIN               (0x25)
#define ADDR_OVERCURRENT_PROTECTION_TIME   (0x26)
#define ADDR_VELOCITY_I_GAIN               (0x27)
#define ADDR_TORQUE_ENABLE                 (0x28)
#define ADDR_GOAL_ACCELERATION             (0x29)
#define ADDR_GOAL_POSITION                 (0x2A)
#define ADDR_OPERATION_HOURS               (0x2C)
#define ADDR_GOAL_VELOCITY                 (0x2E)
#define ADDR_TORQUE_LIMIT                  (0x30)
#define ADDR_LOCK                          (0x37)
#define ADDR_PRESENT_POSITION              (0x38)
#define ADDR_PRESENT_VELOCITY              (0x3A)
#define ADDR_PRESENT_PWM                   (0x3C)
#define ADDR_PRESENT_INPUT_VOLTAGE         (0x3E)
#define ADDR_PRESENT_TEMPERATURE           (0x3F)
#define ADDR_SYNC_WRITE_FLAG               (0x40)
#define ADDR_HARDWARE_ERROR_STATE          (0x41)
#define ADDR_MOVING_STATUS                 (0x42)
#define ADDR_PRESENT_CURRENT               (0x45)

/* Memory size */
#define SIZE_FIRMWARE_MAIN_VERSION_NO      (1)
#define SIZE_FIRMWARE_SECONDARY_VERSION_NO (1)
#define SIZE_SERVO_MAIN_VERSION            (1)
#define SIZE_SERVO_SUB_VERSION             (1)
#define SIZE_ID                            (1)
#define SIZE_BAUD_RATE                     (1)
#define SIZE_RETURN_DELAY_TIME             (1)
#define SIZE_STATUS_RETURN_LEVEL           (1)
#define SIZE_MIN_POSITION_LIMIT            (2)
#define SIZE_MAX_POSITION_LIMIT            (2)
#define SIZE_MAX_TEMPERATURE_LIMIT         (1)
#define SIZE_MAX_INPUT_VOLTAGE             (1)
#define SIZE_MIN_INPUT_VOLTAGE             (1)
#define SIZE_MAX_TORQUE_LIMIT              (2)
#define SIZE_SETTING_BYTE                  (1)
#define SIZE_PROTECTION_SWITCH             (1)
#define SIZE_LED_ALARM_CONDITIONS          (1)
#define SIZE_POSITION_P_GAIN               (1)
#define SIZE_POSITION_D_GAIN               (1)
#define SIZE_POSITION_I_GAIN               (1)
#define SIZE_PUNCH                         (2)
#define SIZE_CW_DEAD_BAND                  (1)
#define SIZE_CCW_DEAD_BAND                 (1)
#define SIZE_OVERLOAD_CURRENT              (2)
#define SIZE_ANGULAR_RESOLUTION            (1)
#define SIZE_POSITION_OFFSET_VALUE         (2)
#define SIZE_WORK_MODE                     (1)
#define SIZE_PROTECT_TORQUE                (1)
#define SIZE_OVERLOAD_PROTECTION_TIME      (1)
#define SIZE_OVERLOAD_TORQUE               (1)
#define SIZE_VELOCITY_P_GAIN               (1)
#define SIZE_OVERCURRENT_PROTECTION_TIME   (1)
#define SIZE_VELOCITY_I_GAIN               (1)
#define SIZE_TORQUE_ENABLE                 (1)
#define SIZE_GOAL_ACCELERATION             (1)
#define SIZE_GOAL_POSITION                 (2)
#define SIZE_OPERATION_HOURS               (2)
#define SIZE_GOAL_VELOCITY                 (2)
#define SIZE_TORQUE_LIMIT                  (2)
#define SIZE_LOCK                          (1)
#define SIZE_PRESENT_POSITION              (2)
#define SIZE_PRESENT_VELOCITY              (2)
#define SIZE_PRESENT_PWM                   (2)
#define SIZE_PRESENT_INPUT_VOLTAGE         (1)
#define SIZE_PRESENT_TEMPERATURE           (1)
#define SIZE_SYNC_WRITE_FLAG               (1)
#define SIZE_HARDWARE_ERROR_STATE          (1)
#define SIZE_MOVING_STATUS                 (1)
#define SIZE_PRESENT_CURRENT               (2)

#define MAX_POSITION                       (4095)
#define MIN_POSITION                       (0)
#define MAX_VELOCITY                       (254)
#define MIN_VELOCITY                       (-254)
#define MAX_TORQUE                         (32767)
#define MIN_TORQUE                         (-32768)
#define MAX_UNDEFINED                      (32767)
#define MIN_UNDEFINED                      (-32768)


/**
 * 動作モード列挙型
 */
typedef enum {
  enOptions_TorqueDisable = 0x00,
  enOptions_TorqueEnable  = 0x01,
  enOptions_ControlPosition = 0x00,
  enOptions_ControlVelocity = 0x01,
    enOptions_ControlTorque     = 0x08,
    enOptions_ControlFForward   = 0x0C,
    enOptions_ServoNormal       = 0x00,
    enOptions_ServoClone        = 0x40,
    enOptions_ServoReverse      = 0x80
} EN_OPTIONS;

/*
 * システムエラー列挙型
 */
typedef enum {
  enSysErrors_Voltage     = 0x01,
  enSysErrors_Sensor      = 0x02,
  enSysErrors_Temperature = 0x04,
  enSysErrors_Current     = 0x08,
  enSysErrors_Angle       = 0x10,
  enSysErrors_Overload    = 0x20  
} EN_SYS_ERRORS;

/**
 * モーターステータス異常列挙型
 */
typedef enum {
    enMotorErrors_Temperature   = 0x01,
    enMotorErrors_LockDetect    = 0x02,
    enMotorErrors_CurrentLimit  = 0x04,
    enMotorErrors_HallIC        = 0x08
} EN_MOTOR_ERRORS;

/**
 * UART受信エラー列挙型
 */
typedef enum {
    enUartErrors_Framing        = 0x01,
    enUartErrors_Parity         = 0x02,
    enUartErrors_Break          = 0x04,
    enUartErrors_Overrun        = 0x08
} EN_UART_ERRORS;

/**
 * コマンドエラーなど列挙型
 */
typedef enum {
    enCmdErrors_CheckSum        = 0x01,
    enCmdErrors_Length          = 0x02,
    enCmdErrors_Size            = 0x04,
    enCmdErrors_Address         = 0x08,
    enCmdErrors_WrongCmd        = 0x10
} EN_COMMAND_ERRORS;

/**
 * 軌道生成タイプ列挙型
 */
typedef enum {
    enTrajErrors_Normal         = 0x00,
    enTrajErrors_Even           = 0x01,
    enTrajErrors_ThirdPoly      = 0x03,
    enTrajErrors_FourthPoly     = 0x04,
    enTrajErrors_FifthPoly      = 0x05
} EN_TRAJECTORY_TYPES;

/**
 * コマンド列挙型
 */
typedef enum {
    enCmdTypes_Load             = 0x01,
    enCmdTypes_Save             = 0x02,
    enCmdTypes_Read             = 0x03,
    enCmdTypes_Write            = 0x04,
    enCmdTypes_Reset            = 0x05,
    enCmdTypes_Position         = 0x06
} EN_COMMAND_TYPES;

/**
 * プロトコルオプション列挙型
 */
typedef enum {
    enProtocolOptions_ErrStat   = 0x00,
    enProtocolOptions_SysStat   = 0x01,
    enProtocolOptions_MotorStat = 0x02,
    enProtocolOptions_UartStat  = 0x03,
    enProtocolOptions_CmdStat   = 0x04,
    enProtocolOptions_ClearStat = 0x80
} EN_PROTOCOL_OPTIONS;

#endif  // STS3215_DESIGN_HPP_
