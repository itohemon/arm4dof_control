#ifndef STS3215_PORT_HPP_
#define STS3215_PORT_HPP_

#include <string>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "sts3215_design.hpp"
#include "sts3215_servo.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

#define PROTOCOL_VERSION (1.0)

/**
 * STS3215通信ポートクラス
 */
class Sts3215Port
{
public:
  Sts3215Port(std::string dev_name, uint32_t baudrate);
  ~Sts3215Port();
  bool is_init(void);

  void load(Sts3215Servo &servo);
  void save(Sts3215Servo &servo);
  // void reset(Sts3215Servo &servo);
  void clean(void);

  void readCurrent(Sts3215Servo &servo);
  void writeDesired(Sts3215Servo &servo);
  void writeRunMode(Sts3215Servo &servo);

private:
  bool initialize(void);
  void create_multi_packet();
  void create_single_packet();
  uint8_t calc_sum(uint8_t* data, uint8_t length);
  bool read_port(uint8_t* data, uint8_t length, uint8_t timeout_msec=30);
  bool write_port(uint8_t* data, uint8_t length);
  void wait(uint16_t msec);

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  std::string     dev_name_;
  uint32_t        baudrate_;
  bool            init_result_;
};

#endif  // STS3215_PORT_HPP_
