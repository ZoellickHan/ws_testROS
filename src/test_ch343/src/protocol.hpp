#ifndef RM_SERIAL_DRIVER__PROTOCOL_HPP_
#define RM_SERIAL_DRIVER__PROTOCOL_HPP_

#include <sys/cdefs.h>

#include <algorithm>
#include <boost/cstdint.hpp>
#include <cstdint>
#include <iostream>
#include <vector>

// Protocol version: v3.0 (RM2024)

// note: CRC operations are defined in CRC.hpp
/*
 * CRC parameters:
 * width = 16, i.e. CRC-16
 * init = 0xFFFF
 * poly = 0x1189
 * final_xor_value = 0 (do nothing)
 */

// UART parameters:
/*
 * baud_rate = 460800
 * stop bits = 1
 * parity = none
 * hardware_flow_ctrl = no
 * data_size = 8-bit
 */

namespace rm_serial_driver
{
// 0x stands for msg received (1 - 10 for msg from pcb to nuc)
// 1x stands for msg sent (11 - 20 for msg from nuc to pcb)
// 2x stands for ring buffer msg sent (21 - 30 for msg from nuc to pcb)
enum CommunicationType : uint8_t 
{
  BEAT_MSG = 0x01,                 // for test
  GIMBAL_MSG = 0x02,               // gimbal packet received for hero and infantry
  SENTRY_GIMBAL_MSG = 0x03,        // packet received for sentry

  TWOCRC_GIMBAL_MSG = 0x04,       // for ring buffer
  TWOCRC_GIMBALSTATUS_MSG = 0x05, // for ring buffer, sentry uses

  GIMBAL_CMD = 0x11,              // gimbal command sent for hero and infantry
  SENTRY_GIMBAL_CMD = 0x12,       // gimbal command sent for sentry

  TWOCRC_GIMBAL_CMD = 0x21,       // for ring buffer 
  TWOCRC_CHASSIS_CMD = 0x22,      // for ring buffer sentry use
};

// *********************************//
// * protocol without ring buffer * //
// *********************************//

struct BeatMsg  // BEAT_MSG
{
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;

  uint32_t beat = 0;
  
  uint16_t checksum = 0;
} __attribute__((packed));

struct GimbalMsg  // GIMBAL_MSG
{
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  
  uint8_t cur_cv_mode;
  uint8_t target_color;
  float bullet_speed;
  float q_w;
  float q_x;
  float q_y;
  float q_z;
  
  uint16_t checksum = 0;
} __attribute__((packed));

struct SentryGimbalMsg  // SENTRY_GIMBAL_MSG
{
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;

  uint8_t target_color;
  float left_gimbal_q_w;
  float left_gimbal_q_x;
  float left_gimbal_q_y;
  float left_gimbal_q_z;
  float right_gimbal_q_w;
  float right_gimbal_q_x;
  float right_gimbal_q_y;
  float right_gimbal_q_z;

  float chassis_vel_x;
  float chassis_vel_y;
  float chassis_vel_w;

  uint8_t path_planner;
  uint8_t control_mode;
  uint8_t action;

  uint16_t checksum = 0;
} __attribute__((packed));

struct GimbalCommand  // GIMBAL_CMD
{
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;

  float target_pitch;
  float target_yaw;
  uint8_t shoot_mode;
  
  uint8_t crc_1;
  uint8_t crc_2;
} __attribute__((packed));

struct SentryGimbalCommand  // SENTRY_GIMBAL_CMD
{
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;

  float left_target_pitch;
  float left_target_yaw;
  uint8_t left_shoot_mode;

  float right_target_pitch;
  float right_target_yaw;
  uint8_t right_shoot_mode;

  float vel_x;
  float vel_y;
  float vel_w;

  uint8_t crc_1;
  uint8_t crc_2;

} __attribute__((packed));


inline GimbalMsg fromVector(const std::vector<uint8_t> & data)
{
  GimbalMsg received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

inline std::vector<uint8_t> toVector(const GimbalCommand & data)
{
  std::vector<uint8_t> sent_packet(sizeof(GimbalCommand));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(GimbalCommand), sent_packet.begin());
  return sent_packet;
}

inline SentryGimbalMsg fromSentryVector(const std::vector<uint8_t> & data)
{
  SentryGimbalMsg received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

inline std::vector<uint8_t> toSentryVector(const SentryGimbalCommand & data)
{
  std::vector<uint8_t> sent_packet(sizeof(SentryGimbalCommand));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SentryGimbalCommand), sent_packet.begin());
  return sent_packet;
}

// **************************** //
// * protocol for ring buffer * //
// **************************** //

struct Header  
{
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  uint8_t crc_1;
  uint8_t crc_2;
} __attribute__((packed));


struct TwoCRC_GimbalMsg  // TWOCRC_GIMBAL_MSG, also 0x04
{
  Header header;
  
  uint8_t cur_cv_mode;
  uint8_t target_color;
  float bullet_speed;
  float q_w;
  float q_x;
  float q_y;
  float q_z;
  uint16_t checksum = 0;

} __attribute__((packed));

//5+2+4+16+2
struct TwoCRC_SentryGimbalMsg // TWOCRC_GIMBALSTATUS_MSG, also 0x05
{
  Header header;

  uint8_t cur_cv_mode;
  uint8_t target_color;
  float bullet_speed;
  float q1[4];
  float q2[10];
  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));


struct TwoCRC_GimbalCommand // TWOCRC_GIMBAL_CMD, also 0x21
{
  Header header;

  float target_pitch;
  float target_yaw;
  uint8_t shoot_mode;

  uint8_t crc_3;
  uint8_t crc_4;
} __attribute__((packed));

struct TwoCRC_ChassisCommand  // TWOCRC_CHASSIS_CMD = 0x22
{
  Header header;

  float vel_x;
  float vel_y;
  float vel_w;

  uint8_t crc_3;
  uint8_t crc_4;
} __attribute__((packed));

inline Header fromHeaderVector(const std::vector<uint8_t> & data)
{
  Header received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

inline TwoCRC_GimbalMsg fromTwoCRCVector(const std::vector<uint8_t> & data)
{
  TwoCRC_GimbalMsg received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

inline std::vector<uint8_t> toTwoCRCVector(const TwoCRC_GimbalCommand & data )
{
  std::vector<uint8_t> sent_packet(sizeof(TwoCRC_GimbalCommand));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(TwoCRC_GimbalCommand), sent_packet.begin());
  return sent_packet;
}

inline TwoCRC_SentryGimbalMsg fromSentryTwoCRCVector(const std::vector<uint8_t> & data)
{
  TwoCRC_SentryGimbalMsg  received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

inline std::vector<uint8_t> toSentryTwoCRCVectorChassis(const TwoCRC_ChassisCommand  & data )
{
  std::vector<uint8_t> sent_packet(sizeof(TwoCRC_ChassisCommand ));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(TwoCRC_ChassisCommand), sent_packet.begin());
  return sent_packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PROTOCOL_HPP_