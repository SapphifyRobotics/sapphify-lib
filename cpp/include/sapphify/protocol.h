// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sapphify LLC
//
// SAPPHIFY CAN protocol constants — the C++ mirror of SapphifyProtocol.java.
//
// FIRST's "Requirements for FIRST CAN Nodes" states that a device must "provide software library
// support for C++, and Java". This header is that support at the protocol level: header-only,
// freestanding-friendly, no WPILib and no heap, so the same file compiles into robot code, into
// our firmware, and into a team's own driver.
//
// Every number here also appears in SAPPHIFY_CAN_SPECIFICATION.md and in SapphifyProtocol.java.
// Three copies is two too many, which is why the specification is normative and both libraries
// are checked against it by their self-checks rather than by eye.

#ifndef SAPPHIFY_PROTOCOL_H_
#define SAPPHIFY_PROTOCOL_H_

#include <cstdint>

namespace sapphify {

/// FRC CAN manufacturer ID.
///
/// BETA VALUE. 8 is the FIRST "Team Use" identifier and is correct only for pre-release
/// hardware. A released build must carry the ID assigned to SAPPHIFY by FIRST; this is the one
/// place it is written, so the assignment changes a single line.
inline constexpr std::uint8_t kManufacturerId = 8;

/// FRC device types used by SAPPHIFY devices. Device numbers are scoped per type, so an AHRS and
/// an encoder may both be device 0.
enum class DeviceType : std::uint8_t {
  kAhrs = 4,     ///< ROTEM. FIRST device type 4, "Gyro Sensor".
  kEncoder = 7,  ///< Absolute encoder.
  kBridge = 10,  ///< USB-CAN FD bridge. "Miscellaneous".
};

/// Factory default device number.
inline constexpr std::uint8_t kDefaultDeviceNumber = 0;

/// Protocol document version this header implements.
inline constexpr const char* kProtocolVersion = "0.9-draft";

/// Packs a 6-bit API class and 4-bit API index into the 10-bit API ID.
constexpr std::uint16_t ApiId(std::uint8_t api_class, std::uint8_t api_index) {
  return static_cast<std::uint16_t>((api_class << 4) | (api_index & 0x0F));
}

namespace api {
inline constexpr std::uint16_t kStatusOrientation = ApiId(0, 0);
inline constexpr std::uint16_t kStatusRates = ApiId(0, 1);
inline constexpr std::uint16_t kStatusAccel = ApiId(0, 2);
inline constexpr std::uint16_t kStatusEuler = ApiId(0, 3);

inline constexpr std::uint16_t kStatusQuality = ApiId(1, 0);
inline constexpr std::uint16_t kStatusBias = ApiId(1, 1);
inline constexpr std::uint16_t kStatusVibration = ApiId(1, 2);
inline constexpr std::uint16_t kStatusMag = ApiId(1, 3);

inline constexpr std::uint16_t kStatusHealth = ApiId(2, 0);
inline constexpr std::uint16_t kStatusCalibration = ApiId(2, 1);
inline constexpr std::uint16_t kStatusCan = ApiId(2, 2);
inline constexpr std::uint16_t kStatusIdentity = ApiId(2, 3);

inline constexpr std::uint16_t kStatusFdComposite = ApiId(3, 0);

inline constexpr std::uint16_t kCmdZeroYaw = ApiId(16, 0);
inline constexpr std::uint16_t kCmdSetYaw = ApiId(16, 1);
inline constexpr std::uint16_t kCmdResetEstimator = ApiId(16, 2);
inline constexpr std::uint16_t kCmdIdentify = ApiId(16, 3);
inline constexpr std::uint16_t kCmdSelfTest = ApiId(16, 4);

inline constexpr std::uint16_t kCfgSetDeviceNumber = ApiId(32, 0);
inline constexpr std::uint16_t kCfgSetRate = ApiId(32, 1);
inline constexpr std::uint16_t kCfgSetMountPose = ApiId(32, 2);
inline constexpr std::uint16_t kCfgSetMagEnable = ApiId(32, 3);
inline constexpr std::uint16_t kCfgCommit = ApiId(32, 4);
inline constexpr std::uint16_t kCfgRead = ApiId(32, 5);
}  // namespace api

/// Health flag bits. Identical on every SAPPHIFY device.
namespace flag {
inline constexpr std::uint32_t kCalValid = 1u << 0;
inline constexpr std::uint32_t kCalStale = 1u << 1;
inline constexpr std::uint32_t kMountSet = 1u << 2;
inline constexpr std::uint32_t kMagEnabled = 1u << 3;
inline constexpr std::uint32_t kMagDisturbed = 1u << 4;
inline constexpr std::uint32_t kGyroSat = 1u << 5;
inline constexpr std::uint32_t kAccelSat = 1u << 6;
inline constexpr std::uint32_t kHighVibration = 1u << 7;
inline constexpr std::uint32_t kFifoOverrun = 1u << 8;
inline constexpr std::uint32_t kTimeDiscontinuity = 1u << 9;
inline constexpr std::uint32_t kNoHost = 1u << 10;
inline constexpr std::uint32_t kIdConflict = 1u << 11;
inline constexpr std::uint32_t kBusOffRecovered = 1u << 12;
inline constexpr std::uint32_t kLogActive = 1u << 13;
inline constexpr std::uint32_t kLogFull = 1u << 14;
inline constexpr std::uint32_t kTempOutOfCal = 1u << 15;
inline constexpr std::uint32_t kNumericalFault = 1u << 16;
inline constexpr std::uint32_t kFirmwareMismatch = 1u << 17;
}  // namespace flag

/// Scaling factors. Decoders must not hard-code these anywhere else.
namespace scale {
inline constexpr double kQuaternion = 32767.0;  ///< int16 divided by this
inline constexpr double kRateDps = 0.02;        ///< angular rate LSB, deg/s
inline constexpr double kAccelG = 0.002;        ///< acceleration LSB, g
inline constexpr double kAngleMillideg = 0.001; ///< uncertainty and drift LSB, deg
inline constexpr double kBiasDps = 0.0001;      ///< gyro bias magnitude LSB, deg/s
}  // namespace scale

/// Builds the 29-bit FRC extended arbitration ID.
///
/// Layout, most significant field first: device type (5), manufacturer (8), API class (6),
/// API index (4), device number (6).
constexpr std::uint32_t ArbitrationId(DeviceType type, std::uint16_t api_id,
                                      std::uint8_t device_number) {
  return (static_cast<std::uint32_t>(type) << 24) |
         (static_cast<std::uint32_t>(kManufacturerId) << 16) |
         (static_cast<std::uint32_t>(api_id & 0x3FF) << 6) |
         (device_number & 0x3F);
}

}  // namespace sapphify

#endif  // SAPPHIFY_PROTOCOL_H_
