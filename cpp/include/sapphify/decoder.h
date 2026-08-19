// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sapphify LLC
//
// Frame decoders — the C++ mirror of RotemDecoder.java.
//
// Pure functions from bytes to values, with no allocation, no exceptions and no dependency on
// WPILib or on any transport. That makes the whole wire protocol testable on any machine with a
// compiler, and it lets the identical source compile into robot code, into a bench tool, and into
// the device firmware that produces these frames in the first place.
//
// All multi-byte fields are little-endian, two's complement where signed.

#ifndef SAPPHIFY_DECODER_H_
#define SAPPHIFY_DECODER_H_

#include <cstddef>
#include <cstdint>

#include "sapphify/protocol.h"

namespace sapphify {

/// Decoded STATUS_ORIENTATION payload: a unit quaternion.
struct Orientation {
  double w, x, y, z;
};

/// Decoded STATUS_RATES or STATUS_ACCEL payload.
struct Vector3 {
  double x, y, z;
  std::uint16_t sequence;
};

/// Decoded STATUS_QUALITY payload.
struct Quality {
  double yaw_sigma_degrees;
  double drift_since_zero_degrees;
  double bias_magnitude_dps;
  std::uint8_t zupt_state;       ///< 0 moving, 1 stationary candidate, 2 applied, 3 inhibited
  std::uint8_t estimator_state;  ///< 0 init, 1 converging, 2 converged, 3 degraded, 4 fault
};

/// Decoded STATUS_HEALTH payload.
struct Health {
  std::uint32_t flags;
  std::uint8_t self_test_result;
  std::uint8_t fault_code;
  std::int8_t die_temperature_c;
  std::uint8_t uptime_minutes;

  constexpr bool Has(std::uint32_t flag_bit) const { return (flags & flag_bit) != 0; }
};

namespace detail {
constexpr std::uint8_t U8(const std::uint8_t* d, std::size_t i) { return d[i]; }

constexpr std::int16_t I16(const std::uint8_t* d, std::size_t i) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(d[i]) |
                                   (static_cast<std::uint16_t>(d[i + 1]) << 8));
}

constexpr std::uint16_t U16(const std::uint8_t* d, std::size_t i) {
  return static_cast<std::uint16_t>(static_cast<std::uint16_t>(d[i]) |
                                    (static_cast<std::uint16_t>(d[i + 1]) << 8));
}

constexpr std::uint32_t U32(const std::uint8_t* d, std::size_t i) {
  return static_cast<std::uint32_t>(d[i]) | (static_cast<std::uint32_t>(d[i + 1]) << 8) |
         (static_cast<std::uint32_t>(d[i + 2]) << 16) | (static_cast<std::uint32_t>(d[i + 3]) << 24);
}
}  // namespace detail

/// Every decoder reports success rather than throwing: a frame arriving with the wrong length is
/// a bus condition, not a programming error, and firmware has no exceptions to throw anyway.
constexpr bool DecodeOrientation(const std::uint8_t* data, std::size_t length, Orientation* out) {
  if (data == nullptr || out == nullptr || length < 8) return false;
  out->w = detail::I16(data, 0) / scale::kQuaternion;
  out->x = detail::I16(data, 2) / scale::kQuaternion;
  out->y = detail::I16(data, 4) / scale::kQuaternion;
  out->z = detail::I16(data, 6) / scale::kQuaternion;
  return true;
}

constexpr bool DecodeRates(const std::uint8_t* data, std::size_t length, Vector3* out) {
  if (data == nullptr || out == nullptr || length < 8) return false;
  out->x = detail::I16(data, 0) * scale::kRateDps;
  out->y = detail::I16(data, 2) * scale::kRateDps;
  out->z = detail::I16(data, 4) * scale::kRateDps;
  out->sequence = detail::U16(data, 6);
  return true;
}

constexpr bool DecodeAccel(const std::uint8_t* data, std::size_t length, Vector3* out) {
  if (data == nullptr || out == nullptr || length < 8) return false;
  out->x = detail::I16(data, 0) * scale::kAccelG;
  out->y = detail::I16(data, 2) * scale::kAccelG;
  out->z = detail::I16(data, 4) * scale::kAccelG;
  out->sequence = detail::U16(data, 6);
  return true;
}

constexpr bool DecodeQuality(const std::uint8_t* data, std::size_t length, Quality* out) {
  if (data == nullptr || out == nullptr || length < 8) return false;
  out->yaw_sigma_degrees = detail::U16(data, 0) * scale::kAngleMillideg;
  out->drift_since_zero_degrees = detail::U16(data, 2) * scale::kAngleMillideg;
  out->bias_magnitude_dps = detail::U16(data, 4) * scale::kBiasDps;
  out->zupt_state = detail::U8(data, 6);
  out->estimator_state = detail::U8(data, 7);
  return true;
}

constexpr bool DecodeHealth(const std::uint8_t* data, std::size_t length, Health* out) {
  if (data == nullptr || out == nullptr || length < 8) return false;
  out->flags = detail::U32(data, 0);
  out->self_test_result = detail::U8(data, 4);
  out->fault_code = detail::U8(data, 5);
  out->die_temperature_c = static_cast<std::int8_t>(data[6]);
  out->uptime_minutes = detail::U8(data, 7);
  return true;
}

}  // namespace sapphify

#endif  // SAPPHIFY_DECODER_H_
