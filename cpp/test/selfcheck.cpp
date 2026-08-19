// Self-check for the SAPPHIFY C++ protocol headers.
//
// Deliberately mirrors the Java SapphifyLibSelfCheck case for case. The two libraries decode the
// same wire format, so any divergence between them is a bug in one of them — and a team writing
// its own driver from the specification can run either to check its own decoder.
//
//   g++ -std=c++20 -Wall -Wextra -Werror -Icpp/include cpp/test/selfcheck.cpp -o selfcheck

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "sapphify/decoder.h"
#include "sapphify/protocol.h"

namespace {
int failures = 0;

void Check(const char* what, bool ok) {
  std::printf("%-58s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok) ++failures;
}

bool Near(double a, double b, double tol) { return std::fabs(a - b) < tol; }
}  // namespace

int main() {
  using namespace sapphify;

  // Arbitration ID layout. The Java side asserts this exact value.
  Check("arbitrationId(AHRS, STATUS_ORIENTATION, 0) == 0x04080000",
        ArbitrationId(DeviceType::kAhrs, api::kStatusOrientation, 0) == 0x04080000u);
  Check("device number lands in the low 6 bits",
        (ArbitrationId(DeviceType::kAhrs, api::kStatusOrientation, 5) & 0x3F) == 5u);
  Check("manufacturer id is the beta Team Use value", kManufacturerId == 8);
  Check("AHRS is FIRST device type 4", static_cast<int>(DeviceType::kAhrs) == 4);

  // API packing.
  Check("apiId(2,3) packs to 0x23", ApiId(2, 3) == 0x23u);
  Check("apiId(32,4) is the config commit", api::kCfgCommit == ApiId(32, 4));

  // Orientation: identity quaternion.
  const std::uint8_t identity[8] = {0xFF, 0x7F, 0, 0, 0, 0, 0, 0};
  Orientation o{};
  Check("identity quaternion decodes", DecodeOrientation(identity, 8, &o));
  Check("identity w == 1.0", Near(o.w, 1.0, 1e-4));
  Check("identity x,y,z == 0", o.x == 0.0 && o.y == 0.0 && o.z == 0.0);

  // Orientation: 90 degrees of yaw, w = z = cos(45 deg).
  const std::int16_t c = static_cast<std::int16_t>(std::lround(std::cos(M_PI / 4) * 32767));
  const std::uint8_t yaw90[8] = {static_cast<std::uint8_t>(c & 0xFF),
                                 static_cast<std::uint8_t>((c >> 8) & 0xFF),
                                 0, 0, 0, 0,
                                 static_cast<std::uint8_t>(c & 0xFF),
                                 static_cast<std::uint8_t>((c >> 8) & 0xFF)};
  Check("90 degree quaternion decodes", DecodeOrientation(yaw90, 8, &o));
  const double yaw_deg =
      std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z)) * 180.0 / M_PI;
  Check("90 degree quaternion yields 90 degrees of yaw", Near(yaw_deg, 90.0, 0.02));

  // Short frames are rejected, not undefined behaviour.
  Check("short frame rejected", !DecodeOrientation(identity, 3, &o));
  Check("null data rejected", !DecodeOrientation(nullptr, 8, &o));

  // Rates and acceleration, matching the Java conversions.
  const std::uint8_t rates[8] = {0x88, 0x13, 0, 0, 0, 0, 1, 0};  // 5000 -> 100 deg/s
  Vector3 v{};
  Check("rates decode", DecodeRates(rates, 8, &v));
  Check("rate LSB is 0.02 deg/s", Near(v.x, 5000 * 0.02, 1e-9));
  Check("sequence survives", v.sequence == 1);

  const std::uint8_t accel[8] = {0, 0, 0, 0, 0xF4, 0x01, 1, 0};  // 500 -> 1.0 g
  Check("accel decodes", DecodeAccel(accel, 8, &v));
  Check("accel LSB is 0.002 g", Near(v.z, 500 * 0.002, 1e-9));

  // Quality.
  const std::uint8_t quality[8] = {0xE8, 0x03, 0x2C, 0x01, 0x64, 0x00, 2, 2};
  Quality q{};
  Check("quality decodes", DecodeQuality(quality, 8, &q));
  Check("yaw sigma in milli-degrees", Near(q.yaw_sigma_degrees, 1.0, 1e-9));
  Check("drift in milli-degrees", Near(q.drift_since_zero_degrees, 0.3, 1e-9));
  Check("bias magnitude scaling", Near(q.bias_magnitude_dps, 0.01, 1e-9));
  Check("zupt applied", q.zupt_state == 2);
  Check("estimator converged", q.estimator_state == 2);

  // Health and flags.
  const std::uint32_t flags = flag::kHighVibration | flag::kMagDisturbed | flag::kCalValid;
  const std::uint8_t health[8] = {static_cast<std::uint8_t>(flags & 0xFF),
                                  static_cast<std::uint8_t>((flags >> 8) & 0xFF),
                                  static_cast<std::uint8_t>((flags >> 16) & 0xFF),
                                  static_cast<std::uint8_t>((flags >> 24) & 0xFF),
                                  0, 0, 0xE7 /* -25 C */, 7};
  Health h{};
  Check("health decodes", DecodeHealth(health, 8, &h));
  Check("calibration valid flag", h.Has(flag::kCalValid));
  Check("high vibration flag", h.Has(flag::kHighVibration));
  Check("magnetic disturbance flag", h.Has(flag::kMagDisturbed));
  Check("id conflict flag absent", !h.Has(flag::kIdConflict));
  Check("die temperature is signed", h.die_temperature_c == -25);
  Check("uptime survives", h.uptime_minutes == 7);

  // Decoders are constexpr, so the compiler proves them at build time too.
  static_assert(ArbitrationId(DeviceType::kAhrs, api::kStatusHealth, 0) == 0x04080800u,
                "arbitration id must be computable at compile time");

  std::printf("\n%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
  return failures == 0 ? 0 : 1;
}
