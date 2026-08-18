package com.sapphify.frc.rotem;

/**
 * Pure frame decoders for the ROTEM CAN protocol.
 *
 * <p>Deliberately free of any WPILib or hardware dependency: every method is a total function from
 * bytes to values. That makes the whole wire protocol unit-testable on a laptop with no board, no
 * bus and no robot, and it makes the published test vectors runnable by anyone implementing their
 * own driver.
 *
 * <p>All multi-byte fields are little-endian, two's complement where signed.
 */
public final class RotemDecoder {

  private RotemDecoder() {}

  /** Decoded {@code STATUS_ORIENTATION} payload: a unit quaternion. */
  public record Orientation(double w, double x, double y, double z) {}

  /** Decoded {@code STATUS_RATES} or {@code STATUS_ACCEL} payload. */
  public record Vector3(double x, double y, double z, int sequence) {}

  /** Decoded {@code STATUS_QUALITY} payload. */
  public record Quality(
      double yawSigmaDegrees,
      double driftSinceZeroDegrees,
      double biasMagnitudeDps,
      int zuptState,
      int estimatorState) {}

  /** Decoded {@code STATUS_HEALTH} payload. */
  public record Health(
      int flags, int selfTestResult, int faultCode, int dieTemperatureC, int uptimeMinutes) {

    public boolean has(int flagBit) {
      return (flags & flagBit) != 0;
    }
  }

  public static Orientation decodeOrientation(byte[] data) {
    require(data, 8, "STATUS_ORIENTATION");
    return new Orientation(
        int16(data, 0) / RotemProtocol.Scale.QUATERNION,
        int16(data, 2) / RotemProtocol.Scale.QUATERNION,
        int16(data, 4) / RotemProtocol.Scale.QUATERNION,
        int16(data, 6) / RotemProtocol.Scale.QUATERNION);
  }

  public static Vector3 decodeRates(byte[] data) {
    return decodeScaledVector(data, RotemProtocol.Scale.RATE_DPS, "STATUS_RATES");
  }

  public static Vector3 decodeAccel(byte[] data) {
    return decodeScaledVector(data, RotemProtocol.Scale.ACCEL_G, "STATUS_ACCEL");
  }

  public static Quality decodeQuality(byte[] data) {
    require(data, 8, "STATUS_QUALITY");
    return new Quality(
        uint16(data, 0) * RotemProtocol.Scale.ANGLE_MILLIDEG,
        uint16(data, 2) * RotemProtocol.Scale.ANGLE_MILLIDEG,
        uint16(data, 4) * RotemProtocol.Scale.BIAS_DPS,
        uint8(data, 6),
        uint8(data, 7));
  }

  public static Health decodeHealth(byte[] data) {
    require(data, 8, "STATUS_HEALTH");
    return new Health(
        (int) uint32(data, 0),
        uint8(data, 4),
        uint8(data, 5),
        (byte) data[6], // signed degrees Celsius
        uint8(data, 7));
  }

  private static Vector3 decodeScaledVector(byte[] data, double lsb, String frame) {
    require(data, 8, frame);
    return new Vector3(
        int16(data, 0) * lsb, int16(data, 2) * lsb, int16(data, 4) * lsb, uint16(data, 6));
  }

  private static void require(byte[] data, int length, String frame) {
    if (data == null || data.length < length) {
      throw new IllegalArgumentException(
          frame + " requires " + length + " bytes, got " + (data == null ? "null" : data.length));
    }
  }

  private static int uint8(byte[] d, int i) {
    return d[i] & 0xFF;
  }

  private static short int16(byte[] d, int i) {
    return (short) ((d[i] & 0xFF) | ((d[i + 1] & 0xFF) << 8));
  }

  private static int uint16(byte[] d, int i) {
    return (d[i] & 0xFF) | ((d[i + 1] & 0xFF) << 8);
  }

  private static long uint32(byte[] d, int i) {
    return (d[i] & 0xFFL)
        | ((d[i + 1] & 0xFFL) << 8)
        | ((d[i + 2] & 0xFFL) << 16)
        | ((d[i + 3] & 0xFFL) << 24);
  }
}
