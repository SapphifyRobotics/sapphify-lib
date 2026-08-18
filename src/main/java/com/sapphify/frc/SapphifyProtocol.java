package com.sapphify.frc;

/**
 * Constants generated from the ROTEM series CAN protocol specification.
 *
 * <p>This file is the single Java-side mirror of {@code ROTEM_SERIES_CAN_SPECIFICATION.md}. It is
 * the only place in the library where a protocol number appears. Firmware, the host tooling and
 * the published test vectors mirror the same specification, so a protocol change is one edit per
 * implementation and never a search for scattered literals.
 *
 * <p>These constants are <b>series-wide</b>. Every SAPPHIFY ROTEM device shares one manufacturer
 * ID, one health-flag vocabulary, one configuration API and one library. Products are separated
 * by {@link DeviceType} and device number, never by a second vendordep.
 */
public final class SapphifyProtocol {

  private SapphifyProtocol() {}

  /**
   * FRC device types used by the ROTEM series.
   *
   * <p>Device numbers are scoped per device type, so an AHRS and an encoder may both be device 0
   * without conflicting.
   */
  public enum DeviceType {
    /** ROTEM AHRS. FIRST device type 4, "Gyro Sensor". */
    AHRS(4),
    /** ROTEM absolute encoder. FIRST device type 7, "Encoder". */
    ENCODER(7),
    /** ROTEM USB-CAN FD bridge. FIRST device type 10, "Miscellaneous". */
    BRIDGE(10);

    private final int id;

    DeviceType(int id) {
      this.id = id;
    }

    public int id() {
      return id;
    }
  }

  /**
   * FRC CAN manufacturer ID.
   *
   * <p><b>Beta value.</b> 8 is the FIRST "Team Use" manufacturer ID and is correct only for
   * pre-release hardware. SAPPHIFY has requested an assignment from the reserved pool (21-255).
   * When it arrives this constant changes and nothing else does — that is the entire reason it
   * exists.
   *
   * <p>A release build must fail its own preflight check while this still reads 8.
   *
   * <p>One ID covers the entire series. Requesting a separate manufacturer ID per product would
   * be wrong and would be refused.
   */
  public static final int MANUFACTURER_ID = 8;

  /** Factory default device number. Configurable and persisted on the device. */
  public static final int DEFAULT_DEVICE_NUMBER = 0;

  /** Version of the protocol document this library implements. */
  public static final String PROTOCOL_VERSION = "0.9-draft";

  /** Periodic status frames. See specification section 3.1. */
  public static final class Api {
    private Api() {}

    public static final int STATUS_ORIENTATION = apiId(0, 0);
    public static final int STATUS_RATES = apiId(0, 1);
    public static final int STATUS_ACCEL = apiId(0, 2);
    public static final int STATUS_EULER = apiId(0, 3);

    public static final int STATUS_QUALITY = apiId(1, 0);
    public static final int STATUS_BIAS = apiId(1, 1);
    public static final int STATUS_VIBRATION = apiId(1, 2);
    public static final int STATUS_MAG = apiId(1, 3);

    public static final int STATUS_HEALTH = apiId(2, 0);
    public static final int STATUS_CALIBRATION = apiId(2, 1);
    public static final int STATUS_CAN = apiId(2, 2);
    public static final int STATUS_IDENTITY = apiId(2, 3);

    public static final int STATUS_FD_COMPOSITE = apiId(3, 0);

    public static final int CMD_ZERO_YAW = apiId(16, 0);
    public static final int CMD_SET_YAW = apiId(16, 1);
    public static final int CMD_RESET_ESTIMATOR = apiId(16, 2);
    public static final int CMD_IDENTIFY = apiId(16, 3);
    public static final int CMD_SELF_TEST = apiId(16, 4);

    public static final int CFG_SET_DEVICE_NUMBER = apiId(32, 0);
    public static final int CFG_SET_RATE = apiId(32, 1);
    public static final int CFG_SET_MOUNT_POSE = apiId(32, 2);
    public static final int CFG_SET_MAG_ENABLE = apiId(32, 3);
    public static final int CFG_COMMIT = apiId(32, 4);
    public static final int CFG_READ = apiId(32, 5);
  }

  /** Packs a 6-bit API class and a 4-bit API index into the 10-bit API ID. */
  public static int apiId(int apiClass, int apiIndex) {
    if (apiClass < 0 || apiClass > 0x3F) {
      throw new IllegalArgumentException("API class out of range: " + apiClass);
    }
    if (apiIndex < 0 || apiIndex > 0x0F) {
      throw new IllegalArgumentException("API index out of range: " + apiIndex);
    }
    return (apiClass << 4) | apiIndex;
  }

  /**
   * Builds the 29-bit FRC extended arbitration ID.
   *
   * <p>Layout, most significant field first: device type (5), manufacturer (8), API class (6),
   * API index (4), device number (6).
   */
  public static int arbitrationId(DeviceType deviceType, int apiId, int deviceNumber) {
    if (apiId < 0 || apiId > 0x3FF) {
      throw new IllegalArgumentException("API ID out of range: " + apiId);
    }
    if (deviceNumber < 0 || deviceNumber > 62) {
      throw new IllegalArgumentException(
          "device number out of range (0-62, 63 reserved): " + deviceNumber);
    }
    return (deviceType.id() << 24) | (MANUFACTURER_ID << 16) | (apiId << 6) | deviceNumber;
  }

  /** Health flag bits. These are identical on every ROTEM device. See specification 3.2. */
  public static final class Flag {
    private Flag() {}

    public static final int CAL_VALID = 1 << 0;
    public static final int CAL_STALE = 1 << 1;
    public static final int MOUNT_SET = 1 << 2;
    public static final int MAG_ENABLED = 1 << 3;
    public static final int MAG_DISTURBED = 1 << 4;
    public static final int GYRO_SAT = 1 << 5;
    public static final int ACCEL_SAT = 1 << 6;
    public static final int HIGH_VIBRATION = 1 << 7;
    public static final int FIFO_OVERRUN = 1 << 8;
    public static final int TIME_DISCONTINUITY = 1 << 9;
    public static final int NO_HOST = 1 << 10;
    public static final int ID_CONFLICT = 1 << 11;
    public static final int BUS_OFF_RECOVERED = 1 << 12;
    public static final int LOG_ACTIVE = 1 << 13;
    public static final int LOG_FULL = 1 << 14;
    public static final int TEMP_OUT_OF_CAL = 1 << 15;
    public static final int NUMERICAL_FAULT = 1 << 16;
    public static final int FIRMWARE_MISMATCH = 1 << 17;
  }

  /** Scaling factors. Decoders must not hard-code these anywhere else. */
  public static final class Scale {
    private Scale() {}

    /** Quaternion components are int16 divided by this. */
    public static final double QUATERNION = 32767.0;

    /** Angular rate LSB, degrees per second. */
    public static final double RATE_DPS = 0.02;

    /** Acceleration LSB, g. */
    public static final double ACCEL_G = 0.002;

    /** Yaw uncertainty and drift-estimate LSB, degrees. */
    public static final double ANGLE_MILLIDEG = 0.001;

    /** Gyro bias magnitude LSB, degrees per second. */
    public static final double BIAS_DPS = 0.0001;
  }
}
