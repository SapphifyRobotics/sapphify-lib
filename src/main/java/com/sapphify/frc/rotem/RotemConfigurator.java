package com.sapphify.frc.rotem;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Applies and reads back device configuration.
 *
 * <p>Obtained from a device with {@code device.getConfigurator()}. Every method returns a {@link
 * RotemStatusCode} and none of them throws for a device condition.
 *
 * <h2>Commits are atomic</h2>
 *
 * An apply stages every value, range-checks it, then commits once. A brown-out mid-commit leaves
 * the previous configuration intact. There is no state in which a device is half configured.
 *
 * <h2>Misuse detection</h2>
 *
 * Applying configuration repeatedly in a robot loop wears the device's flash and floods the bus,
 * and it is nearly always an accident — a {@code configure()} call that belongs in a subsystem
 * constructor ends up in {@code periodic()}. After {@value #MISUSE_WINDOW_SECONDS} seconds of
 * sustained high-rate applies, this class returns {@link RotemStatusCode#FREQUENT_CONFIG_CALLS}
 * instead of continuing to write. A {@value #GRACE_PERIOD_SECONDS}-second grace period after
 * construction keeps legitimate start-up configuration bursts quiet.
 */
public final class RotemConfigurator {

  /** Applies faster than this are considered loop-rate rather than deliberate. */
  static final double MISUSE_RATE_HZ = 1.0;

  /** How long sustained high-rate applies are tolerated before reporting misuse. */
  static final double MISUSE_WINDOW_SECONDS = 3.0;

  /** Start-up window during which misuse detection stays quiet. */
  static final double GRACE_PERIOD_SECONDS = 5.0;

  /** Default time allowed for the device to acknowledge a commit. */
  public static final double DEFAULT_TIMEOUT_SECONDS = 0.100;

  private final RotemTransport transport;
  private final RotemProtocol.DeviceType deviceType;
  private final int deviceNumber;
  private final double constructedAtSeconds;

  private final Object lock = new Object();
  private double lastApplySeconds = Double.NaN;
  private double sustainedSinceSeconds = Double.NaN;

  RotemConfigurator(
      RotemTransport transport, RotemProtocol.DeviceType deviceType, int deviceNumber) {
    this.transport = transport;
    this.deviceType = deviceType;
    this.deviceNumber = deviceNumber;
    this.constructedAtSeconds = transport == null ? 0.0 : transport.currentTimeSeconds();
  }

  /** Applies a whole configuration with the default timeout. */
  public RotemStatusCode apply(RotemAhrsConfiguration configuration) {
    return apply(configuration, DEFAULT_TIMEOUT_SECONDS);
  }

  /**
   * Applies a whole configuration.
   *
   * @param configuration the desired configuration; a freshly constructed instance is the factory
   *     default
   * @param timeoutSeconds how long to wait for the device to acknowledge; zero does not wait
   */
  public RotemStatusCode apply(RotemAhrsConfiguration configuration, double timeoutSeconds) {
    if (configuration == null) {
      throw new IllegalArgumentException("configuration must not be null");
    }
    if (transport == null) {
      return RotemStatusCode.NO_TRANSPORT;
    }

    RotemStatusCode validity = configuration.validate();
    if (validity.isError()) {
      return validity;
    }

    RotemStatusCode misuse = recordApplyAndCheckMisuse(transport.currentTimeSeconds());
    if (misuse.isError()) {
      return misuse;
    }

    RotemStatusCode staged = apply(configuration.mountPose, timeoutSeconds);
    if (staged.isError()) {
      return staged;
    }
    staged = apply(configuration.features, timeoutSeconds);
    if (staged.isError()) {
      return staged;
    }
    return commit(timeoutSeconds);
  }

  /** Applies only the mount pose, leaving everything else untouched. */
  public RotemStatusCode apply(RotemAhrsConfiguration.MountPose mountPose, double timeoutSeconds) {
    RotemStatusCode validity = mountPose.validate();
    if (validity.isError()) {
      return validity;
    }
    ByteBuffer payload = ByteBuffer.allocate(6).order(ByteOrder.LITTLE_ENDIAN);
    payload.putShort(toCentiDegrees(mountPose.yawDegrees));
    payload.putShort(toCentiDegrees(mountPose.pitchDegrees));
    payload.putShort(toCentiDegrees(mountPose.rollDegrees));
    return send(RotemProtocol.Api.CFG_SET_MOUNT_POSE, payload.array());
  }

  /** Applies only the feature flags, leaving everything else untouched. */
  public RotemStatusCode apply(RotemAhrsConfiguration.Features features, double timeoutSeconds) {
    RotemStatusCode validity = features.validate();
    if (validity.isError()) {
      return validity;
    }
    ByteBuffer payload = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
    payload.put((byte) (features.magnetometerFusionEnabled ? 1 : 0));
    payload.put((byte) (features.blackBoxRecorderEnabled ? 1 : 0));
    payload.putShort((short) Math.round(features.hostTimeoutSeconds * 100.0));
    return send(RotemProtocol.Api.CFG_SET_MAG_ENABLE, payload.array());
  }

  /** Assigns a new device number. Persisted; survives power cycles. */
  public RotemStatusCode setDeviceNumber(int newDeviceNumber) {
    if (newDeviceNumber < 0 || newDeviceNumber > 62) {
      return RotemStatusCode.INVALID_PARAMETER;
    }
    RotemStatusCode staged =
        send(RotemProtocol.Api.CFG_SET_DEVICE_NUMBER, new byte[] {(byte) newDeviceNumber});
    return staged.isError() ? staged : commit(DEFAULT_TIMEOUT_SECONDS);
  }

  private RotemStatusCode commit(double timeoutSeconds) {
    return send(RotemProtocol.Api.CFG_COMMIT, new byte[0]);
  }

  private RotemStatusCode send(int apiId, byte[] payload) {
    if (transport == null) {
      return RotemStatusCode.NO_TRANSPORT;
    }
    int arbitrationId = RotemProtocol.arbitrationId(deviceType, apiId, deviceNumber);
    return transport.send(
        new RotemTransport.Frame(arbitrationId, payload, transport.currentTimeSeconds()));
  }

  /**
   * Records an apply and reports whether the caller is applying configuration at loop rate.
   *
   * <p>Visible for testing.
   */
  RotemStatusCode recordApplyAndCheckMisuse(double nowSeconds) {
    synchronized (lock) {
      double previous = lastApplySeconds;
      lastApplySeconds = nowSeconds;

      if (nowSeconds - constructedAtSeconds < GRACE_PERIOD_SECONDS) {
        return RotemStatusCode.OK;
      }
      if (Double.isNaN(previous)) {
        return RotemStatusCode.OK;
      }

      boolean fast = (nowSeconds - previous) < (1.0 / MISUSE_RATE_HZ);
      if (!fast) {
        sustainedSinceSeconds = Double.NaN;
        return RotemStatusCode.OK;
      }
      if (Double.isNaN(sustainedSinceSeconds)) {
        sustainedSinceSeconds = previous;
        return RotemStatusCode.OK;
      }
      return (nowSeconds - sustainedSinceSeconds) >= MISUSE_WINDOW_SECONDS
          ? RotemStatusCode.FREQUENT_CONFIG_CALLS
          : RotemStatusCode.OK;
    }
  }

  private static short toCentiDegrees(double degrees) {
    return (short) Math.round(degrees * 100.0);
  }
}
