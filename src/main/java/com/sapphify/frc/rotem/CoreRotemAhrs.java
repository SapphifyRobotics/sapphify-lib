package com.sapphify.frc.rotem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A ROTEM AHRS, with no dependency on WPILib.
 *
 * <p>This is the layer that actually talks to the device. It is deliberately free of WPILib types
 * so that the same code drives the robot library, the command-line tool, the desktop configurator
 * and log replay. {@code RotemAhrs} is a thin subclass that adds WPILib integration and nothing
 * else — the split is the same one CTRE uses between {@code CorePigeon2} and {@code Pigeon2}, and
 * it is what keeps a bench tool from dragging in a robot framework.
 *
 * <p>Nothing here throws for a device or bus condition; reads return a {@link RotemSignal} carrying
 * a {@link RotemStatusCode}, and commands return a status directly.
 *
 * <h2>Reading orientation</h2>
 *
 * <pre>{@code
 * var imu = new CoreRotemAhrs(0, transport);
 * var yaw = imu.getYaw();
 * if (yaw.isValid()) {
 *   useHeading(yaw.value(), yaw.deviceTimestampSeconds());
 * }
 * }</pre>
 *
 * <h2>Knowing when not to trust it</h2>
 *
 * <pre>{@code
 * var quality = imu.getYawUncertainty();
 * if (quality.isValid() && quality.value() > 2.0) {
 *   // the filter itself says the heading is uncertain
 * }
 * for (String alert : imu.getActiveAlerts()) {
 *   reportToDashboard(alert);
 * }
 * }</pre>
 */
public class CoreRotemAhrs implements AutoCloseable {

  /** Orientation older than this is not usable for pose estimation. */
  public static final double DEFAULT_STALENESS_LIMIT_SECONDS = 0.25;

  private final int deviceNumber;
  private final RotemTransport transport;
  private final RotemConfigurator configurator;

  /**
   * Constructs a device.
   *
   * @param deviceNumber 0 to 62; the factory default is 0
   * @param transport how to reach it; {@link RotemSimTransport} for tests
   */
  public CoreRotemAhrs(int deviceNumber, RotemTransport transport) {
    if (deviceNumber < 0 || deviceNumber > 62) {
      throw new IllegalArgumentException(
          "device number must be 0-62 (63 is reserved), got " + deviceNumber);
    }
    this.deviceNumber = deviceNumber;
    this.transport = transport;
    this.configurator = new RotemConfigurator(transport, deviceType(), deviceNumber);
  }

  /** The FRC device type this class speaks to. Overridden by nothing; AHRS is device type 4. */
  protected RotemProtocol.DeviceType deviceType() {
    return RotemProtocol.DeviceType.AHRS;
  }

  /** This device's number on the bus. */
  public int getDeviceNumber() {
    return deviceNumber;
  }

  /** Configuration interface for this device. */
  public RotemConfigurator getConfigurator() {
    return configurator;
  }

  // ---- Orientation -----------------------------------------------------------------------

  /**
   * Current heading.
   *
   * <ul>
   *   <li><b>Units:</b> deg
   *   <li><b>Default rate:</b> 100 Hz on CAN 2.0 and CAN FD
   *   <li><b>Frame:</b> {@code STATUS_ORIENTATION}
   * </ul>
   *
   * <p>Counter-clockwise positive, matching WPILib convention. Not an absolute heading unless an
   * absolute reference has been accepted — see {@link #getYawUncertainty()}.
   */
  public RotemSignal<Double> getYaw() {
    return decodeOrientation("Yaw", q -> Math.toDegrees(yawFromQuaternion(q)), "deg");
  }

  /**
   * Full orientation as a quaternion, ordered {@code [w, x, y, z]}.
   *
   * <ul>
   *   <li><b>Default rate:</b> 100 Hz
   *   <li><b>Frame:</b> {@code STATUS_ORIENTATION}
   * </ul>
   *
   * <p>The quaternion is the authoritative state on the device; roll, pitch and yaw are derived at
   * the output boundary. Prefer this when you need full 3-D orientation, because deriving Euler
   * angles and recomposing them loses information near gimbal lock.
   */
  public RotemSignal<double[]> getQuaternion() {
    return decodeOrientation(
        "Quaternion", q -> new double[] {q.w(), q.x(), q.y(), q.z()}, "");
  }

  /**
   * The filter's own one-sigma uncertainty in heading.
   *
   * <ul>
   *   <li><b>Units:</b> deg
   *   <li><b>Default rate:</b> 10 Hz
   *   <li><b>Frame:</b> {@code STATUS_QUALITY}
   * </ul>
   *
   * <p>No other FRC IMU publishes this. It is the difference between "our odometry drifted" as a
   * mystery and as a measurement.
   */
  public RotemSignal<Double> getYawUncertainty() {
    return decodeQuality("YawUncertainty", RotemDecoder.Quality::yawSigmaDegrees, "deg");
  }

  /**
   * Estimated heading error accumulated since the last zero.
   *
   * <ul>
   *   <li><b>Units:</b> deg
   *   <li><b>Default rate:</b> 10 Hz
   *   <li><b>Frame:</b> {@code STATUS_QUALITY}
   * </ul>
   *
   * <p>Reported even when magnetometer fusion is disabled: with fusion off the magnetometer still
   * runs as a drift auditor, estimating error without touching the published heading.
   */
  public RotemSignal<Double> getAccumulatedDrift() {
    return decodeQuality("AccumulatedDrift", RotemDecoder.Quality::driftSinceZeroDegrees, "deg");
  }

  /** Decoded device health, including every fault flag. Default rate 4 Hz. */
  public RotemSignal<RotemDecoder.Health> getHealth() {
    return decode(
        "Health",
        RotemProtocol.Api.STATUS_HEALTH,
        RotemDecoder::decodeHealth,
        "",
        new RotemDecoder.Health(0, 0, 0, 0, 0));
  }

  // ---- Commands --------------------------------------------------------------------------

  /** Sets the current heading to zero. */
  public RotemStatusCode zeroYaw() {
    return send(RotemProtocol.Api.CMD_ZERO_YAW, new byte[0]);
  }

  /**
   * Blinks the device's LEDs so a human can find it in a robot.
   *
   * @param seconds how long to blink, 1 to 60
   */
  public RotemStatusCode identify(int seconds) {
    if (seconds < 1 || seconds > 60) {
      return RotemStatusCode.INVALID_PARAMETER;
    }
    return send(RotemProtocol.Api.CMD_IDENTIFY, new byte[] {(byte) seconds});
  }

  /** Runs the device's built-in self test. */
  public RotemStatusCode selfTest() {
    return send(RotemProtocol.Api.CMD_SELF_TEST, new byte[0]);
  }

  /**
   * Requests a publication rate for one status frame.
   *
   * @param apiId one of the {@code STATUS_*} constants in {@link RotemProtocol.Api}
   * @param hz requested rate; zero disables the frame
   */
  public RotemStatusCode setUpdateFrequency(int apiId, double hz) {
    if (transport == null) {
      return RotemStatusCode.NO_TRANSPORT;
    }
    return transport.setUpdateFrequency(
        RotemProtocol.arbitrationId(deviceType(), apiId, deviceNumber), hz);
  }

  // ---- Diagnosis -------------------------------------------------------------------------

  /**
   * Human-readable descriptions of everything currently wrong, ready to show a driver.
   *
   * <p>The WPILib layer feeds these straight into the Alert API. The text names the likely
   * mechanical or electrical cause rather than the bit that was set, because the person reading it
   * in a pit is usually not the person who wrote the robot code.
   *
   * @return an empty list when the device is healthy
   */
  public List<String> getActiveAlerts() {
    List<String> alerts = new ArrayList<>();
    RotemSignal<RotemDecoder.Health> health = getHealth();

    if (!health.isValid()) {
      alerts.add("ROTEM AHRS " + deviceNumber + ": " + health.status().description());
      return alerts;
    }

    RotemDecoder.Health h = health.value();
    if (h.has(RotemProtocol.Flag.ID_CONFLICT)) {
      alerts.add(
          "ROTEM AHRS "
              + deviceNumber
              + ": another device answers on this device number. Give each device a unique number.");
    }
    if (h.has(RotemProtocol.Flag.NUMERICAL_FAULT)) {
      alerts.add(
          "ROTEM AHRS "
              + deviceNumber
              + ": estimator numerical fault. Heading is not trustworthy until it reconverges.");
    }
    if (!h.has(RotemProtocol.Flag.CAL_VALID)) {
      alerts.add(
          "ROTEM AHRS " + deviceNumber + ": no valid factory calibration. Contact SAPPHIFY.");
    }
    if (h.has(RotemProtocol.Flag.HIGH_VIBRATION)) {
      alerts.add(
          "ROTEM AHRS "
              + deviceNumber
              + ": high vibration. Check swerve module bearings and belt tension; the vibration"
              + " analyser names the offending frequency.");
    }
    if (h.has(RotemProtocol.Flag.MAG_DISTURBED)) {
      alerts.add(
          "ROTEM AHRS "
              + deviceNumber
              + ": magnetic disturbance detected, heading correction suspended.");
    }
    if (h.has(RotemProtocol.Flag.GYRO_SAT) || h.has(RotemProtocol.Flag.ACCEL_SAT)) {
      alerts.add(
          "ROTEM AHRS "
              + deviceNumber
              + ": sensor saturated. Motion exceeded the configured range; heading accuracy is"
              + " degraded for this interval.");
    }
    if (h.has(RotemProtocol.Flag.FIFO_OVERRUN)) {
      alerts.add("ROTEM AHRS " + deviceNumber + ": sensor FIFO overrun; samples were lost.");
    }
    if (h.has(RotemProtocol.Flag.TEMP_OUT_OF_CAL)) {
      alerts.add(
          "ROTEM AHRS "
              + deviceNumber
              + ": temperature outside the calibrated range; bias compensation is extrapolated.");
    }
    if (h.has(RotemProtocol.Flag.FIRMWARE_MISMATCH)) {
      alerts.add(
          "ROTEM AHRS "
              + deviceNumber
              + ": firmware and library protocol versions differ. Update the older one.");
    }
    return alerts;
  }

  // ---- Internals -------------------------------------------------------------------------

  private record Quat(double w, double x, double y, double z) {}

  private <T> RotemSignal<T> decodeOrientation(
      String name, java.util.function.Function<Quat, T> map, String units) {
    return decode(
        name,
        RotemProtocol.Api.STATUS_ORIENTATION,
        data -> {
          RotemDecoder.Orientation o = RotemDecoder.decodeOrientation(data);
          return map.apply(new Quat(o.w(), o.x(), o.y(), o.z()));
        },
        units,
        null);
  }

  private RotemSignal<Double> decodeQuality(
      String name, java.util.function.Function<RotemDecoder.Quality, Double> map, String units) {
    return decode(
        name,
        RotemProtocol.Api.STATUS_QUALITY,
        data -> map.apply(RotemDecoder.decodeQuality(data)),
        units,
        Double.NaN);
  }

  private <T> RotemSignal<T> decode(
      String name,
      int apiId,
      java.util.function.Function<byte[], T> decoder,
      String units,
      T fallback) {
    if (transport == null) {
      return RotemSignal.failed(name, RotemStatusCode.NO_TRANSPORT, fallback);
    }
    Optional<RotemTransport.Frame> frame =
        transport.latestFrame(RotemProtocol.arbitrationId(deviceType(), apiId, deviceNumber));
    if (frame.isEmpty()) {
      return RotemSignal.failed(name, RotemStatusCode.DEVICE_NOT_PRESENT, fallback);
    }
    try {
      T value = decoder.apply(frame.get().data());
      // Both timestamps are the frame's arrival time for now. The device-side sample timestamp
      // is not yet carried on classic CAN — that is the one unresolved design point in the
      // protocol specification (section 3.2), and inventing a value here would make latency
      // compensation look implemented when it is not. Age, and therefore staleness, is correct
      // today; latency becomes meaningful when the protocol carries the sample timestamp.
      double arrival = frame.get().timestampSeconds();
      return RotemSignal.of(name, value, units, arrival, arrival);
    } catch (IllegalArgumentException e) {
      return RotemSignal.failed(name, RotemStatusCode.MALFORMED_FRAME, fallback);
    }
  }

  private RotemStatusCode send(int apiId, byte[] payload) {
    if (transport == null) {
      return RotemStatusCode.NO_TRANSPORT;
    }
    return transport.send(
        new RotemTransport.Frame(
            RotemProtocol.arbitrationId(deviceType(), apiId, deviceNumber),
            payload,
            transport.currentTimeSeconds()));
  }

  /** Standard aerospace yaw extraction from a quaternion. */
  private static double yawFromQuaternion(Quat q) {
    return Math.atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }

  @Override
  public void close() {
    // The transport is owned by the caller: several devices share one bus, so closing a device
    // must not close the bus out from under its siblings.
  }
}
