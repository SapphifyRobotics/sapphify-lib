package com.sapphify.frc.hardware;

import com.sapphify.frc.SapphifyStatusCode;

/**
 * The complete configuration of a ROTEM AHRS, as a plain value object.
 *
 * <p>Configuration is a <em>value</em>, not a sequence of setter calls. You build the object you
 * want, apply it, and the device either takes all of it or none of it. A freshly constructed
 * instance is exactly the factory default, so {@code configurator.apply(new
 * RotemConfiguration())} is how you reset a device — there is no separate "factory reset" verb
 * to remember.
 *
 * <p>Sections mirror the configuration API in the protocol specification so that the library, the
 * firmware and the configuration tool cannot drift apart.
 */
public class RotemConfiguration {

  /** How the board is physically mounted relative to the robot chassis. */
  public MountPose mountPose = new MountPose();

  /** Optional behaviour that is off by default. */
  public Features features = new Features();

  /**
   * Physical mounting orientation, in degrees.
   *
   * <p>Applied on the device rather than in robot code, and persisted, so that a board mounted
   * sideways reports robot-frame orientation to everything on the bus — including the
   * configuration tool and the black-box log — rather than only to the one program that remembered
   * to rotate it.
   *
   * <p>Pigeon 2's mount calibration not persisting across power cycles is a documented team
   * complaint. Ours commits atomically to wear-levelled flash with a CRC.
   */
  public static class MountPose {
    /**
     * Rotation about the board Z axis.
     *
     * <ul>
     *   <li><b>Minimum:</b> -180.0
     *   <li><b>Maximum:</b> 180.0
     *   <li><b>Default:</b> 0.0
     *   <li><b>Units:</b> deg
     * </ul>
     */
    public double yawDegrees = 0.0;

    /**
     * Rotation about the board Y axis.
     *
     * <ul>
     *   <li><b>Minimum:</b> -180.0
     *   <li><b>Maximum:</b> 180.0
     *   <li><b>Default:</b> 0.0
     *   <li><b>Units:</b> deg
     * </ul>
     */
    public double pitchDegrees = 0.0;

    /**
     * Rotation about the board X axis.
     *
     * <ul>
     *   <li><b>Minimum:</b> -180.0
     *   <li><b>Maximum:</b> 180.0
     *   <li><b>Default:</b> 0.0
     *   <li><b>Units:</b> deg
     * </ul>
     */
    public double rollDegrees = 0.0;

    SapphifyStatusCode validate() {
      return inRange(yawDegrees) && inRange(pitchDegrees) && inRange(rollDegrees)
          ? SapphifyStatusCode.OK
          : SapphifyStatusCode.INVALID_PARAMETER;
    }

    private static boolean inRange(double v) {
      return !Double.isNaN(v) && v >= -180.0 && v <= 180.0;
    }
  }

  /** Opt-in behaviour. Every default here is the conservative choice. */
  public static class Features {

    /**
     * Whether magnetometer measurements are fused into the heading estimate.
     *
     * <p><b>Default: false, and that is a considered position rather than an oversight.</b> An FRC
     * field is full of motors, steel and current; a silently mag-aided heading that shifts when a
     * neighbouring robot drives past is worse than no magnetometer at all, because the failure is
     * invisible. With fusion off the magnetometer still runs as a <em>drift auditor</em>: it
     * reports estimated heading error since zeroing without touching the published heading.
     *
     * <p>When enabled, corrections remain gated by the disturbance detector and are applied
     * gradually. Heading never steps.
     */
    public boolean magnetometerFusionEnabled = false;

    /**
     * Whether the onboard black-box recorder runs.
     *
     * <p>Default true. It records independently of robot code and independently of the bus, so a
     * team still has ground truth after a crash, a brownout or a lost roboRIO log.
     */
    public boolean blackBoxRecorderEnabled = true;

    /**
     * Seconds without a robot-controller heartbeat before the device reports {@code FLAG_NO_HOST}.
     *
     * <ul>
     *   <li><b>Minimum:</b> 0.1
     *   <li><b>Maximum:</b> 30.0
     *   <li><b>Default:</b> 2.0
     *   <li><b>Units:</b> s
     * </ul>
     */
    public double hostTimeoutSeconds = 2.0;

    SapphifyStatusCode validate() {
      if (Double.isNaN(hostTimeoutSeconds)
          || hostTimeoutSeconds < 0.1
          || hostTimeoutSeconds > 30.0) {
        return SapphifyStatusCode.INVALID_PARAMETER;
      }
      return SapphifyStatusCode.OK;
    }
  }

  /**
   * Checks every field against its documented range.
   *
   * <p>Validation happens on this side of the bus so that an out-of-range value costs nothing and
   * reports precisely, instead of consuming a round trip to be rejected by firmware.
   */
  public SapphifyStatusCode validate() {
    SapphifyStatusCode status = mountPose.validate();
    if (status.isError()) {
      return status;
    }
    return features.validate();
  }
}
