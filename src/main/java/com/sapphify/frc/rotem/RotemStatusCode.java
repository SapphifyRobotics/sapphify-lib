package com.sapphify.frc.rotem;

/**
 * Every outcome a ROTEM API call can produce.
 *
 * <p><b>Nothing in this library throws for a device or bus condition.</b> A device on a CAN bus can
 * legitimately disappear mid-match — a connector vibrates loose, a bus browns out, a team unplugs
 * the wrong thing in the pit. Robot code that dies with an exception in that situation is worse
 * than robot code that keeps driving with a stale heading and a reported fault. Programming errors
 * (a null argument, an out-of-range device number) still throw, because those are bugs rather than
 * field conditions.
 *
 * <p>Each value documents <b>what to do about it</b>, not merely what happened. The message is
 * what a student reads at 2 a.m. in a pit with a match in twenty minutes, and it is the same text
 * the library surfaces through the WPILib Alert API.
 */
public enum RotemStatusCode {

  /** Call succeeded. */
  OK(0, "No error.", true),

  // ---- Bus and presence ------------------------------------------------------------------

  /**
   * No frame has been received from this device within the timeout.
   */
  DEVICE_NOT_PRESENT(
      -100,
      "No ROTEM device answered at this device number. Verify the device is powered (the SYS LED"
          + " is lit), that the CAN wiring is intact, and that the bus is terminated at both ends"
          + " — measure about 60 ohms across CANH and CANL with the robot off. 120 ohms means one"
          + " termination is missing.",
      false),

  /**
   * Frames are arriving but they stopped being fresh.
   */
  SIGNAL_STALE(
      -101,
      "The device was present but its data has stopped updating. Check bus utilisation, and check"
          + " whether this signal's update frequency was set to zero.",
      false),

  /**
   * Two devices are answering on the same Device Type and Device Number.
   */
  DEVICE_ID_CONFLICT(
      -102,
      "Another device is answering on the same device number. Every ROTEM device of the same type"
          + " needs a unique device number. Assign one over USB with no bus and no robot code, or"
          + " over CAN with the configuration tool.",
      false),

  /** The CAN controller entered bus-off and is recovering. */
  BUS_OFF(
      -103,
      "The CAN controller went bus-off, which almost always means a wiring fault or a bit-rate"
          + " mismatch rather than a device fault. Recovery is automatic; if it repeats, inspect"
          + " the wiring before replacing anything.",
      false),

  // ---- Protocol and versioning -----------------------------------------------------------

  /** The device speaks a protocol version this library does not support. */
  PROTOCOL_VERSION_MISMATCH(
      -110,
      "The device firmware and this library implement different protocol versions. Update whichever"
          + " is older; the pairing is reported in the device identity frame.",
      false),

  /**
   * A release build is still carrying the beta manufacturer ID.
   *
   * <p>This exists to make an internal mistake loud rather than silent. Manufacturer ID 8 is the
   * FIRST "Team Use" identifier and must never reach a released product.
   */
  BETA_MANUFACTURER_ID(
      -111,
      "This build is using the FIRST \"Team Use\" manufacturer ID (8), which is valid for"
          + " pre-release hardware only. A released build must carry the manufacturer ID assigned"
          + " to SAPPHIFY by FIRST.",
      false),

  /** A received frame did not carry the number of bytes its type requires. */
  MALFORMED_FRAME(
      -112, "A frame arrived with an unexpected length and was discarded.", false),

  // ---- Configuration ---------------------------------------------------------------------

  /** A configuration value was outside its permitted range. */
  INVALID_PARAMETER(
      -120,
      "A configuration value was outside its permitted range and nothing was applied. The"
          + " permitted range is in the javadoc for the field you set.",
      false),

  /** The device did not acknowledge a configuration commit before the timeout. */
  CONFIG_TIMEOUT(
      -121,
      "The device did not acknowledge the configuration within the timeout. The previous"
          + " configuration is still in effect — a commit is atomic and never leaves the device"
          + " half-configured. Retry with a longer timeout.",
      false),

  /**
   * Configuration is being applied continuously in a robot loop.
   *
   * <p>Copied deliberately from Phoenix's frequent-config detector. Applying configuration every
   * loop iteration wears flash and floods the bus, and it is almost always an accident — a
   * {@code configure()} call that belongs in a constructor ended up in {@code periodic()}.
   */
  FREQUENT_CONFIG_CALLS(
      -122,
      "Configuration is being applied repeatedly in a loop. Configuration belongs in your"
          + " subsystem constructor, not in periodic(). Repeated commits wear the device's flash"
          + " and consume bus bandwidth.",
      false),

  /** Stored configuration failed its CRC check. */
  CONFIG_CORRUPT(
      -123,
      "The stored configuration failed its integrity check and factory defaults were loaded."
          + " Re-apply your configuration and report this — it should not happen.",
      false),

  // ---- Estimator and calibration ---------------------------------------------------------

  /** The device has no valid factory calibration. */
  CALIBRATION_INVALID(
      -130,
      "This device has no valid factory calibration, so its heading accuracy is not the specified"
          + " figure. Contact SAPPHIFY with the serial number from the identity frame.",
      false),

  /** The estimator has not converged yet. */
  ESTIMATOR_NOT_CONVERGED(
      -131,
      "The attitude estimator is still converging. Leave the robot still for a moment after power"
          + " up; the quality frame reports when it has converged.",
      false),

  /** The estimator detected a numerical fault and reinitialised. */
  ESTIMATOR_FAULT(
      -132,
      "The attitude estimator hit a numerical fault and reinitialised itself rather than publish a"
          + " plausible-looking wrong orientation. Heading is not trustworthy until it reports"
          + " converged again.",
      false),

  // ---- Local programming errors ----------------------------------------------------------

  /** The requested operation is not supported by this device type. */
  UNSUPPORTED_ON_DEVICE(
      -140, "This operation is not supported by this ROTEM device type.", false),

  /** No transport has been attached. */
  NO_TRANSPORT(
      -141,
      "This device has no transport attached, so it cannot reach the hardware. Construct it with"
          + " a transport, or use the simulation transport in unit tests.",
      false);

  private final int value;
  private final String description;
  private final boolean ok;

  RotemStatusCode(int value, String description, boolean ok) {
    this.value = value;
    this.description = description;
    this.ok = ok;
  }

  /** The wire/log value. Stable across releases; never renumber these. */
  public int value() {
    return value;
  }

  /** Human-readable explanation including the remedy. Suitable for a dashboard alert verbatim. */
  public String description() {
    return description;
  }

  /** True only for {@link #OK}. */
  public boolean isOK() {
    return ok;
  }

  /** True for anything that is not {@link #OK}. */
  public boolean isError() {
    return !ok;
  }

  /** Looks up a status code by wire value, or {@code null} if unknown. */
  public static RotemStatusCode fromValue(int value) {
    for (RotemStatusCode c : values()) {
      if (c.value == value) {
        return c;
      }
    }
    return null;
  }

  @Override
  public String toString() {
    return name() + " (" + value + "): " + description;
  }
}
