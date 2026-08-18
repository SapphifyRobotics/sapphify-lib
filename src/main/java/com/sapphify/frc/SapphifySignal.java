package com.sapphify.frc;

import java.util.Objects;

/**
 * One timestamped measurement from a ROTEM device.
 *
 * <p>Modelled on Phoenix's {@code StatusSignal}: a signal is an object you hold onto, not a
 * {@code double} you fetch. Holding the object is what makes latency compensation, staleness
 * detection and update-frequency control possible at all — a bare {@code double} has thrown away
 * everything you need to know about how old it is.
 *
 * <p>Two timestamps are carried, and the difference matters:
 *
 * <ul>
 *   <li>{@link #deviceTimestampSeconds()} — when the device sampled the sensor. Use this one for
 *       pose estimation; it is the only timestamp that reflects reality.
 *   <li>{@link #receivedTimestampSeconds()} — when the frame reached this robot. Use this only to
 *       judge staleness.
 * </ul>
 *
 * <p>Instances are immutable. A device getter returns a fresh instance rather than mutating a
 * shared one, so handing a signal to a dashboard, a logger and a control loop cannot produce three
 * different readings of the same variable.
 *
 * @param <T> the measured value type
 */
public final class SapphifySignal<T> {

  private final String name;
  private final T value;
  private final String units;
  private final double deviceTimestampSeconds;
  private final double receivedTimestampSeconds;
  private final SapphifyStatusCode status;

  private SapphifySignal(
      String name,
      T value,
      String units,
      double deviceTimestampSeconds,
      double receivedTimestampSeconds,
      SapphifyStatusCode status) {
    this.name = Objects.requireNonNull(name, "name");
    this.value = value;
    this.units = units == null ? "" : units;
    this.deviceTimestampSeconds = deviceTimestampSeconds;
    this.receivedTimestampSeconds = receivedTimestampSeconds;
    this.status = Objects.requireNonNull(status, "status");
  }

  /** Builds a good measurement. */
  public static <T> SapphifySignal<T> of(
      String name,
      T value,
      String units,
      double deviceTimestampSeconds,
      double receivedTimestampSeconds) {
    return new SapphifySignal<>(
        name, value, units, deviceTimestampSeconds, receivedTimestampSeconds, SapphifyStatusCode.OK);
  }

  /**
   * Builds a failed measurement.
   *
   * <p>A failed read produces one of these rather than an exception or a {@code null}. Callers that
   * ignore the status get a documented fallback value instead of a crash; callers that check it get
   * a remedy they can print.
   */
  public static <T> SapphifySignal<T> failed(String name, SapphifyStatusCode status, T fallbackValue) {
    if (status.isOK()) {
      throw new IllegalArgumentException("failed() requires an error status, got " + status);
    }
    return new SapphifySignal<>(name, fallbackValue, "", Double.NaN, Double.NaN, status);
  }

  /** Signal name, matching the field name in the protocol specification. */
  public String name() {
    return name;
  }

  /** The measured value. On a failed read this is the documented fallback, never {@code null}. */
  public T value() {
    return value;
  }

  /** Unit string as published in the specification, for example {@code "deg"} or {@code "deg/s"}. */
  public String units() {
    return units;
  }

  /** When the <em>device</em> sampled this. The timestamp to use for pose estimation. */
  public double deviceTimestampSeconds() {
    return deviceTimestampSeconds;
  }

  /** When this robot received the frame. Use only to judge staleness. */
  public double receivedTimestampSeconds() {
    return receivedTimestampSeconds;
  }

  /** Outcome of the read. */
  public SapphifyStatusCode status() {
    return status;
  }

  /** True if this measurement is usable. */
  public boolean isValid() {
    return status.isOK();
  }

  /**
   * Transport and processing delay: how long the value had already been true before it arrived.
   *
   * <p>Returns {@link Double#NaN} if either timestamp is unavailable.
   */
  public double latencySeconds() {
    return receivedTimestampSeconds - deviceTimestampSeconds;
  }

  /**
   * Age of this measurement relative to a supplied "now".
   *
   * <p>The caller passes the current time rather than the signal reading a clock, so that unit
   * tests and log replay are deterministic — the same log must produce the same answer on every
   * run, which is impossible if the library samples a wall clock internally.
   */
  public double ageSeconds(double nowSeconds) {
    return nowSeconds - receivedTimestampSeconds;
  }

  /** True if this measurement is older than {@code maxAgeSeconds}, or invalid. */
  public boolean isStale(double nowSeconds, double maxAgeSeconds) {
    return !isValid() || ageSeconds(nowSeconds) > maxAgeSeconds;
  }

  @Override
  public String toString() {
    if (!isValid()) {
      return name + " = <" + status.name() + ">";
    }
    return units.isEmpty() ? name + " = " + value : name + " = " + value + " " + units;
  }
}
