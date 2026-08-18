package com.sapphify.frc.rotem;

import java.util.Optional;

/**
 * How the library reaches a ROTEM device.
 *
 * <p>Everything above this interface — decoding, signals, configuration, alerts — is transport
 * agnostic. That is the whole point: the same core drives the WPILib CAN bus on a robot, SocketCAN
 * on a Linux laptop through our USB-CAN FD bridge, a USB CDC connection to a device on a bench with
 * no robot at all, and a recorded log during replay. Four products, one implementation.
 *
 * <p>It also means the entire library is testable without hardware. {@link RotemSimTransport} is a
 * complete implementation in a few dozen lines.
 *
 * <p>Implementations must be safe to call from a background thread.
 */
public interface RotemTransport extends AutoCloseable {

  /**
   * A frame as it appeared on the wire.
   *
   * @param arbitrationId the 29-bit extended identifier
   * @param data payload, 0-64 bytes
   * @param timestampSeconds when this frame was received, from a monotonic source
   */
  record Frame(int arbitrationId, byte[] data, double timestampSeconds) {

    public Frame {
      if (data == null) {
        throw new IllegalArgumentException("frame data must not be null");
      }
      if (data.length > 64) {
        throw new IllegalArgumentException("CAN FD frames carry at most 64 bytes, got " + data.length);
      }
    }
  }

  /**
   * Most recently received frame with this arbitration ID, if one has ever arrived.
   *
   * <p>Returns the cached frame without blocking. Staleness is the caller's judgement, made with
   * {@link RotemSignal#isStale(double, double)} — this method deliberately does not decide for
   * them, because the acceptable age of a heading and of a calibration-age report differ by three
   * orders of magnitude.
   */
  Optional<Frame> latestFrame(int arbitrationId);

  /** Sends one frame. Returns a status rather than throwing. */
  RotemStatusCode send(Frame frame);

  /**
   * Requests a particular publication rate for one arbitration ID.
   *
   * @param hz requested rate; zero disables the frame entirely
   */
  RotemStatusCode setUpdateFrequency(int arbitrationId, double hz);

  /**
   * Current time from the same monotonic source used to stamp frames.
   *
   * <p>The transport owns the clock so that replay and unit tests are deterministic. Nothing above
   * this interface may read a wall clock.
   */
  double currentTimeSeconds();

  @Override
  void close();
}
