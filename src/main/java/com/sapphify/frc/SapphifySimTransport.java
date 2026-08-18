package com.sapphify.frc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * In-memory transport for unit tests, log replay and simulation.
 *
 * <p>Time is advanced explicitly with {@link #advanceTime(double)} rather than read from a clock,
 * so a test that asserts "this signal is stale after 250 ms" runs identically on a fast laptop, a
 * loaded CI runner and next year. Anything that reads {@code System.nanoTime()} inside the library
 * would make such a test flaky, which is why {@link SapphifyTransport} owns the clock.
 */
public final class SapphifySimTransport implements SapphifyTransport {

  private final Map<Integer, Frame> received = new HashMap<>();
  private final Map<Integer, Double> requestedRates = new HashMap<>();
  private final List<Frame> sent = new ArrayList<>();
  private double now;
  private SapphifyStatusCode sendResult = SapphifyStatusCode.OK;

  /** Injects a frame as if the device had transmitted it, stamped at the current time. */
  public void inject(int arbitrationId, byte[] data) {
    received.put(arbitrationId, new Frame(arbitrationId, data.clone(), now));
  }

  /** Advances the simulated clock. */
  public void advanceTime(double seconds) {
    if (seconds < 0) {
      throw new IllegalArgumentException("time must not move backwards");
    }
    now += seconds;
  }

  /** Forces {@link #send} to fail, so error paths can be tested without hardware. */
  public void failSendsWith(SapphifyStatusCode status) {
    this.sendResult = status;
  }

  /** Every frame the library has transmitted, oldest first. */
  public List<Frame> sentFrames() {
    return List.copyOf(sent);
  }

  /** The rate last requested for an arbitration ID, or empty if never requested. */
  public Optional<Double> requestedRate(int arbitrationId) {
    return Optional.ofNullable(requestedRates.get(arbitrationId));
  }

  @Override
  public Optional<Frame> latestFrame(int arbitrationId) {
    return Optional.ofNullable(received.get(arbitrationId));
  }

  @Override
  public SapphifyStatusCode send(Frame frame) {
    if (sendResult.isOK()) {
      sent.add(frame);
    }
    return sendResult;
  }

  @Override
  public SapphifyStatusCode setUpdateFrequency(int arbitrationId, double hz) {
    if (hz < 0 || hz > 1000) {
      return SapphifyStatusCode.INVALID_PARAMETER;
    }
    requestedRates.put(arbitrationId, hz);
    return SapphifyStatusCode.OK;
  }

  @Override
  public double currentTimeSeconds() {
    return now;
  }

  @Override
  public void close() {
    received.clear();
    sent.clear();
    requestedRates.clear();
  }
}
