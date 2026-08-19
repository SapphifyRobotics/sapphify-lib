package com.sapphify.frc.hardware;

import com.sapphify.frc.SapphifyCanBus;
import com.sapphify.frc.SapphifyStatusCode;
import com.sapphify.frc.SapphifyTransport;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.hal.can.CANReceiveMessage;
import org.wpilib.system.Timer;

/**
 * {@link SapphifyTransport} over the WPILib CAN API.
 *
 * <p>This is the only class in the library that touches WPILib, which is the point: everything
 * above it — decoding, signals, configuration, alerts — is transport agnostic, so the same core
 * drives a robot, a bench tool over SocketCAN, and log replay.
 *
 * <p>WPILib's {@code CAN} object is itself scoped to one device (bus, device number, manufacturer,
 * device type), so this class holds one per device it is asked about. That mirrors how WPILib
 * expects vendors to use it and keeps the receive filtering in the HAL rather than in Java.
 */
public final class WpiCanTransport implements SapphifyTransport {

  private final int busId;
  private final int manufacturerId;
  private final Map<Integer, CAN> handles = new HashMap<>();
  private final CANReceiveMessage scratch = new CANReceiveMessage();

  /** Opens a transport on the given bus. */
  public WpiCanTransport(SapphifyCanBus canBus, int manufacturerId) {
    if (canBus.busId() < 0) {
      throw new IllegalArgumentException(
          "bus " + canBus + " has no WPILib id; it is usable off-robot only");
    }
    this.busId = canBus.busId();
    this.manufacturerId = manufacturerId;
  }

  /**
   * Registers a device so its frames can be read.
   *
   * <p>Must be called before {@link #latestFrame(int)} will return anything for that device.
   * {@code CoreRotem} does this through {@link Rotem}.
   */
  public void openDevice(int deviceType, int deviceNumber) {
    handles.computeIfAbsent(
        key(deviceType, deviceNumber),
        k -> new CAN(busId, deviceNumber, manufacturerId, deviceType));
  }

  @Override
  public Optional<Frame> latestFrame(int arbitrationId) {
    CAN can = handles.get(keyFromArbitrationId(arbitrationId));
    if (can == null) {
      return Optional.empty();
    }
    int apiId = (arbitrationId >> 6) & 0x3FF;
    synchronized (scratch) {
      if (!can.readPacketLatest(apiId, scratch)) {
        return Optional.empty();
      }
      byte[] copy = new byte[scratch.length];
      System.arraycopy(scratch.data, 0, copy, 0, scratch.length);
      // CANReceiveMessage.timestamp is microseconds on the same monotonic clock Timer exposes.
      return Optional.of(new Frame(arbitrationId, copy, scratch.timestamp / 1_000_000.0));
    }
  }

  @Override
  public SapphifyStatusCode send(Frame frame) {
    CAN can = handles.get(keyFromArbitrationId(frame.arbitrationId()));
    if (can == null) {
      return SapphifyStatusCode.DEVICE_NOT_PRESENT;
    }
    int apiId = (frame.arbitrationId() >> 6) & 0x3FF;
    // writePacketNoThrow rather than writePacket: a bus fault is a field condition, and this
    // library's contract is that nothing throws for one.
    int status = can.writePacketNoThrow(apiId, frame.data(), frame.data().length, 0);
    return status == 0 ? SapphifyStatusCode.OK : SapphifyStatusCode.BUS_OFF;
  }

  @Override
  public SapphifyStatusCode setUpdateFrequency(int arbitrationId, double hz) {
    if (hz < 0 || hz > 1000) {
      return SapphifyStatusCode.INVALID_PARAMETER;
    }
    // Rate is a device-side setting: the request goes out as a configuration frame and the device
    // changes its own publication rate. Nothing about the WPILib receive side needs to change.
    int apiId = (arbitrationId >> 6) & 0x3FF;
    byte[] payload = new byte[4];
    int centihz = (int) Math.round(hz * 100.0);
    payload[0] = (byte) (apiId & 0xFF);
    payload[1] = (byte) ((apiId >> 8) & 0xFF);
    payload[2] = (byte) (centihz & 0xFF);
    payload[3] = (byte) ((centihz >> 8) & 0xFF);
    return send(
        new Frame(
            (arbitrationId & ~(0x3FF << 6))
                | (com.sapphify.frc.SapphifyProtocol.Api.CFG_SET_RATE << 6),
            payload,
            currentTimeSeconds()));
  }

  @Override
  public double currentTimeSeconds() {
    return Timer.getTimestamp();
  }

  @Override
  public void close() {
    handles.values().forEach(CAN::close);
    handles.clear();
  }

  private static int key(int deviceType, int deviceNumber) {
    return (deviceType << 8) | deviceNumber;
  }

  private static int keyFromArbitrationId(int arbitrationId) {
    return key((arbitrationId >> 24) & 0x1F, arbitrationId & 0x3F);
  }
}
