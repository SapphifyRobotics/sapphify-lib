package com.sapphify.frc;

import java.util.Objects;

/**
 * Which CAN bus a device is on.
 *
 * <p>The roboRIO had one CAN bus, so a device address was just a number. Systemcore has five
 * (`can_s0` … `can_s4`), and a Motioncore expander adds twenty more (`can_d0` … `can_d19`), all
 * addressed as SocketCAN interfaces. A device number alone no longer identifies a device.
 *
 * <p>Every 2027 vendor library added a bus to its device constructors for exactly this reason —
 * Redux takes {@code new Canandgyro(0, 4)}, CTRE takes {@code new TalonFX(0, CANBus.systemCore(4))}.
 * This is a value object rather than a bare string on purpose: Phoenix shipped a string overload
 * in an early 2027 alpha and then removed it, because {@code "can_s4"}, {@code "cans4"} and
 * {@code "socketcan:can_s4"} are all things people type and only one of them works.
 *
 * <p>Defaults to {@link #DEFAULT}, which is `can_s0` — the same default every other vendor chose,
 * so a team moving between vendors is not surprised.
 */
public final class SapphifyCanBus {

  /** The bus a device uses when none is specified: Systemcore bus 0. */
  public static final SapphifyCanBus DEFAULT = systemCore(0);

  private final String interfaceName;
  private final int busId;

  private SapphifyCanBus(String interfaceName, int busId) {
    this.interfaceName = interfaceName;
    this.busId = busId;
  }

  /**
   * A CAN bus built into Systemcore.
   *
   * @param busId 0 to 4, matching the `can_s0` … `can_s4` interfaces and the port labels
   */
  public static SapphifyCanBus systemCore(int busId) {
    if (busId < 0 || busId > 4) {
      throw new IllegalArgumentException(
          "Systemcore has buses 0 through 4, got " + busId);
    }
    // WPILib's CANBusMap numbers these 0-4, and org.wpilib.hardware.bus.CAN takes that integer
    // as its leading constructor argument.
    return new SapphifyCanBus("can_s" + busId, busId);
  }

  /**
   * A device channel on a Motioncore expander.
   *
   * @param channel 0 to 19, matching the `can_d0` … `can_d19` interfaces
   */
  public static SapphifyCanBus motionCore(int channel) {
    if (channel < 0 || channel > 19) {
      throw new IllegalArgumentException(
          "Motioncore has channels 0 through 19, got " + channel);
    }
    // CANBusMap continues the same integer space: CAN_D0 is bus id 5, through CAN_D19 at 24.
    return new SapphifyCanBus("can_d" + channel, 5 + channel);
  }

  /**
   * A bus named by its SocketCAN interface directly.
   *
   * <p>The escape hatch for a bus this library does not know about yet, and for non-robot use on a
   * Linux host with our USB-CAN FD bridge. Accepts a bare interface name or a {@code socketcan:}
   * prefixed one.
   */
  public static SapphifyCanBus socketCan(String interfaceName) {
    Objects.requireNonNull(interfaceName, "interfaceName");
    String name = interfaceName.startsWith("socketcan:")
        ? interfaceName.substring("socketcan:".length())
        : interfaceName;
    if (name.isBlank()) {
      throw new IllegalArgumentException("interface name must not be blank");
    }
    int id = -1;
    try {
      if (name.startsWith("can_s")) {
        id = Integer.parseInt(name.substring(5));
      } else if (name.startsWith("can_d")) {
        id = 5 + Integer.parseInt(name.substring(5));
      }
    } catch (NumberFormatException ignored) {
      // A bus we cannot map to a WPILib id. Legal off-robot; see busId().
    }
    return new SapphifyCanBus(name, id);
  }

  /**
   * The WPILib bus id, as {@code org.wpilib.hardware.bus.CAN} expects it.
   *
   * <p>WPILib numbers buses in one integer space rather than by name: {@code CANBusMap.CAN_S0}
   * through {@code CAN_S4} are 0 to 4, and {@code CAN_D0} through {@code CAN_D19} continue at 5
   * through 24. The 2027 {@code CAN} constructor takes that integer first, so a library that only
   * kept the interface name would have nothing to pass it.
   *
   * @return the bus id, or -1 for a SocketCAN interface with no WPILib equivalent — legal for
   *     bench and Linux host use through our USB-CAN bridge, but not addressable from robot code
   */
  public int busId() {
    return busId;
  }

  /** The SocketCAN interface name, for example {@code "can_s0"}. */
  public String interfaceName() {
    return interfaceName;
  }

  /** True if this is one of Systemcore's built-in buses rather than a Motioncore channel. */
  public boolean isSystemCore() {
    return interfaceName.startsWith("can_s");
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof SapphifyCanBus b && interfaceName.equals(b.interfaceName);
  }

  @Override
  public int hashCode() {
    return interfaceName.hashCode();
  }

  @Override
  public String toString() {
    return interfaceName;
  }
}
