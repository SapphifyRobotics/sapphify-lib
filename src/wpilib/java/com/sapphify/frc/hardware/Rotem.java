package com.sapphify.frc.hardware;

import com.sapphify.frc.SapphifyCanBus;
import com.sapphify.frc.SapphifyProtocol;
import com.sapphify.frc.SapphifySignal;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.driverstation.Alert;
import org.wpilib.epilogue.Logged;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.SimDevice;
import org.wpilib.hardware.hal.SimDouble;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.util.sendable.Sendable;
import org.wpilib.util.sendable.SendableBuilder;

/**
 * ROTEM, with WPILib integration.
 *
 * <p>The WPILib-aware subclass of {@link CoreRotem}. It adds only what needs WPILib types —
 * geometry accessors, the Alert API, Sendable, simulation and usage reporting — and nothing else.
 * Keeping the split sharp is what lets the command-line tool, the desktop configurator and log
 * replay use {@code CoreRotem} without dragging a robot framework in behind them.
 *
 * <pre>{@code
 * public class Drivetrain extends SubsystemBase {
 *   private final Rotem imu = new Rotem(0);
 *
 *   @Override
 *   public void periodic() {
 *     imu.refreshAlerts();                 // publishes faults to the dashboard
 *     poseEstimator.update(imu.getRotation2d(), modulePositions);
 *   }
 * }
 * }</pre>
 */
@Logged
public class Rotem extends CoreRotem implements Sendable, AutoCloseable {

  private final WpiCanTransport ownedTransport;
  private final List<AlertBinding> alerts = new ArrayList<>();

  private final SimDevice simDevice;
  private final SimDouble simYaw;
  private final SimDouble simYawUncertainty;

  /** One WPILib Alert bound to one health flag, so a fault can never silently stop reporting. */
  private record AlertBinding(int flag, Alert alert) {}

  /** Constructs a device on the default bus, Systemcore {@code can_s0}. */
  public Rotem(int deviceNumber) {
    this(deviceNumber, SapphifyCanBus.DEFAULT);
  }

  /** Constructs a device on a named bus. */
  public Rotem(int deviceNumber, SapphifyCanBus canBus) {
    this(deviceNumber, canBus, new WpiCanTransport(canBus, SapphifyProtocol.MANUFACTURER_ID));
  }

  private Rotem(int deviceNumber, SapphifyCanBus canBus, WpiCanTransport transport) {
    super(deviceNumber, canBus, transport);
    this.ownedTransport = transport;
    transport.openDevice(SapphifyProtocol.DeviceType.AHRS.id(), deviceNumber);

    // 2027 replaced the enum-based HAL.report with a string-based reportUsage. Vendors are
    // expected to call it so that usage reports and log analysis can attribute the device.
    HAL.reportUsage("ROTEM", deviceNumber, "SapphifyLib");

    // Simulation. SimDevice.create returns null outside simulation, and every accessor below
    // checks for that rather than branching on a RobotBase.isSimulation() call.
    simDevice = SimDevice.create("SAPPHIFY:ROTEM", deviceNumber);
    if (simDevice != null) {
      simYaw = simDevice.createDouble("Yaw", SimDevice.Direction.INPUT, 0.0);
      simYawUncertainty =
          simDevice.createDouble("YawUncertainty", SimDevice.Direction.INPUT, 0.0);
    } else {
      simYaw = null;
      simYawUncertainty = null;
    }

    bindAlert(SapphifyProtocol.Flag.ID_CONFLICT, Alert.Level.HIGH);
    bindAlert(SapphifyProtocol.Flag.NUMERICAL_FAULT, Alert.Level.HIGH);
    bindAlert(SapphifyProtocol.Flag.FIRMWARE_MISMATCH, Alert.Level.HIGH);
    bindAlert(SapphifyProtocol.Flag.HIGH_VIBRATION, Alert.Level.MEDIUM);
    bindAlert(SapphifyProtocol.Flag.MAG_DISTURBED, Alert.Level.MEDIUM);
    bindAlert(SapphifyProtocol.Flag.GYRO_SAT, Alert.Level.MEDIUM);
    bindAlert(SapphifyProtocol.Flag.ACCEL_SAT, Alert.Level.MEDIUM);
    bindAlert(SapphifyProtocol.Flag.FIFO_OVERRUN, Alert.Level.MEDIUM);
    bindAlert(SapphifyProtocol.Flag.TEMP_OUT_OF_CAL, Alert.Level.LOW);
    bindAlert(SapphifyProtocol.Flag.CAL_STALE, Alert.Level.LOW);
  }

  private void bindAlert(int flag, Alert.Level level) {
    alerts.add(new AlertBinding(flag, new Alert("ROTEM " + getDeviceNumber(), "", level)));
  }

  // ---- Geometry, matching what WPILib's own IMU exposes ------------------------------------

  /** Heading as a {@link Rotation2d}. This is what a pose estimator consumes. */
  public Rotation2d getRotation2d() {
    if (simYaw != null) {
      return Rotation2d.fromDegrees(simYaw.get());
    }
    SapphifySignal<Double> yaw = getYaw();
    return Rotation2d.fromDegrees(yaw.isValid() ? yaw.value() : 0.0);
  }

  /** Full orientation. Derived from the quaternion, never from re-composed Euler angles. */
  public Rotation3d getRotation3d() {
    return new Rotation3d(getQuaternionWpi());
  }

  /** Orientation as a WPILib {@link Quaternion}. */
  public Quaternion getQuaternionWpi() {
    SapphifySignal<double[]> q = getQuaternion();
    if (!q.isValid()) {
      return new Quaternion();
    }
    double[] v = q.value();
    return new Quaternion(v[0], v[1], v[2], v[3]);
  }

  // ---- Diagnostics -------------------------------------------------------------------------

  /**
   * Publishes current faults to the driver station.
   *
   * <p>Call from {@code periodic()}. Alert has built-in change detection, so calling it every loop
   * is the intended usage rather than something to optimise around.
   */
  public void refreshAlerts() {
    SapphifySignal<RotemDecoder.Health> health = getHealth();
    if (!health.isValid()) {
      alerts.forEach(b -> b.alert().set(false));
      return;
    }
    List<String> messages = getActiveAlerts();
    for (AlertBinding b : alerts) {
      boolean active = health.value().has(b.flag());
      if (active) {
        // Reuse the core layer's wording, which names the likely mechanical cause rather than
        // the bit that was set.
        messages.stream()
            .filter(m -> matches(m, b.flag()))
            .findFirst()
            .ifPresent(m -> b.alert().setText(m));
      }
      b.alert().set(active);
    }
  }

  private static boolean matches(String message, int flag) {
    return switch (flag) {
      case SapphifyProtocol.Flag.ID_CONFLICT -> message.contains("device number");
      case SapphifyProtocol.Flag.NUMERICAL_FAULT -> message.contains("numerical");
      case SapphifyProtocol.Flag.HIGH_VIBRATION -> message.contains("vibration");
      case SapphifyProtocol.Flag.MAG_DISTURBED -> message.contains("magnetic");
      case SapphifyProtocol.Flag.FIFO_OVERRUN -> message.contains("FIFO");
      case SapphifyProtocol.Flag.TEMP_OUT_OF_CAL -> message.contains("temperature");
      case SapphifyProtocol.Flag.FIRMWARE_MISMATCH -> message.contains("firmware");
      default -> message.contains("saturated");
    };
  }

  // ---- Dashboard ---------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    // "Gyro" gives the compass widget on dashboards that render Sendable types.
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", () -> getRotation2d().getDegrees(), null);
    builder.addDoubleProperty("Yaw uncertainty (deg)", () -> value(getYawUncertainty()), null);
    builder.addDoubleProperty("Drift since zero (deg)", () -> value(getAccumulatedDrift()), null);
    builder.addBooleanProperty("Healthy", () -> getActiveAlerts().isEmpty(), null);
    builder.addStringProperty("Bus", () -> getCanBus().toString(), null);
  }

  private static double value(SapphifySignal<Double> signal) {
    return signal.isValid() ? signal.value() : Double.NaN;
  }

  // ---- Simulation --------------------------------------------------------------------------

  /** Simulation control, in the shape of WPILib's own {@code OnboardIMUSim}. */
  public RotemSim getSimState() {
    return new RotemSim(simYaw, simYawUncertainty);
  }

  /** Sets simulated device state. Mirrors {@code OnboardIMUSim}'s naming and units. */
  public static final class RotemSim {
    private final SimDouble yaw;
    private final SimDouble uncertainty;

    RotemSim(SimDouble yaw, SimDouble uncertainty) {
      this.yaw = yaw;
      this.uncertainty = uncertainty;
    }

    /** Sets simulated yaw, in radians, matching {@code OnboardIMUSim.setYaw(double)}. */
    public void setYaw(double angleRad) {
      if (yaw != null) {
        yaw.set(Math.toDegrees(angleRad));
      }
    }

    /** Sets the simulated one-sigma yaw uncertainty, in degrees. */
    public void setYawUncertainty(double degrees) {
      if (uncertainty != null) {
        uncertainty.set(degrees);
      }
    }
  }

  @Override
  public void close() {
    alerts.forEach(b -> b.alert().close());
    alerts.clear();
    if (simDevice != null) {
      simDevice.close();
    }
    ownedTransport.close();
  }
}
