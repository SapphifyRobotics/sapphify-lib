package com.sapphify.frc.hardware;

import com.sapphify.frc.SapphifyProtocol;
import com.sapphify.frc.SapphifySimTransport;
import com.sapphify.frc.SapphifyStatusCode;
import com.sapphify.frc.SapphifyCanBus;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Zero-dependency self-check for the ROTEM library.
 *
 * <p>Runs with nothing but a JDK — no Gradle, no JUnit, no WPILib, no hardware:
 *
 * <pre>{@code
 * javac -d /tmp/sapphify $(find src/main/java -name '*.java')
 * java -cp /tmp/sapphify com.sapphify.frc.hardware.SapphifyLibSelfCheck
 * }</pre>
 *
 * <p>It lives in the main source tree rather than the test tree precisely so that it needs no
 * build system to run, and {@code build.gradle} excludes it from the published jar so it is not
 * shipped to teams.
 *
 * <p>That property is deliberate. A team writing its own driver from the published specification
 * should be able to verify our decoders against theirs without installing a build system, and a
 * reviewer at FIRST should be able to run it in under a minute. Determinism comes from
 * {@link SapphifySimTransport} owning the clock; nothing in the library reads a wall clock.
 */
public class SapphifyLibSelfCheck {
  static int fail = 0;
  static void check(String what, boolean ok) {
    System.out.printf("%-58s %s%n", what, ok ? "PASS" : "FAIL");
    if (!ok) fail++;
  }
  static int RotemProtocolRates() {
    return SapphifyProtocol.arbitrationId(SapphifyProtocol.DeviceType.AHRS, SapphifyProtocol.Api.STATUS_RATES, 0);
  }
  static int RotemProtocolAccel() {
    return SapphifyProtocol.arbitrationId(SapphifyProtocol.DeviceType.AHRS, SapphifyProtocol.Api.STATUS_ACCEL, 0);
  }
  public static void main(String[] a) {
    var t = new SapphifySimTransport();
    var imu = new CoreRotem(0, t);
    check("no frame -> DEVICE_NOT_PRESENT", imu.getYaw().status()==SapphifyStatusCode.DEVICE_NOT_PRESENT);
    int orient = SapphifyProtocol.arbitrationId(SapphifyProtocol.DeviceType.AHRS, SapphifyProtocol.Api.STATUS_ORIENTATION, 0);
    t.inject(orient, new byte[]{(byte)0xFF,(byte)0x7F,0,0,0,0,0,0});
    var yaw = imu.getYaw();
    check("identity quaternion -> yaw ~0", yaw.isValid() && Math.abs(yaw.value())<1e-6);
    check("yaw units = deg", "deg".equals(yaw.units()));
    short c=(short)Math.round(Math.cos(Math.PI/4)*32767);
    ByteBuffer b=ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    b.putShort(c); b.putShort((short)0); b.putShort((short)0); b.putShort(c);
    t.inject(orient,b.array());
    double y90=imu.getYaw().value();
    check("90 deg quaternion -> yaw ~90 (got "+String.format("%.3f",y90)+")", Math.abs(y90-90.0)<0.02);
    check("quaternion length 4", imu.getQuaternion().value().length==4);
    check("fresh signal not stale", !imu.getYaw().isStale(t.currentTimeSeconds(),0.25));
    t.advanceTime(1.0);
    check("stale after 1s vs 0.25s limit", imu.getYaw().isStale(t.currentTimeSeconds(),0.25));
    check("latency 0 until protocol carries sample time", imu.getYaw().latencySeconds()==0.0);
    check("age reflects 1s gap", Math.abs(imu.getYaw().ageSeconds(t.currentTimeSeconds())-1.0)<1e-9);
    t.inject(orient,new byte[]{1,2,3});
    check("short frame -> MALFORMED_FRAME", imu.getYaw().status()==SapphifyStatusCode.MALFORMED_FRAME);
    int health=SapphifyProtocol.arbitrationId(SapphifyProtocol.DeviceType.AHRS,SapphifyProtocol.Api.STATUS_HEALTH,0);
    int flags=SapphifyProtocol.Flag.HIGH_VIBRATION|SapphifyProtocol.Flag.MAG_DISTURBED;
    ByteBuffer hb=ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    hb.putInt(flags); hb.put((byte)0); hb.put((byte)0); hb.put((byte)30); hb.put((byte)5);
    t.inject(health,hb.array());
    var alerts=imu.getActiveAlerts();
    check("2 faults + missing CAL_VALID -> 3 alerts", alerts.size()==3);
    check("vibration alert names mechanical cause", alerts.stream().anyMatch(s->s.contains("bearings")));
    check("zeroYaw ok", imu.zeroYaw().isOK());
    check("identify(0) rejected", imu.identify(0)==SapphifyStatusCode.INVALID_PARAMETER);
    check("identify(5) ok", imu.identify(5).isOK());
    var cfg=new RotemConfiguration();
    check("default config valid", cfg.validate().isOK());
    check("mag fusion off by default", !cfg.features.magnetometerFusionEnabled);
    cfg.mountPose.yawDegrees=200;
    check("out-of-range mount -> INVALID_PARAMETER", imu.getConfigurator().apply(cfg)==SapphifyStatusCode.INVALID_PARAMETER);
    cfg.mountPose.yawDegrees=90;
    check("valid config applies", imu.getConfigurator().apply(cfg).isOK());
    var t2=new SapphifySimTransport(); var imu2=new CoreRotem(1,t2);
    t2.advanceTime(10.0);
    var good=new RotemConfiguration();
    SapphifyStatusCode last=SapphifyStatusCode.OK;
    for(int i=0;i<200;i++){ last=imu2.getConfigurator().apply(good); t2.advanceTime(0.02); }
    check("4s of loop-rate applies -> FREQUENT_CONFIG_CALLS", last==SapphifyStatusCode.FREQUENT_CONFIG_CALLS);
    check("null transport -> NO_TRANSPORT", new CoreRotem(2,null).zeroYaw()==SapphifyStatusCode.NO_TRANSPORT);
    boolean threw=false; try{ new CoreRotem(63,t);}catch(IllegalArgumentException e){threw=true;}
    check("device 63 reserved -> throws", threw);
    check("status codes carry remedies", SapphifyStatusCode.DEVICE_NOT_PRESENT.description().contains("60 ohms"));
    // CAN bus selection: Systemcore has five buses, Motioncore adds twenty channels.
    check("default bus is can_s0", SapphifyCanBus.DEFAULT.interfaceName().equals("can_s0"));
    check("systemCore(4) -> can_s4", SapphifyCanBus.systemCore(4).interfaceName().equals("can_s4"));
    check("motionCore(2) -> can_d2", SapphifyCanBus.motionCore(2).interfaceName().equals("can_d2"));
    check("socketCan strips prefix", SapphifyCanBus.socketCan("socketcan:can_s1").interfaceName().equals("can_s1"));
    // WPILib numbers buses in one integer space: CAN_S0..S4 = 0..4, CAN_D0..D19 = 5..24.
    check("can_s0 -> busId 0", SapphifyCanBus.DEFAULT.busId()==0);
    check("can_s4 -> busId 4", SapphifyCanBus.systemCore(4).busId()==4);
    check("can_d0 -> busId 5", SapphifyCanBus.motionCore(0).busId()==5);
    check("can_d19 -> busId 24", SapphifyCanBus.motionCore(19).busId()==24);
    check("unknown interface -> busId -1", SapphifyCanBus.socketCan("vcan0").busId()==-1);
    boolean b1=false; try{ SapphifyCanBus.systemCore(5);}catch(IllegalArgumentException e){b1=true;}
    check("systemCore(5) rejected", b1);
    boolean b2=false; try{ SapphifyCanBus.motionCore(20);}catch(IllegalArgumentException e){b2=true;}
    check("motionCore(20) rejected", b2);
    var busImu = new CoreRotem(0, SapphifyCanBus.systemCore(4), t);
    check("device reports its bus", busImu.getCanBus().interfaceName().equals("can_s4"));
    t.inject(health, hb.array());
    check("alert names the bus, not just the device number",
        busImu.getActiveAlerts().stream().anyMatch(s2 -> s2.contains("can_s4/0")));
    // WPILib-compatible surface: names, units and sign convention must match OnboardIMU.
    t.inject(orient, b.array());
    check("getYawRadians ~ pi/2 for the 90 deg quaternion",
        Math.abs(imu.getYawRadians() - Math.PI/2) < 5e-4);
    ByteBuffer rb = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    rb.putShort((short)5000); rb.putShort((short)0); rb.putShort((short)0); rb.putShort((short)1);
    t.inject(RotemProtocolRates(), rb.array());
    check("getGyroRateX converts to rad/s",
        Math.abs(imu.getGyroRateX() - Math.toRadians(5000*0.02)) < 1e-9);
    ByteBuffer ab = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    ab.putShort((short)0); ab.putShort((short)0); ab.putShort((short)500); ab.putShort((short)1);
    t.inject(RotemProtocolAccel(), ab.array());
    check("getAccelZ converts g to m/s^2",
        Math.abs(imu.getAccelZ() - (500*0.002*9.80665)) < 1e-9);
    check("setUpdateFrequency reaches transport",
        imu.setUpdateFrequency(SapphifyProtocol.Api.STATUS_ORIENTATION,250).isOK()
        && t.requestedRate(orient).orElse(0.0)==250.0);
    System.out.println(fail==0?"\nALL PASS":"\n"+fail+" FAILED");
    System.exit(fail==0?0:1);
  }
}
