package com.sapphify.frc.hardware;

import com.sapphify.frc.SapphifyProtocol;
import com.sapphify.frc.SapphifySimTransport;
import com.sapphify.frc.SapphifyStatusCode;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Zero-dependency self-check for the ROTEM library.
 *
 * <p>Runs with nothing but a JDK — no Gradle, no JUnit, no WPILib, no hardware:
 *
 * <pre>{@code
 * cd src/main/java && java com/sapphify/frc/hardware/SapphifyLibSelfCheck.java
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
    check("setUpdateFrequency reaches transport",
        imu.setUpdateFrequency(SapphifyProtocol.Api.STATUS_ORIENTATION,250).isOK()
        && t.requestedRate(orient).orElse(0.0)==250.0);
    System.out.println(fail==0?"\nALL PASS":"\n"+fail+" FAILED");
    System.exit(fail==0?0:1);
  }
}
