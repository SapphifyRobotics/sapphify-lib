# SapphifyLib

The WPILib vendor library for SAPPHIFY FRC CAN devices.

**Status: pre-alpha.** The protocol layer, device layer and decoders are implemented and tested;
the CAN transport binding to WPILib is not written yet. It is public this early on purpose — an
open library that appears only once it is finished is indistinguishable from a closed one.

## Devices

| Device | What it is | Status |
|---|---|---|
| **ROTEM** | CAN FD attitude and heading reference | first article built |
| — | absolute encoder | in design |
| — | USB-CAN FD bridge | in design |

One library covers all of them. Install `SapphifyLib` once and every SAPPHIFY device you ever buy
is already supported — no second vendordep, no second tool, no second set of habits.

## Using it

```java
import com.sapphify.frc.hardware.CoreRotem;

var imu = new CoreRotem(0, transport);                              // can_s0, the default
var far = new CoreRotem(0, SapphifyCanBus.systemCore(4), transport); // can_s4

var yaw = imu.getYaw();
if (yaw.isValid()) {
    useHeading(yaw.value(), yaw.deviceTimestampSeconds());
}

// The part no other FRC IMU offers: the device's own opinion of its accuracy.
var sigma = imu.getYawUncertainty();

// Everything wrong, phrased for a person standing in a pit.
for (String alert : imu.getActiveAlerts()) {
    reportToDashboard(alert);
}
```

## Design decisions

- **One library for every device.** All SAPPHIFY devices share one FIRST manufacturer ID and one
  vendordep; products are separated by device type and device number, never by a second install.
- **`com.sapphify.frc` is device-agnostic; `com.sapphify.frc.hardware` holds the devices.** The
  shared layer carries status codes, signals, the protocol and the transport abstraction.
- **Pure Java over the WPILib CAN API.** No JNI, no native binaries, no per-platform build matrix.
  That is what makes the Systemcore arm64 port nearly free and keeps the vendordep a single
  artifact.
- **Nothing throws for a device or bus condition.** A device on a CAN bus can legitimately
  disappear mid-match. Reads return a `SapphifySignal` carrying a `SapphifyStatusCode`; every code
  documents its remedy, and that text is what the dashboard shows.
- **The transport owns the clock.** Nothing in the library reads a wall clock, so unit tests and
  log replay are deterministic.
- **Every protocol number lives in `SapphifyProtocol`.** No literals anywhere else — which matters
  most for the manufacturer ID, since FIRST has not assigned ours yet and beta builds run on the
  "Team Use" ID 8.

## Verifying it, with nothing installed

No Gradle, no JUnit, no WPILib, no hardware — a bare JDK is enough:

```bash
javac -d /tmp/sapphify $(find src/main/java -name '*.java')
java -cp /tmp/sapphify com.sapphify.frc.hardware.SapphifyLibSelfCheck
```

31 checks covering frame decoding, staleness, fault reporting, configuration validation, misuse
detection and CAN bus selection. A team writing its own driver from the published protocol specification can verify its
decoders against ours the same way.

## Installing (once published)

```
https://frcsdk.sapphify.com/SapphifyLib-2027.json
```

Licence: MIT.
