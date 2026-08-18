# ROTEMLib

The WPILib vendor library for the SAPPHIFY ROTEM device series.

**Status: pre-alpha skeleton.** The protocol constants and frame decoders are real and tested;
the CAN transport and the device classes are not written yet. It is public this early on purpose
— an open library that appears only when it is finished is indistinguishable from a closed one.

## Design decisions

- **One library for the whole series.** ROTEM AHRS, encoder and bridge share one manufacturer ID
  and one vendordep. A team installs `ROTEMLib` once.
- **Pure Java over the WPILib CAN API.** No JNI, no native binaries, no per-platform build
  matrix. This is what makes the Systemcore (arm64) port nearly free and keeps the vendordep a
  single Maven artifact.
- **Decoders are pure functions.** `RotemDecoder` has no WPILib or hardware dependency, so the
  entire wire protocol is testable on a laptop with no board, no bus and no robot — and the
  published test vectors are runnable by anyone writing their own driver.
- **Every protocol number lives in `RotemProtocol`.** No literals anywhere else. The manufacturer
  ID in particular is one constant, because FIRST has not assigned ours yet and beta builds run
  on the "Team Use" ID 8.

## Installing (once published)

```
https://maven.sapphify.com/ROTEMLib-2027.json
```

Licence: MIT.
