# Telescope Configuration

`hardware.yaml` is the single configuration surface for the telescoping sonar
arm hardware attached to the Teensy 4.1 timing bridge.

The configuration contains explicitly marked dummy values until the assembled
mechanism is measured. Motion remains disabled until the `configured` and
`wiring_verified` flags are valid. The
following values must be filled in before enabling motion:

- minimum and maximum arm length;
- pulley pitch diameter or measured linear travel per motor revolution;
- motor-to-pulley ratio;
- encoder mounting location and direction sign;
- sonar offset from the arm tip;
- verified wiring for the motor driver, encoder, and redundant limit inputs.

The encoder position is maintained as a signed count. Homing the minimum
limit establishes `encoder_zero_count_at_min_length`; the measured count span
between the minimum and maximum limits is stored as
`calibrated_count_span_to_max_length`. For a constant-pitch pulley, the runtime
mapping is:

```text
length_m = min_length_m
         + (encoder_count - encoder_zero_count_at_min_length)
           / calibrated_count_span_to_max_length
           * (max_length_m - min_length_m)
```

If the cable winds directly onto a spool, travel per revolution changes with
the cable layer. In that case `transmission_type` must be changed to a spool
model and the controller must use the spool radius and cable diameter rather
than the constant-pitch equation. The controller must never infer a usable
geometry from zero-valued placeholder fields.

Every valid, debounced minimum-limit event rebases the encoder zero and sets
the reported arm length to `minimum_length_datum_m` (normally zero). A valid
maximum-limit event clamps the reported length to `max_length_m`; it does not
rebase the minimum reference. Limit contacts must remain stable for the
configured debounce interval, agree with their NO/NC plausibility check, and
match the commanded travel direction before they are accepted as reference
events.

The encoder is an NPN open-collector output. Its pull-up must be to the Teensy
3.3 V rail, not the encoder supply voltage. The limit inputs use the SPDT
contacts as normally-open and normally-closed signals. The two contacts
provide electrical plausibility checking, but they are not independent
mechanical safety channels because they belong to the same switch. The MCU
must stop motion on disagreement or an invalid contact combination; a
separate hardwired emergency-stop or second mechanical switch is still
required for independent protection.
