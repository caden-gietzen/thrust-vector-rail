# Encoder Reliability Investigation Under High-Frequency Servo Excitation

## Overview

During high-frequency servo excitation tests, the quadrature encoder measurement system exhibited apparent drift and loss of positional consistency after repeated bidirectional motion. The issue became increasingly noticeable during aggressive pseudo-random excitation and rapid back-and-forth motion.

This document summarizes:

- Observed behavior
- Initial hypotheses
- Potential root causes
- Proposed diagnostic methodology
- Recommended next steps

The goal is not only to determine whether the encoder itself is unreliable, but to characterize the operational limits of the entire measurement chain:

```
Mechanical system → Encoder → Electrical signaling → RP2040 decoding → Software handling
```

---

# Observed Behavior

## Primary Symptoms

The following behaviors were observed during aggressive servo motion:

- Encoder count drift accumulates over time
- Encoder does not reliably return to its original count after repeated oscillatory motion
- Drift magnitude appears correlated with:
    - High-frequency motion
    - Rapid direction reversals
    - Larger command amplitudes
- Small-amplitude or slower motions appear significantly more reliable

Observed failure modes include:

```
Expected:
0 → +N → 0 → -N → 0

Observed:
0 → +N → +12 → -N+8 → +15
```

This suggests cumulative count integrity loss.

---

# Potential Root Causes

## 1. Excessive Edge Rate

The encoder may be generating transitions faster than the decoding pipeline can reliably process.

Potential bottlenecks include:

- PIO handling
- FIFO draining
- Interrupt latency
- MicroPython polling latency
- State machine update timing

Symptoms would likely worsen as:

```
counts/second ↑
```

---

## 2. Invalid Quadrature Transitions During Rapid Reversal

Fast direction changes may produce transient invalid states such as:

```
00 → 11
```

instead of valid Gray-code transitions:

```
00 → 01 → 11
```

This can occur due to:

- Channel skew
- Noise
- Contact bounce
- Weak pull-ups
- Signal ringing

Rapid reversal is especially suspicious because the observed failures correlate strongly with aggressive oscillatory motion.

---

## 3. Electrical Noise

Servo current transients may inject noise into encoder lines.

Potential contributors:

- Shared ground impedance
- Insufficient decoupling
- Long signal wires
- Weak pull-up resistors
- PWM-induced EMI

This is especially plausible if:

- Errors increase with servo load
- Errors correlate with rapid acceleration
- Errors worsen at higher PWM amplitudes

---

## 4. Mechanical Slip or Compliance

The encoder may not perfectly represent actual mechanical motion.

Potential issues include:

- Belt slip
- Shaft slip
- Backlash
- Compliance in linkage
- Coupling looseness

This could falsely appear as encoder drift despite correct electrical counting.

---

## 5. Decoder Logic / Firmware Errors

The custom RP2040 PIO + C module implementation may contain edge cases involving:

- FIFO handling
- Overflow conditions
- Invalid transition interpretation
- Timing assumptions
- Synchronization issues

This possibility must remain open until isolated experimentally.

---

# Key Diagnostic Principle

The objective is not merely to determine whether drift exists.

The objective is to determine:

```
Under what operating conditions does count integrity begin to degrade?
```

This effectively defines the safe operational envelope of the measurement system.

---

# Proposed Diagnostic Strategy

# Phase 1 — Static Mapping

## Objective

Create a repeatable mapping between:

```
Servo PWM ↔ Settled Encoder Count
```

## Procedure

1. Zero encoder at center position
2. Sweep slowly through PWM values
3. Allow full settling at each step
4. Record:
    - PWM command
    - Final encoder count

## Purpose

This provides:

- Approximate angular calibration
- Repeatable reference positions
- Baseline hysteresis assessment

---

# Phase 2 — Bidirectional Return-to-Zero Testing

## Objective

Detect cumulative count drift.

## Procedure

Perform repeated motion cycles:

```
Center→ Positive displacement→ Negative displacement→ Return center
```

Then compare final encoder count against initial count.

---

## Example Test Sequence

```
1. Center servo
2. Zero encoder
3. Command +A
4. Wait for settling
5. Command -A
6. Wait for settling
7. Command center
8. Wait for settling
9. Record final encoder count
10.Repeat
```

---

# Recommended Sweep Parameters

## Command Amplitudes

```
50µs 100µs 200µs 400µs 600µs
```

## Dwell Times

```
1000ms 500ms 250ms 100ms 50ms
```

---

# Metrics to Record

For each cycle:

```
start_count
end_count
net_drift
peak_count_rate
move_time
command_amplitude
dwell_time
```

---

# Most Important Derived Metric

## Drift Per Cycle

```
drift_per_cycle = final_count_after_cycle - initial_count
```

A reliable system should maintain:

```
drift_per_cycle ≈ 0
```

over many repetitions.

---

# Phase 3 — Count Rate Characterization

## Objective

Estimate the maximum reliable count rate.

Compute:

```
counts_per_second = abs(diff(counts) / diff(time))
```

Then correlate drift with count rate.

---

# Desired Outcome

Determine whether a threshold exists such as:

```
Reliable below:
3000 counts/sec

Unreliable above:
4500 counts/sec
```

This effectively defines encoder bandwidth in practical terms.

---

# Phase 4 — Directional Error Analysis

## Objective

Determine whether failures are direction-dependent.

Compare:

```
CW motion error
vs
CCW motion error
```

Direction-specific failures often indicate:

- Quadrature timing asymmetry
- Signal skew
- One weak channel
- Pull-up imbalance

---

# Important Limitation

Using servo angle as “ground truth” is imperfect.

The following effects become entangled:

```
Servo nonlinearity
Mechanical backlash
Encoder errors
Compliance
Linkage hysteresis
```

Therefore:

```
Servo-commanded angle ≠ perfect truth reference
```

The strongest diagnostic is repeated return-to-origin consistency.

---

# Recommended Next Steps

## Immediate

1. Implement automated bidirectional drift testing
2. Log:
    - Final drift
    - Peak count rate
    - Direction
    - Amplitude
3. Generate plots:
    - Drift vs cycle number
    - Drift vs count rate
    - Drift vs command amplitude

---

## Electrical Validation

If drift persists:

- Probe encoder A/B lines with oscilloscope or logic analyzer
- Check:
    - Voltage swing
    - Ringing
    - Timing skew
    - Invalid transitions

---

## Firmware Validation

Add instrumentation for:

- Invalid quadrature transitions
- FIFO overflow detection
- Missed state transitions

Potentially compare:

```
PIO implementation
vs
simple interrupt-based decoder
```

under identical motion conditions.

---

# Preliminary Conclusion

Current evidence suggests the issue is likely not true encoder “drift,” but rather degradation of count integrity during aggressive bidirectional motion.

The most likely causes are:

1. High edge-rate limitations
2. Invalid quadrature transitions during reversal
3. Electrical noise during aggressive servo actuation

The proposed diagnostic workflow aims to empirically determine the reliability envelope of the encoder measurement system and identify the dominant failure mechanism.