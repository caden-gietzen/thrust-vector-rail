## Correct workflow

### Stage 1 — PRBS scouting test

Use **Pseudo-Random Binary Signal (PRBS)** first to get a rough Bode-style estimate and identify:

- approximate servo bandwidth,
- approximate delay,
- whether the servo behaves like first-order, second-order, or rate-limited dynamics,
- what frequency range is worth exciting with PRPS.

This is the same logic as the [thrust identification workflow](../../experiments/thrust_identification/procedure.md), except now:

$$
G_{\text{servo}}(j\omega)=\frac{\Theta(j\omega)}{U(j\omega)}
$$​

where:

- $G_{\text{servo}}(j\omega)$ is the servo frequency response,
- $j\omega$ is the imaginary frequency variable,
- $\omega$ is angular frequency in radians per second,
- $\Theta(j\omega)$ is encoder-measured servo angle,
- $U(j\omega)$ is commanded servo PWM in microseconds.

### Stage 2 — PRPS focused test

Then use **Pseudo-Random Periodic Signal (PRPS)** over the useful range. This gives cleaner phase/magnitude points because the injected frequencies are deliberate and repeatable.

The dangerous mistake would be jumping straight to a wide PRPS range like `0.05–10 Hz` before knowing where the servo response falls apart. You may waste time collecting high-frequency data where the servo is saturated, rate-limited, or barely moving.

## Encoder angle conversion

The servo output shaft and the encoder are coupled by a GT2 timing belt with
**equal pulleys (16T : 16T)**, so the rotational ratio is exactly $1:1$:

$$
\theta_{\text{servo}} = \theta_{\text{encoder}}
$$

This conversion is **spec-derived and exact**, not empirically calibrated. GT2 is
a *toothed* belt — it does not slip, and equal integer tooth counts give an exact
angular ratio; belt tension does not change counts-per-degree. The encoder is
**600 pulses per revolution** and the quadrature decoder counts all four edges, so:

$$N_{\text{counts/rev}} = 600 \times 4 = 2400$$

$$
\theta_{\text{deg}} = 360 \cdot \frac{\text{count}}{2400} = 0.15 \cdot \text{count}
\quad(\text{exact})
$$

This is the single source of truth in
[`encoderAngleScale()`](../../analysis/utils/encoderAngleScale.m). Scripts pull the
scale from there rather than hardcoding $2400$.

**Why spec, not empirical.** Unlike the linear rail encoder — where the belt
converts rotation to *translation* and the pitch-line diameter introduces a real
scale error (nominal $60$ vs measured $64.81$ counts/mm, see
[encoder_calibration/results.md](../../experiments/encoder_calibration/results.md)) —
a $1:1$ *angular* coupling has no such ambiguity. The earlier "$\approx 2414$
counts/rev" figure came from reading a sweep as $180^\circ$ off video/protractor;
at the exact $2400$ scale that sweep was actually $\approx 181^\circ$, i.e. a $\sim
1^\circ$ ($0.6\%$) protractor misread, not a physical scale. A half-revolution
sweep is retained only as a coarse **no-tooth-skip sanity check** (fast moves can
drop counts — see the encoder-mismatch validation), not as the calibration basis.
Backlash ($\approx 2.4^\circ$) is a separate additive offset, not a scale error.