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

Because both pulleys are 20T, the mechanical ratio is:

$$
\theta_{\text{servo}} = \theta_{\text{encoder}}
$$

So:

$$
\theta_{\text{rad}} = \frac{2\pi \cdot \text{count}}{N_{\text{counts/rev}}}
$$​

where:

- $\theta_{\text{rad}}$​ is servo output angle in radians,
- `count` is the encoder count,
- $N_{\text{counts/rev}}$​ is encoder counts per full revolution.

If your encoder is **600 pulses per revolution** and the quadrature decoder counts all four edges, then:

$$N_{\text{counts/rev}} = 2400$$

That gives:

$$
\theta_{\text{deg}} = 360 \cdot \frac{\text{count}}{2400}
$$​

But verify this experimentally. Command a slow sweep or manually rotate the servo pulley one full revolution if mechanically possible and check the count change. Do not blindly trust the 2400 number until confirmed — see [Section 2 of the servo identification results](../../experiments/servo_identification/results.md#2-encoder-count-to-angle-calibration) for the measured calibration (1207 counts over a half-revolution, confirming ≈ 2414 counts/rev).