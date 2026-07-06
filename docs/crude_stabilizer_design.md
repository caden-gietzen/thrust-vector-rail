# Crude Stabilizer Design

> **Status:** Draft for review. This note documents the first closed-loop stabilizer used to
> bootstrap safe closed-loop identification. It is intentionally short and recruiter-facing:
> the goal is to show the control reasoning without turning this into a full controller
> theory report.

This stabilizer is the bridge between the open-loop subsystem models in
[rough_truth_model.md](rough_truth_model.md) and the closed-loop identification campaign.
The rail cannot be safely excited open-loop across the operating envelope, so the first
controller only needs to be conservative, robust, and boring enough to keep the cart away
from the end-stops while identification data is collected.

The finalized controller structure is implemented in
[ctrl_fblin.m](../tvr_stabilizer/ctrl_fblin.m). The simulation plant and nominal design
parameters are in [plant.m](../tvr_stabilizer/plant.m) and
[params.m](../tvr_stabilizer/params.m).

---

## 1. Reduced control model

For the bootstrap stabilizer, thrust is held approximately constant during a run:

$$
T(t) \approx T_0
$$

The rail-direction equations of motion reduce to

$$
\dot{p} = v
$$

$$
\dot{v} = \frac{T_0}{m}\sin\theta - d
$$

where $p$ is rail position, $v$ is rail velocity, $\theta$ is thrust-vector angle, $m$ is
cart mass, and $d$ is the acceleration-equivalent disturbance from friction and other
unmodeled effects.

The important nonlinearity is geometric, not mysterious:

$$
F_\text{rail} = T_0\sin\theta
$$

For a fixed $T_0$, the servo angle controls the rail acceleration through $\sin\theta$.
That makes the system a good fit for input/output linearization at this early phase.

---

## 2. Input/output linearization

Define a virtual input

$$
u = \sin\theta
$$

so the reduced plant becomes

$$
\ddot{p} = \frac{T_0}{m}u - d
$$

Ignoring the disturbance for the nominal control derivation, choose $u$ so the rail sees a
commanded acceleration $a_c$:

$$
u_\text{cmd} = \frac{m}{T_0}a_c
$$

Then reconstruct the physical servo-angle command with

$$
\theta_\text{cmd} = \sin^{-1}\!\left(u_\text{cmd}\right)
$$

The implementation clips $u_\text{cmd}$ before applying $\sin^{-1}(\cdot)$:

$$
u = \operatorname{sat}_{[-0.999,\ 0.999]}(u_\text{cmd})
$$

This keeps the controller inside the real servo-angle envelope and prevents invalid
inverse-sine commands.

---

## 3. PID acceleration command

The controller regulates tracking error

$$
e = p - r
$$

where $r$ is the reference position. The integral state is

$$
\dot{\sigma} = e
$$

The commanded acceleration is

$$
a_c = \ddot{r} - k_i\sigma - k_1e - k_2(\dot{p} - \dot{r})
$$

Substituting this into the feedback-linearized plant gives the nominal closed-loop error
dynamics

$$
\ddot{e} + k_2\dot{e} + k_1e + k_i\sigma = 0,
\qquad
\dot{\sigma} = e
$$

or equivalently

$$
s^3 + k_2s^2 + k_1s + k_i = 0
$$

for the integral-augmented error dynamics. The initial pole-placement template used
$\omega = 6$ rad/s with

$$
k_1 = 3\omega^2,
\qquad
k_2 = 3\omega
$$

and a softer integral gain than the exact triple-pole value:

$$
k_i = 0.5\omega^3
$$

This keeps the proportional and damping response reasonably quick while reducing the
chance that the integrator fights saturation, stiction, servo lag, or model mismatch too
aggressively.

---

## 4. Bode and margin check

The PID gains are justified by the open-loop return ratio

$$
L(s) = C(s)P(s)
$$

where $C(s)$ is the PID acceleration law and $P(s)$ is the rough linearized plant seen
inside unity feedback. This is the right place to discuss gain margin, phase margin, and
crossover frequency: the stabilizer is being sized for robust closed-loop identification,
not for the fastest possible reference tracking.

Three loop models are useful, in increasing order of realism:

1. **Ideal feedback-linearized model.** This uses only
   $P(s)=1/s^2$. It is useful for intuition and pole-placement algebra, but it is not a
   safe hardware justification because it ignores the identified servo lag and delay.
2. **Rough identified actuator model.** This augments the double integrator with the
   identified servo dynamics,

   $$
   P_\theta(s) =
   \frac{1}{s^2}\frac{1}{1+\tau_\theta s}e^{-L_\theta s}
   $$

   using the servo lag and delay from
   [rough_truth_model.md](rough_truth_model.md).

   > **⚠ Servo params updated 2026-07-06 — margins below pending recomputation.** The
   > loop-margin figures in the remainder of this section were computed against the
   > *superseded slow servo* ($\tau_\theta = 24.4~\text{ms}$, $L_\theta = 28.8~\text{ms}$).
   > The canonical model is now the faster upgraded servo ($\tau_\theta = 17.1~\text{ms}$,
   > $L_\theta = 13.4~\text{ms}$, ±15° rung). Because the new servo is *faster*, the phase
   > margins below are **pessimistic** — the design is safe, but the specific numbers
   > (crossover, PM) must be recomputed against the updated model before the next tuning pass.
3. **Implemented-loop linearization.** This linearizes the actual
   [tvr_sim.slx](../tvr_stabilizer/tvr_sim.slx) loop with friction and measurement noise
   disabled, reference amplitude set to zero, the transport delay represented by a
   third-order Padé approximation, and the velocity filter included. This is the
   authoritative small-signal margin check because it matches the implemented controller
   wiring in [ctrl_fblin.m](../tvr_stabilizer/ctrl_fblin.m).

The original $\omega = 6~\text{rad/s}$ pole-placement seed is too aggressive once the
identified actuator dynamics are included. The preliminary rough-loop check gave only
about $19^\circ$ phase margin at $\omega_c \approx 17.25~\text{rad/s}$, or
$2.75~\text{Hz}$. The implemented-loop linearization was worse, reporting a negative
phase margin with the same gains. That contradicts the rough-model target of a slow
$0.1$-$0.3~\text{Hz}$ first hardware loop.

The revised first-hardware tune is therefore

$$
\omega = 0.6~\text{rad/s}
$$

$$
k_1 = 3\omega^2 = 1.08
$$

$$
k_2 = 3\omega = 1.8
$$

$$
k_i = 0.5\omega^3 = 0.108
$$

The softened integral convention is intentional. It keeps integral action available for
Coulomb friction without letting the integrator dominate the delay-limited loop.

The margin sweep in [init_stabilizer_bode.m](../tvr_stabilizer/init_stabilizer_bode.m)
prints the ideal, rough-actuator, and implemented-loop margins for
$\omega \in \{0.3,0.6,1.0,1.8,3.0,6.0\}$. It then selects the **largest**
$\omega$ whose implemented-loop row satisfies the first-hardware margin targets:

$$
f_c \le 0.30~\text{Hz},
\qquad
PM \ge 60^\circ,
\qquad
GM \ge 12~\text{dB}
$$

That encoded rule selects $\omega=0.6~\text{rad/s}$ as the first hardware candidate:

| $\omega$ | $\omega_c$ | Crossover | Phase margin | Gain margin | Interpretation |
|---:|---:|---:|---:|---:|---|
| $0.6~\text{rad/s}$ | $\approx 1.88~\text{rad/s}$ | $\approx 0.30~\text{Hz}$ | $\approx 64^\circ$ | $\approx 23~\text{dB}$ | first hardware candidate |
| $1.0~\text{rad/s}$ | $\approx 3.15~\text{rad/s}$ | $\approx 0.50~\text{Hz}$ | $\approx 58^\circ$ | $\approx 18~\text{dB}$ | simulation-only backup |
| $1.8~\text{rad/s}$ | $\approx 5.70~\text{rad/s}$ | $\approx 0.91~\text{Hz}$ | $\approx 47^\circ$ | $\approx 13~\text{dB}$ | usable margin but too quick for first hardware |
| $3.0~\text{rad/s}$ | $\approx 9.44~\text{rad/s}$ | $\approx 1.50~\text{Hz}$ | $\approx 31^\circ$ | $\approx 8~\text{dB}$ | too little margin |

The velocity filter is treated as part of the implemented controller, not as a
separate inner velocity loop. Its cutoff is chosen relative to the outer
position-loop crossover because any filter lag appears inside the same return ratio used
for the phase-margin check. A practical rule is

$$
\omega_{f,v} \ge 5\text{--}10\,\omega_c
$$

where $\omega_{f,v}$ is the velocity-filter cutoff and $\omega_c$ is the implemented-loop
crossover. For the first hardware candidate, $\omega_c \approx 1.88~\text{rad/s}$. The
current first-order velocity filter uses

$$
\tau_v = 0.01~\text{s}
$$

so

$$
\omega_{f,v} = \frac{1}{\tau_v} = 100~\text{rad/s}
$$

or

$$
f_{f,v} \approx 15.9~\text{Hz}
$$

This is about $53$ times the implemented-loop crossover. The filter therefore contributes
only about

$$
-\tan^{-1}(\omega_c\tau_v) \approx -1.1^\circ
$$

of phase lag at crossover, so it does not materially erode the documented phase margin.
The value can be lowered later if differentiated encoder noise dominates, but any change
must be rechecked with [init_stabilizer_bode.m](../tvr_stabilizer/init_stabilizer_bode.m)
or [loop_margin.m](../tvr_stabilizer/loop_margin.m).

Friction is deliberately excluded from the small-signal loop-margin linearization.
Gain margin and phase margin are properties of the local return ratio; Coulomb friction,
stiction, saturation, and direction-dependent breakaway are nonlinear disturbance and
authority limits, not clean multiplicative loop-gain changes. Including the current
tanh-smoothed friction model in the linearization would make the result depend strongly
on the artificial slope chosen near $v=0$ and would obscure the actuator-delay margin that
the Bode check is meant to measure.

That does **not** mean friction can be ignored when selecting the crude PID. The actual
first-hardware choice is a balance between two tests:

- the friction-off implemented-loop linearization must retain enough delay robustness;
- the nonlinear closed-loop simulation used for PRPS and Monte Carlo validation must run
  with friction enabled and show enough authority to move, track, and stay away from the
  rail limits without sustained saturation.

In other words, the Bode margin check answers "will this loop remain stable around the
nominal operating point despite actuator lag and delay?" The nonlinear friction-enabled
simulation answers "does this conservative loop have enough authority to be useful on the
real rail?" If $\omega=0.6~\text{rad/s}$ is too sluggish to overcome friction, increasing
to $\omega=1.0~\text{rad/s}$ is a legitimate trade: it spends some phase margin to buy
low-speed authority. That decision should be made explicitly from the paired linear-margin
and friction-enabled nonlinear results.

If a candidate passes the friction-off return-ratio check but performs poorly with
friction enabled, there are only two honest design moves. One is to make friction part of
the control-oriented plant treatment, such as through an explicit friction feedforward,
disturbance estimate, or revised nonlinear simulation model used during tuning. The other
is to relax the first-hardware gain-margin and phase-margin targets enough to raise the
PID gains. The second option is acceptable for a bootstrap controller only if the reduced
margins are documented and the friction-enabled nonlinear simulations still show no rail
limit violations or sustained actuator saturation.

The acceptance targets for the first hardware stabilizer are:

- crossover near $0.3~\text{Hz}$ or lower;
- phase margin preferably at least $60^\circ$;
- gain margin preferably at least $12~\text{dB}$;
- PRPS reference components for closed-loop ID below the selected crossover, assigned in
  [run_mc_campaign.m](../tvr_stabilizer/monte_carlo/run_mc_campaign.m);
- enough commanded authority to move through the provisional friction model in nonlinear
  PRPS simulation;
- no rail-limit violation or sustained servo saturation in friction-enabled nonlinear
  simulation.

If the $\omega=0.6~\text{rad/s}$ controller is too sluggish to overcome friction and
stiction in nonlinear simulation, the next candidate is $\omega=1.0~\text{rad/s}$. That
trade should be documented explicitly as a robustness-for-authority exchange, not hidden
as a retune.

## 5. What is deliberately not inverted

The controller in [ctrl_fblin.m](../tvr_stabilizer/ctrl_fblin.m) inverts only the static
geometry $u=\sin\theta$. It does **not** invert the identified servo lag, servo delay,
thrust dynamics, or friction model.

That is intentional. The rough design notes considered a fuller backstepping-style
approach through the actuator dynamics, but that would require trusting the least certain
parts of the model too much. In particular:

- pure transport delay is awkward to backstep through directly;
- Pade or predictor approximations would add model-dependent states before the model has
  been earned in closed loop;
- friction is still provisional and asymmetric, so exact cancellation would be fragile;
- the first controller only needs to be safe enough for closed-loop identification.

The chosen design uses the clean, high-confidence nonlinearity directly and lets robust
feedback absorb the actuator and friction imperfections.

---

## 6. Saturation and anti-windup

The practical command sequence is:

$$
u_\text{raw}
= \frac{m}{T_0}
\left(\ddot{r} - k_i\sigma - k_1e - k_2(\dot{p} - \dot{r})\right)
$$

$$
u = \operatorname{sat}_{[-0.999,\ 0.999]}(u_\text{raw})
$$

$$
\theta_\text{cmd} = \sin^{-1}(u)
$$

When saturation occurs, the integrator is conditionally frozen if it would continue
pushing the command farther into saturation. This is a small but important hardware-safety
detail: the rail is low-authority and friction-dominated, so a naive integrator can build a
large stored command while the servo is already pinned near its limit.

---

## 7. Validation role

This controller is validated in simulation against a rough truth model that includes the
identified servo delay/lag, thrust uncertainty, measurement noise, and friction
disturbance overbounds. The current Monte Carlo scripts vary the truth plant while keeping
the controller design fixed; that is the important robustness test for this phase.

The current validation artifacts are written by campaign under:

- [`data/processed/controller_validation/tvr_stabilizer/monte_carlo/`](../data/processed/controller_validation/tvr_stabilizer/monte_carlo/)
- [`plots/controller_validation/tvr_stabilizer/monte_carlo/`](../plots/controller_validation/tvr_stabilizer/monte_carlo/)
- [`reports/controller_validation/tvr_stabilizer/monte_carlo/`](../reports/controller_validation/tvr_stabilizer/monte_carlo/)

The 500-case tracking sweep completed with no unstable simulations and no rail-limit
violations. Many cases briefly touched the servo command limit, but the saturation
fraction stayed small. The strict tracking-success threshold is not yet met, which is
acceptable for this phase: the crude stabilizer is a bootstrap controller for safe
closed-loop data collection, not the final 15 mm / 7 mm (R4/R5) tracking controller.

---

## 8. Why this is the right Phase 1 controller

The design matches the current project maturity:

- it uses the first-principles nonlinear geometry instead of pretending the plant is
  globally linear;
- it avoids exact cancellation of poorly trusted actuator and friction dynamics;
- it includes integral action because Coulomb friction is large compared with available
  rail force;
- it is simple enough to port to embedded code for closed-loop identification;
- it creates the safe operating condition needed to collect the data that will produce the
  later earned model.

After closed-loop identification, this controller becomes the baseline. The later
gain-scheduled LQR and estimator work should beat it on precision, bandwidth, and
disturbance rejection, but this stabilizer earns the data that makes those later designs
credible.
