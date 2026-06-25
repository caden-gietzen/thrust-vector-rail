# Uncertainty Quantification

> **Status:** Phase 1 uncertainty summary for the rough truth model and crude stabilizer.
> This document explains how uncertainty was quantified from the hardware identification
> data, how each uncertainty artifact should be used in simulation, and where the current
> limits of confidence are.

The project does not treat uncertainty as a single generic error bar. Each subsystem has a
different dominant source of uncertainty, so each one uses a method matched to the data:

- Servo dynamics: bootstrap Monte Carlo over repeated PRPS data units.
- Fixed thrust: pre-run and post-run baselines at the intended feedforward command.
- Friction: residual force fitting plus conservative spatial overbounds.

Those artifacts feed the [rough truth model](rough_truth_model.md) and the simulation
validation of the [crude stabilizer](crude_stabilizer_design.md). The purpose is not to
claim the plant is fully known; it is to make the model uncertainty explicit enough that
the first closed-loop controller can be stress-tested before hardware deployment.

---

## 1. Why uncertainty quantification matters here

The current phase is a bootstrap problem:

$$
\text{rough model} \rightarrow \text{crude stabilizer} \rightarrow
\text{closed-loop ID data} \rightarrow \text{earned model}
$$

The rail cannot be safely excited open-loop across the full operating envelope, so the
first stabilizer must be designed before the final closed-loop model exists. Uncertainty
quantification is what keeps that honest. Instead of tuning the stabilizer against one
best-fit plant, the controller is checked against plausible variations in actuator
dynamics, thrust level, and friction disturbance.

For this phase, the uncertainty artifacts answer three practical questions:

1. How much do the identified actuator parameters move when the finite dataset is
   reweighted?
2. How much fixed thrust is actually produced at the chosen feedforward command under the
   observed voltage and run conditions?
3. How large can the unmodeled friction residual be after the simple rough truth model has
   explained the parts it can explain?

---

## 2. Servo dynamics: bootstrap Monte Carlo over PRPS data

The servo model used in the [rough truth model](rough_truth_model.md) is a
first-order-plus-delay transfer function:

$$
G_\theta(s) =
\frac{\theta(s)}{u_\theta(s)}
= \frac{K_\theta}{1 + \tau_\theta s}e^{-L_\theta s}
$$

The nominal fit is documented in
[experiments/servo_identification/results.md](../experiments/servo_identification/results.md),
and the uncertainty procedure is documented in
[experiments/servo_identification/bootstrap_uncertainty.md](../experiments/servo_identification/bootstrap_uncertainty.md).

### Data structure

The servo PRPS campaign used repeated periodic excitation across multiple command
amplitudes. Because each PRPS period is a complete excitation cycle, the bootstrap unit can
be a whole period rather than an arbitrary time slice. For the current dataset, the default
period bootstrap uses:

$$
4\ \text{amplitudes}
\times 4\ \text{training files per amplitude}
\times 4\ \text{periods per file}
= 64\ \text{period units}
$$

Stratified sampling preserves the amplitude mix by resampling the same number of units
from each run group. This matters because the servo has mild amplitude-dependent dynamics:
the gain is stable, but the fitted time constant varies with excursion size.

### Procedure

The MATLAB entry point is
[bootstrap_servo_fopd_parameters.m](../analysis/system_identification/servo_identification/servo_prps_log/bootstrap_servo_fopd_parameters.m).
Each bootstrap draw:

1. Samples complete PRPS units with replacement.
2. Reassembles those units into a synthetic training dataset.
3. Estimates the empirical frequency-response function.
4. Refits the same first-order-plus-delay model family.
5. Scores the result against fixed held-out validation data.
6. Stores $K_\theta$, $\tau_\theta$, $L_\theta$, fit metrics, validation metrics, and
   sampled-unit metadata.

The result is a cloud of physically paired parameter samples:

$$
\left\{K_\theta^{(i)},\ \tau_\theta^{(i)},\ L_\theta^{(i)}\right\}_{i=1}^{N}
$$

### How it is used

Simulation Monte Carlo should sample complete rows from the bootstrap cloud, not sample
independent min/max ranges for each parameter. A row preserves the empirical correlation
between gain, lag, and delay that was produced by the same resampled dataset.

This is a genuine Monte Carlo uncertainty set: it estimates how sensitive the selected
model family is to the finite PRPS dataset. It does not reopen servo model selection and it
does not claim to capture every possible hardware fault or future mechanical change.

---

## 3. Thrust: fixed-command baseline envelope

The dynamic thrust model is identified as:

$$
G_T(s) =
\frac{\Delta T(s)}{\Delta u_T(s)}
= \frac{K_T}{1 + \tau_Ts}e^{-L_Ts}
$$

That dynamic model is documented in
[experiments/thrust_identification/results.md](../experiments/thrust_identification/results.md).
For the crude stabilizer, however, the motor is held near a fixed feedforward command and
the servo angle regulates rail acceleration. Therefore the immediately useful uncertainty
is not only the global dynamic gain; it is the actual fixed thrust produced at:

$$
u_T^\ast = 1825\ \text{microseconds}
$$

The fixed-command uncertainty procedure is documented in
[experiments/thrust_identification/fixed_pwm_uncertainty.md](../experiments/thrust_identification/fixed_pwm_uncertainty.md).

### Data structure

The accepted thrust PRPS files include three segment types:

- `baseline_pre`: fixed PWM before PRPS excitation.
- `prps`: dynamic excitation around the center command.
- `baseline_post`: fixed PWM after PRPS excitation.

For fixed-thrust uncertainty, only the `baseline_pre` and `baseline_post` segments are
used. These baseline segments directly measure the constant-thrust condition that the
crude stabilizer relies on. They also capture run-to-run effects such as battery voltage,
warming, load-cell drift, and post-excitation changes.

For the current $1825$ microsecond analysis, the default unit is the final settled tail of
one baseline segment. There are six baseline units:

$$
3\ \text{files} \times 2\ \text{baseline segments per file} = 6\ \text{units}
$$

### Procedure

The MATLAB entry point is
[bootstrap_thrust_fixed_pwm_baseline.m](../analysis/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_thrust_fixed_pwm_baseline.m).
For each baseline unit, the script records mean thrust, thrust variation, battery voltage,
and current when available. Each bootstrap draw samples the baseline units with
replacement and computes a resampled mean fixed thrust and mean voltage.

The thrust artifact is therefore two things at once:

- a bootstrap distribution of the observed mean fixed thrust;
- a raw observed min/max envelope across the actual baseline units.

Because only six baseline units are available, the conservative design range should come
from the observed envelope, while the bootstrap distribution is useful for understanding
how the mean changes when the available baseline units are reweighted.

### How it is used

The crude stabilizer should test low-thrust and high-thrust cases separately:

- Low $T^\ast$ checks whether servo authority is still sufficient to overcome friction and
  track position.
- High $T^\ast$ checks whether the same gains become too aggressive or drive servo
  saturation.

This is deliberately operating-point-specific. It is not a replacement for the global
thrust dynamic model; it is the uncertainty that matters most for the Phase 1 stabilizer.

---

## 4. Friction: residual fitting and spatial overbounds

Friction is the least-settled subsystem and is treated more conservatively than the
actuators. The current results are documented in
[experiments/friction_identification/results.md](../experiments/friction_identification/results.md),
and the simulation overbound procedure is documented in
[experiments/friction_identification/friction_disturbance_overbound.md](../experiments/friction_identification/friction_disturbance_overbound.md).

### Residual method

The friction campaign used open-loop runs that pushed the cart across the rail. During
those runs, the identified servo and thrust models predict the rail-direction force:

$$
\hat{F}_\text{rail}(t)
= \hat{T}(t)\sin\!\left(\hat{\theta}(t)\right)
$$

Encoder data provides position, velocity, and acceleration estimates. The unexplained
force residual is attributed to friction and other rail-direction disturbances:

$$
\hat{F}_\text{friction}(t)
= \hat{T}(t)\sin\!\left(\hat{\theta}(t)\right)
- M\hat{\ddot{x}}(t)
$$

This method is intentionally model-aware. It does not fit friction from motion alone; it
subtracts the force already explained by the rough actuator model and asks what remains.

### Dynamic friction fit

The first layer fits a simple viscous-plus-Coulomb structure:

$$
\hat{F}_\text{friction}(v)
= -bv - \mu_c\operatorname{sign}(v)
$$

The current multi-angle pass estimates approximately:

$$
b \approx 2.3\ \text{N s/m},
\qquad
\mu_c \approx 1.0\ \text{N}
$$

Direction-dependent fits are also kept because the data show meaningful asymmetry. The
friction artifact therefore carries symmetric, high, low, and asymmetric-worst cases
rather than one overconfident nominal value.

### Spatial residual overbound

After subtracting the rough truth model and the crude dynamic friction fit, the remaining
residual still has structure versus rail position. That remainder is interpreted
conservatively as possible spatial friction or rail disturbance:

$$
F_\text{spatial}(p,r)
= \sum_i A_i(r)
\exp\!\left(
-\frac{\left(p-p_i(r)\right)^2}{2\sigma_i(r)^2}
\right)
$$

The exported simulation disturbance family is:

$$
F_\text{fric}(p,v,r)
= -b_\pm(r)v
-\mu_\pm(r)\operatorname{sign}(v)
-F_\text{spatial}(p,r)
$$

with a separate stiction and breakaway rule near $v = 0$.

### How it is used

Unlike the servo bootstrap, the friction artifact should not be read as a probability
distribution. It is a provisional robustness overbound built from candidate data. Its job
is to make the crude stabilizer survive:

- high Coulomb friction;
- direction-asymmetric friction;
- stiction and breakaway;
- localized rail-position disturbances;
- residual force not explained by the current rough truth model.

This is the right posture for Phase 1 because friction is exactly where the current model
is least earned. The controller should be robust to it, not delicately tuned around one
friction coefficient.

---

## 5. Summary of simulation inputs

| Subsystem | Artifact | Simulation use | Interpretation |
|---|---|---|---|
| Servo | Bootstrap cloud of $K_\theta$, $\tau_\theta$, and $L_\theta$ | Sample complete parameter rows in Monte Carlo | Dataset sensitivity for the selected FOPD model |
| Thrust | Fixed-$u_T^\ast$ baseline units and bootstrap means | Sweep low, nominal, and high fixed thrust | Observed operating-point envelope, including voltage/run variation |
| Friction | Dynamic cases, stiction range, and spatial bump overbounds | Stress-test controller robustness | Conservative disturbance family, not a probability law |

Together these artifacts convert the rough truth model from a single nominal plant into a
family of plausible plants. That is the key validation step before giving the crude
stabilizer authority over hardware.
