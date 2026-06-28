# Pre-Overhaul Baseline

> **Status:** Historical baseline for before/after comparison. This preserves metrics from the first hardware and analysis pass before requirements, feasibility, hardware selection, and identification are revisited.

This document is not the front-door project narrative and should not be treated as the current requirements source. Its purpose is to preserve the old hardware and analysis baseline so future work can show what improved after revised requirements, updated feasibility analysis, hardware changes, and re-identification.

## 1. Why Preserve This

The project is moving back to day-zero assumptions and requirements so feasibility and hardware selection can be redone with a cleaner flowdown. Once new hardware is selected and re-identified, the original hardware values below provide the comparison point for showing actual gains:

- more thrust authority,
- faster or better-behaved vectoring,
- larger usable force bandwidth,
- reduced sensitivity to friction and saturation,
- improved closed-loop operating envelope,
- tighter uncertainty bounds and validation margins.

## 2. Source Inventory

This baseline pulls from the stable result and report artifacts available before the overhaul:

| Area | Source |
|---|---|
| Plant structure | [plant_model_structure.md](plant_model_structure.md), [rough_truth_model.md](rough_truth_model.md) |
| Encoder calibration | [experiments/encoder_calibration/results.md](../experiments/encoder_calibration/results.md) |
| Load-cell validation | [hardware_validation/hx711_load_cell_validation.md](hardware_validation/hx711_load_cell_validation.md) |
| Servo identification | [experiments/servo_identification/results.md](../experiments/servo_identification/results.md), [bootstrap_uncertainty.md](../experiments/servo_identification/bootstrap_uncertainty.md) |
| Thrust identification | [experiments/thrust_identification/results.md](../experiments/thrust_identification/results.md), [fixed_pwm_uncertainty.md](../experiments/thrust_identification/fixed_pwm_uncertainty.md) |
| Friction identification | [experiments/friction_identification/results.md](../experiments/friction_identification/results.md), [friction_disturbance_overbound.md](../experiments/friction_identification/friction_disturbance_overbound.md) |
| Uncertainty summary | [uncertainty_quantification.md](uncertainty_quantification.md) |
| Feasibility snapshot | [hardware_selection.md](hardware_selection.md) |
| Generated reports | `reports/system_identification/...`, `reports/controller_validation/...` |

## 3. Baseline Model Structure

The old baseline used the same structural state:

$$
\mathbf{x} = \begin{bmatrix} p \\ v \\ \theta \\ T \end{bmatrix}
$$

and the same thrust-vector coupling:

$$
F_x = T\sin\theta
$$

Those structural choices remain valid. The values below are the pre-overhaul hardware realization of that structure.

## 4. Measurement Baselines

| Metric | Baseline value | Notes |
|---|---:|---|
| Vehicle mass | $m \approx 0.4536$ kg | original cart measurement |
| Design mass uncertainty used in rough model | $\sigma_m = 0.136$ kg | roughly $30\%$ mass uncertainty |
| Rail end-stop separation | $316.5$ mm | encoder calibration traverse |
| Encoder scale | $64.810$ counts/mm, $64{,}810.4$ counts/m | measured, use in firmware |
| Encoder nominal scale | $60.000$ counts/mm | GT2 + 20-tooth pulley nominal |
| Encoder deviation from nominal | $+8.02\%$ | systematic geometry/belt difference |
| Encoder calibration repeatability | $0.01\%$ | max-min over mean |
| Encoder quantization floor | approximately $0.015$ mm/count | derived from measured scale |
| HX711 calibration slope | $415.008037$ counts/g | load-cell validation fit |
| HX711 calibration intercept | $148.655610$ counts | load-cell validation fit |
| HX711 fit RMSE | $258.75$ counts, approximately $0.62$ g | calibration validation |
| HX711 calibration masses | $0$, $50$, $100$, $200$, $500$ g | validation dataset |

## 5. Servo / Vectoring Baseline

The latest accepted pre-overhaul servo model is the two-band, large-signal operating-amplitude FOPD from [experiments/servo_identification/results.md](../experiments/servo_identification/results.md):

$$
G_\theta(s) = \frac{0.0014787}{1 + 0.02989s}e^{-0.03543s}
\quad [\text{rad}/\mu\text{s}]
$$

| Metric | Baseline value | Notes |
|---|---:|---|
| Accepted gain | $K_\theta = 0.0014787$ rad/$\mu$s | large-signal operating-amplitude fit |
| Accepted time constant | $\tau_\theta = 29.89$ ms | two-band re-run |
| Accepted delay | $L_\theta = 35.43$ ms | two-band re-run |
| Combined lag + delay | approximately $65$ ms | contributes approximately $23^\circ$ phase at $1$ Hz |
| Recommended initial rail-loop bandwidth | $\le 1$ Hz | pre-overhaul design guidance |
| Step-reconstruction corner | approximately $3.4$ Hz | from step response analysis |
| Peak measured slew rate | approximately $240^\circ$/s | fastest measured small-step transient |
| Conservative excitation slew ceiling | $120^\circ$/s | used for PRPS redesign |
| Loss-free encoder/vectoring envelope | approximately $\pm 36^\circ$ | fast moves beyond this lost counts |
| Fast-move count loss at large excursion | up to $5.4^\circ$/cycle at $\pm 55^\circ$ | encoder/vectoring reliability concern |
| Backlash / hysteresis | approximately $\pm 2^\circ$; $2.40^\circ$ center gap | carried as nonlinearity |
| Responsive static command range | $306$-$2681$ $\mu$s | discovery sweep |
| Static travel span | approximately $215^\circ$ | measured responsive travel |
| $180^\circ$ travel requirement | met with margin | pre-overhaul hardware capability |
| Lash-free static gain | $-0.092427^\circ/\mu$s, $-0.00161315$ rad/$\mu$s | branch-mean fit |
| Lash-free neutral | $1423.7$ $\mu$s | branch-mean fit |
| Branch slope agreement | $2.6\%$ | static map linearity |
| Within-branch gain ripple | $7.0\%$ up, $7.9\%$ down | residual nonlinearity |

### 5.1 Servo Uncertainty Metrics

| Metric | Baseline value | Notes |
|---|---:|---|
| Two-band bootstrap draws | $300$ | accepted two-band result |
| $K_\theta$ 95% CI | $0.0014781$-$0.0014793$ rad/$\mu$s | relative SE $0.04\%$ |
| $\tau_\theta$ 95% CI | $29.69$-$30.10$ ms | relative SE $0.7\%$ |
| $L_\theta$ 95% CI | $35.02$-$35.82$ ms | relative SE $1.1\%$ |
| Trusted high-band coherence | $\gamma^2 \ge 0.90$ to $10.775$ Hz | measured high-band edge |
| High-frequency multiplicative uncertainty | $W(f)$ overlap max $0.12$, extension max $0.42$ | small-signal band not merged |
| Amplitude-dependence decision | SEPARATE | small-signal high band outside large-signal CI tube |
| Static-vs-dynamic gain gap | approximately $8\%$ | large-signal $K$ below static chord |

### 5.2 Superseded Servo Snapshot

The older README-era servo values are retained only as history because they appeared in earlier narrative docs:

| Metric | Superseded value |
|---|---:|
| Prior-pass gain | $K_\theta = 0.001556$ rad/$\mu$s |
| Prior-pass time constant | $\tau_\theta = 24.4$ ms |
| Prior-pass delay | $L_\theta = 28.8$ ms |
| Prior-pass frequency range | $0.10$-$3.05$ Hz |
| Prior-pass held-out FRF error | worst case $0.5$ dB / $2.6^\circ$ |
| Prior-pass train-vs-validation gap | training $0.29$ dB vs validation $\le 0.5$ dB |
| Prior-pass bootstrap draws | $500$, all successful |
| Prior-pass bootstrap 95% CI | $K$: $0.0015793$-$0.0015889$ rad/$\mu$s; $\tau$: $23.75$-$32.08$ ms; $L$: $27.43$-$31.26$ ms |

## 6. Thrust Baseline

The pre-overhaul thrust model is the global first-order-plus-delay fit over the usable range:

$$
G_T(s) = \frac{0.00414}{1 + 0.0781s}e^{-0.0252s}
\quad [\text{N}/\mu\text{s}]
$$

| Metric | Baseline value | Notes |
|---|---:|---|
| Dynamic gain | $K_T = 0.00414$ N/$\mu$s | global $1100$-$1950$ $\mu$s |
| Dynamic time constant | $\tau_T = 78.1$ ms | global FOPD |
| Dynamic delay | $L_T = 25.2$ ms | global FOPD |
| Dominant bandwidth | $2.04$ Hz, $12.8$ rad/s | thrust actuator |
| PRPS excitation band | $0.075$-$2.55$ Hz | $22$ frequency points |
| Median PRPS sample period | $23$ ms | approximately $43$ Hz median |
| Median coherence | $\ge 0.996$ across runs | clean input-output relation |
| Global training magnitude RMSE | $0.187$ dB | frequency-domain fit |
| Global validation magnitude RMSE | $0.410$ dB | frequency-domain validation |
| Time-domain validation RMSE | $0.123$ N | $60$-$70$ s validation window |
| Time-domain validation relative RMSE | approximately $3.95\%$ | global validation window |

### 6.1 Static Thrust Map

| Metric | Baseline value | Notes |
|---|---:|---|
| Usable command range | $1075$-$1950$ $\mu$s | lower arm threshold to upper plateau |
| Usable thrust range | $0.23$-$4.17$ N | static sweep |
| Observed force range | $-0.116$ to $4.364$ N | full sweep raw range |
| Battery voltage range during sweep | $10.70$-$12.21$ V | voltage sag visible |
| Degree-4 static-map RMSE | $0.0234$ N | preferred global static map |
| Degree-4 static-map MAE | $0.0169$ N | preferred global static map |
| Degree-2 static-map RMSE | $0.0465$ N | comparison model |
| Degree-2 static-map MAE | $0.0396$ N | comparison model |
| Degree-4 improvement over degree-2 | approximately $50\%$ RMSE reduction | nonlinear map needed |
| Tare bias | approximately $-0.015$ N | motor-off setpoints |
| Motor-off one-sigma noise | approximately $0.016$ N | zero-force uncertainty |
| Zero-force two-sigma uncertainty | approximately $\pm 0.032$ N | force below this indistinguishable from zero |
| Hysteresis mean | $0.0286$ N | up/down sweep |
| Hysteresis max | $0.0579$ N | up/down sweep |
| Hysteresis relative scale | less than $1\%$ of usable range | likely negligible for initial design |

Static polynomial, with $\hat{u} = (u - 1525.0)/256.2$:

$$
F_{ss}(\hat{u}) =
1.828\times 10^{-5}\hat{u}^4
+ 0.053686\hat{u}^3
+ 0.17600\hat{u}^2
+ 1.06699\hat{u}
+ 1.74620
\quad [\text{N}]
$$

### 6.2 Thrust Operating-Region Metrics

| Region | Model | $K_T$ (N/$\mu$s) | $\tau_1$ (ms) | $L$ (ms) | Train RMSE | Val RMSE |
|---|---|---:|---:|---:|---:|---:|
| global $1100$-$1950$ | FOPD | $0.00414$ | $78.1$ | $25.2$ | $0.187$ dB | $0.410$ dB |
| local $1100$-$1350$ | second-order lag + delay | $0.00338$ | $148.3$ | $9.79$ | $0.122$ dB | $1.074$ dB |
| local $1400$-$1650$ | FOPD | $0.00405$ | $78.0$ | $29.1$ | $0.196$ dB | $0.643$ dB |
| local $1400$-$1950$ | FOPD | $0.00503$ | $58.0$ | $26.4$ | $0.223$ dB | $0.450$ dB |
| local $1700$-$1950$ | FOPD | $0.00647$ | $42.0$ | $27.2$ | $0.179$ dB | $0.458$ dB |

The static local gains also split into two near-linear regimes:

| Static region | Local gain | Linear RMSE | Degree-4 RMSE over same region |
|---|---:|---:|---:|
| $1075$-$1650$ $\mu$s | $0.00371$ N/$\mu$s | $0.0274$ N | $0.0159$ N |
| $1650$-$1950$ $\mu$s | $0.00654$ N/$\mu$s | $0.0335$ N | $0.0348$ N |

The upper-region gain is approximately $76\%$ larger than the lower-region gain. Dynamic local gain rises from approximately $0.0034$ to $0.0065$ N/$\mu$s, a roughly $91\%$ increase.

### 6.3 Fixed-Thrust Uncertainty at $u_T^\ast = 1825\ \mu\text{s}$

| Metric | Baseline value | Notes |
|---|---:|---|
| Baseline units | $6$ | $3$ files times pre/post baseline |
| Tail samples per unit | $40$ | approximately $0.8$ s |
| Observed thrust envelope | $2.9103$-$3.2389$ N | conservative fixed-thrust range |
| Observed voltage envelope | $10.639$-$11.446$ V | same baseline units |
| Bootstrap draws | $500$ | baseline-unit resampling |
| Bootstrap mean thrust 95% interval | $2.9451$-$3.1283$ N | reweighted mean |
| Bootstrap median mean thrust | $3.0288$ N | fixed-command estimate |
| Recommended conservative range | $T^\ast(1825\ \mu\text{s}) \in [2.910,\ 3.239]$ N | use raw envelope for design |

## 7. Friction and Disturbance Baseline

Friction is the least-settled subsystem. The baseline preserves both the fitted values and the overbound used for robustness testing.

| Metric | Baseline value | Notes |
|---|---:|---|
| Friction dataset | $18$ candidate runs | not finalized |
| Valid residual samples | $2132$ | overbound export |
| Selected model | viscous + Coulomb | pooled multi-angle pass |
| Pooled viscous coefficient | $b = 2.3154\ \text{N}\cdot\text{s}/\text{m}$ | current best estimate |
| Pooled Coulomb coefficient | $\mu_c = 1.0226$ N | current best estimate |
| Pooled RMSE | $0.4360$ N | current fit |
| Mass-sensitivity range for $b$ | $2.2882$-$2.3425\ \text{N}\cdot\text{s}/\text{m}$ | $M = 0.454 \pm 0.136$ kg |
| Mass-sensitivity range for $\mu_c$ | $1.0210$-$1.0242$ N | nearly mass-insensitive |
| Directional asymmetry in $\mu_c$ | $28.7\%$ | significant |
| Directional asymmetry in $b$ | $7.8\%$ | smaller but nonzero |
| Earlier Coulomb-only estimate | $\mu_c \approx 0.82$ N | superseded but retained as model-selection context |
| Earlier asymmetry estimate | approximately $42\%$ | superseded single-angle pass |

### 7.1 Directional Friction Fits

| Source | Direction | $b$ ($\text{N}\cdot\text{s}/\text{m}$) | $\mu_c$ (N) | RMSE (N) | $n$ |
|---|---|---:|---:|---:|---:|
| friction analysis | positive dir | $2.3345$ | $1.1335$ | $0.2988$ | $1238$ |
| friction analysis | negative dir | $2.5246$ | $0.8492$ | $0.5362$ | $894$ |
| overbound export | positive velocity | $2.4970$ | $0.8611$ | $0.4965$ | $892$ |
| overbound export | negative velocity | $2.3536$ | $1.1250$ | $0.3505$ | $1240$ |

The test design confounds servo direction and motion direction, so these asymmetries are important robustness metrics but not yet fully deconfounded friction physics.

### 7.2 Friction Overbound for Simulation

| Case | $b_+$ | $\mu_+$ | $b_-$ | $\mu_-$ |
|---|---:|---:|---:|---:|
| low | $1.9976$ | $0.6409$ | $1.8829$ | $0.9049$ |
| nominal | $2.4970$ | $0.8611$ | $2.3536$ | $1.1250$ |
| high | $2.9964$ | $1.3014$ | $2.8243$ | $1.5653$ |
| asymmetric worst | $3.1213$ | $1.5653$ | $1.7652$ | $0.7510$ |

| Overbound metric | Baseline value |
|---|---:|
| Breakaway range | $0.458$-$1.546$ N |
| Spatial position range | $-0.2861$ to $0.2944$ m |
| Spatial bump count | $2$-$5$ |
| Spatial bump width | $0.0150$-$0.0800$ m |
| Spatial bump amplitude bound | $0.4403$ N |

## 8. Feasibility and Hardware-Selection Snapshot

These are pre-overhaul feasibility numbers. They should be recalculated after the revised requirements and assumptions are finalized.

| Metric | Pre-overhaul value | Notes |
|---|---:|---|
| Required motor thrust | $1.12$ kgf, $11.0$ N | old feasibility criterion |
| Required vectoring bandwidth | $17.4$ Hz | old feasibility criterion |
| Feedback crossover implied by old analysis | $5.8$ Hz | disturbance-rejection sizing |
| Candidate current motor thrust | approximately $0.43$ kgf | failed authority criterion |
| Candidate current vectoring bandwidth | approximately $5$ Hz | failed bandwidth criterion |
| Feasibility verdict for current hardware | FAIL / FAIL | authority and bandwidth |
| Upgrade B placeholder | $2.0$ kgf, $20$ Hz | first old passing candidate |
| Upgrade C placeholder | $3.0$ kgf, $30$ Hz | old passing candidate |
| Maximum rail force at $20^\circ$ with old thrust max | $4.17\sin 20^\circ = 1.43$ N | from rough truth model |
| Friction / max rail force ratio | approximately $1.0/1.43 = 0.7$ | low-authority, friction-dominated |

## 9. Provisional Controller-Report Metrics

Generated Monte Carlo/controller reports exist in `reports/controller_validation/`. These are not final qualification metrics, but they are useful as pre-overhaul smoke-test baselines.

| Report class | Representative metric | Baseline value |
|---|---|---:|
| PRPS Monte Carlo, $N=20$, $0.05$ m excitation | mean `max_abs_error_m` | $0.0545$ m |
| PRPS Monte Carlo, $N=20$, $0.05$ m excitation | mean `saturation_fraction` | $0.0487$ |
| PRPS Monte Carlo, $N=20$, $0.05$ m excitation | worst rail margin | $0.1256$ m |
| PRPS Monte Carlo, $N=10$, $0.05$ m to $1$ Hz excitation | mean `max_abs_error_m` | $0.0572$ m |
| PRPS Monte Carlo, $N=10$, $0.05$ m to $1$ Hz excitation | saturation fraction | $0$ |
| Smoke track, $N=1$ | `max_abs_error_m` | $0.0389$ m |
| Smoke track, $N=1$ | rail margin | $0.2364$ m |

These values should not be over-interpreted. They are included so that later controller and hardware revisions can show whether early simulation behavior improved, especially saturation incidence, rail margin, and tracking error under comparable scenarios.

## 10. Comparison Metrics for the Next Hardware Pass

When the updated hardware is selected and re-identified, compare against this baseline using:

| Metric | Desired direction |
|---|---|
| Maximum thrust | increase |
| Usable rail force at required vector angle | increase |
| Vectoring bandwidth | increase |
| Vectoring delay | decrease |
| Vectoring uncertainty | decrease or move outside operating band |
| Thrust bandwidth | increase or remain non-limiting |
| Fixed-command thrust envelope width | decrease |
| Voltage sensitivity | decrease or become better modeled |
| Friction / available-force ratio | decrease |
| Authority margin over requirement | increase |
| Saturation incidence during qualification motion | decrease |
| Closed-loop tracking envelope | increase |
| Tracking error over the required envelope | decrease |
| Model validation error | decrease |
| Monte Carlo envelope coverage | improve |

The key claim to prove later is not simply that the new hardware is different. The claim is that revised requirements and feasibility analysis selected hardware that measurably improves authority, bandwidth, uncertainty, and closed-loop operating margin.
