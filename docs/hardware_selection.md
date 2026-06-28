# Vectoring Hardware Selection & Overhaul Requirements

> **Status:** Forward-looking, **day-zero, plant-agnostic** analysis. It answers "what motor and what servo must the overhauled rail carry to hold the precision spec over the required motion envelope?" using only $m$, $A$, $f$, the error spec, $\theta_{\max}$, and a friction over-bound; none of these require the rail's identified transfer functions. Requirement-derived inputs trace to [requirements.md](requirements.md); margin and conservative over-bounds are added here before evaluating hardware. Every number below is reproduced by [`analysis/feasibility/capability_maps.py`](../analysis/feasibility/capability_maps.py); the companion per-frequency envelope detail lives in [`analysis/feasibility/plant_authority_envelope.py`](../analysis/feasibility/plant_authority_envelope.py).

This document turns the a priori requirements in [requirements.md](requirements.md) into stricter hardware-selection criteria with margin. It sits upstream of the per-stage acceptance metrics in [project_metrics.md](project_metrics.md) and feeds the truth-model parameters consolidated in [rough_truth_model.md](rough_truth_model.md).

## 1. The decision this drives

The current rail is **authority-poor and slow-vectoring** at the required motion envelope: it fails on both thrust and servo bandwidth. The question is not "is it ad hoc" but "which knob do I buy": a stronger motor (authority) or a faster servo (bandwidth), and how much of each. The output is a single pair of numbers: **required motor thrust** and **required vectoring bandwidth**, each traceable to the worst-case mass and the binding physical constraint it came from. That pair is what goes to a hardware vendor.

## 2. Requirements

| Requirement | Value | Tag | Rationale |
|---|---|---|---|
| Reference motion envelope | $A = 150$ mm sinusoid at $f = 1$ Hz | REQUIREMENT | Requirements R1-R2 in [requirements.md](requirements.md); the feedforward channel must *produce* it. |
| Precision spec (peak position error) | $\lvert e\rvert \le 5$ mm | REQUIREMENT | Top-level hardware tracking requirement R5 in [requirements.md](requirements.md). |
| Vehicle mass range | $0.45$-$0.75$ kg | REQUIREMENT | Design mass envelope R3 in [requirements.md](requirements.md). |
| Usable vectoring deflection | $\theta_\max = 45^\circ$ | FEASIBILITY ASSUMPTION | Candidate usable deflection for sizing; replace if selected hardware differs. |
| Friction over-bound | $F_d = \mu_s(mg + N_\text{preload})$, $\mu_s = 0.15$, $N_\text{preload} = 15$ N | FEASIBILITY MARGIN | Conservative breakaway model used to avoid sizing hardware on the edge. |

## 3. Architecture: two channels, one actuator (2-DOF)

The reference and the disturbance are **distinct jobs** and must not be conflated (running the reference amplitude through the feedback sensitivity badly over-specifies crossover â€” the error this analysis exists to correct).

- **Feedforward channel â€” owns the known reference.** Its job is *producing* the $150$ mm @ $1$ Hz motion. Feasibility is an **authority + actuator-bandwidth** check, never a crossover check: the motor must generate the peak lateral force, and the servo must render the inverse-dynamics command (content at $f_\text{ref}$ and harmonics) with low distortion.
- **Feedback channel â€” owns disturbance rejection only.** It does **not** chase the reference. Its input is the friction over-bound $F_d$ as an input-referred force; its crossover is solved from that disturbance and its error-budget share alone.

The two channels share **one** actuator, so the authority ledger is checked **simultaneously**, not as independent budgets (see [Â§7](#7-shared-actuator-coupling-check)).

## 4. Error budget

One peak-error spec, allocated by root-sum-square across independent contributors:

$$
e_\text{total} = \sqrt{e_\text{ff}^2 + e_\text{fb}^2 + e_\text{quant}^2 + e_\text{model}^2} \le 5\ \text{mm}
$$

| Contributor | Allocation | Actual @ selected HW | Drives |
|---|---|---|---|
| Encoder quantization $e_\text{quant}$ | $0.015$ mm | $0.015$ mm | encoder $64.8$ counts/mm |
| Model-error reserve $e_\text{model}$ | $1.000$ mm | $1.000$ mm | held for sim/ID mismatch |
| Feedforward residual $e_\text{ff}$ | $0.35$ mm | $0.25$ mm | servo bandwidth |
| Feedback (friction) $e_\text{fb}$ | $4.89$ mm | $4.89$ mm | crossover |
| **RSS total** | $5.00$ mm | $4.99$ mm | $\le 5$ mm spec |

**The split is optimized, not assumed.** Feedback ($e_\text{fb}\propto F_d/\omega_c^2$) is the expensive channel; the feedforward residual is cheap once the servo is fast. Minimizing the required servo bandwidth over the feedforward/feedback allocation hands feedback **essentially the entire** remaining budget (optimal $\approx 0.3\%$ feedforward / $99.7\%$ feedback). This lowers the required bandwidth from $20.7$ Hz (naive $50/50$) to $17.4$ Hz. The optimum sits near the boundary, so a small feedforward margin may be worth holding for robustness â€” the $1$ mm model reserve already provides global cushion.

## 5. Hardware buy criteria (the two numbers)

> ### **Required motor thrust: $1.12$ kgf ($11.0$ N)** â€” from authority, **heavy** mass ($0.75$ kg).
> ### **Required vectoring bandwidth: $17.4$ Hz** (crossover $5.8$ Hz) â€” from feedback disturbance rejection, **light** mass ($0.45$ kg).

The two requirements come from **opposite mass ends**, and each must be met at its own worst case:

- **Authority** scales with mass (more inertia to accelerate), so it is worst at the **heavy** end.
- **Disturbance-rejection bandwidth** scales as $e_\text{fb}\propto 1/m$ (less inertia resists the excursion), so it is worst at the **light** end. $F_d$ is itself recomputed per mass through its $mg$ term.

### 5.1 Authority (feedforward)

$$
T_\text{req} = \frac{m\,A\,(2\pi f)^2 + F_d}{\sin\theta_\max}
$$

At heavy mass: peak lateral force $m A(2\pi f)^2 = 4.44$ N, plus breakaway $F_d = 3.35$ N, over $\sin 45^\circ$ gives $11.0$ N $= 1.12$ kgf. The $f^2$ term makes frequency the dominant cost â€” see [Â§6](#6-sensitivities-and-the-levers). The required-thrust map over the whole $(A,f)$ plane, with candidate-motor contours and the aspiration marker, is [`select_authority_map.png`](../plots/feasibility/select_authority_map.png).

### 5.2 Bandwidth (feedback)

Feedback rejecting a **step** input force (breakaway at motion onset/reversal) to its budget share:

$$
e_\text{fb} \approx \frac{F_d}{m\,\omega_c^2}\quad\Longrightarrow\quad \omega_c \ge \sqrt{\frac{F_d}{m\,e_\text{fb}}}
$$

At light mass with $F_d = 2.91$ N and $e_\text{fb} = 4.89$ mm: $\omega_c \ge 5.8$ Hz. The servo bandwidth must exceed the crossover by the phase-margin factor (see [Â§10](#10-assumptions-and-what-to-refine)):

$$
f_\text{servo} \ge \frac{f_c}{K_\text{WC}},\qquad K_\text{WC} = 0.333\ (\text{servo} \approx 3\times\text{crossover})
$$

giving $17.4$ Hz. The feedforward channel imposes two further (here non-binding) lower bounds: the residual bound $f_\text{servo} \ge f_\text{ref}\sqrt{A/2e_\text{ff}}$ and a command-rendering floor $f_\text{servo} \ge 5 f_\text{ref}$.

## 6. Sensitivities and the levers

| Lever | Effect | Takeaway |
|---|---|---|
| **Reference frequency** ($0.5 \to 2$ Hz at $A=150$ mm) | thrust $0.64 \to 3.05$ kgf | $f^2$ law â€” $1$ Hz costs $1.8\times$ what $0.5$ Hz does; the cheapest authority relief is a slower or smaller aspiration. |
| **Friction** ($\mu_s\ 0.02 \to 0.15$) | required bandwidth $6.6 \to 17.4$ Hz | $\sqrt{F_d}$ law â€” **measuring the real breakaway force is the single cheapest way to relax the servo spec**; slick bearings could halve it. |
| **Budget split** (FF share) | $20.7$ Hz (naive $50/50$) $\to 17.4$ Hz (optimal) | feedback is the expensive channel; give it nearly all the budget. |

The friction and budget-split curves are [`select_bandwidth_drivers.png`](../plots/feasibility/select_bandwidth_drivers.png); the budget waterfall and the $f^2$ authority cost are [`select_budget_and_freq.png`](../plots/feasibility/select_budget_and_freq.png).

## 7. Shared-actuator coupling check

For a sinusoid, velocity reverses (breakaway $F_d$) at the **position extremes**, where acceleration is also peak â€” so the feedforward peak force and the breakaway friction land at the **same instant**. The actuator must cover both at once:

$$
F_\text{total} = m A (2\pi f)^2 + F_d = 4.44 + 3.35 = 7.79\ \text{N}\ \Rightarrow\ T = F_\text{total}/\sin\theta_\max = 11.0\ \text{N} = 1.12\ \text{kgf}
$$

This **equals** the required-thrust headline â€” the coupling *is* the authority number, which is why the reserve in the authority formula is taken as $F_d$ itself, not a separate margin.

## 8. Secondary checks

- **Slew.** Peak commanded vectoring rate to render the reference is $\approx 145^\circ$/s at the required thrust and heavy mass, inside the $240^\circ$/s servo limit â€” the reference is not slew-limited once authority is met.
- **Sensor-noise upper bound on crossover.** The encoder quantization noise, propagated through the closed-loop noise bandwidth, sets an *upper* limit on usable crossover; it is **non-binding** here (the encoder is far from the limit). A naive velocity-differentiation scheme would lower that upper edge and should be re-checked once the estimator is designed.

## 9. Feasibility verdict

Need thrust $\ge 1.12$ kgf **and** servo bandwidth $\ge 17.4$ Hz, across the mass range:

| Candidate | Thrust (heavy) | Bandwidth (light) | Verdict |
|---|---|---|---|
| current ($0.43$ kgf, $5$ Hz) | FAIL | FAIL | **FAIL** |
| upgrade A ($1.0$ kgf, $15$ Hz) | FAIL | FAIL | **FAIL** |
| upgrade B ($2.0$ kgf, $20$ Hz) | PASS | PASS | **OK** |
| upgrade C ($3.0$ kgf, $30$ Hz) | PASS | PASS | **OK** |

The matrix figure is [`select_feasibility_matrix.png`](../plots/feasibility/select_feasibility_matrix.png). **Upgrade B is the cheapest viable pair** â€” and it only clears bandwidth *because* the budget split is optimized ($20$ Hz $> 17.4$ Hz required, but $20 < 20.7$ Hz at the naive split). That margin is thin and rests entirely on the conservative friction bound, which is the strongest argument for measuring breakaway before committing.

## 10. Assumptions and what to refine

In rough priority order:

1. **Friction over-bound $F_d$ ($\mu_s$, $N_\text{preload}$).** Drives the bandwidth requirement via $\sqrt{F_d}$ and is currently a deliberate worst case. A direct **breakaway-force measurement** (ramp-to-motion, already supported by [`firmware/pico_micropython/lib/encoder_home.py`](../firmware/pico_micropython/lib/encoder_home.py)) is the highest-value next step; see the friction-mechanism status in [`experiments/friction_identification/results.md`](../experiments/friction_identification/results.md).
2. **$K_\text{WC} = 0.333$ (servo $= 3\times$ crossover).** A phase-margin **placeholder**, not a derived value â€” refine once a plant model exists and a real phase budget can be set.
3. **Aspirational motion ($150$ mm @ $1$ Hz) and mass range.** These define the envelope; if the real mission is slower or lighter, the authority requirement drops on the $f^2$/$m$ laws.
4. **Candidate actuator specs.** Only the current motor ($0.425$ kgf) and servo (~$5$ Hz) are MEASURED; the upgrade pairs are datasheet-class placeholders â€” replace with real vendor numbers before purchase.
5. **Model-error reserve ($1$ mm) and the near-boundary budget split.** Consider holding a small feedforward margin for robustness rather than the strict bandwidth-minimizing optimum.

## 11. Reproduce

```sh
.venv/Scripts/python analysis/feasibility/capability_maps.py
```

Prints the buy criteria, the budget ledger, the coupling/slew/noise checks, the per-candidate feasibility table, and all sensitivity sweeps; writes the four `select_*.png` figures to [`plots/feasibility/`](../plots/feasibility/). All parameters live in a tagged config block at the top of the script.
