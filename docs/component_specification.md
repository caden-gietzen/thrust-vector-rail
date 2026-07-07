# Component Specification Sheet

> **Status:** Purchase-facing required-spec sheet — the terminal artifact of [feasibility](feasibility.md). It converts the a priori [requirements](requirements.md) and [qualification maneuvers](qualification_test_plan.md) into a **required specification for every purchased component**, so each part has a documented reason for why it is needed to meet the requirements. Every number here is a **conservative a priori over-bound derived before identification** — specs come first, and later subsystem identification must land *inside* these bounds (if identification ever exceeds a bound, the bound — not the requirement — is what is re-examined). Nothing here uses a measured transfer function as an input; identified values appear only as *confirmation* callouts.

## 1. Philosophy: over-bound first, identify second

A component's specification must hold **before** that component is characterized — you can't identify a servo's transfer function to decide which servo to buy. So every input a purchased part is sized against is a deliberate over-bound of the worst case:

- **Mass** is taken at the heavy end of the design range ($0.75$ kg) wherever more mass is worse (authority) and at the light end ($0.45$ kg) wherever less mass is worse (feedback excursion, since $e \propto 1/m$).
- **Disturbance** is the full [R13](requirements.md#5-disturbance-rejection) $1.0$ N injected force step, taken as coincident with breakaway friction (worst-case timing).

**Friction is handled differently — and deliberately so.** Friction is not a spec of a purchased part; it is a **disturbance** the drive must overcome, and it is a property of the **rail mechanism** (bearings, pulley, rail, encoder mount), *independent* of which motor, prop, servo, or ESC is bolted on. So it is **characterized first**, then the actuators are sized to beat it with margin. This is the correct real-world order — you measure a plant's parasitic loads before sizing its drive — and it is not circular: measuring friction does not depend on the selection decision. The friction over-bound used here is the identified **worst-direction breakaway**, $\mu_c(1+\text{asym}) \approx 1.30$ N, taken once (not double-inflated). It is **PROVISIONAL pending re-identification on the 2026-06-26 rebuilt mechanism** ([friction_identification/results.md](../experiments/friction_identification/results.md)); that re-run is the step that *locks* this input. If post-rebuild friction comes in lower (the expectation), every downstream spec only gains margin.

A component **passes** only if its required spec is met with these inputs. That is what lets the purchase be justified without waiting on the purchased part's own identification.

## 2. Shared over-bound inputs

Every component spec flows from this one table. Each value is tagged **REQUIREMENT** (traces to a requirement), **DISTURBANCE** (a characterized parasitic load, over-bounded), or **ASSUMPTION** (a design choice, replace with vendor data when a part is chosen).

| Quantity | Symbol | Value | Tag | Trace / rationale |
|---|---|---:|---|---|
| Reference amplitude | $A$ | $100$ mm | REQUIREMENT | [R1](requirements.md#1-operating-envelope) |
| Reference frequency | $f_\text{ref}$ | $0.5$ Hz | REQUIREMENT | [R2](requirements.md#1-operating-envelope) |
| Heavy mass (authority) | $m_H$ | $0.75$ kg | REQUIREMENT | [qualification_test_plan.md §2](qualification_test_plan.md#2-common-setup-and-scoring) design range |
| Light mass (feedback) | $m_L$ | $0.45$ kg | REQUIREMENT | [qualification_test_plan.md §2](qualification_test_plan.md#2-common-setup-and-scoring) design range |
| Peak tracking error | $e_\text{peak}$ | $15$ mm | REQUIREMENT | [R4](requirements.md#2-tracking-performance) |
| Injected disturbance | $F_\text{inj}$ | $1.0$ N | REQUIREMENT | [R13](requirements.md#5-disturbance-rejection) / M3 |
| Coulomb friction (nominal) | $\mu_c$ | $1.0$ N | DISTURBANCE | identified pooled breakaway ([friction_identification/results.md](../experiments/friction_identification/results.md)); provisional, pre-rebuild |
| Directional asymmetry | asym | $+30\%$ | DISTURBANCE | identified worst-direction inflation |
| Usable vector deflection | $\theta_\max$ | $45^\circ$ | ASSUMPTION | geometric ceiling for the authority floor |
| Encoder scale | — | $64.8$ counts/mm | REQUIREMENT-derived | [encoder_calibration/results.md](../experiments/encoder_calibration/results.md); sets the quantization floor |

**Friction disturbance over-bound (motor must physically overcome it)** — the identified nominal breakaway inflated *once* to the worst direction (no separate "slack" factor; the earlier $1.625$ N figure double-counted asymmetry against a $1.25$ slack allowance):

$$
F_\text{fric} = \mu_c\,(1 + \text{asym}) = 1.0 \times 1.30 = 1.30\ \text{N}
$$

**Governing rail-axis force over-bound** — the larger of the two coincident worst cases the actuator must produce:

$$
F_{x,\max} = \max\underbrace{\big(m_H A (2\pi f_\text{ref})^2 + F_\text{fric}\big)}_{\text{tracking: peak accel + breakaway} = 0.74 + 1.30 = 2.04\ \text{N}},\ \underbrace{\big(F_\text{inj} + F_\text{fric}\big)}_{\text{M3 hold: reject step + breakaway} = 1.0 + 1.30 = 2.30\ \text{N}} = 2.30\ \text{N}
$$

The M3 hold case governs: the motor must break friction *and* counter the full injected disturbance at once. Everything downstream sizes off $F_{x,\max} = 2.30$ N.

## 3. The spec sheet

One row per purchased component. **Required spec** is the over-bound-derived number the part must meet; **Trace** is the requirement it flows from; **Why** is the one-line justification.

| Component | Required spec | Trace | Why it gets us to the requirement |
|---|---|---|---|
| **Motor + prop (thrust)** | static thrust $\ge 0.95$ kgf ($\approx 9.3$ N; a $\sim 1$ kgf part clears it); hard floor $\ge 0.33$ kgf | [R7](requirements.md#3-actuation-requirements)/[R8](requirements.md#3-actuation-requirements), $F_{x,\max}$ | Produces the $2.30$ N rail force at a *small* vector angle so the servo's slew limit does not bind the vectoring bandwidth — see [§4.1](#41-motor--prop-thrust). |
| **Servo (vectoring)** | **hard:** delay $\le 25$ ms, bandwidth $\ge 5$ Hz; **target:** bandwidth $\sim 10$ Hz; slew $\ge 900\,^\circ$/s, travel $\ge \pm 45^\circ$ | [R9](requirements.md#3-actuation-requirements), [R13](requirements.md#5-disturbance-rejection)/M3, M1/M2 | Delay + $\ge 5$ Hz support the M3 crossover (M1/M3 need no more); $\sim 10$ Hz is a conservative margin from M2's open-loop feedforward screen — see [§4.2](#42-servo-vectoring-actuator). |
| **ESC** | continuous current $\ge 30$ A, matched to motor $K_v$ and cell count, fast PWM/OneShot | motor peak current | Delivers motor peak current with $\gtrsim 1.5\times$ margin and no thermal fold-back during a run — see [§4.3](#43-esc). |
| **Battery / supply** | cell count matched to motor (rig runs $\approx 6$S); low sag; $\ge 25$C; $\ge 1.5$ Ah **or** a bench DC supply | thrust repeatability, run time | Holds voltage steady so the (voltage-sensitive) thrust map is repeatable across a campaign — see [§4.4](#44-battery--power-supply). |
| **Encoder** | $\ge 60$ counts/mm; quadrature | [R10](requirements.md#4-sensing-and-logging-requirements), error budget | Position quantum $\le 0.02$ mm keeps the sensing floor negligible in the $15$ mm budget — see [§4.5](#45-encoder). |
| **Load cell + ADC** | range $\ge$ motor max thrust (**$\ge 20$ N** for the upgraded rig); resolution $\le 0.02$ N | thrust ID / [R11](requirements.md#4-sensing-and-logging-requirements) | Covers the full thrust command range for static + dynamic thrust identification; the installed cell is **under-range** — see [§4.6](#46-load-cell--adc). |
| **MCU (Pico)** | control loop $\ge 200$ Hz; servo PWM resolution $\le 0.5$ µs; HW quadrature; $\ge 2$ PWM, $\ge 1$ ADC, $\ge 1$ UART | [R11](requirements.md#4-sensing-and-logging-requirements)/[R12](requirements.md#4-sensing-and-logging-requirements), crossover | Samples $\gg 20\times$ the actuator bandwidth and resolves the servo command finely enough that compute adds no meaningful error or delay — see [§4.7](#47-mcu-raspberry-pi-pico). |

## 4. Derivations (why each number)

### 4.1 Motor + prop (thrust)

**Hard authority floor.** At the geometric deflection ceiling $\theta_\max = 45^\circ$, the minimum thrust to produce the governing rail force is

$$
T_\text{floor} = \frac{F_{x,\max}}{\sin\theta_\max} = \frac{2.30}{\sin 45^\circ} = 3.25\ \text{N} = 0.33\ \text{kgf}.
$$

A part below this cannot produce the required force at all. But the floor is **not** the recommended buy, because sizing to it forces the servo to swing to $45^\circ$, and a large deflection amplitude collides with the servo slew limit.

**Small-deflection target (the real spec).** Rail force is $F_x = T\sin\theta$, so buying thrust *beyond* the floor shrinks the required deflection $\theta_\text{req} = \arcsin(F_{x,\max}/T)$. A smaller swing raises the slew-limited vectoring ceiling $f_\text{slew} = \dot\theta_\max / (2\pi\,\theta_\text{req})$. To keep that ceiling above the $\sim 10$ Hz servo *target* ([§4.2](#42-servo-vectoring-actuator)) with a $900\,^\circ$/s servo, the worst-case deflection must satisfy $\theta_\text{req} \le 14.3^\circ$, which requires

$$
T \ge \frac{F_{x,\max}}{\sin 14.3^\circ} = \frac{2.30}{0.247} = 9.3\ \text{N} = 0.95\ \text{kgf}.
$$

So the spec is **$\ge 0.95$ kgf ($\approx 9.3$ N) static thrust** — a $\sim 1$ kgf-class part clears it — which holds the worst-case deflection to $\le 14.3^\circ$ and the slew-limited ceiling to $\ge 10$ Hz, clearing the $\sim 10$ Hz vectoring target.

> **Current-hardware status (identified confirmation).** The propulsion was **upgraded** after the original static sweep. The constant-thrust-hold campaign (2026-07-03/05, [fit_thrust_voltage_model.m](../analysis/system_identification/thrust_identification/thrust_constant_hold_log/fit_thrust_voltage_model.m)) measures $\approx 9.5$ N at only $40\%$ command (at $V \approx 22.3$ V), so the current hardware **already clears the $0.95$ kgf ($9.3$ N) target with substantial throttle headroom** — thrust authority is met, and the binding constraint is the servo vectoring bandwidth, not thrust. The earlier swept static map ($\approx 4.2$ N max, [thrust_identification/results.md](../experiments/thrust_identification/results.md)) is **stale**: it predates the prop/motor/mount change. The open thrust item is therefore *not* a bigger motor but **re-running the static command→thrust sweep on the current hardware** to lock the absolute map and confirm the operating-point deflection.

**Thrust dynamics are not a driver in Phase 1.** The crude bootstrap stabilizer holds thrust constant and vectors only ([crude-stabilizer scope](rough_truth_model.md)), so the motor/prop only needs *steady, repeatable* thrust at the hold setpoint — there is no thrust-bandwidth requirement yet. (The identified thrust FOPD $\tau \approx 78$ ms is recorded for later phases but does not size the purchase.)

### 4.2 Servo (vectoring actuator)

The servo has two genuinely hard, maneuver-derived constraints and a third that is a **conservative target, not a bare requirement**. Keeping them separate matters, because at face value M1 and M3 do *not* demand a fast servo — the candidate screen in [analyze_requirement_feasibility.m](../analysis/feasibility/analyze_requirement_feasibility.m) couples bandwidth and delay in each row, which hides which one is actually binding.

**Hard constraint 1 — delay $\le \sim 25$ ms.** M3 rejects the injected force step at a crossover set by the light mass (worst for $e \propto 1/m$):

$$
f_c = \frac{1}{2\pi}\sqrt{\frac{F_\text{inj}}{m_L\,e_\text{peak}}} = \frac{1}{2\pi}\sqrt{\frac{1.0}{0.45\cdot 0.015}} = 1.94\ \text{Hz},
$$

and the servo must leave phase headroom there ($\text{lag} \le 45^\circ$ on $G_\text{act}(s) = e^{-sT_d}/(\tau s + 1)$). That budget is **delay-dominated**: at $1.94$ Hz, $25$ ms of pure delay costs $17.5^\circ$ while a $10$ Hz pole costs only $11^\circ$. Delay — not bandwidth — is what a slow servo runs out of first. (The old servo's $\approx 29$ ms delay is exactly why it failed and was replaced.)

**Hard constraint 2 — bandwidth $\ge \sim 5$ Hz.** With delay held at $\sim 25$ ms, the M3 phase/gain screen passes for bandwidth $\ge 4$ Hz (at $3$ Hz the lag hits $50^\circ > 45^\circ$). M1 is irrelevant here — its $\theta_\text{ff}$ energy is entirely below $0.4$ Hz. So **M1 and M3 together only ask for roughly a $5$ Hz / $\le 25$ ms servo.**

**Conservative target — bandwidth $\sim 10$ Hz.** The push above $\sim 5$ Hz comes **only from M2**, through a deliberately conservative screen: M2's feedforward-rendering error — the position error *if the servo rendered the $0.5$ Hz sine by open-loop inverse dynamics alone* — reaches the $15$ mm limit around $8$ Hz at $25$ ms delay. But M2 is tracked **closed-loop**, so feedback corrects that rendering residual; the $12.8$ mm is an open-loop worst case, not a real tracking error. Crediting feedback relaxes the bandwidth back toward the M3-driven $\sim 5$ Hz. The $\sim 10$ Hz target therefore carries margin from (a) this pure-feedforward M2 screen and (b) a "servo $\approx 3\times$ crossover" rule of thumb — it is a robustness cushion, not a maneuver requirement.

**Slew $\ge 900\,^\circ$/s** and **travel $\ge \pm 45^\circ$** are set by [§4.1](#41-motor--prop-thrust): the servo must swing the worst-case deflection without slew-limiting the vectoring bandwidth it is designed to (the $\sim 10$ Hz target).

> **Status.** The upgraded digital servo (re-identified 2026-06-29) delivers a $\approx 9.3$ Hz corner, $\approx 878\,^\circ$/s slew, and $\approx 13.4$ ms delay ([servo_identification/results.md](../experiments/servo_identification/results.md)) — it clears both hard constraints with room and sits essentially on the $\sim 10$ Hz target. The hardware decision is unaffected by the hard-vs-target distinction; the reframing is about *why* the number is what it is.

### 4.3 ESC

The ESC must pass the motor's peak current with margin. For a $\approx 0.95$ kgf static-thrust motor/prop, an a priori static efficiency of $6$–$8$ gf/W puts electrical power at $\approx 118$–$158$ W; the peak current then depends on pack voltage — $\approx 18$ A at $3$S ($11.1$ V) down to $\approx 7$ A at the current rig's $\approx 6$S ($22$ V). Spec **continuous $\ge 30$ A** ($\gtrsim 1.5\times$ the worst-case $3$S peak) so the ESC never thermally folds back mid-run. Match its cell-count and PWM/OneShot protocol to the chosen motor and to the Pico's PWM output.

> These wattage/current numbers are a priori sizing bands, not identified — replace with the vendor thrust/power table once a motor/prop is chosen.

### 4.4 Battery / power supply

The identified static thrust map is **voltage-sensitive** ([thrust_identification/results.md](../experiments/thrust_identification/results.md)): sag between a fresh and a depleted pack shifts the command-to-thrust map, which corrupts thrust repeatability across a campaign. Two acceptable options:

- **Bench DC supply (preferred for a testbed):** a regulated supply at the motor voltage eliminates sag entirely, giving a repeatable thrust map run-to-run. This is the better choice for a bench rig that never needs to be untethered.
- **Battery:** cell count to match the chosen motor (the current upgraded rig runs $\approx 6$S / $22$ V), **$\ge 25$C** and **$\ge 1.5$ Ah** so the peak draw is a modest fraction of capacity (low sag) and a working session spans many runs between swaps.

Either way the sizing driver is **thrust-map repeatability**, not run time.

### 4.5 Encoder

Position is the measured output ([R10](requirements.md#4-sensing-and-logging-requirements)) and its quantization is a floor in the error budget. At $\ge 60$ counts/mm the quantum is $\le 0.017$ mm, which is $\sim 0.1\%$ of the $15$ mm budget — negligible, as required. The installed encoder is $64.8$ counts/mm ($0.015$ mm quantum), already inside spec. Quadrature (two channels) is required for direction.

### 4.6 Load cell + ADC

The load cell is not in the control loop; it exists to **identify** the thrust actuator ([R11](requirements.md#4-sensing-and-logging-requirements)). Its range must cover the motor's **maximum** thrust across the full command sweep, with **$\le 0.02$ N resolution** to capture the static map and PRPS dynamics cleanly. Since the upgraded rig already reads $\approx 9.5$ N at only $40\%$ command, full-throttle output is well above that, so the cell needs a **$\ge 20$ N** range with margin.

> **Gap callout.** The installed HX711 cell appears **under-range for the upgraded motor**: the constant-thrust-hold campaign flagged readings clipping near $\approx 8$ N as non-physical ([analyze_thrust_constant_hold.m](../analysis/system_identification/thrust_identification/thrust_constant_hold_log/analyze_thrust_constant_hold.m)). A higher-capacity cell is needed before the current-hardware thrust re-sweep ([§5](#5-what-remains-to-finalize)) can characterize the full command→thrust map. (The earlier HX711 *reliability* issue — distinct from range — was resolved via driver power-cycle + connection rework, [hx711_load_cell_validation.md](hardware_validation/hx711_load_cell_validation.md).)

### 4.7 MCU (Raspberry Pi Pico)

- **Control/log loop rate.** Digital control needs $f_s \gtrsim 20$–$30\times$ the closed-loop bandwidth. With a $\le 10$ Hz actuator bandwidth and $\approx 2$ Hz crossover, **$f_s \ge 200$ Hz** is the floor; the RP2040 ($125$ MHz, dual-core) runs the loop at $\ge 1$ kHz with room to spare. This traces to [R11](requirements.md#4-sensing-and-logging-requirements)/[R12](requirements.md#4-sensing-and-logging-requirements) (log fast enough to recompute every metric offline).
- **Servo PWM resolution.** To keep command quantization out of the error budget, the servo channel must resolve $\le 0.05^\circ$ of vector angle. At a typical $\sim 0.09^\circ$/µs servo, that is **$\le 0.5$ µs** PWM step; the Pico's $16$-bit PWM at $50$ Hz gives $\approx 0.3$ µs, inside spec.
- **Encoder decode.** Peak quadrature edge rate at the $0.15$ m/s peak velocity is $\approx 40$ kedge/s — trivial for the RP2040 **PIO** hardware quadrature decoder (MHz-class), which offloads decode from the CPU. HW (PIO) decode is required so the loop is not stolen by edge counting.
- **I/O channel count.** The rig needs $\ge 2$ PWM (servo + ESC), $\ge 1$ PIO state machine (quadrature), GPIO for the HX711 bit-bang, $\ge 1$ ADC (current sense), and $\ge 1$ UART (battery telemetry). The RP2040 covers all of these, which is why it is sufficient — the spec is *channel availability*, and the Pico clears it.

## 5. What remains to finalize

1. **Lock the friction disturbance bound (prerequisite).** Re-run friction identification on the 2026-06-26 rebuilt mechanism and lock the worst-direction breakaway. This is the one input the sheet treats as characterized-first, so locking it *locks the thrust spec*. If breakaway comes in under the provisional $1.30$ N (the expectation), every downstream spec keeps its margin; if it exceeds it, revisit $F_{x,\max}$. The current $1.30$ N is pre-rebuild and provisional.
2. **Pick concrete parts** and replace the ASSUMPTION-tagged bands (motor $K_v$, prop diameter/pitch, ESC model, cell count) with vendor datasheet numbers, then re-confirm each row passes.
3. **Re-run the static thrust sweep on the current (upgraded) hardware.** The swept command→thrust map is stale — it predates the prop/motor/mount change. The hold campaign shows the current rig makes $\approx 9.5$ N at $40\%$, already clearing the $0.95$ kgf target, but the absolute map must be re-identified to lock the operating-point deflection (see [§4.1](#41-motor--prop-thrust)). Thrust authority is **not** the open hardware gap — vectoring bandwidth is.

## 6. Reproduce

The actuation-side numbers (force envelope, thrust floor, servo bandwidth/delay screen) are reproduced by:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); run('c:/dev/thrust-vector-rail/analysis/feasibility/analyze_requirement_feasibility.m')"
```

The propulsion-chain and compute sizing (thrust → deflection → current → ESC/battery, loop rate, PWM resolution) are reproduced by [`analyze_propulsion_sizing.m`](../analysis/feasibility/analyze_propulsion_sizing.m).
