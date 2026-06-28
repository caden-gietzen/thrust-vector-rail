# Actuator Modeling Approach

> **Status:** Design rationale. This captures the actuator-modeling reasoning that sits upstream of feasibility, hardware selection, and detailed system identification. It formalizes the day-zero conclusion that the rail does not directly command or measure lateral force; it composes lateral force from thrust magnitude, vector angle, and the vectoring mechanism dynamics.

This document records the actuator decomposition that motivates the rest of the project narrative. It follows the step-zero plant abstraction in [plant_model_structure.md](plant_model_structure.md): first define how the actuator can produce rail-direction force, then decide what must be measured, then check whether the available motor and vectoring hardware can meet the motion and precision requirements. The current feasibility analysis in [hardware_selection.md](hardware_selection.md), the assembled rough plant in [rough_truth_model.md](rough_truth_model.md), and the subsystem identification work in [system_identification.md](system_identification.md) all depend on this modeling choice.

## 1. Core conclusion

The rail-direction actuator is not a directly measured force source. With the current sensors, the project can directly measure:

- servo or vector angle dynamics,
- static or dynamic thrust magnitude, and
- cart position along the rail.

It cannot directly measure the full input-output transfer function from commanded lateral force to actual lateral force. The lateral force model must therefore be composed from pieces:

$$
u_\theta \rightarrow \theta \rightarrow F_x
$$

with thrust supplied by the motor command:

$$
u_T \rightarrow T
$$

The central geometric model is:

$$
F_x = T\sin\theta
$$

where $F_x$ is rail-direction force, $T$ is thrust magnitude, and $\theta$ is the thrust-vector angle relative to the vertical axis.

This makes the first actuator question a modeling question, not a controller question: **what force bandwidth and authority can be inferred from the measured thrust and vectoring subsystems?**

The resulting feasibility convention is that lateral-force actuation bandwidth can be approximated as equal to vectoring actuation bandwidth, provided thrust is held near a fixed operating point and the angle-to-force projection remains quasi-static. That approximation is what feeds the hardware-selection problem: seek a motor with enough thrust authority and a vectoring mechanism with enough bandwidth to satisfy the rail's motion and precision requirements. The full nonlinear sine projection remains in simulation, but the vectoring bandwidth is the practical buy-down number for lateral-force bandwidth.

## 2. Why this should precede feasibility

Feasibility should not start from "pick a motor" or "tune a controller." It should start from the mechanism that accelerates the cart. For this rail, acceleration is generated indirectly: the motor produces thrust magnitude, the servo redirects that thrust, and only the horizontal component accelerates the cart.

That means early feasibility must check two coupled actuator limits:

- **Authority:** can the available thrust magnitude and vector angle produce the required peak rail force?
- **Bandwidth:** can the vectoring mechanism render the commanded force direction fast enough for the required motion and disturbance rejection?

This is the conceptual input to [hardware_selection.md](hardware_selection.md): motor sizing is an authority problem, while servo selection is a vectoring bandwidth problem. The two cannot be fully separated because they meet through $F_x = T\sin\theta$.

## 3. Constant-thrust operating assumption

For the first control-oriented model, choose a fixed motor operating point and hold thrust approximately constant:

$$
T(t) \approx T_0
$$

The servo or vectoring mechanism dynamics are represented as:

$$
\frac{\Theta(s)}{\Theta_\text{cmd}(s)} = G_\theta(s)
$$

The rail force is then:

$$
F_x(t) = T_0\sin\theta(t)
$$

For small angles about $\theta = 0$:

$$
F_x(t) \approx T_0\theta(t)
$$

For small-signal motion about an operating angle $\theta_0$:

$$
\Delta F_x \approx T_0\cos(\theta_0)\Delta\theta
$$

and therefore:

$$
\frac{\Delta F_x(s)}{\Delta\Theta_\text{cmd}(s)}
\approx T_0\cos(\theta_0)G_\theta(s)
$$

This is the key early modeling result: under constant-thrust, small-signal assumptions, the lateral-force bandwidth is inherited from the servo or vectoring angle bandwidth. The factor $T_0\cos(\theta_0)$ changes force amplitude, not bandwidth.

In simulation, the model should keep both pieces explicit: a linear servo-angle response followed by a static nonlinear force projection,

$$
\theta_\text{cmd} \rightarrow G_\theta(s) \rightarrow \theta
\rightarrow F_x = T_0\sin\theta
$$

For small perturbations around zero, this reduces to:

$$
F_x \approx T_0\theta
$$

which is the approximation that makes lateral-force bandwidth inherit the servo-angle bandwidth. For larger sweeps, including the current $\pm 30^\circ$ operating envelope, the simulation should retain the full sine projection $F_x = T_0\sin\theta$ so that force amplitude, saturation, and harmonic distortion are not hidden by the small-angle approximation.

## 4. Commanded-force interpretation

If a controller commands desired lateral force $F_{x,\text{cmd}}$ while thrust is held at $T_0$, the corresponding angle command is:

$$
\theta_\text{cmd} = \sin^{-1}\left(\frac{F_{x,\text{cmd}}}{T_0}\right)
$$

For small angles:

$$
\theta_\text{cmd} \approx \frac{F_{x,\text{cmd}}}{T_0}
$$

The actual angle follows the vectoring dynamics:

$$
\theta(s) = G_\theta(s)\theta_\text{cmd}(s)
$$

and the actual lateral force is:

$$
F_x(s) \approx T_0\theta(s)
$$

Substituting the small-angle command mapping gives:

$$
F_x(s) \approx T_0G_\theta(s)\frac{F_{x,\text{cmd}}(s)}{T_0}
$$

so:

$$
\frac{F_x(s)}{F_{x,\text{cmd}}(s)} \approx G_\theta(s)
$$

This is useful but must be described carefully. It does **not** mean the project has measured lateral-force dynamics directly. It means the commanded-force to inferred-force bandwidth is approximately the measured vectoring-angle bandwidth when thrust is constant and the angle-to-force map is quasi-static.

## 5. What each experiment contributes

### 5.1 Static thrust map

The thrust test identifies the relationship between motor command and thrust magnitude:

$$
T = f(u_T)
$$

This gives the available force scale and the feasible constant-thrust operating points. It answers the authority question: for a given maximum vector angle, how much rail force can the system produce?

The current thrust results are documented in [experiments/thrust_identification/results.md](../experiments/thrust_identification/results.md).

### 5.2 Static vectoring geometry

The vectoring geometry maps thrust magnitude and angle to rail force:

$$
F_x = T\sin\theta
$$

For a target force amplitude $A_F$ at thrust $T_0$, the required angle amplitude is:

$$
A_\theta = \sin^{-1}\left(\frac{A_F}{T_0}\right)
$$

This relationship is also where saturation enters. If $\lvert F_x\rvert > T_0\sin\theta_{\max}$, the required rail force is outside the vectoring authority of that motor-angle pair.

### 5.3 Dynamic servo or vectoring model

The servo identification measures the dynamic relationship between angle command and actual angle:

$$
G_\theta(s) = \frac{\theta(s)}{\theta_\text{cmd}(s)}
$$

or, when using PWM directly:

$$
G_{\theta u}(s) = \frac{\theta(s)}{u_\theta(s)}
$$

This determines attenuation and phase lag. If:

$$
\theta_\text{cmd}(t) = A_{\theta,\text{cmd}}\sin(2\pi f t)
$$

then the measured angle response is approximately:

$$
\theta(t) =
\left|G_\theta(j2\pi f)\right|A_{\theta,\text{cmd}}
\sin(2\pi f t + \phi_\theta)
$$

The inferred lateral-force amplitude is:

$$
A_{F_x}(f) \approx
T_0\left|G_\theta(j2\pi f)\right|A_{\theta,\text{cmd}}
$$

and the inferred lateral-force phase lag is approximately the angle phase lag:

$$
\phi_{F_x}(f) \approx \phi_\theta(f)
$$

The current servo results are documented in [experiments/servo_identification/results.md](../experiments/servo_identification/results.md).

## 6. Reporting convention

The project should distinguish two bandwidths:

1. **Measured servo/vector angle bandwidth.**

This is directly identified from the angle response:

   $$
   G_\theta(j\omega) = \frac{\theta(j\omega)}{\theta_\text{cmd}(j\omega)}
   $$

The bandwidth is the frequency where:

   $$
   20\log_{10}\left(
   \frac{\left|G_\theta(j2\pi f_\text{BW})\right|}
        {\left|G_\theta(0)\right|}
   \right) = -3
   $$

2. **Inferred lateral-force bandwidth.**

This is model-derived from the measured angle dynamics and static thrust geometry:

   $$
   f_{\text{BW},F_x} \approx f_{\text{BW},\theta}
   $$

for a specified thrust level $T_0$, operating angle $\theta_0$, and command amplitude.

The phrase **measured lateral-force bandwidth** should be avoided unless a lateral force sensor is added. The preferred terms are **inferred lateral-force bandwidth** or **model-derived vectoring-force bandwidth**.

Suggested report language:

> Because lateral force was not directly measured, the vectoring-force bandwidth was inferred by combining the measured servo angle dynamics with a static thrust-vectoring model. At approximately constant thrust $T_0$, lateral force was modeled as $F_x = T_0\sin\theta$. Linearizing about operating angle $\theta_0$ gives $\Delta F_x \approx T_0\cos(\theta_0)\Delta\theta$, so the small-signal lateral-force bandwidth is approximately equal to the measured servo-angle bandwidth. This estimate assumes thrust magnitude remains constant during vectoring and that the angle-to-force mapping is quasi-static.

## 7. Failure modes and limitations

This composed actuator model is defensible, but it should not be oversold as a fully measured lateral-force model. The approximation can break if:

- thrust magnitude changes while vectoring because of airflow interaction, frame blockage, battery sag, or ESC behavior;
- large vector angles make the sine mapping meaningfully nonlinear;
- the servo bandwidth measured without thrust load is optimistic relative to powered operation;
- backlash, compliance, or friction make the measured servo angle differ from the true thrust vector angle;
- the load cell measures thrust magnitude or a vertical component, not dynamic lateral force.

These define the honesty boundary for the current hardware and the uncertainty cases that should be carried into simulation.

## 8. Ideal-world validation tests

If time and hardware access were unlimited, the current sensor suite could still increase confidence in the composed model through the tests below. These are not blockers for the current phase. The practical project path is to carry these effects as bounded modeling uncertainties, stress them in Monte Carlo simulation, and constrain operation below the identified vectoring bandwidth so the failure modes remain outside the intended operating envelope.

### 8.1 Thrust constancy versus vector angle

Hold motor command fixed and sweep vector angle while measuring thrust:

$$
T_\text{meas}(\theta)
$$

If thrust stays approximately constant over the planned vectoring range:

$$
T(\theta) \approx T_0
$$

then the inferred lateral-force model is stronger. If thrust varies with angle, the static geometry should become:

$$
F_x = T(\theta)\sin\theta
$$

### 8.2 Powered servo bandwidth

Run servo PRPS while the motor is producing a representative constant thrust. Compare:

$$
G_{\theta,\text{unpowered}}(j\omega)
$$

against:

$$
G_{\theta,\text{powered}}(j\omega)
$$

If this test were performed and powered bandwidth were lower, the powered result would become the vectoring bandwidth used for feasibility and controller design. Without that test, the unpowered result should be treated as optimistic and protected with bandwidth margin in simulation and operation.

### 8.3 Repeat across thrust levels

Repeat the powered servo identification at several thrust operating points:

$$
T_0 \in \{T_1, T_2, \ldots, T_n\}
$$

Then report a bandwidth map:

$$
f_{\text{BW},\theta}(T_0, A_\theta)
$$

and infer:

$$
f_{\text{BW},F_x}(T_0, A_F)
$$

using:

$$
A_\theta = \sin^{-1}\left(\frac{A_F}{T_0}\right)
$$

## 9. Clean final structure this enables

This modeling approach suggests the following actuator-characterization sequence:

1. **Static thrust map:** identify $T = f(u_T)$.
2. **Static vectoring map:** identify $\theta = g(u_\theta)$ and use $F_x = T\sin\theta$.
3. **Dynamic vectoring model:** identify $G_\theta(s) = \theta(s)/\theta_\text{cmd}(s)$.
4. **Inferred lateral-force model:** compose a linear vectoring response with the static nonlinear projection $F_x(t) = T_0\sin\theta(t)$; use $F_x \approx T_0\theta$ only for small-signal bandwidth interpretation.
5. **Limitation statement:** explicitly state that lateral force bandwidth is inferred, not directly measured.

The rough truth model in [rough_truth_model.md](rough_truth_model.md) is the current implementation of this structure using identified servo, thrust, and provisional friction parameters. Later closed-loop identification should test how well this composed open-loop model predicts the actual coupled rail dynamics.

## 10. Bottom line

Given the current hardware:

$$
\text{directly measured: servo/vector angle bandwidth}
$$

$$
\text{directly measured: thrust magnitude}
$$

$$
\text{inferred: lateral-force bandwidth through } F_x = T_0\sin\theta
$$

The main linearized relationship is:

$$
\Delta F_x(s) \approx T_0\cos(\theta_0)\Delta\Theta(s)
$$

so:

$$
f_{\text{BW},F_x} \approx f_{\text{BW},\theta}
$$

under constant-thrust, small-signal, quasi-static force-mapping assumptions.
