# Gain Scheduling — Approach and Simulation Tiers

## Simplified Nonlinear Model

State: $x = \begin{bmatrix} p \\ v \end{bmatrix}$, inputs: $u = \begin{bmatrix} \theta \\ T \end{bmatrix}$ (servo angle, thrust command)

$$\dot{x} = \begin{bmatrix} v \\ \frac{1}{m}(T \sin\theta - bv) \end{bmatrix}$$

## Linearization Analysis

Jacobians at operating point $\beta = (\theta_r, T_r)$:

$$A(\beta) = \frac{\partial F}{\partial x} = \begin{bmatrix} 0 & 1 \\ 0 & -b/m \end{bmatrix} \quad \leftarrow \text{constant (not } \beta\text{-dependent)}$$

$$B(\beta) = \frac{\partial F}{\partial u} = \begin{bmatrix} 0 & 0 \\ \frac{T}{m}\cos\theta & \frac{1}{m}\sin\theta \end{bmatrix}$$

**Key observation:** $A$ is constant; all operating-point dependence lives in $B$.

| $B$ entry | Dependence | Implication |
|---|---|---|
| $B_\theta = \frac{T}{m}\cos\theta$ | scales with $T$ and $\cos\theta$ | servo authority increases with thrust; diminishes as $\theta \to 90°$ |
| $B_T = \frac{1}{m}\sin\theta$ | scales with $\sin\theta$ | thrust authority increases with deflection |

$\theta_r$ affects both channels: it sets $B_T = \frac{1}{m}\sin\theta_r$ directly, and modulates $B_\theta$ through $\cos\theta_r$. $T_r$ only affects $B_\theta$ — by structure, differentiating $\frac{T\sin\theta}{m}$ with respect to $T$ drops $T$ out entirely, so $B_T$ carries no $T$ dependence. At small angles $B_T \approx 0$, meaning the thrust channel has little direct lateral authority near hover.

**Scheduling variable candidates:**

1. $\beta = \theta_r$ — servo angle only
2. $\beta = (\theta_r, T_r)$ — operating-point pair
3. $\beta = F_{x,r}$ — reference lateral force ($F_{x,r} = m a_y + b v_r$), a single scalar that encapsulates both

## Simulation Tiers

### Tier 1 — Fixed-Gain LQR (no scheduling)

Linearize once about $\theta = 0$, $T_r = 1700\ \mu\text{s}$ (~2.57 N from static thrust map deg-4 fit — see [thrust identification results](../experiments/thrust_identification/results.md)).

Design a single LQR gain matrix $K$ on the constant linearized plant. Baseline for comparing scheduled variants.

### Tier 2 — Angle-Scheduled LQR ($\beta = \theta_r$)

Linearize over a grid of $\theta_r$ values (e.g. $0°,\ 5°,\ 10°,\ \ldots,\ 60°$) at fixed nominal thrust.

Interpolate gain matrices $K(\theta_r)$. Validates whether angle alone captures enough variation.

### Tier 3 — Pair-Scheduled LQR ($\beta = (T_r,\ \theta_r)$)

Linearize over a 2D grid of $(T_r,\ \theta_r)$ pairs.

Interpolate $K(T_r,\ \theta_r)$. Motivated by the linear scaling of servo authority with thrust:
$B_\theta = \frac{T}{m}\cos\theta$ — at high thrust, the same angle command produces proportionally more lateral force.

## Operating-Point Notes

| Quantity | Value |
|---|---:|
| Tier 1 nominal command | $1700\ \mu\text{s}$ |
| Tier 1 nominal thrust | ~2.57 N (deg-4 polynomial) |
| Effective thrust range | $1075$–$1950\ \mu\text{s}$ → 0.23–4.17 N |
| Gain crossover (static map) | ~$1650\ \mu\text{s}$ ($K_T$ jumps 76%) |
