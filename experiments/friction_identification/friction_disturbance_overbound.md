# Friction Disturbance Overbound Procedure

## Purpose

The current friction data show large Coulomb friction, directional asymmetry, stiction, and intermittent residual structure along the rail. A single fitted pair of $b$ and $\mu_c$ is not enough to stress-test the crude stabilizer.

This procedure builds a provisional V&V disturbance family from the current `candidate/` friction data. It is intended for controller validation, not as a finalized friction identification result.

## Disturbance Form

The simulation-facing disturbance family is:

$$
F_\text{fric}(p, v, r) =
-b_\pm(r)v
-\mu_\pm(r)\operatorname{sign}(v)
-F_\text{spatial}(p, r)
$$

with a separate stiction rule near $v = 0$. The sign $\pm$ is selected by velocity direction.

The spatial term is represented by bounded localized bumps:

$$
F_\text{spatial}(p,r) =
\sum_i A_i(r)\exp\left(-\frac{(p-p_i(r))^2}{2\sigma_i(r)^2}\right)
$$

This lets the simulator generate multiple rail-friction realizations without assuming the friction is purely velocity-dependent.

## Data and Method

The analysis entry point is [`build_friction_disturbance_overbound.m`](../../analysis/system_identification/friction_identification/friction_sweep_log/build_friction_disturbance_overbound.m).

It reuses the same residual method as [`analyze_friction_sweep.m`](../../analysis/system_identification/friction_identification/friction_sweep_log/analyze_friction_sweep.m):

$$
\hat{F}_\text{friction}(t) =
\hat{F}_\text{thrust}(t)\sin(\hat{\theta}(t))
-M\hat{\ddot{x}}(t)
$$

The script:

1. Loads the current `candidate/` friction sweep runs.
2. Reconstructs position, velocity, and acceleration from encoder counts.
3. Predicts thrust and servo dynamics using the identified actuator models.
4. Computes the signed friction residual.
5. Fits direction-dependent dynamic friction by velocity sign.
6. Extracts breakaway/stiction bounds from halt classifications.
7. Subtracts the dynamic fit and analyzes remaining residual force versus rail position.
8. Exports dynamic, stiction, and spatial bounds for simulation.

## Outputs

Numeric outputs and figures are written to [`plots/system_identification/friction_identification/friction_sweep_log/friction_disturbance_overbound/`](../../plots/system_identification/friction_identification/friction_sweep_log/friction_disturbance_overbound/). The generated report is written to [`reports/system_identification/friction_identification/friction_sweep_log/friction_disturbance_overbound/`](../../reports/system_identification/friction_identification/friction_sweep_log/friction_disturbance_overbound/).

The main artifacts are:

- `friction_overbound_params.mat` -- MATLAB struct for controller simulations.
- `friction_overbound_params.json` -- portable dynamic/stiction/spatial bounds.
- `friction_residual_samples.csv` -- signed residual samples used to derive the overbound.
- `friction_overbound_summary.csv` -- compact numeric summary.
- `friction_disturbance_overbound.report.md` -- generated report.
- `friction_force_vs_velocity_envelope.png` -- measured residuals and simulation cases.
- `friction_residual_vs_position.png` -- residual remainder versus rail position.
- `friction_spatial_bump_realizations.png` -- example bounded spatial disturbances.

## Simulation Interface

The exported struct contains:

```matlab
frictionOverbound.dynamic.low
frictionOverbound.dynamic.nominal
frictionOverbound.dynamic.high
frictionOverbound.dynamic.asymmetric_worst

frictionOverbound.stiction.breakaway_low_N
frictionOverbound.stiction.breakaway_high_N
frictionOverbound.stiction.v_epsilon_mps

frictionOverbound.spatial.bump_count_range
frictionOverbound.spatial.amplitude_abs_max_N
frictionOverbound.spatial.width_range_m
frictionOverbound.spatial.position_range_m
frictionOverbound.spatial.seed
```

For conservative crude-stabilizer tests, use the `high` or `asymmetric_worst` dynamic case, the high stiction threshold, and sampled spatial bump realizations.

## Interpretation

This overbound intentionally mixes identified structure and conservative padding. It should be used to answer:

- Can the controller recover from high breakaway friction?
- Does the controller remain stable under direction-asymmetric friction?
- Does position regulation survive localized rail disturbance bumps?
- Is integral action sufficient when Coulomb friction consumes a large fraction of available rail force?

It should not be interpreted as a probability distribution over physical friction. The current data are still untriaged candidate data, and friction remains the least-settled subsystem.
