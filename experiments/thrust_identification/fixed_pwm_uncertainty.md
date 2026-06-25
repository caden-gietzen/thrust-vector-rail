# Fixed-PWM Thrust Uncertainty Procedure

## Purpose

The crude stabilizer holds thrust at a fixed feedforward PWM and regulates rail position primarily through servo angle. For that phase, the most useful thrust uncertainty is not the full dynamic model uncertainty, but the actual thrust produced at the selected fixed command.

This procedure estimates the observed fixed-thrust envelope at:

$$
u_T^\ast = 1825~\mu\text{s}
$$

using the baseline segments in the accepted thrust PRPS files whose center command is 1825 $\mu$s.

## Data Used

The relevant files are the `local_1700_1950` PRPS runs in [`data/raw/system_identification/thrust_identification/thrust_prps_daq_voltage/accepted/`](../../data/raw/system_identification/thrust_identification/thrust_prps_daq_voltage/accepted/).

Each file contains:

- `baseline_pre`: fixed PWM at the PRPS center command before excitation
- `prps`: dynamic excitation around the center command
- `baseline_post`: fixed PWM at the PRPS center command after excitation

Only the baseline segments are used for this fixed-thrust analysis. The PRPS excitation segment is useful for dynamic identification, but it is not a direct steady-state constant-thrust measurement.

## Bootstrap Units

Each bootstrap unit is the final settled tail of one baseline segment. By default, the analysis uses the final 40 samples:

$$
40~\text{samples} \times 0.02~\text{s/sample} \approx 0.8~\text{s}
$$

For the current dataset, there are six units:

$$
3~\text{files} \times 2~\text{baseline segments per file} = 6~\text{baseline units}
$$

The units are intentionally not split into smaller windows by default. Adjacent windows from the same baseline share the same battery state, thermal condition, and load-cell behavior, so treating them as independent would overstate the effective sample count.

## Procedure

The analysis entry point is [`bootstrap_thrust_fixed_pwm_baseline.m`](../../analysis/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_thrust_fixed_pwm_baseline.m).

For each baseline unit, the script records:

- mean thrust
- thrust standard deviation
- thrust min/max
- mean battery voltage
- voltage min/max
- mean current, when available

For each bootstrap draw:

1. Sample the six baseline units with replacement.
2. Compute the mean fixed thrust of the resampled units.
3. Compute the mean voltage of the resampled units.
4. Store the sampled unit IDs for auditability.

The bootstrap summarizes how the observed units reweight the mean. Because only six units are available, the raw observed min/max envelope is also reported and should be used as the conservative design range.

## Outputs

Numeric outputs and figures are written to [`plots/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_fixed_pwm_1825/`](../../plots/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_fixed_pwm_1825/). The generated report is written to [`reports/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_fixed_pwm_1825/`](../../reports/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_fixed_pwm_1825/).

The main artifacts are:

- `fixed_pwm_baseline_units.csv` -- one row per baseline unit.
- `fixed_pwm_bootstrap_samples.csv` -- one row per bootstrap draw.
- `fixed_pwm_bootstrap_summary.csv` -- observed and bootstrap percentile summaries.
- `fixed_pwm_bootstrap_samples.mat` -- full MATLAB output.
- `bootstrap_thrust_fixed_pwm_baseline.report.md` -- generated report.
- `fixed_pwm_thrust_histogram.png` -- bootstrap mean thrust distribution with observed envelope markers.
- `fixed_pwm_thrust_vs_voltage.png` -- observed baseline units and bootstrap means in force-voltage space.

## Running

Default run:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); run('c:/dev/thrust-vector-rail/analysis/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_thrust_fixed_pwm_baseline.m')"
```

Short debug run:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); N_BOOT=20; run('c:/dev/thrust-vector-rail/analysis/system_identification/thrust_identification/thrust_prps_daq_voltage/bootstrap_thrust_fixed_pwm_baseline.m')"
```

## Interpretation

This analysis estimates:

$$
T^\ast = F_{ss}(1825~\mu\text{s})
$$

under the voltage and run conditions represented in the accepted PRPS baseline segments.

It does not replace the dynamic thrust model:

$$
G_T(s) = \frac{K_T}{1 + \tau_T s}e^{-L_Ts}
$$

Instead, it provides a fixed-thrust envelope for the rough stabilizer. The low end of the observed envelope should be used when checking minimum servo authority, and the high end should be used when checking saturation and aggressiveness.
