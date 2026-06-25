# Servo PRPS Bootstrap Uncertainty Procedure

## Purpose

The nominal PRPS identification selects one best-fit first-order-plus-delay servo model. That model is useful for design, but it does not show how sensitive the fitted parameters are to the finite dataset.

The bootstrap procedure estimates that sensitivity by repeatedly resampling the existing PRPS data, refitting the same model family, and saving the resulting parameter cloud. The cloud is intended for downstream simulation sweeps and robustness checks.

## Model Family

The bootstrap does not re-open model selection. It uses the already-selected first-order-plus-delay model:

$$
G(s) = \frac{K}{1 + \tau s} e^{-Ls}
$$

where $K$ is the servo gain in rad/$\mu$s, $\tau$ is the actuator time constant, and $L$ is the effective delay.

## Bootstrap Units

Sampling with replacement means a synthetic training set may include the same unit more than once and omit other units.

Two unit definitions are supported:

- **Period bootstrap:** each unit is one complete repeated PRPS period, identified by `period_index`.
- **Episode bootstrap:** each unit is one complete CSV episode.

The default is period bootstrap. The servo PRPS firmware logs complete periodic excitation repeats using `period_index` and `period_sample_index`, so complete PRPS periods can be resampled without cutting through the middle of an excitation cycle.

For the current dataset, period mode uses 64 units:

$$
4\ \text{amplitudes} \times 4\ \text{training files per amplitude} \times 4\ \text{periods per file} = 64\ \text{periods}
$$

With stratified sampling enabled, each bootstrap draw preserves the original amplitude mix by resampling the same number of units from each `run_name`.

## Procedure

The analysis entry point is [`bootstrap_servo_fopd_parameters.m`](../../analysis/system_identification/servo_identification/servo_prps_log/bootstrap_servo_fopd_parameters.m).

For each bootstrap draw:

1. Load the PRPS training files from `candidate/training`.
2. Normalize the CSVs using the same sign convention and reconstruction logic as the nominal PRPS analysis.
3. Split the training data into period or episode units.
4. Sample units with replacement, preserving the amplitude mix by default.
5. Reassemble the sampled units into one synthetic training table.
6. Estimate an empirical frequency response from the synthetic training table.
7. Fit one first-order-plus-delay model.
8. Score that model against the fixed validation data in `candidate/validation`.
9. Store $K$, $\tau$, $L$, fit metrics, validation metrics, and sampled-unit metadata.

Failed or unphysical fits are kept in the raw sample table with `fit_status = "failed"`, but they are excluded from the summary percentiles.

## Outputs

Numeric outputs and figures are written to [`plots/system_identification/servo_identification/servo_prps_log/bootstrap_fopd/`](../../plots/system_identification/servo_identification/servo_prps_log/bootstrap_fopd/). The generated report is written to [`reports/system_identification/servo_identification/servo_prps_log/bootstrap_fopd/`](../../reports/system_identification/servo_identification/servo_prps_log/bootstrap_fopd/).

The main artifacts are:

- `bootstrap_parameter_samples.mat` -- full parameter cloud, bootstrap metadata, FRFs, and validation FRF.
- `bootstrap_parameter_samples.csv` -- one row per bootstrap draw.
- `bootstrap_parameter_summary.csv` -- mean, standard deviation, and percentile summary for $K$, $\tau$, and $L$.
- `bootstrap_servo_fopd_parameters.report.md` -- generated summary report.
- `bootstrap_parameter_histograms.png` -- marginal distributions.
- `bootstrap_parameter_pairs.png` -- parameter correlation view.
- `bootstrap_bode_envelope.png` -- frequency-response envelope implied by the parameter cloud.

Downstream simulations should prefer the full correlated sample cloud over independent min/max parameter sweeps:

```matlab
load bootstrap_parameter_samples.mat
sample = validSamples(i, :);

K = sample.K_rad_per_us;
tau = sample.tau_s;
L = sample.delay_s;
```

Using complete sample rows preserves correlations between $K$, $\tau$, and $L$ that would be lost if each parameter were swept independently.

## Running

Default period-bootstrap run:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); run('c:/dev/thrust-vector-rail/analysis/system_identification/servo_identification/servo_prps_log/bootstrap_servo_fopd_parameters.m')"
```

Short debug run:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); N_BOOT=5; RESAMPLE_MODE='period'; run('c:/dev/thrust-vector-rail/analysis/system_identification/servo_identification/servo_prps_log/bootstrap_servo_fopd_parameters.m')"
```

Episode-bootstrap debug run:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); N_BOOT=5; RESAMPLE_MODE='episode'; run('c:/dev/thrust-vector-rail/analysis/system_identification/servo_identification/servo_prps_log/bootstrap_servo_fopd_parameters.m')"
```

The console line `Bootstrap draws: ...` is the definitive check for the active value of `N_BOOT`.

## Interpretation

The bootstrap does not replace the nominal identified model. It answers a different question: given the observed PRPS data and the selected model family, how much do the fitted parameters move when the available repeated measurements are reweighted?

A tight parameter cloud indicates the FOPD fit is insensitive to which periods or episodes dominate the dataset. A wider cloud indicates that some units carry meaningful leverage over the fit. In later simulation work, the cloud can be used directly as a Monte Carlo uncertainty set for actuator dynamics.
