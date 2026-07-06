#!/usr/bin/env python3
"""
design_servo_prps_excitation.py

Laptop-side designer for the re-run servo PRPS excitation. The Pico firmware
(servo_prps_log.py) only *plays back* a multisine; all of the design intelligence
lives here so the RP2040 stays a dumb, deterministic player.

Pipeline (mirrors experiments/servo_identification/results.md "Transition"):

  1. Build a line grid spanning ~f_min..f_max, weighted around and above the
     step-reconstruction corner f_c (~3.4 Hz) so phase and rolloff across the
     corner are *measured*, not extrapolated. Snap each line to a period bin
     (integer multiple of 1/period) and de-duplicate.
  2. Apply the per-line amplitude taper: A_flat for f <= f_c, 1/f for f > f_c.
     This is the envelope the firmware receives as PRPS_LINE_AMPLITUDES; peak
     normalization on the Pico makes the overall scale come from the TEST_RUNS
     amplitude (sized in step 4).
  3. Assign Schroeder phases (power-weighted by the taper), then run a
     crest-factor minimization pass (random restarts) over the taper-weighted
     multisine. Lower crest => more in-band power under the same peak command.
  4. Size the per-run command amplitude (us) so the realized peak command
     velocity equals a fraction of the 120 deg/s ceiling, using the *same*
     per-sample-difference metric the firmware guard checks, and additionally
     clamp to the encoder loss-free envelope (+/-36 deg).

Outputs PRPS_PHASES / PRPS_LINE_AMPLITUDES aligned to the snapped-bin order the
firmware reconstructs, and (optionally) writes them straight into the
servo_prps_log.orchestrate.json sidecar as a multi-realization campaign.

Realizations: top-of-band SNR is recovered by averaging more periods and more
*realizations* (distinct crest-min phase sets), never by raising amplitude past
the velocity ceiling. --realizations N emits N segments, each its own phase set.
"""

import argparse
import base64
import json
import math
import os
import sys

import numpy as np


# ----------------------------------------------------------------------
# Servo truth values (single source of truth)
# ----------------------------------------------------------------------
# Pull the zero-angle center and static gain from the firmware truth module
# (firmware/pico_micropython/lib/servo_static_map.py) so the laptop designer and
# the on-device player agree on one source. Falls back to the last-known
# literals if the module can't be imported.
_FIRMWARE_LIB = os.path.join(
    os.path.dirname(__file__), "..", "firmware", "pico_micropython", "lib"
)
sys.path.insert(0, os.path.abspath(_FIRMWARE_LIB))
try:
    from servo_static_map import NEUTRAL_US as TRUTH_CENTER_US
    from servo_static_map import DEG_PER_US as TRUTH_DEG_PER_US
except Exception:
    TRUTH_CENTER_US = 1431
    TRUTH_DEG_PER_US = 0.091092

# Repo-relative path the orchestrator uploads to /lib so the firmware import of
# the truth module succeeds on a fresh device.
TRUTH_MODULE_UPLOAD = {
    "local": "firmware/pico_micropython/lib/servo_static_map.py",
    "remote": "lib/servo_static_map.py",
}


# ----------------------------------------------------------------------
# Frequency grid
# ----------------------------------------------------------------------

def build_weighted_grid(f_min, f_max, f_c, n_below, n_above):
    """
    Log-spaced grid that is denser around and above the corner f_c.

    n_below lines populate [f_min, f_c); n_above lines populate [f_c, f_max].
    n_above is normally larger so the band around/above the corner is weighted.
    """
    below = np.exp(np.linspace(math.log(f_min), math.log(f_c), n_below, endpoint=False))
    above = np.exp(np.linspace(math.log(f_c), math.log(f_max), n_above))
    return np.concatenate([below, above])


def snap_to_bins(freqs_hz, period_s):
    """Snap to integer multiples of the fundamental (1/period); de-duplicate, sort."""
    fundamental = 1.0 / period_s
    bins = []
    seen = set()
    for f in np.sort(freqs_hz):
        if f <= 0:
            continue
        k = int(round(f / fundamental))
        if k < 1:
            k = 1
        if k in seen:
            continue
        seen.add(k)
        bins.append(k)
    bins = np.array(sorted(bins), dtype=int)
    return bins, bins * fundamental


# ----------------------------------------------------------------------
# Amplitude taper
# ----------------------------------------------------------------------

def taper_amplitudes(freqs_hz, f_c):
    """A_flat below f_c, 1/f above. Normalized so max == 1."""
    a = np.where(freqs_hz <= f_c, 1.0, f_c / freqs_hz)
    return a / a.max()


# ----------------------------------------------------------------------
# Multisine evaluation
# ----------------------------------------------------------------------

def eval_multisine(bins, amplitudes, phases, samples_per_period):
    """One period of the (un-normalized) taper-weighted multisine, sampled on the
    same integer grid the firmware uses."""
    n = np.arange(samples_per_period)
    y = np.zeros(samples_per_period)
    for k, a, phi in zip(bins, amplitudes, phases):
        y += a * np.sin(2.0 * math.pi * k * n / samples_per_period + phi)
    return y


def crest_factor(y):
    rms = math.sqrt(np.mean(y * y))
    if rms <= 0:
        return 1e9
    return float(np.max(np.abs(y)) / rms)


def peak_norm_diff(y):
    """max |y[n+1]-y[n]| / max|y|, wrap-around included (matches firmware guard)."""
    peak = float(np.max(np.abs(y)))
    if peak <= 0:
        return 0.0
    d = np.max(np.abs(np.diff(y, append=y[0])))
    return float(d / peak)


# ----------------------------------------------------------------------
# Phase design
# ----------------------------------------------------------------------

def schroeder_phases(amplitudes):
    """Power-weighted Schroeder phases (low crest seed)."""
    p = amplitudes ** 2
    p = p / p.sum()
    phases = np.zeros(len(amplitudes))
    for n in range(1, len(amplitudes)):
        phases[n] = phases[0] - 2.0 * math.pi * np.sum((n - np.arange(n)) * p[:n])
    return np.mod(phases, 2.0 * math.pi)


def minimize_crest(bins, amplitudes, samples_per_period, restarts, rng):
    """Seed with Schroeder, then random-restart search; keep the lowest crest."""
    best_phases = schroeder_phases(amplitudes)
    best_y = eval_multisine(bins, amplitudes, best_phases, samples_per_period)
    best_cf = crest_factor(best_y)

    for _ in range(restarts):
        phases = rng.uniform(0.0, 2.0 * math.pi, size=len(bins))
        y = eval_multisine(bins, amplitudes, phases, samples_per_period)
        cf = crest_factor(y)
        if cf < best_cf:
            best_cf, best_phases, best_y = cf, phases, y

    return best_phases, best_cf, best_y


# ----------------------------------------------------------------------
# Amplitude sizing against the velocity ceiling + encoder envelope
# ----------------------------------------------------------------------

def size_amplitude_us(y, dt_s, deg_per_us, vel_limit_deg_s, vel_fraction,
                      encoder_lossfree_deg, encoder_margin):
    """
    Largest command amplitude (us) s.t. realized peak velocity <= vel_fraction *
    ceiling AND command stays inside the encoder loss-free envelope.
    """
    pnd = peak_norm_diff(y)  # per-sample normalized slope
    # vel = amp_us * pnd / dt_s * deg_per_us  <= vel_fraction * vel_limit
    vel_amp = (vel_fraction * vel_limit_deg_s) / (pnd / dt_s * abs(deg_per_us))
    # |command_deg| <= margin * lossfree  => amp_us * 1.0 (normalized peak) <= ...
    enc_amp = (encoder_margin * encoder_lossfree_deg) / abs(deg_per_us)
    amp = min(vel_amp, enc_amp)
    binding = "velocity" if vel_amp <= enc_amp else "encoder-envelope"
    return amp, binding, vel_amp, enc_amp, pnd


# ----------------------------------------------------------------------
# Orchestrate sidecar emission
# ----------------------------------------------------------------------

def command_table_b64(y):
    """
    Base64'd little-endian float32 of one period of the NORMALIZED command
    waveform (y / max|y|), matching the firmware's peak normalization. The Pico
    plays this back by index with no trig, so the command tick stays
    deterministic. Decodes on-device into an array('f') of len(y) samples.
    """
    peak = float(np.max(np.abs(y)))
    if peak <= 0.0:
        peak = 1.0
    norm = (np.asarray(y, dtype=np.float64) / peak).astype("<f4")
    return base64.b64encode(norm.tobytes()).decode("ascii")


def build_segment(name, log_base, cfg_common, freqs, phases, amplitudes,
                  amplitude_us, center_us, table_b64, num_periods):
    config = dict(cfg_common)
    config.update({
        "LOG_FILE_BASE": log_base,
        "NUM_ACQUISITION_SETS": 1,
        "NUM_PERIODS_PER_RUN": int(num_periods),
        "USE_MANUAL_FREQUENCY_LIST": True,
        "MANUAL_EXCITED_FREQS_HZ": [round(float(f), 6) for f in freqs],
        "PRPS_PHASES": [round(float(p), 6) for p in phases],
        "PRPS_LINE_AMPLITUDES": [round(float(a), 6) for a in amplitudes],
        "PRPS_COMMAND_TABLE_B64": table_b64,
        "TEST_RUNS": [[name, int(center_us), int(round(amplitude_us))]],
    })
    return {"name": name, "script": "servo_prps_log.py", "config": config}


def split_periods(num_periods, max_per_segment):
    """
    Spread num_periods across the fewest sub-segments that each hold
    <= max_per_segment periods, as evenly as possible. Each sub-segment is its
    own Pico run (own baseline + CSV), so its file stays under the flash budget;
    they share the same phase set and pool in analysis (the prior a/b split).
    """
    if max_per_segment < 1:
        max_per_segment = 1
    if num_periods <= max_per_segment:
        return [num_periods]
    n_sub = -(-num_periods // max_per_segment)  # ceil
    base, rem = divmod(num_periods, n_sub)
    return [base + (1 if i < rem else 0) for i in range(n_sub)]


def max_periods_per_segment(samples_per_period, args):
    """
    Largest period count whose CSV fits the Pico flash budget. The firmware logs
    every prps sample plus a fixed baseline/settle/post block; row width is taken
    from a real prior file. Returns >= 1.
    """
    budget_rows = args.pico_flash_kb * 1024.0 / args.bytes_per_row
    prps_rows = budget_rows - args.baseline_rows
    return max(1, int(prps_rows // samples_per_period))


def design_band(label, name_prefix, f_min, f_max, f_corner, n_below, n_above,
                num_periods, args, samples_per_period, dt_s, seed_base):
    """Design one band (grid -> taper -> crest-min phases -> amplitude sizing)
    and return (segments, (f_lo, f_hi))."""
    grid = build_weighted_grid(f_min, f_max, f_corner, n_below, n_above)
    bins, freqs = snap_to_bins(grid, args.period_s)
    amplitudes = taper_amplitudes(freqs, f_corner)

    print()
    print("== {} band ==".format(label))
    print("  band                : {:.3f} .. {:.3f} Hz, corner {:.2f} Hz".format(
        freqs[0], freqs[-1], f_corner))
    print("  lines (kept)        :", len(bins))
    print("  num_periods         :", num_periods)
    top_spc = samples_per_period / bins[-1]
    print("  top-line samples/cyc:", "{:.1f}".format(top_spc))
    if top_spc < 8:
        print("  WARNING: top line < 8 samples/cycle; consider smaller --command-dt-ms")

    cfg_common = {
        "COOLDOWN_BETWEEN_SETS_MS": 0,
        "PRPS_PERIOD_S": args.period_s,
        "NUM_PERIODS_PER_RUN": num_periods,
        "COMMAND_UPDATE_DT_MS": args.command_dt_ms,
        "SERVO_FREQ_HZ": args.servo_freq_hz,
        "PEAK_VELOCITY_LIMIT_DEG_S": args.vel_limit,
        "SERVO_STATIC_DEG_PER_US": args.deg_per_us,
    }

    # Split each realization's periods across sub-segments so no single CSV
    # overruns Pico flash (the proven a/b pattern, now budget-driven).
    max_pp = max_periods_per_segment(samples_per_period, args)
    chunks = split_periods(num_periods, max_pp)
    est_kb = (max(chunks) * samples_per_period + args.baseline_rows) * args.bytes_per_row / 1024.0
    print("  flash budget        : {:.0f} KB/CSV -> <= {} periods/segment".format(
        args.pico_flash_kb, max_pp))
    if len(chunks) > 1:
        print("  period split        : {} periods -> {} segments {} (largest CSV ~{:.0f} KB)".format(
            num_periods, len(chunks), chunks, est_kb))
    else:
        print("  period split        : none ({} periods in 1 segment, CSV ~{:.0f} KB)".format(
            num_periods, est_kb))

    segments = []
    for r in range(args.realizations):
        seed = seed_base + r
        rng = np.random.default_rng(seed)
        phases, cf, y = minimize_crest(bins, amplitudes, samples_per_period,
                                       args.restarts, rng)
        amp_us, binding, vel_amp, enc_amp, pnd = size_amplitude_us(
            y, dt_s, args.deg_per_us, args.vel_limit, args.vel_fraction,
            args.encoder_lossfree_deg, args.encoder_margin)
        realized_vel = amp_us * pnd / dt_s * abs(args.deg_per_us)
        realized_deg = amp_us * abs(args.deg_per_us)

        print()
        print("  realization {} (seed {})".format(r, seed))
        print("    crest factor      : {:.3f}".format(cf))
        print("    amplitude_us      : {} (binding: {})".format(int(round(amp_us)), binding))
        print("    realized peak vel : {:.1f} deg/s (ceiling {:.0f})".format(
            realized_vel, args.vel_limit))
        print("    realized peak ang : +/-{:.1f} deg (loss-free +/-{:.0f})".format(
            realized_deg, args.encoder_lossfree_deg))

        # One precomputed table is shared by all sub-segments of this realization.
        table_b64 = command_table_b64(y)
        for sub_idx, sub_periods in enumerate(chunks):
            if len(chunks) > 1:
                name = "{}_r{:02d}{}".format(name_prefix, r, chr(ord("a") + sub_idx))
            else:
                name = "{}_r{:02d}".format(name_prefix, r)
            segments.append(build_segment(
                name, name, cfg_common, freqs, phases, amplitudes, amp_us,
                args.center_us, table_b64, sub_periods))

    return segments, (float(freqs[0]), float(freqs[-1]))


def linear_block_grid(f_min, lin_top, corner, period_s, n_lines):
    """
    Log-spaced line grid for one ladder rung's linear block, spanning
    [f_min, lin_top]. When the corner sits inside the band the grid is weighted
    around it (so the rolloff is measured, not extrapolated); otherwise (large
    rungs whose lin_top is already below the corner) it is plain log-spaced.
    """
    if corner < lin_top * 0.99:
        n_below = max(4, n_lines // 2)
        n_above = max(3, n_lines - n_below)
        grid = build_weighted_grid(f_min, lin_top, corner, n_below, n_above)
    else:
        grid = np.exp(np.linspace(math.log(f_min), math.log(lin_top), n_lines))
    return snap_to_bins(grid, period_s)


def single_tone_table_and_freq(mult, f_slew, fundamental, coh_ceiling, samples_per_period):
    """
    Snap mult*f_slew to a period bin, clip to the coherence ceiling, and build a
    single-tone (crest-free) command table for the slew probe. Returns
    (freq_hz, bin_k, table_b64, peak_norm_diff) or None if the tone would exceed
    the ceiling after snapping to a bin already used.
    """
    f_req = min(mult * f_slew, coh_ceiling)
    k = max(1, int(round(f_req / fundamental)))
    n = np.arange(samples_per_period)
    y = np.sin(2.0 * math.pi * k * n / samples_per_period)
    return k * fundamental, k, command_table_b64(y), peak_norm_diff(y)


def design_amplitude_ladder(args, samples_per_period, dt_s):
    """
    Fixed-amplitude ladder: one rung per angle amplitude. Each rung gets a
    velocity-bounded linear PRPS block (top frequency capped by the rung's own
    slew knee and the coherence ceiling) and, when the slew knee falls inside the
    trusted band, a set of single-tone slew probes bracketing the knee. Amplitude
    is FIXED per rung (the theta we characterize); frequency content is derived
    from the slew/corner/coherence limits -- the inverse of the size-amplitude
    bands above.
    """
    ladder_deg = [float(x) for x in args.ladder_deg.split(",") if x.strip()]
    probe_span = [float(x) for x in args.probe_span.split(",") if x.strip()]
    deg_per_us = abs(args.deg_per_us)
    fundamental = 1.0 / args.period_s
    coh = args.coh_ceiling_hz

    cfg_common = {
        "COOLDOWN_BETWEEN_SETS_MS": 0,
        "PRPS_PERIOD_S": args.period_s,
        "COMMAND_UPDATE_DT_MS": args.command_dt_ms,
        "SERVO_FREQ_HZ": args.servo_freq_hz,
        "PEAK_VELOCITY_LIMIT_DEG_S": args.safety_vel_limit,
        "SERVO_STATIC_DEG_PER_US": args.deg_per_us,
    }

    max_pp = max_periods_per_segment(samples_per_period, args)
    lin_chunks = split_periods(args.ladder_num_periods, max_pp)
    probe_chunks = split_periods(args.ladder_probe_periods, max_pp)

    print()
    print("== amplitude ladder ==")
    print("  slew_est {:.0f} deg/s | corner {:.1f} Hz | coh ceiling {:.1f} Hz".format(
        args.slew_est_deg_s, args.corner_est_hz, coh))
    print("  rungs (deg)         :", ", ".join("{:g}".format(d) for d in ladder_deg))
    print("  lin periods/split   : {} -> {} (probe {} -> {})".format(
        args.ladder_num_periods, lin_chunks, args.ladder_probe_periods, probe_chunks))
    print("  {:>5} {:>7} {:>8} {:>9} {:>7} {:>8}".format(
        "theta", "amp_us", "f_slew", "lin_top", "probed", "lin_vel"))

    segments = []
    theta_lo, theta_hi = min(ladder_deg), max(ladder_deg)

    for theta in ladder_deg:
        amp_us = theta / deg_per_us
        f_slew = args.slew_est_deg_s / (2.0 * math.pi * theta)
        lin_top = min(args.lin_top_frac * f_slew, coh)
        # Keep at least a usable band even for the biggest rungs.
        lin_top = max(lin_top, args.f_min * 2.5)

        tag = "a{:02d}".format(int(round(theta)))

        # --- linear block: fixed amplitude, several crest-min realizations ---
        bins, freqs = linear_block_grid(
            args.f_min, lin_top, args.corner_est_hz, args.period_s, args.ladder_lin_lines)
        amplitudes = taper_amplitudes(freqs, args.corner_est_hz)

        realized_vel_report = 0.0
        for r in range(args.ladder_realizations):
            rng = np.random.default_rng(args.base_seed + r + int(round(theta)) * 100)
            phases, cf, y = minimize_crest(bins, amplitudes, samples_per_period,
                                           args.restarts, rng)
            pnd = peak_norm_diff(y)
            realized_vel = amp_us * pnd / dt_s * deg_per_us
            realized_vel_report = max(realized_vel_report, realized_vel)
            if realized_vel > args.safety_vel_limit:
                print("  WARNING: {} lin realization {} command vel {:.0f} deg/s "
                      "exceeds safety {:.0f}".format(tag, r, realized_vel, args.safety_vel_limit))
            table_b64 = command_table_b64(y)
            for sub_idx, sub_periods in enumerate(lin_chunks):
                suffix = chr(ord("a") + sub_idx) if len(lin_chunks) > 1 else ""
                name = "ladder_{}_lin_r{:02d}{}".format(tag, r, suffix)
                segments.append(build_segment(
                    name, name, cfg_common, freqs, phases, amplitudes, amp_us,
                    args.center_us, table_b64, sub_periods))

        # --- slew probes: only where the knee sits inside the trusted band ---
        probed = f_slew <= args.probe_when_knee_below_hz
        if probed:
            used_bins = set()
            for mult in probe_span:
                f_hz, k, table_b64, pnd = single_tone_table_and_freq(
                    mult, f_slew, fundamental, coh, samples_per_period)
                if k in used_bins:
                    continue
                used_bins.add(k)
                probe_vel = amp_us * pnd / dt_s * deg_per_us
                if probe_vel > args.safety_vel_limit:
                    print("  WARNING: {} probe {:.2f}Hz command vel {:.0f} deg/s "
                          "exceeds safety {:.0f}".format(tag, f_hz, probe_vel, args.safety_vel_limit))
                for sub_idx, sub_periods in enumerate(probe_chunks):
                    suffix = chr(ord("a") + sub_idx) if len(probe_chunks) > 1 else ""
                    name = "ladder_{}_probe_f{}{}".format(
                        tag, ("{:.2f}".format(f_hz)).replace(".", "p"), suffix)
                    segments.append(build_segment(
                        name, name, cfg_common, [round(f_hz, 6)], [0.0], [1.0],
                        amp_us, args.center_us, table_b64, sub_periods))

        print("  {:>5.0f} {:>7.0f} {:>8.2f} {:>9.2f} {:>7} {:>7.0f}".format(
            theta, amp_us, f_slew, lin_top, "yes" if probed else "no", realized_vel_report))

    description = (
        "Servo amplitude-bandwidth ladder: {:g}-{:g} deg rungs. Each rung is a "
        "fixed-amplitude linear PRPS block (top freq = min({:.2f}*f_slew(theta), "
        "{:.1f} Hz coherence ceiling)) plus single-tone slew probes on rungs whose "
        "knee f_slew = slew_est/(2*pi*theta) <= {:.1f} Hz. slew_est {:.0f} deg/s, "
        "corner {:.1f} Hz (measured from the 2026-06-29 upgraded-servo step test). "
        "Maps f_vec(theta) = min(linear corner, slew knee) to invert the feasibility "
        "thrust trade.").format(
            theta_lo, theta_hi, args.lin_top_frac, coh,
            args.probe_when_knee_below_hz, args.slew_est_deg_s, args.corner_est_hz)

    return segments, description


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--f-min", type=float, default=0.3)
    ap.add_argument("--f-max", type=float, default=15.0)
    ap.add_argument("--f-corner", type=float, default=3.4)
    ap.add_argument("--n-below", type=int, default=8, help="lines below the corner")
    ap.add_argument("--n-above", type=int, default=18, help="lines at/above the corner")
    # Period is bounded by Pico RAM, not by the science: the firmware allocates a
    # single-period log buffer of 11*samples_per_period bytes (plus a 4*spp command
    # table). At 5 ms, 10 s -> 2000 samples -> ~22 KB buffer, the footprint the
    # firmware was designed around. Longer periods (e.g. 40 s -> 86 KB) MemoryError
    # on the RP2040. Recover low-frequency averaging with more periods, not a longer
    # one.
    ap.add_argument("--period-s", type=float, default=10.0)
    ap.add_argument("--command-dt-ms", type=float, default=5.0)
    # PWM frame rate the servo is driven at. The command tick (--command-dt-ms)
    # should match its period: standard analog servos are 50 Hz (20 ms), the
    # upgraded high-speed digital servo accepts 330 Hz (~3 ms). Emitted per
    # segment so the on-device PWM refresh matches the playback tick.
    ap.add_argument("--servo-freq-hz", type=int, default=330,
                    help="PWM frame rate (Hz); should pair with --command-dt-ms")
    ap.add_argument("--num-periods", type=int, default=8)
    ap.add_argument("--center-us", type=int, default=TRUTH_CENTER_US,
                    help="oscillation center (us); defaults to the truth zero-angle center")
    ap.add_argument("--deg-per-us", type=float, default=TRUTH_DEG_PER_US,
                    help="servo static gain magnitude (deg/us); defaults to the truth value")
    ap.add_argument("--vel-limit", type=float, default=120.0, help="deg/s ceiling")
    ap.add_argument("--vel-fraction", type=float, default=0.85,
                    help="target fraction of the ceiling to leave guard margin")
    ap.add_argument("--encoder-lossfree-deg", type=float, default=36.0)
    ap.add_argument("--encoder-margin", type=float, default=0.8)
    # Pico flash budget: each segment is its own CSV, written whole before it is
    # pulled+deleted. A realization with more periods than fit the budget is split
    # across sub-segments (a/b/c ...). 915 KB fit on the rail previously; default
    # 800 KB leaves margin. bytes-per-row/baseline-rows are measured from a real
    # log (111 B/row, ~1100 baseline+settle+post rows at the firmware defaults).
    ap.add_argument("--pico-flash-kb", type=float, default=800.0,
                    help="per-CSV flash budget (KB); triggers period splitting")
    ap.add_argument("--bytes-per-row", type=float, default=115.0,
                    help="measured CSV row width (bytes) for the size estimate")
    ap.add_argument("--baseline-rows", type=int, default=1100,
                    help="non-prps rows per file (baseline + settle + post)")
    ap.add_argument("--restarts", type=int, default=6000,
                    help="random phase restarts per realization for crest-min")
    ap.add_argument("--realizations", type=int, default=4,
                    help="distinct crest-min phase sets (for averaging SNR)")
    ap.add_argument("--base-seed", type=int, default=7001)
    ap.add_argument("--write-orchestrate", type=str, default=None,
                    help="path to servo_prps_log.orchestrate.json to overwrite")

    # Combined two-band campaign (large-signal low + small-signal high).
    ap.add_argument("--combined", action="store_true",
                    help="emit both a large-signal low band and a small-signal high band")
    ap.add_argument("--low-f-max", type=float, default=10.0,
                    help="upper edge of the large-signal low band")
    ap.add_argument("--low-f-corner", type=float, default=None,
                    help="taper corner for the low band; below it amplitude is flat, "
                         "above it rolls off 1/f. Default = low-f-max (flat across the "
                         "whole low band). Set < low-f-max to keep low frequencies "
                         "large-signal while tapering the extension up to low-f-max.")
    ap.add_argument("--low-n-below", type=int, default=18)
    ap.add_argument("--low-n-above", type=int, default=1)
    ap.add_argument("--low-num-periods", type=int, default=6)
    ap.add_argument("--seed-offset-high", type=int, default=1000,
                    help="seed offset so the high band gets distinct phase sets")

    # Amplitude-bandwidth ladder (fixed-amplitude rungs; frequency capped by slew).
    ap.add_argument("--amplitude-ladder", action="store_true",
                    help="emit a fixed-amplitude ladder mapping f_vec(theta)")
    ap.add_argument("--ladder-deg", type=str, default="5,7,9,11,13,15",
                    help="comma-separated angle-amplitude rungs (deg)")
    ap.add_argument("--ladder-realizations", type=int, default=3,
                    help="crest-min phase sets per linear block (averaging SNR)")
    ap.add_argument("--ladder-num-periods", type=int, default=3,
                    help="periods per linear-block realization (3 fits one segment at "
                         "the default 10 s period; averaging SNR comes from realizations)")
    ap.add_argument("--ladder-probe-periods", type=int, default=3,
                    help="periods per single-tone slew probe")
    ap.add_argument("--ladder-lin-lines", type=int, default=14,
                    help="excited lines in each rung's linear block")
    ap.add_argument("--slew-est-deg-s", type=float, default=900.0,
                    help="measured saturation slew (deg/s); centers the slew knee")
    ap.add_argument("--corner-est-hz", type=float, default=9.0,
                    help="measured effective linear corner (Hz)")
    ap.add_argument("--coh-ceiling-hz", type=float, default=15.0,
                    help="hard top-frequency cap (no line excited above this)")
    ap.add_argument("--lin-top-frac", type=float, default=0.9,
                    help="linear-block top = this * f_slew(theta), then min'd with coh ceiling")
    ap.add_argument("--probe-span", type=str, default="0.6,0.8,1.0,1.2,1.4",
                    help="probe tone frequencies as multiples of f_slew(theta)")
    ap.add_argument("--probe-when-knee-below-hz", type=float, default=15.0,
                    help="only probe rungs whose slew knee <= this (inside the trusted band)")
    ap.add_argument("--safety-vel-limit", type=float, default=1400.0,
                    help="per-segment PEAK_VELOCITY_LIMIT_DEG_S abort backstop (deg/s)")

    args = ap.parse_args()

    dt_s = args.command_dt_ms / 1000.0
    samples_per_period = int(round(args.period_s / dt_s))
    fundamental = 1.0 / args.period_s

    # The firmware buffers one full period of 11-byte log records in a single
    # contiguous bytearray; on the RP2040 anything past ~24 KB tends to MemoryError
    # once the heap is fragmented by the config + command table. Warn loudly here so
    # a too-long period is caught on the laptop, not on the rail.
    log_buffer_kb = 11.0 * samples_per_period / 1024.0
    pico_buffer_ceiling_kb = 24.0

    print("PRPS excitation design")
    print("  period_s            :", args.period_s, "(fundamental {:.4f} Hz)".format(fundamental))
    print("  command_dt_ms       :", args.command_dt_ms,
          "(update {:.1f} Hz, {} samples/period)".format(1000.0 / args.command_dt_ms, samples_per_period))
    print("  pico log buffer     : {:.1f} KB / period".format(log_buffer_kb))
    if log_buffer_kb > pico_buffer_ceiling_kb:
        print("  WARNING: {:.1f} KB single-period log buffer exceeds the ~{:.0f} KB the "
              "RP2040 can reliably allocate.".format(log_buffer_kb, pico_buffer_ceiling_kb))
        print("           Shorten --period-s (more --num-periods recovers averaging) "
              "to avoid a MemoryError on the Pico.")
    if not args.amplitude_ladder:
        print("  velocity ceiling    : {:.0f} deg/s ({:.0f}% guard-margin target)".format(
            args.vel_limit, args.vel_fraction * 100))

    if args.amplitude_ladder:
        segments, description = design_amplitude_ladder(args, samples_per_period, dt_s)
    elif args.combined:
        # Low band: flat amplitude (corner = upper edge, so taper is all-flat),
        # large signal, anchors the nominal fit. High band: tapered, crosses the
        # corner, small signal, measures rolloff.
        low_corner = args.low_f_corner if args.low_f_corner is not None else args.low_f_max
        # n_above only matters when the corner is below the band edge (i.e. there
        # is a tapered region to populate); keep at least a few lines up there.
        low_n_above = args.low_n_above
        if low_corner < args.low_f_max and low_n_above < args.n_above:
            low_n_above = args.n_above
        low_segments, low_band = design_band(
            "large-signal low", "lowband_largesig",
            args.f_min, args.low_f_max, low_corner,
            args.low_n_below, low_n_above, args.low_num_periods,
            args, samples_per_period, dt_s, args.base_seed)
        high_segments, high_band = design_band(
            "small-signal high", "highband_smallsig",
            args.f_min, args.f_max, args.f_corner,
            args.n_below, args.n_above, args.num_periods,
            args, samples_per_period, dt_s, args.base_seed + args.seed_offset_high)
        segments = low_segments + high_segments
        description = (
            "Combined re-run servo PRPS campaign. Large-signal low band "
            "{:.2f}-{:.2f} Hz (clears the +/-2 deg backlash; anchors the nominal "
            "K/tau/L fit) + small-signal high band {:.2f}-{:.2f} Hz (crosses the "
            "{:.1f} Hz corner to measure rolloff/phase). Both use Schroeder+crest-min "
            "phases under a {:.0f} deg/s ceiling, {} realizations each. Analysis fits "
            "the low band, then tests the high-band FRF against that fit's CI tube: "
            "merge if consistent, else route the high band to high-frequency "
            "uncertainty.").format(
                low_band[0], low_band[1], high_band[0], high_band[1],
                args.f_corner, args.vel_limit, args.realizations)
    else:
        segments, band = design_band(
            "single", "rerun_servo_prps",
            args.f_min, args.f_max, args.f_corner,
            args.n_below, args.n_above, args.num_periods,
            args, samples_per_period, dt_s, args.base_seed)
        description = (
            "Re-run servo PRPS campaign: weighted {:.2f}-{:.2f} Hz grid (corner "
            "{:.1f} Hz), Schroeder+crest-min phases under a {:.0f} deg/s velocity "
            "ceiling, A_flat/1f taper, {} realizations x {} periods.").format(
                band[0], band[1], args.f_corner, args.vel_limit,
                args.realizations, args.num_periods)

    if args.write_orchestrate:
        doc = {
            "orchestration_version": 1,
            "description": description,
            "remote_config_path": "run_config.json",
            "write_manifest": False,
            "uploads": [TRUTH_MODULE_UPLOAD],
            "segments": segments,
        }
        with open(args.write_orchestrate, "w") as f:
            json.dump(doc, f, indent=2)
            f.write("\n")
        print()
        print("Total segments:", len(segments))
        print("Wrote orchestrate sidecar:", os.path.abspath(args.write_orchestrate))
    else:
        print()
        print("Total segments:", len(segments))
        print("(dry run; pass --write-orchestrate <path> to emit the sidecar)")


if __name__ == "__main__":
    main()
