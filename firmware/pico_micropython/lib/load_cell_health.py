# load_cell_health.py
# Weightless per-run HX711 health probe for the thrust-measurement workflow.
#
# Motivation:
#   The known-weight calibration run before each thrust dataset checks absolute
#   scale/linearity, but it does NOT quantify how noisy, drifty, or flaky the
#   sensor was on that particular day. An intermittent HX711 connection that
#   returns days later shows up as read dropouts, bit-corruption outliers, or an
#   elevated zero-load noise floor -- all visible with NO known weights, just a
#   quiet zero-load hold. This probe logs that hold to a small sidecar CSV so
#   every thrust dataset carries an instrument-health fingerprint that analysis
#   can gate on (reject runs with excessive noise / drift / dropouts).
#
# It is deliberately weightless and fully automated: motors off, no load, no
# human step. Run it right after tare/zero, while the rig is already idle.
#
# Output: a sidecar CSV (one row per read ATTEMPT, so dropouts are visible)
#   <LOG_FILE_BASE>_load_cell_health.csv
# The firmware prints a quick on-bench summary; the authoritative stats and the
# threshold decision are recomputed in MATLAB (analysis/utils/loadCellHealthGate.m)
# from the raw rows. The "Saved:" line lets run_pico_and_pull.py pull the
# sidecar alongside the dataset it vouches for.

from utime import sleep_ms, ticks_ms, ticks_diff

HEALTH_CSV_HEADER = "t_ms,sample_idx,ok,raw_count\n"


def _cfg_get(cfg, key, default):
    # MicroPython dicts support .get, but treat an explicit None as "use default"
    # so an orchestrator override of null falls back cleanly.
    try:
        val = cfg[key]
    except KeyError:
        return default
    return default if val is None else val


def probe_zero_load_health(hx, cfg):
    """Log a weightless zero-load hold to a sidecar CSV and print a bench summary.

    Preconditions: motors OFF, no load on the cell, no current flowing (the same
    idle state as the initial tare). The probe does not need or use the tare
    offset -- it logs raw counts, and noise/drift are offset-invariant.

    Config keys (all optional, with defaults):
      LOG_FILE_BASE            base name for the sidecar (set by the caller)
      HEALTH_PROBE_SECONDS     probe duration          (default 20)
      HEALTH_SAMPLE_DELAY_MS   pacing between reads     (default 90)
      HEALTH_SETTLE_DELAY_MS   settle before probing    (default 1000)

    Returns a summary dict and prints "Saved: <name>.csv" so the orchestrator
    pulls the sidecar. Never raises on a read failure -- a dropout is data.
    """
    base = _cfg_get(cfg, "LOG_FILE_BASE", "load_cell")
    probe_s = _cfg_get(cfg, "HEALTH_PROBE_SECONDS", 20)
    delay_ms = _cfg_get(cfg, "HEALTH_SAMPLE_DELAY_MS", 90)
    settle_ms = _cfg_get(cfg, "HEALTH_SETTLE_DELAY_MS", 1000)

    filename = "{}_load_cell_health.csv".format(base)

    print()
    print("Load-cell health probe (weightless zero-load hold)")
    print("  duration (s):", probe_s)
    print("  sidecar:", filename)
    print("Keep motors OFF and the cell unloaded; settling...")
    sleep_ms(settle_ms)

    # Running accumulators for the on-bench summary. The authoritative numbers
    # are recomputed in MATLAB from the raw rows; these just give bench feedback.
    n = 0            # good reads
    dropouts = 0
    s_y = 0.0        # sum(raw)
    s_yy = 0.0       # sum(raw^2)
    s_t = 0.0        # sum(t_s)
    s_tt = 0.0       # sum(t_s^2)
    s_ty = 0.0       # sum(t_s * raw)
    r_min = None
    r_max = None

    f = open(filename, "w")
    try:
        f.write(HEALTH_CSV_HEADER)

        start = ticks_ms()
        sample_idx = 0
        while ticks_diff(ticks_ms(), start) < probe_s * 1000:
            t_ms = ticks_diff(ticks_ms(), start)
            sample_idx += 1
            try:
                raw = hx.read()
                ok = 1
            except OSError:
                raw = 0
                ok = 0

            f.write("{},{},{},{}\n".format(t_ms, sample_idx, ok, raw))
            if sample_idx % 25 == 0:
                f.flush()

            if ok:
                t_s = t_ms / 1000.0
                n += 1
                s_y += raw
                s_yy += raw * raw
                s_t += t_s
                s_tt += t_s * t_s
                s_ty += t_s * raw
                if r_min is None or raw < r_min:
                    r_min = raw
                if r_max is None or raw > r_max:
                    r_max = raw
            else:
                dropouts += 1

            if delay_ms > 0:
                sleep_ms(delay_ms)

        f.flush()
    finally:
        f.close()

    attempts = n + dropouts
    mean = s_y / n if n else 0.0
    var = (s_yy / n - mean * mean) if n else 0.0
    if var < 0.0:
        var = 0.0            # guard tiny negative from float round-off
    std = var ** 0.5
    ptp = (r_max - r_min) if (r_min is not None) else 0.0

    # Least-squares slope of raw vs time (counts/s).
    drift_cps = 0.0
    if n >= 2:
        denom = n * s_tt - s_t * s_t
        if denom != 0.0:
            drift_cps = (n * s_ty - s_t * s_y) / denom

    summary = {
        "attempts": attempts,
        "good": n,
        "dropouts": dropouts,
        "mean": mean,
        "std": std,
        "ptp": ptp,
        "drift_cps": drift_cps,
        "seconds": probe_s,
    }

    print("Health probe summary:")
    print("  reads ok / attempts:", n, "/", attempts, "(dropouts:", dropouts, ")")
    print("  zero-load mean (count):", mean)
    print("  noise std (count):", std)
    print("  peak-to-peak (count):", ptp)
    print("  drift (count/s):", drift_cps)
    print("Saved:", filename)

    return summary
