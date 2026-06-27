# encoder_home.py
#
# Powered encoder homing utility — call this once at the start of any test to
# establish an absolute position datum and infer the rail length.
#
# WHY THIS EXISTS:
#   The quadrature encoder is INCREMENTAL: its count is relative to wherever it
#   powered on. Every run therefore needs a homing pass to (a) set an absolute
#   zero at a known physical reference (an end stop) and (b) learn where the
#   opposite stop sits in count-space. From that count span and the trusted
#   spec-derived scale we infer the usable rail length.
#
# WHAT IT DOES NOT DO:
#   - It does NOT re-measure counts/mm. The scale is treated as a fixed,
#     spec-derived constant (SCALE_COUNTS_PER_MM below). Rail length is INFERRED
#     from it, not the other way around.
#   - It does NOT depend on any identified friction model. The breakaway ESC
#     command is DISCOVERED each leg by ramping throttle up from gentle until
#     motion is detected. This keeps homing valid regardless of friction state.
#
# CONVENTION:
#   "home" = the end stop reached by driving at angle HOME_ANGLE_DEG. The encoder
#   is zeroed there, so after homing position counts run 0 .. +span toward the
#   far stop. Flip the sign of HOME_ANGLE_DEG if your geometry homes the other
#   way.
#
# USAGE (from any test, after hardware is set up):
#
#       import encoder_home
#       # servo: machine.PWM ; escs: list[machine.PWM] ; encoder already init'd
#       datum = encoder_home.home(servo, escs)
#       if not datum["ok"]:
#           raise RuntimeError("Homing failed: " + datum["status"])
#       # datum["rail_length_mm"], datum["span_counts"], datum["count_max"] ...
#       # Cart is parked at home (count == 0).
#
#   Override any default by passing a dict:
#       datum = encoder_home.home(servo, escs, {"NUM_ROUND_TRIPS": 3})
#
# Hardware (defaults match the rest of the firmware):
#   Servo PWM: GP15   ESC PWM: GP13, GP14   Encoder: GP18/GP19 (caller inits)

import time
import encoder
import servo_static_map


# ============================================================
# Defaults  (override via the `overrides` arg to home())
# ============================================================

DEFAULTS = {
    # ---- Scale (spec-derived, trusted; rail length is inferred from this) ----
    "SCALE_COUNTS_PER_MM": 64.810,   # experiments/encoder_calibration/results.md

    # ---- Servo mapping (single source: lib/servo_static_map.py) ----
    "SERVO_NEUTRAL_US": servo_static_map.NEUTRAL_US,
    "SERVO_DEG_PER_US": servo_static_map.DEG_PER_US,   # |static gain|, deg/us
    "SERVO_HARD_MIN_US": 1000,
    "SERVO_HARD_MAX_US": 2000,

    # ---- ESC limits ----
    "PWM_SAFE_US": 1000,

    # ---- Homing motion ----
    # Small angle, per design: keep the vectoring deflection gentle.
    # HOME_ANGLE_DEG drives toward the HOME stop; -HOME_ANGLE_DEG drives to far.
    "HOME_ANGLE_DEG": -20.0,
    "NUM_ROUND_TRIPS": 2,            # span samples = 2 * NUM_ROUND_TRIPS
    "SERVO_SETTLE_MS": 700,          # minimum dwell after the servo flip

    # ---- Rest settle (before motor on / breakaway test) ----
    # The servo flip itself nudges the cart; the breakaway test must start from
    # true rest, not from leftover flip motion. After SERVO_SETTLE_MS we wait
    # until the encoder is quiet, then capture the resting start count.
    "REST_WINDOW_MS": 400,           # quiet window that confirms "at rest"
    "REST_THRESHOLD_COUNTS": 10,     # max-min within window below this == at rest
    "REST_MAX_WAIT_MS": 4000,        # cap; proceed anyway if never fully settles

    # ---- Throttle ramp-to-motion (friction-agnostic breakaway search) ----
    "HOMING_ESC_START_US": 1150,     # gentle starting command
    "HOMING_ESC_STEP_US": 25,        # increment per dwell while no motion
    "HOMING_ESC_MAX_US": 1800,       # cap — keep homing gentle
    "RAMP_DWELL_MS": 400,            # time at each command level to detect motion
    "MOVE_DETECT_COUNTS": 50,        # ~0.77 mm: "we are moving"

    # ---- Halt (endstop) detection ----
    "COMMAND_DT_MS": 20,             # 50 Hz control/sample loop
    "HALT_DURATION_S": 1.2,          # quiet-count window to declare a stop
    "HALT_THRESHOLD_COUNTS": 15,     # max-min within window below this == quiet
    "LEG_TIMEOUT_S": 25,             # hard per-leg time limit

    # ---- Acceptance ----
    # A traverse's span must be within this fraction of the running mean to be
    # trusted; otherwise repeatability is flagged.
    "REPEATABILITY_WARN_PCT": 2.0,

    "VERBOSE": True,
}


# ============================================================
# Small helpers (self-contained so the module needs no host script)
# ============================================================

def _clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _log(cfg, *args):
    if cfg["VERBOSE"]:
        print(*args)


def _angle_to_servo_us(cfg, angle_deg):
    us = cfg["SERVO_NEUTRAL_US"] - angle_deg / cfg["SERVO_DEG_PER_US"]
    us = int(us + 0.5)
    return int(_clamp(us, cfg["SERVO_HARD_MIN_US"], cfg["SERVO_HARD_MAX_US"]))


def _write_servo_us(cfg, servo, pulse_us):
    pulse_us = int(_clamp(pulse_us, cfg["SERVO_HARD_MIN_US"], cfg["SERVO_HARD_MAX_US"]))
    servo.duty_ns(pulse_us * 1000)


def _pwm_us_to_duty_u16(pwm_us):
    return int(int(pwm_us) * 65535 // 20000)


def _set_esc_us(cfg, escs, pwm_us):
    pwm_us = _clamp(int(pwm_us), cfg["PWM_SAFE_US"], 2000)
    duty = _pwm_us_to_duty_u16(pwm_us)
    for esc in escs:
        esc.duty_u16(duty)


def _sleep_to_next_sample(cfg, loop_start_ms):
    elapsed = time.ticks_diff(time.ticks_ms(), loop_start_ms)
    remaining = cfg["COMMAND_DT_MS"] - elapsed
    if remaining > 0:
        time.sleep_ms(remaining)


def _wait_until_at_rest(cfg):
    """
    Block until the encoder count has been quiet for REST_WINDOW_MS, then return
    the resting count. Caps out at REST_MAX_WAIT_MS so a never-quite-still rig
    cannot hang the run. Assumes the motor is already OFF (ESC at safe).
    """
    dt = cfg["COMMAND_DT_MS"]
    window = max(2, int(cfg["REST_WINDOW_MS"] / dt))
    buf = [encoder.get_count()] * window
    idx = 0
    start_ms = time.ticks_ms()

    while True:
        time.sleep_ms(dt)
        count = encoder.get_count()
        buf[idx] = count
        idx = (idx + 1) % window

        if max(buf) - min(buf) < cfg["REST_THRESHOLD_COUNTS"]:
            return count
        if time.ticks_diff(time.ticks_ms(), start_ms) > cfg["REST_MAX_WAIT_MS"]:
            _log(cfg, "    WARNING: cart never fully settled; proceeding.")
            return count


# ============================================================
# Single leg: drive toward one end stop until halt
# ============================================================

def _drive_to_endstop(cfg, servo, escs, angle_deg, leg_name):
    """
    Drive toward one end stop. The servo is held at the (max) target angle for
    the whole leg; the throttle is ramped up to HOMING_ESC_MAX_US in steps. Halt
    detection and the leg timeout are DELIBERATELY disabled during the ramp and
    only engage once the throttle is at max — so a stick-slip pause at a low
    breakaway thrust can't be misread as the end stop mid-rail.

    Returns (final_count, status, breakaway_us) where status is one of:
        "endstop"   — at max thrust, count went quiet AND we had displaced from
                      the leg start: end stop reached.
        "no_motion" — at max thrust, count quiet with ~no displacement (already
                      at this stop, or could not break away even at max).
        "timeout"   — ran out of time after reaching max thrust.

    breakaway_us is the ESC command (us) at which the cart first displaced more
    than MOVE_DETECT_COUNTS from the leg start, or None if it never moved. The
    static thrust map turns this into a breakaway force for friction ID.
    """
    dt_ms = cfg["COMMAND_DT_MS"]
    servo_us = _angle_to_servo_us(cfg, angle_deg)

    _log(cfg, "  [{}] servo {:+.1f} deg ({} us); ramping throttle to max...".format(
        leg_name, angle_deg, servo_us))

    # Servo to (max) angle and let it settle BEFORE any thrust — so "max angle"
    # holds for the entire driven portion of the leg. The servo flip itself
    # nudges the cart, so after the fixed dwell we wait for it to come to rest
    # and only then capture start_count and turn the motor on — otherwise
    # leftover flip motion would be misread as thrust breakaway.
    _set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
    _write_servo_us(cfg, servo, servo_us)
    time.sleep_ms(cfg["SERVO_SETTLE_MS"])
    start_count = _wait_until_at_rest(cfg)
    _log(cfg, "    at rest after flip (count {}); motor on.".format(start_count))

    esc_us = cfg["HOMING_ESC_START_US"]
    _set_esc_us(cfg, escs, esc_us)

    halt_window = max(1, int(cfg["HALT_DURATION_S"] * 1000 / dt_ms))
    halt_buf = [start_count] * halt_window
    halt_idx = 0
    samples = 0
    leg_timeout_ms = int(cfg["LEG_TIMEOUT_S"] * 1000)

    at_max = esc_us >= cfg["HOMING_ESC_MAX_US"]
    ramp_ref_ms = time.ticks_ms()
    max_start_ms = time.ticks_ms()   # timeout reference; reset when max reached
    breakaway_us = None              # ESC command at first detected motion
    if at_max:
        _log(cfg, "    at max thrust ({} us); halt/timeout active.".format(esc_us))

    status = "timeout"

    while True:
        loop_start = time.ticks_ms()
        count = encoder.get_count()

        # Record the command at which motion first breaks away (any phase).
        if breakaway_us is None and abs(count - start_count) >= cfg["MOVE_DETECT_COUNTS"]:
            breakaway_us = esc_us
            _log(cfg, "    breakaway at {} us".format(esc_us))

        if not at_max:
            # Ramp phase: step throttle toward max. No halt detection, no
            # timeout — the cart may legitimately stall-and-go here.
            if time.ticks_diff(loop_start, ramp_ref_ms) >= cfg["RAMP_DWELL_MS"]:
                esc_us = min(esc_us + cfg["HOMING_ESC_STEP_US"],
                             cfg["HOMING_ESC_MAX_US"])
                _set_esc_us(cfg, escs, esc_us)
                ramp_ref_ms = loop_start
                if esc_us >= cfg["HOMING_ESC_MAX_US"]:
                    # Entering max-thrust phase: arm timeout and halt window from
                    # the current position so they only reflect behavior at max.
                    at_max = True
                    max_start_ms = loop_start
                    halt_buf = [count] * halt_window
                    halt_idx = 0
                    samples = 0
                    _log(cfg, "    at max thrust ({} us); halt/timeout active.".format(
                        esc_us))
        else:
            # Max-thrust phase: timeout + halt detection active.
            if time.ticks_diff(loop_start, max_start_ms) > leg_timeout_ms:
                status = "timeout"
                break

            halt_buf[halt_idx] = count
            halt_idx = (halt_idx + 1) % halt_window
            samples += 1
            if samples >= halt_window:
                if max(halt_buf) - min(halt_buf) < cfg["HALT_THRESHOLD_COUNTS"]:
                    # Quiet at max thrust. Did we actually travel this leg?
                    if abs(count - start_count) < cfg["MOVE_DETECT_COUNTS"]:
                        status = "no_motion"
                    else:
                        status = "endstop"
                    break

        _sleep_to_next_sample(cfg, loop_start)

    # Always cut throttle at the end of a leg.
    _set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
    final_count = encoder.get_count()
    _log(cfg, "  [{}] {} at count {} (breakaway {} us)".format(
        leg_name, status, final_count, breakaway_us))
    return final_count, status, breakaway_us


# ============================================================
# Public API
# ============================================================

def home(servo, escs, overrides=None):
    """
    Run the powered homing sequence and return a datum dict.

    Assumes the caller has already initialized the servo (machine.PWM), the ESC
    list (list of machine.PWM), and the encoder (encoder.init_configured(...)).

    Returns a dict:
        {
          "ok":               bool,
          "status":           str,            # "ok" or a failure reason
          "rail_length_mm":   float,
          "span_counts":      float,          # mean over all traverses
          "count_max":        int,            # far-stop count (home == 0)
          "scale_counts_per_mm": float,
          "spans_counts":     [int, ...],     # per-traverse samples
          "repeatability_pct": float,
          "legs":             [ {leg dict}, ... ],
        }

    Each leg dict (one driven traverse):
        { "index", "name", "direction" ("home"/"far"), "angle_deg", "status",
          "breakaway_us" (or None), "start_count", "end_count", "span_counts"
          (None for the initial zeroing leg) }

    On return the cart is parked at the home stop and the encoder reads ~0.
    """
    cfg = dict(DEFAULTS)
    if overrides:
        cfg.update(overrides)

    home_angle = cfg["HOME_ANGLE_DEG"]
    far_angle = -home_angle
    legs = []

    _log(cfg, "=" * 60)
    _log(cfg, "ENCODER HOMING")
    _log(cfg, "  home angle : {:+.1f} deg   round trips : {}".format(
        home_angle, cfg["NUM_ROUND_TRIPS"]))
    _log(cfg, "  scale      : {:.3f} counts/mm (spec, fixed)".format(
        cfg["SCALE_COUNTS_PER_MM"]))
    _log(cfg, "=" * 60)

    result = {
        "ok": False,
        "status": "",
        "rail_length_mm": 0.0,
        "span_counts": 0.0,
        "count_max": 0,
        "scale_counts_per_mm": cfg["SCALE_COUNTS_PER_MM"],
        "ramp_step_us": cfg["HOMING_ESC_STEP_US"],
        "spans_counts": [],
        "repeatability_pct": 0.0,
        "legs": legs,
    }

    # ---- Leg 0: reach the home stop and zero there. ----
    # If the cart is already at the home stop it won't move; that's acceptable
    # for this first leg (it is, after all, at a stop). If it does move, its
    # breakaway is a valid home-direction friction sample.
    start0 = encoder.get_count()
    end0, status, brk0 = _drive_to_endstop(cfg, servo, escs, home_angle, "to_home")
    legs.append({
        "index": 0, "name": "to_home", "direction": "home",
        "angle_deg": home_angle, "status": status, "breakaway_us": brk0,
        "start_count": start0, "end_count": end0, "span_counts": None,
    })
    if status == "no_motion":
        _log(cfg, "  (no motion toward home — assuming already at home stop)")
    elif status != "endstop":
        result["status"] = "home leg failed: " + status
        _safe_park(cfg, servo, escs)
        return result

    encoder.zero()
    time.sleep_ms(100)
    home_count = encoder.get_count()
    _log(cfg, "  Zeroed at home. count =", home_count)

    # ---- Round trips: each leg after zeroing yields one span sample. ----
    spans = []
    far_count = 0
    leg_index = 1
    for trip in range(cfg["NUM_ROUND_TRIPS"]):
        # home -> far
        start_count = encoder.get_count()
        far_count, status, brk = _drive_to_endstop(
            cfg, servo, escs, far_angle, "to_far[{}]".format(trip + 1))
        span = abs(far_count - home_count)
        legs.append({
            "index": leg_index, "name": "to_far[{}]".format(trip + 1),
            "direction": "far", "angle_deg": far_angle, "status": status,
            "breakaway_us": brk, "start_count": start_count,
            "end_count": far_count, "span_counts": span,
        })
        leg_index += 1
        if status != "endstop":
            result["status"] = "far leg failed (trip {}): {}".format(trip + 1, status)
            _safe_park(cfg, servo, escs)
            return result
        spans.append(span)

        # far -> home
        start_count = encoder.get_count()
        back_count, status, brk = _drive_to_endstop(
            cfg, servo, escs, home_angle, "to_home[{}]".format(trip + 1))
        span = abs(far_count - back_count)
        legs.append({
            "index": leg_index, "name": "to_home[{}]".format(trip + 1),
            "direction": "home", "angle_deg": home_angle, "status": status,
            "breakaway_us": brk, "start_count": start_count,
            "end_count": back_count, "span_counts": span,
        })
        leg_index += 1
        if status != "endstop":
            result["status"] = "return leg failed (trip {}): {}".format(trip + 1, status)
            _safe_park(cfg, servo, escs)
            return result
        spans.append(span)
        home_count = back_count   # update zero reference to absorb any drift

    _safe_park(cfg, servo, escs)

    # ---- Reduce ----
    mean_span = sum(spans) / len(spans)
    rail_mm = mean_span / cfg["SCALE_COUNTS_PER_MM"]
    spread = max(spans) - min(spans)
    repeat_pct = 100.0 * spread / mean_span if mean_span > 0 else 0.0

    result.update({
        "ok": True,
        "status": "ok",
        "rail_length_mm": rail_mm,
        "span_counts": mean_span,
        "count_max": int(round(mean_span)),
        "spans_counts": spans,
        "repeatability_pct": repeat_pct,
    })

    _log(cfg, "-" * 60)
    _log(cfg, "HOMING RESULT")
    _log(cfg, "  per-traverse spans (counts):", spans)
    _log(cfg, "  mean span     : {:.1f} counts".format(mean_span))
    _log(cfg, "  rail length   : {:.2f} mm".format(rail_mm))
    _log(cfg, "  repeatability : {:.2f}% (warn > {:.1f}%)".format(
        repeat_pct, cfg["REPEATABILITY_WARN_PCT"]))
    if repeat_pct > cfg["REPEATABILITY_WARN_PCT"]:
        _log(cfg, "  WARNING: span repeatability exceeds threshold.")
    _log(cfg, "  Cart parked at home (count ~ 0).")
    _log(cfg, "=" * 60)

    return result


def _safe_park(cfg, servo, escs):
    _set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
    time.sleep_ms(300)
    _write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])
    time.sleep_ms(300)
