# lqr_tier1.py
#
# Fixed-gain LQR closed-loop controller for the thrust-vector rail vehicle.
# Tier 1: single linearization at theta*=0, T*=T_r_nom (2.574 N, 1700 µs).
#
# States:  x = [p (m), v (m/s), theta (rad), T (N)]
# Inputs:  u = [theta_cmd (rad), T_cmd (N)]
# Trim:    x* = [0, 0, 0, 2.574],  u* = [0, 2.574]
# Law:     u = u* - K_fixed @ (x - x*)
#
# State estimation:
#   p     : encoder count delta from reference, scaled by COUNTS_PER_M
#   v     : windowed finite-difference velocity with exponential smoothing
#   theta : Euler integration of first-order servo model on prev theta_cmd
#   T     : Euler integration of first-order thrust model on prev T_cmd
#
# Actuator inversion:
#   theta_cmd (rad) → servo_us  via servo calibration (neutral = 1350 µs)
#   T_cmd (N)       → esc_us    via Newton's method on the degree-4 polynomial
#
# Run phases:
#   arm       : ESC at safe (1000 µs), servo at neutral — 3 s
#   spool     : ESC at nominal (1700 µs), servo at neutral — motor settles to T_nom
#   user_wait : hold spool; operator places cart at desired reference position
#   control   : LQR active; data logged to CSV
#
# Hardware:
#   Servo PWM : GP15
#   ESC PWM   : GP13, GP14
#   Encoder A : GP18,  Encoder B : GP19
#
# Encoder calibration : 64,810.4 counts/m  (experiments/encoder_calibration/results.md)
# Servo calibration   : neutral=1350 µs, gain=0.091092 deg/µs
#                       (experiments/servo_identification/results.md)
# Thrust polynomial   : degree-4, mu=(1525.0 µs, 256.2 µs)
#                       (experiments/thrust_identification/results.md)
# LQR gains           : analysis/control_design/gain_scheduling/gain_schedule_tier1.mat
#                       (trim_analysis.m TIER=1, du_theta_max=60° servo hardware limit;
#                        dominant poles ≈ −6.3±1.8i rad/s, bandwidth ≈ 1 Hz)

import time
import math
from machine import Pin, PWM
import encoder


# ==============================================================================
# LQR parameters  (gain_schedule_tier1.mat, TIER=1)
# ==============================================================================

# K_fixed: 2×4  [theta_cmd row; T_cmd row]  cols = [p, v, theta, T]
K_FIXED = (
    (10.472, 3.268, 0.704, 0.0),
    (0.0,    0.0,   0.0,   0.414),
)
X_STAR = (0.0, 0.0, 0.0, 2.574)   # [p*, v*, theta*, T*]
U_STAR = (0.0, 2.574)             # [theta_cmd*, T_cmd*]


# ==============================================================================
# Thrust polynomial  (degree-4, fit range 1100–1950 µs)
# ==============================================================================
# F_ss(u_hat) = p0*u_hat^4 + p1*u_hat^3 + p2*u_hat^2 + p3*u_hat + p4
# u_hat = (esc_us - POLY_MU) / POLY_SIGMA

THRUST_POLY = (1.828e-5, 0.053686, 0.17600, 1.06699, 1.74620)
POLY_MU     = 1525.0   # µs
POLY_SIGMA  = 256.2    # µs


# ==============================================================================
# Configuration
# ==============================================================================


def make_config():
    return {
        # ---- file / logging ----
        "LOG_FILE_BASE":         "lqr_tier1",
        "PRINT_EVERY_N_SAMPLES": 10,

        # ---- servo ----
        "SERVO_PIN":             15,
        "SERVO_FREQ_HZ":         50,
        "SERVO_NEUTRAL_US":      1350,
        "SERVO_HARD_MIN_US":     1000,
        "SERVO_HARD_MAX_US":     2000,
        "SERVO_DEG_PER_US":      0.091092,    # deg per µs, from servo identification
        "SERVO_MAX_ANGLE_RAD":   math.pi / 3, # ±60° saturation limit

        # ---- ESC ----
        "ESC_PINS":              [13, 14],
        "ESC_FREQ_HZ":           50,
        "PWM_SAFE_US":           1000,
        "ESC_NOMINAL_US":        1700,        # → T_r_nom ≈ 2.574 N
        "ESC_HARD_MIN_US":       1100,
        "ESC_HARD_MAX_US":       1950,

        # ---- encoder ----
        "ENC_A_PIN":             18,
        "ENC_B_PIN":             19,
        "ENC_DIR":               -1,           # flip to -1 if position sign is wrong
        "MAX_STEP_RATE":         1000000,
        "DRAIN_HZ":              10000,
        "COUNTS_PER_M":          64810.4,

        # ---- controller ----
        "DT_MS":                 20,          # 50 Hz
        "V_WIN":                 5,           # velocity window (samples = 100 ms at 50 Hz)
        "V_ALPHA":               0.3,         # EMA weight on previous v_smooth

        # ---- state estimator (first-order plant models) ----
        "TAU_SERVO":             0.0244,      # s
        "TAU_THRUST":            0.0781,      # s
        "T_R_NOM":               2.574,        # N — nominal thrust (1700 µs)

        # ---- thrust command limits ----
        "T_CMD_MIN_N":           0.23,        # N — motor arm threshold; keep motor spinning
        "T_CMD_MAX_N":           4.17,        # N — static plateau at 1950 µs

        # ---- reference trajectory ----
        # List of (p_ref_m, hold_s) waypoints executed in order.
        # The controller steps to each p_ref and holds for hold_s seconds.
        # Place the cart near the center of the rail before starting.
        "P_REF_WAYPOINTS": [
            (-0.10, 5.0),
            ( 0.10, 5.0),
            (-0.10, 5.0),
            ( 0.10, 5.0),
        ],

        # ---- run timing (seconds) ----
        "ARM_DURATION_S":        3,
        "SPOOL_DURATION_S":      4,           # must be >> tau_thrust (~78 ms) to settle
        "USER_WAIT_S":           10,
        "CONTROL_DURATION_S":    20,          # must be >= sum of waypoint hold durations

        # ---- safety ----
        "P_LIMIT_M":             0.6,         # abort if |p| exceeds this (m)
    }


def finalize_config(cfg):
    cfg["DT_S"]            = cfg["DT_MS"] / 1000.0
    cfg["ARM_SAMPLES"]     = int(cfg["ARM_DURATION_S"]     / cfg["DT_S"])
    cfg["SPOOL_SAMPLES"]   = int(cfg["SPOOL_DURATION_S"]   / cfg["DT_S"])
    cfg["CONTROL_SAMPLES"] = int(cfg["CONTROL_DURATION_S"] / cfg["DT_S"])
    # Servo: us_per_rad = (deg_per_rad) / (deg_per_us) = (180/pi) / DEG_PER_US
    cfg["SERVO_US_PER_RAD"] = (180.0 / math.pi) / cfg["SERVO_DEG_PER_US"]
    return cfg


# ==============================================================================
# Utilities
# ==============================================================================


def clamp(val, lo, hi):
    return lo if val < lo else (hi if val > hi else val)


def sleep_to_next(loop_start_ms, dt_ms):
    elapsed = time.ticks_diff(time.ticks_ms(), loop_start_ms)
    rem = dt_ms - elapsed
    if rem > 0:
        time.sleep_ms(rem)


# ==============================================================================
# Thrust polynomial inversion  (Newton's method)
# ==============================================================================


def _poly(u_hat):
    c = THRUST_POLY
    return c[0]*u_hat**4 + c[1]*u_hat**3 + c[2]*u_hat**2 + c[3]*u_hat + c[4]


def _poly_d(u_hat):
    c = THRUST_POLY
    return 4*c[0]*u_hat**3 + 3*c[1]*u_hat**2 + 2*c[2]*u_hat + c[3]


# Normalized bounds corresponding to ESC_HARD_MIN/MAX
_U_HAT_LO = (1100.0 - POLY_MU) / POLY_SIGMA   # ≈ -1.660
_U_HAT_HI = (1950.0 - POLY_MU) / POLY_SIGMA   # ≈  1.659


def thrust_to_esc(T_N, u_hat_guess):
    """
    Invert the static thrust polynomial to find ESC µs for thrust T_N.
    Returns (esc_us, u_hat) — pass u_hat back as the next guess for warm-starting.
    """
    T_N   = max(0.23, min(4.17, T_N))
    u_hat = u_hat_guess
    for _ in range(6):
        df = _poly_d(u_hat)
        if abs(df) < 1e-9:
            break
        step   = (_poly(u_hat) - T_N) / df
        u_hat -= step
        u_hat  = max(_U_HAT_LO, min(_U_HAT_HI, u_hat))
        if abs(step) < 1e-6:
            break
    esc_us = int(clamp(int(POLY_MU + u_hat * POLY_SIGMA + 0.5), 1100, 1950))
    return esc_us, u_hat


# ==============================================================================
# LQR control law
# ==============================================================================


def lqr_u(x, cfg, p_ref):
    """u = u* - K_fixed @ (x - x*)  →  (theta_cmd_rad, T_cmd_N)"""
    dx = (x[0]-p_ref, x[1]-X_STAR[1], x[2]-X_STAR[2], x[3]-X_STAR[3])
    K  = K_FIXED
    theta_cmd = U_STAR[0] - (K[0][0]*dx[0] + K[0][1]*dx[1] + K[0][2]*dx[2] + K[0][3]*dx[3])
    T_cmd     = U_STAR[1] - (K[1][0]*dx[0] + K[1][1]*dx[1] + K[1][2]*dx[2] + K[1][3]*dx[3])
    theta_cmd = clamp(theta_cmd, -cfg["SERVO_MAX_ANGLE_RAD"], cfg["SERVO_MAX_ANGLE_RAD"])
    T_cmd     = clamp(T_cmd, cfg["T_CMD_MIN_N"], cfg["T_CMD_MAX_N"])
    return theta_cmd, T_cmd


# ==============================================================================
# Servo helpers
# ==============================================================================


def write_servo_us(cfg, servo, us):
    us = int(clamp(us, cfg["SERVO_HARD_MIN_US"], cfg["SERVO_HARD_MAX_US"]))
    servo.duty_ns(us * 1000)


def theta_to_servo_us(cfg, theta_rad):
    us = cfg["SERVO_NEUTRAL_US"] - theta_rad * cfg["SERVO_US_PER_RAD"]
    return int(clamp(us, cfg["SERVO_HARD_MIN_US"], cfg["SERVO_HARD_MAX_US"]))


def setup_servo(cfg):
    servo = PWM(Pin(cfg["SERVO_PIN"]))
    servo.freq(cfg["SERVO_FREQ_HZ"])
    write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])
    time.sleep_ms(500)
    return servo


# ==============================================================================
# ESC helpers
# ==============================================================================


def _duty_u16(pwm_us):
    return int(int(pwm_us) * 65535 // 20000)


def set_esc_us(cfg, escs, us):
    us = int(clamp(us, cfg["PWM_SAFE_US"], 2000))
    d  = _duty_u16(us)
    for esc in escs:
        esc.duty_u16(d)


def setup_escs(cfg):
    escs = []
    for pin_num in cfg["ESC_PINS"]:
        esc = PWM(Pin(pin_num, Pin.OUT))
        esc.freq(cfg["ESC_FREQ_HZ"])
        escs.append(esc)
    set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
    return escs


# ==============================================================================
# Encoder helpers
# ==============================================================================


def setup_encoder_hw(cfg):
    encoder.init_configured(
        cfg["ENC_A_PIN"], cfg["ENC_B_PIN"],
        cfg["MAX_STEP_RATE"], cfg["DRAIN_HZ"],
    )
    time.sleep_ms(100)


# ==============================================================================
# State estimator
# ==============================================================================


class StateEstimator:
    """
    Estimates [p, v, theta_hat, T_hat] from encoder count and previous commands.

    p         : encoder-derived position relative to a captured reference count
    v         : windowed finite-difference velocity, exponentially smoothed
    theta_hat : Euler forward integration of tau_servo first-order model
    T_hat     : Euler forward integration of tau_thrust first-order model
    """

    def __init__(self, cfg, count_ref):
        self._cfg       = cfg
        self._count_ref = count_ref
        self._cpm       = cfg["COUNTS_PER_M"] * cfg["ENC_DIR"]
        dt              = cfg["DT_S"]
        self._alpha     = cfg["V_ALPHA"]
        N               = cfg["V_WIN"]

        # First-order Euler coefficients
        self._a_srv = 1.0 - dt / cfg["TAU_SERVO"]
        self._b_srv = dt / cfg["TAU_SERVO"]
        self._a_thr = 1.0 - dt / cfg["TAU_THRUST"]
        self._b_thr = dt / cfg["TAU_THRUST"]

        # State
        self.theta_hat = 0.0
        self.T_hat     = cfg["T_R_NOM"]   # motor fully spooled at spool exit
        self._v_smooth = 0.0

        # Circular buffer for velocity window
        self._buf     = [count_ref] * N
        self._buf_N   = N
        self._buf_idx = 0

    def update(self, count, theta_cmd_prev, T_cmd_prev):
        """
        Update state estimates.  theta_cmd_prev and T_cmd_prev are the commands
        that were applied to the hardware during the PREVIOUS sample.
        Returns (p, v, theta_hat, T_hat).
        """
        # Store latest count and advance index; oldest entry is now at buf_idx
        self._buf[self._buf_idx] = count
        self._buf_idx = (self._buf_idx + 1) % self._buf_N
        oldest = self._buf[self._buf_idx]

        p     = (count - self._count_ref) / self._cpm
        v_raw = (count - oldest) / (self._buf_N * self._cfg["DT_S"] * self._cpm)
        self._v_smooth = self._alpha * self._v_smooth + (1.0 - self._alpha) * v_raw

        self.theta_hat = self._a_srv * self.theta_hat + self._b_srv * theta_cmd_prev
        self.T_hat     = self._a_thr * self.T_hat     + self._b_thr * T_cmd_prev

        return p, self._v_smooth, self.theta_hat, self.T_hat


# ==============================================================================
# CSV logging
# ==============================================================================


def write_csv_header(f):
    f.write(
        "t_ms,t_s,phase,"
        "count,p_m,p_ref_m,v_mps,"
        "theta_hat_rad,T_hat_N,"
        "theta_cmd_rad,T_cmd_N,"
        "servo_us,esc_us\n"
    )


def write_csv_row(f, t0_ms, phase, count, p, p_ref, v, theta_hat, T_hat,
                  theta_cmd, T_cmd, servo_us, esc_us):
    t_ms = time.ticks_diff(time.ticks_ms(), t0_ms)
    f.write("{},{:.3f},{},{},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{},{}\n".format(
        int(t_ms), t_ms / 1000.0, phase,
        int(count), p, p_ref, v, theta_hat, T_hat,
        theta_cmd, T_cmd, int(servo_us), int(esc_us),
    ))


# ==============================================================================
# Control run
# ==============================================================================


def run_control(cfg, servo, escs):
    dt_ms    = cfg["DT_MS"]
    log_file = cfg["LOG_FILE_BASE"] + "_set01.csv"

    # ── arm ────────────────────────────────────────────────────────────────────
    print("arm ({} s): ESC at safe, servo at neutral".format(cfg["ARM_DURATION_S"]))
    write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])
    set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
    for _ in range(cfg["ARM_SAMPLES"]):
        sleep_to_next(time.ticks_ms(), dt_ms)

    # ── spool ──────────────────────────────────────────────────────────────────
    print("spool ({} s): ESC at {} us, waiting for thrust to settle".format(
        cfg["SPOOL_DURATION_S"], cfg["ESC_NOMINAL_US"]))
    set_esc_us(cfg, escs, cfg["ESC_NOMINAL_US"])
    for _ in range(cfg["SPOOL_SAMPLES"]):
        sleep_to_next(time.ticks_ms(), dt_ms)

    # ── user wait ──────────────────────────────────────────────────────────────
    print()
    print("=" * 60)
    print("lqr_tier1 — FIXED-GAIN LQR CONTROL TEST")
    print()
    print("  Motor is spooled at nominal ({} us, ~{:.3f} N).".format(
        cfg["ESC_NOMINAL_US"], cfg["T_R_NOM"]))
    print("  Servo is at 0 deg — no lateral force.")
    print()
    print("  Place the cart at the desired reference position.")
    print("  That position becomes p = 0 m for the controller.")
    print()
    print("  Control starts in {} s...".format(cfg["USER_WAIT_S"]))
    print("=" * 60)
    for i in range(cfg["USER_WAIT_S"], 0, -1):
        print(i)
        time.sleep(1)

    count_ref = encoder.get_count()
    print("Reference count: {}".format(count_ref))

    # ── control ────────────────────────────────────────────────────────────────
    est          = StateEstimator(cfg, count_ref)
    u_hat_guess  = (cfg["ESC_NOMINAL_US"] - POLY_MU) / POLY_SIGMA  # warm-start at nominal
    theta_cmd    = 0.0
    T_cmd        = cfg["T_R_NOM"]
    servo_us     = cfg["SERVO_NEUTRAL_US"]
    esc_us       = cfg["ESC_NOMINAL_US"]
    print_n      = cfg["PRINT_EVERY_N_SAMPLES"]
    p_limit      = cfg["P_LIMIT_M"]

    # Build waypoint schedule: list of (p_ref_m, end_sample) in execution order
    waypoints = cfg["P_REF_WAYPOINTS"]
    schedule  = []
    cursor    = 0
    for p_ref_m, hold_s in waypoints:
        cursor += int(hold_s / cfg["DT_S"])
        schedule.append((p_ref_m, cursor))
    wpt_idx = 0
    p_ref   = schedule[0][0] if schedule else 0.0

    print("control ({} s): LQR active. Logging to {}".format(
        cfg["CONTROL_DURATION_S"], log_file))
    print("Reference waypoints:")
    for i, (pr, es) in enumerate(schedule):
        print("  [{:d}] p_ref={:+.3f} m  until sample {:d}".format(i, pr, es))

    t0_ms   = time.ticks_ms()
    aborted = False

    with open(log_file, "w") as f:
        write_csv_header(f)

        for sample in range(cfg["CONTROL_SAMPLES"]):
            loop_start = time.ticks_ms()

            # Advance reference waypoint
            if wpt_idx < len(schedule) - 1 and sample >= schedule[wpt_idx][1]:
                wpt_idx += 1
                p_ref = schedule[wpt_idx][0]
                print("waypoint {:d}: p_ref={:+.3f} m  (sample {:d})".format(
                    wpt_idx, p_ref, sample))

            count = encoder.get_count()
            p, v, theta_hat, T_hat = est.update(count, theta_cmd, T_cmd)
            x = (p, v, theta_hat, T_hat)

            theta_cmd, T_cmd = lqr_u(x, cfg, p_ref)

            servo_us            = theta_to_servo_us(cfg, theta_cmd)
            esc_us, u_hat_guess = thrust_to_esc(T_cmd, u_hat_guess)

            write_servo_us(cfg, servo, servo_us)
            set_esc_us(cfg, escs, esc_us)

            write_csv_row(f, t0_ms, "control",
                          count, p, p_ref, v, theta_hat, T_hat,
                          theta_cmd, T_cmd, servo_us, esc_us)

            if sample % print_n == 0:
                print("s{:4d}  p={:+.4f}m  ref={:+.4f}m  err={:+.4f}m"
                      "  v={:+.5f}m/s  th_cmd={:+.4f}rad  T_cmd={:.3f}N"
                      "  srv={:d}us  esc={:d}us".format(
                    sample, p, p_ref, p - p_ref, v,
                    theta_cmd, T_cmd, servo_us, esc_us))

            if abs(p) > p_limit:
                print("SAFETY STOP: |p|={:.3f} m exceeds limit {:.3f} m".format(
                    abs(p), p_limit))
                aborted = True
                break

            sleep_to_next(loop_start, dt_ms)

    if not aborted:
        print("Control complete.")
    print("Saved:", log_file)
    return log_file


# ==============================================================================
# Hardware setup / shutdown
# ==============================================================================


def setup_hardware(cfg):
    print("Servo on GP{}".format(cfg["SERVO_PIN"]))
    servo = setup_servo(cfg)
    print("ESC on pins", cfg["ESC_PINS"])
    escs = setup_escs(cfg)
    print("Encoder on GP{}/GP{}".format(cfg["ENC_A_PIN"], cfg["ENC_B_PIN"]))
    setup_encoder_hw(cfg)
    print("Initial encoder count:", encoder.get_count())
    return servo, escs


def safe_shutdown(cfg, servo, escs):
    print("Safe shutdown.")
    if escs:
        set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
        time.sleep_ms(500)
    if servo:
        write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])
        time.sleep_ms(500)


# ==============================================================================
# Main
# ==============================================================================


def main():
    cfg = finalize_config(make_config())

    print()
    print("lqr_tier1 — Fixed-Gain LQR, Tier 1")
    print("  K_servo  (theta_cmd): [{:.4f}  {:.4f}  {:.4f}  {:.4f}]".format(*K_FIXED[0]))
    print("  K_thrust (T_cmd)    : [{:.4f}  {:.4f}  {:.4f}  {:.4f}]".format(*K_FIXED[1]))
    print("  x*: p=0 m  v=0 m/s  theta=0 rad  T={:.4f} N".format(X_STAR[3]))
    print("  DT: {} ms  |  ENC_DIR: {}  |  COUNTS_PER_M: {:.1f}".format(
        cfg["DT_MS"], cfg["ENC_DIR"], cfg["COUNTS_PER_M"]))
    print()

    servo = None
    escs  = []

    try:
        servo, escs = setup_hardware(cfg)
        run_control(cfg, servo, escs)

    except KeyboardInterrupt:
        print("Interrupted.")

    finally:
        safe_shutdown(cfg, servo, escs)


main()
