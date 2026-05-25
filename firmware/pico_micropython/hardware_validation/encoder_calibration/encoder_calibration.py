# encoder_calibration.py
#
# Purpose:
#   Calibrate the encoder count-to-position conversion by manually traversing
#   the full rail length between the two measured end stops.
#
# Procedure (printed on startup):
#   1. Position the vehicle at END STOP A and hold still.
#   2. The script detects stillness and zeros the encoder at that position.
#   3. Move to END STOP B and hold.  Move back to END STOP A and hold.
#   4. Repeat for 3 round trips (6 one-way traversals total).
#   5. The script computes and prints counts-per-mm from all traversals.
#
# Output CSV columns:
#   t_ms, count, count_delta, traversal, segment
#
# Hardware:
#   Encoder A: GP18
#   Encoder B: GP19

import time
import encoder


# ============================================================
# Configuration
# ============================================================

ENDSTOP_DISTANCE_MM = 316.5

ENC_A_PIN = 18
ENC_B_PIN = 19
MAX_STEP_RATE = 1_000_000
DRAIN_HZ = 10_000

SAMPLE_DT_MS = 20

# Declare "at end stop" when count has not moved more than this
# over the last SETTLE_STABLE_MS milliseconds.
SETTLE_THRESHOLD_COUNTS = 8
SETTLE_STABLE_MS = 1500

# How long to log while holding at each end stop after detection.
SETTLE_HOLD_MS = 2000

NUM_ROUND_TRIPS = 3   # 6 one-way traversals total

# Warn and move on if an end stop is not reached within this time.
TRAVERSE_TIMEOUT_MS = 60_000

LOG_FILE = "encoder_calibration.csv"


# ============================================================
# Hardware
# ============================================================

def setup_encoder():
    print("Initializing encoder.")
    print("  A pin:", ENC_A_PIN, "  B pin:", ENC_B_PIN)
    encoder.init_configured(ENC_A_PIN, ENC_B_PIN, MAX_STEP_RATE, DRAIN_HZ)
    time.sleep_ms(100)


# ============================================================
# Utilities
# ============================================================

def countdown(message, seconds):
    print()
    print(message)
    for remaining in range(seconds, 0, -1):
        print(remaining)
        time.sleep(1)
    print("Go.")


# ============================================================
# End-stop detection
# ============================================================

def wait_for_still(f, t0_ms, count_zero, traversal, segment, prompt):
    """
    Block until the encoder has been stable for SETTLE_STABLE_MS.
    Logs every SAMPLE_DT_MS throughout.  Returns the stable count.
    """
    print()
    print(prompt)
    print("  Waiting for vehicle to hold still...")

    stable_ref_count = None
    stable_ref_time = None
    start_ms = time.ticks_ms()

    while True:
        now_ms = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now_ms, start_ms)

        count = encoder.get_count()
        count_delta = count - count_zero
        t_ms = time.ticks_diff(now_ms, t0_ms)

        f.write("{},{},{},{},{}\n".format(
            int(t_ms), int(count), int(count_delta), int(traversal), segment
        ))

        if stable_ref_count is None:
            stable_ref_count = count
            stable_ref_time = now_ms
        elif abs(count - stable_ref_count) > SETTLE_THRESHOLD_COUNTS:
            stable_ref_count = count
            stable_ref_time = now_ms
        else:
            stable_for_ms = time.ticks_diff(now_ms, stable_ref_time)
            if stable_for_ms >= SETTLE_STABLE_MS:
                print("  Stable. count_delta =", count_delta)
                return count

        if elapsed_ms > TRAVERSE_TIMEOUT_MS:
            print("  WARNING: timeout — continuing with current count.")
            return count

        time.sleep_ms(SAMPLE_DT_MS)


def hold_and_log(f, t0_ms, count_zero, traversal, segment, duration_ms):
    """Log at a settled position for duration_ms."""
    start_ms = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        now_ms = time.ticks_ms()
        count = encoder.get_count()
        count_delta = count - count_zero
        t_ms = time.ticks_diff(now_ms, t0_ms)
        f.write("{},{},{},{},{}\n".format(
            int(t_ms), int(count), int(count_delta), int(traversal), segment
        ))
        time.sleep_ms(SAMPLE_DT_MS)


# ============================================================
# Calibration output
# ============================================================

def compute_and_print_results(traversal_deltas):
    n = len(traversal_deltas)
    total = 0
    for d in traversal_deltas:
        total += d
    mean_counts = total / n

    counts_per_mm = mean_counts / ENDSTOP_DISTANCE_MM
    counts_per_m = counts_per_mm * 1000.0

    nominal_cpm = 60.0    # counts/mm for GT2 20-tooth pulley, 2400 CPR

    print()
    print("============================================================")
    print("ENCODER CALIBRATION RESULTS")
    print("============================================================")
    print("End-stop separation:", ENDSTOP_DISTANCE_MM, "mm")
    print()
    print("Per-traversal count deltas:")
    for i in range(n):
        direction = "A -> B" if i % 2 == 0 else "B -> A"
        print("  Traversal {:d} ({:s}):  {:d} counts".format(
            i + 1, direction, int(traversal_deltas[i])
        ))

    print()
    print("Mean counts / full traverse: {:d}".format(int(mean_counts)))
    print("Counts per mm (measured):    {:.3f}".format(counts_per_mm))
    print("Counts per m  (measured):    {:.1f}".format(counts_per_m))
    print()
    print("Nominal counts per mm:       {:.3f}  (GT2, 20-tooth, 2400 CPR)".format(nominal_cpm))
    print("Nominal counts per m:        {:.1f}".format(nominal_cpm * 1000.0))
    deviation_pct = 100.0 * (counts_per_m - nominal_cpm * 1000.0) / (nominal_cpm * 1000.0)
    print("Deviation from nominal:      {:.2f}%".format(deviation_pct))

    max_d = max(traversal_deltas)
    min_d = min(traversal_deltas)
    spread_pct = 100.0 * (max_d - min_d) / mean_counts if mean_counts > 0 else 0.0
    print()
    print("Repeatability (max-min / mean): {:.2f}%".format(spread_pct))
    if spread_pct <= 1.0:
        print("Repeatability: PASS (<= 1%)")
    else:
        print("Repeatability: WARNING (> 1% -- consider re-running)")
    print("============================================================")


# ============================================================
# Main calibration sequence
# ============================================================

def run_calibration():
    setup_encoder()

    print()
    print("============================================================")
    print("ENCODER CALIBRATION")
    print("  End-stop separation:", ENDSTOP_DISTANCE_MM, "mm")
    print("  Round trips:", NUM_ROUND_TRIPS, "(", NUM_ROUND_TRIPS * 2, "traversals)")
    print("  Output file:", LOG_FILE)
    print("============================================================")
    print()
    print("STEP 1: Move the vehicle to END STOP A and hold it there.")
    print("        The script will detect stillness and zero the encoder.")

    countdown("Move to END STOP A now. Detection starts in:", 10)

    with open(LOG_FILE, "w") as f:
        f.write("t_ms,count,count_delta,traversal,segment\n")

        t0_ms = time.ticks_ms()

        # Wait for user to settle at end stop A.
        wait_for_still(
            f, t0_ms, 0, 0, "initial",
            "Hold still at END STOP A to zero the encoder."
        )

        print("  Zeroing encoder at END STOP A.")
        encoder.zero()
        time.sleep_ms(100)
        count_zero = encoder.get_count()   # should be 0

        hold_and_log(f, t0_ms, count_zero, 0, "settle_A_0", SETTLE_HOLD_MS)

        traversal_deltas = []
        traversal_num = 0

        for trip in range(NUM_ROUND_TRIPS):
            # ---- A -> B ----
            traversal_num += 1
            start_count = encoder.get_count()
            print()
            print("STEP: Traversal {:d} of {:d}  (A -> B)".format(
                traversal_num, NUM_ROUND_TRIPS * 2
            ))
            countdown("Move vehicle to END STOP B now. Detection starts in:", 10)

            end_count = wait_for_still(
                f, t0_ms, count_zero,
                traversal_num,
                "traverse_AB_{:d}".format(trip + 1),
                "Moving A -> B.  Hold at END STOP B.",
            )
            hold_and_log(
                f, t0_ms, count_zero,
                traversal_num,
                "settle_B_{:d}".format(trip + 1),
                SETTLE_HOLD_MS,
            )
            traversal_deltas.append(abs(end_count - start_count))

            # ---- B -> A ----
            traversal_num += 1
            start_count = encoder.get_count()
            print()
            print("STEP: Traversal {:d} of {:d}  (B -> A)".format(
                traversal_num, NUM_ROUND_TRIPS * 2
            ))
            countdown("Move vehicle back to END STOP A now. Detection starts in:", 10)

            end_count = wait_for_still(
                f, t0_ms, count_zero,
                traversal_num,
                "traverse_BA_{:d}".format(trip + 1),
                "Moving B -> A.  Hold at END STOP A.",
            )
            hold_and_log(
                f, t0_ms, count_zero,
                traversal_num,
                "settle_A_{:d}".format(trip + 1),
                SETTLE_HOLD_MS,
            )
            traversal_deltas.append(abs(end_count - start_count))

    print()
    print("Saved:", LOG_FILE)

    compute_and_print_results(traversal_deltas)


# ============================================================
# Entry point
# ============================================================

try:
    run_calibration()
finally:
    print()
    print("Encoder calibration done.")
