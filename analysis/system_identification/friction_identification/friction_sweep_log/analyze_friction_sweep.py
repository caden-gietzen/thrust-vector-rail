#!/usr/bin/env python3
"""
analyze_friction_sweep.py

Residual-based friction identification from fixed-angle, stepped-ESC runs.
Python port of analyze_friction_sweep.m

Method:
    For each run the servo and thrust dynamic models are applied via lsim()
    to predict the rail-direction force.  Friction is the residual:

        F_friction(t) = F_thrust(t) * sin(theta(t)) - M * xddot(t)

    Stiction is identified from runs where the encoder shows no motion after
    thrust is applied.

Models:
    Servo:  G_servo(s)  = 0.001556 / (1 + 0.0244*s) * exp(-0.0288*s)  [rad/µs]
    Thrust: G_thrust(s) = 0.00414  / (1 + 0.0781*s) * exp(-0.0252*s)  [N/µs]

Input data:
    data/raw/system_identification/friction_identification/
        friction_sweep_log/accepted/   (preferred)
        friction_sweep_log/candidate/  (fallback)

Output:
    Friction model: F_friction = b*xdot + mu*sign(xdot)   [N·s/m, N]
    Stiction:       F_stiction = F_thrust_breakaway * sin(theta_fixed)   [N]
    Figures + markdown report written to the mirrored plots/ directory.

Usage:
    python3 analyze_friction_sweep.py             # run immediately
    python3 analyze_friction_sweep.py --wait-for N  # poll until N files present
"""

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal


# ── Physical / Model Parameters ────────────────────────────────────────────

M_KG        = 0.4536       # vehicle mass (kg) — re-weigh and update
M_SIGMA_KG  = 0.136        # ±0.3 lb uncertainty (kg)

K_SERVO     = 0.001556     # rad/µs
TAU_SERVO   = 0.0244       # s
L_SERVO     = 0.0288       # s
U_NEUTRAL   = 1431         # µs

K_T         = 0.00414      # N/µs
TAU_T       = 0.0781       # s
L_T         = 0.0252       # s
U_T_MIN     = 1075         # µs

C_M         = 64810.4      # counts/m

SG_WINDOW_S          = 0.20
SG_POLY              = 3
V_MIN_MPS            = 0.01
SIMPLICITY_TOLERANCE = 0.10


# ── Utilities ───────────────────────────────────────────────────────────────

def find_repo_root(start: Path) -> Path:
    p = start.resolve()
    while p != p.parent:
        if (p / '.git').is_dir():
            return p
        p = p.parent
    raise RuntimeError(f'Could not find repo root above {start}')


def pade3(T: float):
    """3rd-order Padé approximation of e^(-T*s).

    Returns (num, den) coefficient arrays in descending power of s,
    matching MATLAB pade(T, 3).
    """
    num = np.array([-T**3 / 120.0, T**2 / 10.0, -T / 2.0, 1.0])
    den = np.array([ T**3 / 120.0, T**2 / 10.0,  T / 2.0, 1.0])
    return num, den


def build_fopdt_sys(K: float, tau: float, L: float) -> signal.lti:
    """
    Build K / (tau*s + 1) * e^(-L*s) using a 3rd-order Padé for the delay.
    Returns a scipy.signal.lti object.
    """
    pade_num, pade_den = pade3(L)
    num = np.polymul(np.array([K]), pade_num)          # K * pade_num
    den = np.polymul(np.array([tau, 1.0]), pade_den)   # (tau*s+1) * pade_den
    return signal.lti(num, den)


def rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.asarray(x) ** 2)))


# ── Data loading ─────────────────────────────────────────────────────────────

REQUIRED_COLS = {'t_s', 'phase', 'servo_us', 'esc_pwm_us', 'count', 'count_from_start'}


def load_runs(data_dir: Path) -> list:
    csv_files = sorted(data_dir.glob('*.csv'))
    if not csv_files:
        return []

    runs = []
    for fpath in csv_files:
        try:
            df = pd.read_csv(fpath)
        except Exception as exc:
            print(f'  Skipping {fpath.name}: {exc}')
            continue

        if not REQUIRED_COLS.issubset(df.columns):
            print(f'  Skipping {fpath.name} — unexpected columns.')
            continue

        run = {'filename': fpath.name, 'df': df}

        run_mask = df['phase'] == 'run'
        if run_mask.any():
            run['servo_us'] = float(df.loc[run_mask, 'servo_us'].median())
            run['esc_us']   = float(df.loc[run_mask, 'esc_pwm_us'].median())
        else:
            run['servo_us'] = float(df['servo_us'].median())
            run['esc_us']   = float(df['esc_pwm_us'].median())

        theta_deg = -(run['servo_us'] - U_NEUTRAL) * 0.091092
        run['theta_cmd_deg'] = theta_deg
        run['theta_cmd_rad'] = math.radians(theta_deg)
        run['angle_sign']    = int(np.sign(theta_deg)) if theta_deg != 0.0 else 1

        phases       = set(df['phase'].unique())
        has_stiction = 'stiction_halt' in phases
        has_endstop  = 'endstop_halt'  in phases
        has_timeout  = 'timeout_end'   in phases

        run['is_stiction'] = has_stiction and not has_endstop and not has_timeout
        run['is_motion']   = (has_endstop or has_timeout or
                              (run_mask.any() and
                               df.loc[run_mask, 'count_from_start'].abs().max() >= 15))

        halt_lbl = ('stiction' if has_stiction else
                    'endstop'  if has_endstop  else
                    'timeout'  if has_timeout  else 'unclear')
        print(f'  Run {len(runs)+1:2d}: servo={round(run["servo_us"]):4d} µs  '
              f'esc={round(run["esc_us"]):4d} µs  '
              f'angle={theta_deg:+.1f}°  halt={halt_lbl}')
        runs.append(run)

    return runs


# ── Per-run kinematics + friction residual ───────────────────────────────────

def process_motion_run(run: dict, G_servo: signal.lti, G_thrust: signal.lti):
    """
    Compute position, velocity, acceleration, and friction residual for one
    motion run.  Returns a dict of arrays, or None if the run lacks sufficient
    data.
    """
    df   = run['df']
    mask = df['phase'].isin(['run', 'endstop_halt', 'timeout_end'])
    if mask.sum() < 20:
        return None

    t_s   = df.loc[mask, 't_s'].values.astype(float)
    cnt   = df.loc[mask, 'count_from_start'].values.astype(float)
    sv_us = df.loc[mask, 'servo_us'].values.astype(float)
    ec_us = df.loc[mask, 'esc_pwm_us'].values.astype(float)

    x_m = cnt / C_M

    dt = float(np.median(np.diff(t_s)))
    if dt <= 0 or dt > 0.1:
        return None

    N = len(t_s)
    t_lsim = np.arange(N) * dt

    # Savitzky-Golay smooth then differentiate (matches MATLAB sgolayfilt + gradient)
    win_n = round(SG_WINDOW_S / dt)
    if win_n % 2 == 0:
        win_n += 1
    win_n = max(int(win_n), SG_POLY + 2)

    x_filt = signal.savgol_filter(x_m, win_n, SG_POLY)
    xdot   = np.gradient(x_filt, dt)
    xddot  = np.gradient(xdot,   dt)

    # Simulate servo angle
    u_servo_dev = sv_us - U_NEUTRAL
    _, theta_rad, _ = signal.lsim(G_servo, u_servo_dev, t_lsim)

    # Simulate thrust
    u_thrust_dev = np.maximum(0.0, ec_us - U_T_MIN)
    _, F_thrust, _ = signal.lsim(G_thrust, u_thrust_dev, t_lsim)
    F_thrust = np.maximum(0.0, F_thrust)   # clamp negative Padé artefacts

    F_rail = F_thrust * np.sin(theta_rad)
    F_fric = F_rail - M_KG * xddot

    # Trim transient edges and near-zero-velocity samples
    n_trim = round(0.5 / dt)
    idx_valid = np.abs(xdot) >= V_MIN_MPS
    idx_valid[:min(n_trim, N)] = False
    idx_valid[max(0, N - n_trim):] = False

    if idx_valid.sum() < 5:
        return None

    return {
        'x_m': x_m, 'xdot': xdot, 'xddot': xddot,
        'F_rail': F_rail, 'F_fric': F_fric, 'F_thrust': F_thrust,
        't_s': t_s, 'theta_rad': theta_rad,
        'xdot_valid':  xdot[idx_valid],
        'Ffric_valid': F_fric[idx_valid],
        'idx_valid':   idx_valid,
    }


# ── Main analysis ────────────────────────────────────────────────────────────

def run_analysis(data_dir: Path, plot_dir: Path) -> None:
    print(f'\nLoading {sum(1 for _ in data_dir.glob("*.csv"))} CSV file(s) from:\n  {data_dir}\n')

    runs = load_runs(data_dir)
    if not runs:
        sys.exit('ERROR: No valid runs loaded.')
    print(f'\nTotal runs: {len(runs)}\n')

    G_servo  = build_fopdt_sys(K_SERVO, TAU_SERVO, L_SERVO)
    G_thrust = build_fopdt_sys(K_T,     TAU_T,     L_T)

    # ── Stiction analysis ────────────────────────────────────────────────────
    print('=== Stiction Analysis ===')
    print(f'{"angle":8s}  {"ESC(µs)":8s}  {"F_thrust_ss(N)":14s}  {"F_rail_ss(N)":14s}  motion?')

    for run in runs:
        delta_u           = max(0.0, run['esc_us'] - U_T_MIN)
        run['F_thrust_ss'] = K_T * delta_u
        run['F_rail_ss']   = run['F_thrust_ss'] * math.sin(run['theta_cmd_rad'])
        motion_lbl = 'YES' if run['is_motion'] else 'no (stiction)'
        print(f'{run["theta_cmd_deg"]:+7.1f}°  {round(run["esc_us"]):7d}   '
              f'{run["F_thrust_ss"]:13.4f}   {run["F_rail_ss"]:13.4f}   {motion_lbl}')

    for sgn in [1, -1]:
        dir_runs     = [r for r in runs if r['angle_sign'] == sgn]
        stiction_r   = [r for r in dir_runs if r['is_stiction']]
        motion_r     = [r for r in dir_runs if r['is_motion']]
        if stiction_r:
            r_hi = max(stiction_r, key=lambda r: r['esc_us'])
            print(f'\n  Direction {sgn*20:+.0f}°: stiction confirmed at ESC ≤ {r_hi["esc_us"]:.0f} µs')
            print(f'    → F_thrust_ss ≤ {r_hi["F_thrust_ss"]:.4f} N, '
                  f'F_rail ≤ {r_hi["F_rail_ss"]:.4f} N (below stiction threshold)')
        if motion_r:
            r_lo = min(motion_r, key=lambda r: r['esc_us'])
            print(f'  Direction {sgn*20:+.0f}°: first motion at ESC = {r_lo["esc_us"]:.0f} µs')
            print(f'    → F_stiction ≈ {r_lo["F_rail_ss"]:.4f} N (F_rail at breakaway)')
    print()

    # ── Process motion runs ──────────────────────────────────────────────────
    all_xdot = []; all_Ffric = []; all_esc = []; all_dir = []; all_xddot = []

    for idx, run in enumerate(runs):
        if not run['is_motion']:
            continue
        res = process_motion_run(run, G_servo, G_thrust)
        if res is None:
            print(f'  Run {idx+1} ({run["filename"]}): insufficient data, skipping.')
            continue
        run.update(res)
        n_v = int(res['idx_valid'].sum())
        all_xdot.append(res['xdot_valid'])
        all_Ffric.append(res['Ffric_valid'])
        all_esc.append(np.full(n_v, run['esc_us']))
        all_dir.append(np.full(n_v, run['angle_sign']))
        all_xddot.append(res['xddot'][res['idx_valid']])

    if not all_xdot:
        sys.exit('ERROR: No valid motion data found across all runs.')

    all_xdot  = np.concatenate(all_xdot)
    all_Ffric = np.concatenate(all_Ffric)
    all_esc   = np.concatenate(all_esc)
    all_dir   = np.concatenate(all_dir)
    all_xddot = np.concatenate(all_xddot)

    print(f'Pooled {len(all_xdot)} valid samples from motion runs.\n')

    # ── Fit friction model candidates ────────────────────────────────────────
    sgn_v = np.sign(all_xdot)

    # 1. Viscous only: F = b * v
    b_visc,  = np.linalg.lstsq(all_xdot[:, None], all_Ffric, rcond=None)[0]
    rmse_visc = rms(b_visc * all_xdot - all_Ffric)

    # 2. Coulomb only: F = mu * sign(v)
    mu_coul   = float(np.mean(all_Ffric * sgn_v))
    rmse_coul = rms(mu_coul * sgn_v - all_Ffric)

    # 3. Viscous + Coulomb: F = b*v + mu*sign(v)
    A_vc             = np.column_stack([all_xdot, sgn_v])
    params_vc        = np.linalg.lstsq(A_vc, all_Ffric, rcond=None)[0]
    b_vc, mu_vc      = float(params_vc[0]), float(params_vc[1])
    rmse_vc          = rms(b_vc * all_xdot + mu_vc * sgn_v - all_Ffric)

    # Model selection with simplicity tolerance
    rmse_best_1 = min(rmse_visc, rmse_coul)
    if rmse_vc < (1.0 - SIMPLICITY_TOLERANCE) * rmse_best_1:
        selected_model = 'viscous_coulomb'
        b_sel, mu_sel, rmse_sel = b_vc, mu_vc, rmse_vc
    elif rmse_visc <= rmse_coul:
        selected_model = 'viscous'
        b_sel, mu_sel, rmse_sel = b_visc, 0.0, rmse_visc
    else:
        selected_model = 'coulomb'
        b_sel, mu_sel, rmse_sel = 0.0, mu_coul, rmse_coul

    # ── Mass uncertainty propagation ─────────────────────────────────────────
    # F_fric(M') = F_fric_nominal + (M_nominal - M') * xddot
    M_lo = M_KG - M_SIGMA_KG
    M_hi = M_KG + M_SIGMA_KG

    Ffric_lo = all_Ffric + (M_KG - M_lo) * all_xddot
    Ffric_hi = all_Ffric + (M_KG - M_hi) * all_xddot

    p_lo = np.linalg.lstsq(A_vc, Ffric_lo, rcond=None)[0]
    p_hi = np.linalg.lstsq(A_vc, Ffric_hi, rcond=None)[0]

    b_lo  = float(min(p_lo[0], p_hi[0]));  b_hi  = float(max(p_lo[0], p_hi[0]))
    mu_lo = float(min(p_lo[1], p_hi[1]));  mu_hi = float(max(p_lo[1], p_hi[1]))

    # ── Summary ──────────────────────────────────────────────────────────────
    print('=' * 60)
    print('Friction Model Comparison')
    print(f'  {"viscous":20s}  RMSE = {rmse_visc:.4f} N')
    print(f'  {"coulomb":20s}  RMSE = {rmse_coul:.4f} N')
    print(f'  {"viscous_coulomb":20s}  RMSE = {rmse_vc:.4f} N')
    print()
    print(f'=== Selected model: {selected_model} ===')
    if b_sel != 0:
        print(f'  b   = {b_sel:.4f} N·s/m   (viscous coefficient)')
    if mu_sel != 0:
        print(f'  mu  = {mu_sel:.4f} N       (Coulomb friction force)')
    print(f'  RMSE = {rmse_sel:.4f} N')
    print()
    print(f'=== Mass sensitivity (M = {M_KG:.3f} ± {M_SIGMA_KG:.3f} kg) ===')
    print(f'  b  ∈ [{b_lo:.4f}, {b_hi:.4f}] N·s/m')
    print(f'  mu ∈ [{mu_lo:.4f}, {mu_hi:.4f}] N')
    print('=' * 60)

    # ── Directional asymmetry ─────────────────────────────────────────────────
    dir_signs  = [1, -1]
    dir_labels = ['+20 deg', '-20 deg']
    b_dir      = [float('nan')] * 2
    mu_dir     = [float('nan')] * 2
    rmse_dir_v = [float('nan')] * 2
    n_dir      = [0, 0]
    asym_flag  = False
    b_asym = mu_asym = float('nan')

    print('=== Directional Asymmetry ===')
    for di, (sgn_d, lbl) in enumerate(zip(dir_signs, dir_labels)):
        mask_d    = all_dir == sgn_d
        n_dir[di] = int(mask_d.sum())
        if n_dir[di] < 10:
            print(f'  {lbl}: too few samples ({n_dir[di]}), skipping.')
            continue
        xd = all_xdot[mask_d];  fd = all_Ffric[mask_d]
        p_d = np.linalg.lstsq(np.column_stack([xd, np.sign(xd)]), fd, rcond=None)[0]
        b_dir[di]      = float(p_d[0])
        mu_dir[di]     = float(p_d[1])
        rmse_dir_v[di] = rms(fd - (p_d[0]*xd + p_d[1]*np.sign(xd)))
        print(f'  {lbl}:  b = {b_dir[di]:.4f} N·s/m,  mu = {mu_dir[di]:.4f} N,'
              f'  RMSE = {rmse_dir_v[di]:.4f} N  (n={n_dir[di]})')

    have_both_dirs = not (math.isnan(b_dir[0]) or math.isnan(b_dir[1]))
    if have_both_dirs:
        b_asym  = abs(b_dir[0]  - b_dir[1])  / (sum(abs(v) for v in b_dir)  / 2) * 100
        mu_asym = abs(mu_dir[0] - mu_dir[1]) / (sum(abs(v) for v in mu_dir) / 2) * 100
        print(f'  Delta_b = {b_asym:.1f}%,  Delta_mu = {mu_asym:.1f}%')
        asym_flag = max(b_asym, mu_asym) > 20
        if asym_flag:
            print('  *** Significant asymmetry (>20%) — direction-dependent model recommended. ***')
        else:
            print('  Asymmetry within 20% — pooled model is adequate.')
    print()

    # ── Figures ───────────────────────────────────────────────────────────────
    plot_dir.mkdir(parents=True, exist_ok=True)

    v_range   = np.linspace(all_xdot.min(), all_xdot.max(), 200)
    sgn_range = np.sign(v_range)

    # Figure 1 — Representative run (highest-ESC motion run at +angle)
    pos_motion = [i for i, r in enumerate(runs)
                  if r['is_motion'] and r.get('angle_sign', 0) > 0 and 'xdot' in r]
    if pos_motion:
        rr = runs[max(pos_motion, key=lambda i: runs[i]['esc_us'])]
        fig, axes = plt.subplots(3, 1, figsize=(10, 8))
        fig.suptitle(f'Representative run: ESC = {round(rr["esc_us"])} µs, '
                     f'angle = {rr["theta_cmd_deg"]:+.0f}°')
        axes[0].plot(rr['t_s'], rr['x_m'] * 1000)
        axes[0].set_ylabel('Position (mm)');  axes[0].grid(True)
        axes[1].plot(rr['t_s'], rr['xdot'])
        axes[1].set_ylabel('Velocity (m/s)'); axes[1].grid(True)
        axes[2].plot(rr['t_s'], rr['xddot'])
        axes[2].set_ylabel('Accel (m/s²)');  axes[2].set_xlabel('Time (s)'); axes[2].grid(True)
        plt.tight_layout()
        fig.savefig(plot_dir / 'fig1_representative_run.png', dpi=150)
        plt.close(fig)

    # Figure 2 — Stiction boundary
    fig, ax = plt.subplots(figsize=(max(8, len(runs) * 0.7), 5))
    for k, run in enumerate(runs):
        color = [0.2, 0.6, 0.2] if run['is_motion'] else [0.8, 0.2, 0.2]
        ax.bar(k + 1, run['esc_us'], color=color)
    ax.set_xticks(range(1, len(runs) + 1))
    ax.set_xticklabels(
        [f'{r["theta_cmd_deg"]:+.0f}°/{round(r["esc_us"])}' for r in runs],
        rotation=45, ha='right', fontsize=8)
    ax.set_ylabel('ESC command (µs)')
    ax.set_title('Halt classification by run (green = motion, red = stiction)')
    ax.grid(True);  plt.tight_layout()
    fig.savefig(plot_dir / 'fig2_stiction_boundary.png', dpi=150)
    plt.close(fig)

    # Figure 3 — Friction scatter vs velocity
    esc_unique = np.unique(all_esc)
    cmap_vals  = plt.cm.viridis(np.linspace(0, 1, len(esc_unique)))
    fig, ax = plt.subplots(figsize=(10, 6))
    for ek, esc_v in enumerate(esc_unique):
        mask = all_esc == esc_v
        ax.scatter(all_xdot[mask], all_Ffric[mask], s=6, color=cmap_vals[ek],
                   label=f'{round(esc_v)} µs')
    ax.plot(v_range, b_visc * v_range,                     'b--', lw=1.5,
            label=f'Viscous (b={b_visc:.4f})')
    ax.plot(v_range, mu_coul * sgn_range,                  'r--', lw=1.5,
            label=f'Coulomb (μ={mu_coul:.4f})')
    ax.plot(v_range, b_vc * v_range + mu_vc * sgn_range,  'k-',  lw=2,
            label=f'Viscous+Coulomb (b={b_vc:.4f}, μ={mu_vc:.4f})')
    ax.set_xlabel('Velocity (m/s)'); ax.set_ylabel('F_friction (N)')
    ax.set_title('Friction residual vs velocity — all motion runs')
    ax.legend(loc='best', fontsize=8); ax.grid(True); plt.tight_layout()
    fig.savefig(plot_dir / 'fig3_friction_scatter.png', dpi=150)
    plt.close(fig)

    # Figure 4 — Residual histogram
    if selected_model == 'viscous_coulomb':
        resid = all_Ffric - (b_vc * all_xdot + mu_vc * sgn_v)
    elif selected_model == 'viscous':
        resid = all_Ffric - b_visc * all_xdot
    else:
        resid = all_Ffric - mu_coul * sgn_v
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(resid, bins=40)
    ax.set_xlabel('Residual (N)'); ax.set_ylabel('Count')
    ax.set_title(f'Residual histogram — {selected_model} model  (RMSE = {rmse_sel:.4f} N)')
    ax.grid(True); plt.tight_layout()
    fig.savefig(plot_dir / 'fig4_residual_histogram.png', dpi=150)
    plt.close(fig)

    # Figure 5 — Per-direction scatter
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for di, (sgn_d, lbl) in enumerate(zip(dir_signs, dir_labels)):
        ax = axes[di]
        for run in runs:
            if not run['is_motion'] or run.get('angle_sign', 0) != sgn_d:
                continue
            if 'xdot_valid' not in run:
                continue
            ax.scatter(run['xdot_valid'], run['Ffric_valid'], s=6,
                       label=f'{round(run["esc_us"])} µs')
        ax.plot(v_range, b_vc * v_range + mu_vc * sgn_range, 'k-', lw=2, label='pooled fit')
        ax.set_xlabel('Velocity (m/s)'); ax.set_ylabel('F_friction (N)')
        ax.set_title(f'Direction {sgn_d * 20:+.0f}°')
        ax.legend(loc='best', fontsize=8); ax.grid(True)
    plt.tight_layout()
    fig.savefig(plot_dir / 'fig5_per_direction.png', dpi=150)
    plt.close(fig)

    # Figure 6 — Mass sensitivity bands
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(all_xdot, all_Ffric, s=4, color=[0.7, 0.7, 0.7])
    ax.plot(v_range, b_vc * v_range + mu_vc * sgn_range,  'k-',  lw=2,   label='nominal fit')
    ax.plot(v_range, b_lo * v_range + mu_lo * sgn_range,  'b--', lw=1.5,
            label=f'M - σ ({M_lo:.3f} kg)')
    ax.plot(v_range, b_hi * v_range + mu_hi * sgn_range,  'r--', lw=1.5,
            label=f'M + σ ({M_hi:.3f} kg)')
    ax.set_xlabel('Velocity (m/s)'); ax.set_ylabel('F_friction (N)')
    ax.set_title('Mass sensitivity: friction model bands over ±0.3 lb uncertainty')
    ax.legend(loc='best'); ax.grid(True); plt.tight_layout()
    fig.savefig(plot_dir / 'fig6_mass_sensitivity.png', dpi=150)
    plt.close(fig)

    # Figure 7 — Per-direction model comparison
    if have_both_dirs:
        v_sweep   = np.linspace(all_xdot.min(), all_xdot.max(), 300)
        sgn_sweep = np.sign(v_sweep)
        dir_colors = [[0.0, 0.9, 1.0], [1.0, 0.65, 0.0]]
        fig, ax = plt.subplots(figsize=(10, 6))
        for di, (sgn_d, lbl) in enumerate(zip(dir_signs, dir_labels)):
            mask_d = all_dir == sgn_d
            ax.scatter(all_xdot[mask_d], all_Ffric[mask_d], s=4,
                       color=dir_colors[di], label=f'{lbl} data')
        for di, lbl in enumerate(dir_labels):
            ax.plot(v_sweep, b_dir[di]*v_sweep + mu_dir[di]*sgn_sweep, '-',
                    color=dir_colors[di], lw=2,
                    label=f'{lbl} fit  b={b_dir[di]:.4f}  mu={mu_dir[di]:.4f}')
        ax.plot(v_sweep, b_vc*v_sweep + mu_vc*sgn_sweep, '--',
                color=[0.75, 0.75, 0.75], lw=1.5, label='pooled fit')
        ax.set_xlabel('Velocity (m/s)'); ax.set_ylabel('F_friction (N)')
        ax.set_title('Per-direction friction model comparison')
        ax.legend(loc='best', fontsize=8); ax.grid(True); plt.tight_layout()
        fig.savefig(plot_dir / 'fig7_per_dir_comparison.png', dpi=150)
        plt.close(fig)

    # ── Write markdown report ─────────────────────────────────────────────────
    report_path = plot_dir / 'analyze_friction_sweep.report.md'
    with open(report_path, 'w') as f:
        f.write('# Friction Sweep Analysis Report\n\n')
        f.write(f'**Generated:** {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}  \n')
        f.write(f'**Dataset:** `{data_dir}`  \n')
        f.write(f'**Runs loaded:** {len(runs)}\n\n')

        f.write(f'## Selected Friction Model: `{selected_model}`\n\n')
        if b_sel != 0:
            f.write(f'- **b** (viscous)  = {b_sel:.4f} N·s/m\n')
        if mu_sel != 0:
            f.write(f'- **mu** (Coulomb) = {mu_sel:.4f} N\n')
        f.write(f'- **RMSE** = {rmse_sel:.4f} N\n\n')

        f.write(f'## Mass Sensitivity (M = {M_KG:.3f} +/- {M_SIGMA_KG:.3f} kg)\n\n')
        f.write(f'- b  in [{b_lo:.4f}, {b_hi:.4f}] N·s/m\n')
        f.write(f'- mu in [{mu_lo:.4f}, {mu_hi:.4f}] N\n\n')

        f.write('## Directional Asymmetry\n\n')
        f.write('| Direction | b (N·s/m) | mu (N) | RMSE (N) | n |\n')
        f.write('|-----------|-----------|--------|----------|---|\n')
        for di in range(2):
            if math.isnan(b_dir[di]):
                f.write(f'| {dir_labels[di]} | — | — | — | {n_dir[di]} |\n')
            else:
                f.write(f'| {dir_labels[di]} | {b_dir[di]:.4f} | {mu_dir[di]:.4f} '
                        f'| {rmse_dir_v[di]:.4f} | {n_dir[di]} |\n')
        f.write('\n')

        if have_both_dirs:
            f.write(f'**Delta_b = {b_asym:.1f}%**, **Delta_mu = {mu_asym:.1f}%**\n\n')
            if asym_flag:
                f.write('**WARNING: Significant directional asymmetry (>20%).**\n')
                f.write('Direction-dependent friction model recommended.\n\n')
            else:
                f.write('Asymmetry within 20% — pooled model adequate.\n\n')
            f.write('_Note: for this test design, positive servo direction produces positive_\n')
            f.write('_velocity and negative servo direction produces negative velocity, so_\n')
            f.write('_servo direction and motion direction are confounded in this comparison._\n\n')

    print(f'\nFigures saved to:\n  {plot_dir}')
    print(f'Report written to:\n  {report_path}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description='Friction sweep analysis')
    parser.add_argument(
        '--wait-for', type=int, default=0,
        metavar='N',
        help='Poll candidate/ until N CSV files are present before running (0 = run immediately)')
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    repo_root  = find_repo_root(script_dir)

    # Mirror: analysis/system_identification/... → data/raw/system_identification/...
    rel_path = script_dir.relative_to(repo_root / 'analysis')

    data_base     = repo_root / 'data' / 'raw' / rel_path
    plot_dir      = repo_root / 'plots' / rel_path
    accepted_dir  = data_base / 'accepted'
    candidate_dir = data_base / 'candidate'

    if args.wait_for > 0:
        print(f'Watching {candidate_dir}')
        print(f'Waiting for {args.wait_for} CSV files...')
        while True:
            n = sum(1 for _ in candidate_dir.glob('*.csv'))
            print(f'  {n}/{args.wait_for} files present', flush=True)
            if n >= args.wait_for:
                print(f'  Threshold reached. Starting analysis.\n')
                break
            time.sleep(5)

    if accepted_dir.is_dir() and sum(1 for _ in accepted_dir.glob('*.csv')) > 0:
        data_dir = accepted_dir
    else:
        print('No accepted data found. Using candidate/ ...')
        data_dir = candidate_dir

    run_analysis(data_dir, plot_dir)


if __name__ == '__main__':
    main()
