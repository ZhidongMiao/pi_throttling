# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SIMD Vec processor di/dt transient current throttling simulation platform. Models a power delivery network (PDN) with a 7-state throttle controller FSM, runs 8 benchmark scenarios, and visualizes results in an interactive HTML dashboard.

## Commands

```bash
# Activate venv
source .venv/bin/activate

# Install dependencies (only numpy, scipy)
pip install numpy scipy

# Run simulation + export JSON, then serve dashboard locally
python3 run_sim.py --cycles 700
python3 -m http.server 8080
# → http://localhost:8080/dashboard.html

# One-step: run sim + build standalone HTML (no server needed)
python3 build_standalone.py --cycles 700
# → open dashboard_standalone.html in browser

# Quick PDN calibration check (no JSON output)
python3 pdn_sim3.py
```

## Architecture

Three independent modules + simulation runner:

| Module | Role |
|---|---|
| **`pdn.py`** | PDN model: 7th-order parallel IIR filter bank (7 MACs/cycle). Discretized from 3-mode physics via matrix-exponential ZOH. `PDNModel` (clamped output) and `PDNObserver` (unclamped, for controller feedback). Replaces 480-tap FIR convolution. |
| **`pipeline.py`** | Vector processor: `InstrGroup` (4-issue-port instruction bundle), `PipelineModel` (D2I + I2EX stages). Provides `early_wakeup` and `ex_busy` signals. |
| **`controller.py`** | Throttle controller: 5-state FSM (`IDLE→RAMP→REGULATE→HOLD→RAMPDN`) + PI regulator + 4-layer protection (emergency brake, predictive rate limit, ΔToken limit, M5 anti-resonance). |
| **`pdn_sim3.py`** | Simulation runner: `StimulusGenerator` (10 benchmarks BM1–BM10), `SimResult`, `run_sim()`, `run_all()`. |

**`run_sim.py`** — CLI entry point. Calls `run_all()` and exports `sim_results.json`.

**`build_standalone.py`** — Runs simulation via `run_sim.py`, RLE-compresses bool arrays, inlines JSON as `window._SIM_DATA` into `dashboard.html`, and writes `dashboard_standalone.html`.

**`dashboard.html`** — Chart.js-based interactive dashboard. Loads data from `sim_results.json` (or `window._SIM_DATA` in standalone mode). Supports throttled/baseline/overlay modes, per-benchmark KPI cards, voltage/token/FSM-state charts, and PDN calibration overlay.

## Key Parameters

- `FREQ_GHZ = 1.6`, `V0_MV = 909`, `V_SIGNOFF = 675` (234 mV margin)
- Token table: `mula=3, ln=4, exp=4, mul=2, add=2, mov=1, ld=0, nop=0`
- PDN: 7th-order IIR (3-mode physics, discretized at Ts=0.625ns). State: 7 floats. Matches original 480-tap FIR to machine precision for n<480, with correct infinite tail.
- Hardware constraint: `mula` (EXQ0/EXQ1) and `exp`/`ln` (EXQ2) cannot issue in the same cycle — enforced by `enforce_issue_constraint()`

## Simulation Output

`SimResult` fields include per-cycle traces (`voltage_mv`, `token_actual`, `token_ideal`, `credit`, `fsm_state`, `m5_lock`, `comp_active`, `delta_lim`, `ld_window`, `queue_busy`) and summary KPIs (`max_droop_mv`, `min_voltage_mv`, `ipc_efficiency`, `stall_cycles`).

