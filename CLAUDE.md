# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Goal

**压降 < 80mV，同时最大化 IPC。**

Throttle controller must keep worst-case voltage droop below 80mV (Vmin > 829mV given V0=909mV) across all 10 benchmark scenarios, while minimising IPC loss.

## Commands

```bash
# Activate venv
source .venv/bin/activate

# Install dependencies (only numpy, scipy)
pip install numpy scipy

# Run simulation + export JSON, then serve dashboard locally
python3 run_sim.py --cycles 2000
python3 -m http.server 8080
# → http://localhost:8080/dashboard.html

# One-step: run sim + build standalone HTML (no server needed)
python3 build_standalone.py --cycles 2000
# → open dashboard_standalone.html in browser

# Quick PDN calibration check (no JSON output)
python3 pdn_sim3.py
```

## Architecture

| Module | Role |
|---|---|
| **`pdn.py`** | PDN model: 7th-order parallel IIR filter bank (7 MACs/cycle). Discretized from 3-mode physics via matrix-exponential ZOH. `PDNModel` (clamped output) and `PDNObserver` (unclamped, for controller feedback). |
| **`pipeline.py`** | OOO vector processor: `InstrGroup` (5-issue-port instruction bundle with arch register operands), `PhysRegFile` (100 phys / 32 arch, rename + reader tracking), `PipelineModel` (DEC→rename→SHQ/LDQ→EXQ0/EXQ1/LNQ/STQ→pipeline→WB). Models RAW/WAW/WAR hazards via physical register dependency checking. |
| **`controller.py`** | Throttle controller: 5-state FSM (`IDLE→RAMP→REGULATE→HOLD→RAMPDN`) + PI regulator + 4-layer protection (emergency brake, soft ceiling, predictive rate limit, M5 anti-resonance). Uses internal PDN observer — no hardware ADC. |
| **`pdn_sim3.py`** | Simulation runner: `StimulusGenerator` (10 benchmarks BM1–BM10 with register-aware primitives), `RegState`/`ChainRegState` register allocators, `SimResult`, `run_sim()`, `run_all()`. |

**`run_sim.py`** — CLI entry point. Calls `run_all()` and exports `sim_results.json`.

**`build_standalone.py`** — Runs simulation via `run_sim.py`, RLE-compresses bool arrays, inlines JSON as `window._SIM_DATA` into `dashboard.html`, writes `dashboard_standalone.html`.

**`dashboard.html`** — Chart.js-based interactive dashboard. Loads data from `sim_results.json` (or `window._SIM_DATA` in standalone mode). Supports throttled/baseline/overlay modes, per-benchmark KPI cards, voltage/token/FSM-state charts, Verdi-style instruction timeline with pipeline depth visualization, and PDN calibration overlay.

## Key Parameters

- `FREQ_GHZ = 1.6`, `V0_MV = 909`, `V_SIGNOFF = 675` (234 mV margin)
- Token table: `mula=3, ln=4, exp=4, mul=2, add=2, mov=1, ld=0, st=1, nop=0`
- Pipeline depths: `ln/exp=14, mula=9, ld=10, mul/add=8, mov=6, st=4`
- PDN: 7th-order IIR (3-mode physics, discretized at Ts=0.625ns). State: 7 floats.
- Register file: 32 arch regs, 100 phys regs (68 rename pool).
- Hardware constraint: `mula` (EXQ0/EXQ1) and `exp`/`ln` (LNQ) cannot issue in the same cycle.

### Controller Parameters (v4)

| Layer | Threshold | Action |
|-------|-----------|--------|
| PI regulate | target 68mV droop | Adjust credit 1–3 (Kp=0.10, Ki=0.003, Kd=1.0) |
| Soft ceiling | 38mV droop | Cap credit ≤ 2 |
| Predictive rate | 2.0 mV/cy decline | Cap credit ≤ 1 |
| Emergency brake | 60mV droop | Cap credit ≤ 1, hold 50cy |

Key design: PI target is aspirational (68mV) — the soft ceiling at 38mV is the primary governor, capping credit early to prevent PDN charge deficit accumulation. The 22mV gap between soft ceiling (38mV) and emergency (60mV) provides graduated escalation. Predictive rate limiter at 2.0 mV/cy catches rapid droop before emergency trigger. Credit caps scale by workload width: credit=2 allows full throughput for 2-instr/cycle workloads (MULA×2) but limits 3-instr/cycle workloads (MAL) to 66%.

## Simulation Output

`SimResult` fields include per-cycle traces (`voltage_mv`, `token_actual`, `token_ideal`, `credit`, `fsm_state`, `m5_lock`, `comp_active`, `delta_lim`, `ld_window`, `queue_busy`) and summary KPIs (`max_droop_mv`, `min_voltage_mv`, `ipc_efficiency`, `stall_cycles`).

## Benchmarks

10 benchmarks (BM1–BM10) covering steady-state, max-load, LD-burst, serial RAW-chain, alternating, resonance, random-mixed, and multi-task pipelines. Register dependencies: BM4 uses strict RAW chain (ChainRegState), BM3 uses LD→MULA cross-op RAW, all others use independent register allocation. See `docs/benchmark_sequences.md` for full details.
