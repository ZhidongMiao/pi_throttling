"""
run_sim.py
==========
Runs all benchmarks using pdn_sim3.py and exports sim_results.json
for the dashboard to consume.

Usage:
    python3 run_sim.py [--cycles 2000] [--out sim_results.json]
"""

import json
import argparse
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from pdn_sim3 import (
    PDNModel, PipelineModel, ThrottleController, ThrottleParams, make_default_params,
    StimulusGenerator, run_sim, run_all,
    V0_MV, V_SIGNOFF, STATE_NAMES, TOK
)
from pipeline import PIPE_DEPTH

def results_to_json(results: dict, cycles: int) -> dict:
    """Convert SimResult objects to JSON-serializable dict."""

    benchmarks_meta = []
    for bid, bname, desc in StimulusGenerator.BENCHMARKS:
        benchmarks_meta.append({"id": bid, "name": bname, "desc": desc})

    def instr_type(g) -> str:
        """Map InstrGroup to a compact type string for timeline display."""
        if g.is_empty:                                          return "idle"
        parts = []
        if g.exq0: parts.append(g.exq0.upper())
        if g.exq1: parts.append(g.exq1.upper())
        if g.lnq:  parts.append(g.lnq.upper())
        if g.ldq:  parts.append(g.ldq.upper())
        if g.stq:  parts.append(g.stq.upper())
        return "+".join(parts)

    bm_data = {}
    for key, v in results.items():
        r_on  = v["on"]     # V3 throttled
        r_off = v["off"]    # no throttle baseline
        bid   = v["bid"]

        def ser(r):
            dl  = r.delta_lim if hasattr(r, 'delta_lim') else [False]*r.cycles
            ldw = r.ld_window  if hasattr(r, 'ld_window')  else [False]*r.cycles
            qb  = r.queue_busy if hasattr(r, 'queue_busy') else [False]*r.cycles
            return {
                "voltage_mv":   [round(x, 2) for x in r.voltage_mv],
                "token_actual": r.token_actual,
                "token_ideal":  r.token_ideal,
                "credit":       r.credit,
                "fsm_state":    r.fsm_state,
                "m5_lock":      [bool(x) for x in r.m5_lock],
                "comp_active":  [bool(x) for x in r.comp_active],
                "delta_lim":    [bool(x) for x in dl],
                "ld_window":    [bool(x) for x in ldw],
                "queue_busy":   [bool(x) for x in qb],
                "stall_cycles": r.stall_cycles,
                "min_voltage_mv": round(r.min_voltage_mv, 2),
                "max_droop_mv":   round(r.max_droop_mv, 2),
                "ipc_efficiency": round(r.ipc_efficiency, 4),
                "avg_token":      round(r.avg_token, 3),
                "throttled":      r.throttled,
                "version":        getattr(r, 'version', 'unknown'),
            }

        s_on  = ser(r_on)
        s_off = ser(r_off)

        # Per-queue instruction timeline (RLE per queue for Verdi waveform)
        stim = v["stim"]
        def rle_queue(getter):
            """RLE-encode a per-queue instruction sequence."""
            out = [];  prev = None;  run = 0
            for g in stim:
                t = getter(g) or ""
                if t == prev: run += 1
                else:
                    if prev is not None: out.append([prev, run])
                    prev = t; run = 1
            if prev is not None: out.append([prev, run])
            return out
        iq_exq0 = rle_queue(lambda g: g.exq0)
        iq_exq1 = rle_queue(lambda g: g.exq1)
        iq_lnq  = rle_queue(lambda g: g.lnq)
        iq_ldq  = rle_queue(lambda g: g.ldq)
        iq_stq  = rle_queue(lambda g: g.stq)

        bm_data[bid] = {
            "on":   s_on,     # throttled
            "off":  s_off,    # no throttle baseline
            "desc": v["desc"],
            "bid":  bid,
            "iq_exq0": iq_exq0,
            "iq_exq1": iq_exq1,
            "iq_lnq":  iq_lnq,
            "iq_ldq":  iq_ldq,
            "iq_stq":  iq_stq,
        }

    # PDN calibration trace (Token=20 step)
    pdn_calib = []
    pdn = PDNModel()
    for cy in range(450):
        v = pdn.step(20.0)
        pdn_calib.append(round(v, 2))

    return {
        "meta": {
            "cycles":     cycles,
            "freq_ghz":   1.6,
            "v0_mv":      V0_MV,
            "v_signoff":  V_SIGNOFF,
            "state_names": STATE_NAMES,
            "tok_table":  TOK,
            "pipe_depth":  PIPE_DEPTH,
            "pdn_calib":  pdn_calib,
            "measured_ref": {
                "0":909,"19":835,"40":855,"72":792,
                "114":788,"336":896,"398":888
            },
        },
        "benchmarks": benchmarks_meta,
        "data": bm_data,
    }


def main():
    parser = argparse.ArgumentParser(description="SIMD di/dt Throttle Simulator")
    parser.add_argument("--cycles", type=int, default=2000,
                        help="Simulation cycles per benchmark (default: 2000)")
    parser.add_argument("--out", type=str, default="sim_results.json",
                        help="Output JSON file path")
    parser.add_argument("--pretty", action="store_true",
                        help="Pretty-print JSON (larger file)")
    args = parser.parse_args()

    print(f"[sim] Running {len(StimulusGenerator.BENCHMARKS)} benchmarks × {args.cycles} cycles ...")

    results = run_all(args.cycles)

    print(f"\n{'BM':<6} {'Base':>7} {'Throt':>7} {'ΔDroop':>7}  {'IPC_Base':>8} {'IPC_Throt':>8}")
    print("-"*62)
    for key, v in results.items():
        b=v['off']; r=v['on']
        print(f"{v['bid']:<6} {b.max_droop_mv:6.0f}mV {r.max_droop_mv:6.0f}mV "
              f"{b.max_droop_mv - r.max_droop_mv:6.0f}mV  "
              f"{b.ipc_efficiency:8.1%} {r.ipc_efficiency:8.1%}")

    out_path = os.path.join(os.path.dirname(__file__), args.out)
    payload = results_to_json(results, args.cycles)
    indent = 2 if args.pretty else None
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=indent, separators=(None if indent else (",", ":")))

    size_kb = os.path.getsize(out_path) / 1024
    print(f"\n[sim] Written → {out_path}  ({size_kb:.1f} KB)")
    print("[sim] Done.")


if __name__ == "__main__":
    main()
