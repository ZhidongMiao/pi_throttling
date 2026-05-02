"""
pdn_sim3.py  —  SIMD Vec Processor di/dt Throttle Simulation  v3
=================================================================
Simulation runner: stimulus generation + cycle-level simulation loop.

Modules:
  pdn.py        — PDN model (7th-order parallel IIR)
  pipeline.py   — Vector processor pipeline + instruction model
  controller.py — Throttle controller FSM + PI + protection layers

Usage:
  python3 pdn_sim3.py                # calibration + benchmark summary
  python3 run_sim.py --cycles 2000    # full JSON export for dashboard
"""

import numpy as np
from dataclasses import dataclass
from typing import List
import random

# ── Re-export from sub-modules for backward compatibility ────────────
from pdn import (
    PDNModel, PDNObserver, V0_MV, V_SIGNOFF, V_MARGIN,
    FREQ_GHZ, DT_S
)
from pipeline import (
    InstrGroup, PipelineModel, mg, is_legal_issue, enforce_issue_constraint,
    TOK, PIPE_DEPTH, ARCH_REGS, PHYS_REGS, RENAME_POOL,
)
from controller import (
    State, ThrottleParams, ThrottleController, make_default_params,
    STATE_NAMES,
)

# ══════════════════════════════════════════════════════════════════════
# Stimulus Generator (Task/Function Model)
# ══════════════════════════════════════════════════════════════════════
class StimulusGenerator:
    """Task-granularity stimulus generator with advance-notice signalling.

    Hardware model:
      - Tasks dispatched with ~300-cycle advance notice signal
      - Each task contains multiple functions (60-500 instructions each)
      - Function gaps: 60-100 cycles of idle between functions
      - task_notice field counts down to 0 = task start
    """

    BENCHMARKS = [
        ("BM1","mula_steady_state",  "单Task×3Func: MULA×2稳态, 300cy通知+80cy间隔"),
        ("BM2","mul_add_ln_steady",  "单Task×3Func: MUL+ADD+LN最大负载, 300cy通知"),
        ("BM3","ld_ex_kernel",       "单Task×4Func: LD窗口+MULA×2爆发, 300cy通知"),
        ("BM4","serial_dependency",  "单Task×4Func: 串行MULA依赖链, 300cy通知"),
        ("BM5","mula_vs_mul_add_ln", "单Task×4Func: MULA×2↔MAL交替(每200cy切换), 80cy间隔"),
        ("BM6","ln_dominated",       "单Task×2Func: LN主导持续负载, 300cy通知"),
        ("BM7","sw_resonance",       "单Task×4Func: SW谐振 200cy on/80cy off, 300cy通知"),
        ("BM8","ooo_mixed",          "双Task×2Func: OOO混合负载, 每Task 300cy通知+80cy间隔"),
        ("BM9","multi_task_4",       "4Task×2Func: MULA→MAL→LD→串行MULA, 每Task 300cy通知"),
        ("BM10","multi_task_5",       "5Task×2Func: MAL→交替→随机→LD+MULA→MOV, 每Task 300cy通知"),
    ]

    # Task advance notice (cycles before task dispatch)
    NOTICE = 300

    def __init__(self, seed: int = 42):
        self.rng = random.Random(seed)

    def generate(self, key: str, cycles: int = 2000) -> List[InstrGroup]:
        for bid, bname, _ in self.BENCHMARKS:
            if key in (bid, bname, f"{bid}_{bname}"):
                return getattr(self, f"_{bid.lower()}")(cycles)
        raise ValueError(f"Unknown: {key}")

    # ── Primitive generators ──────────────────────────────────────────
    def _idle(self, n):    return [InstrGroup()] * n
    def _mula2(self, n):   return [mg("mula", "mula")] * n
    def _mal(self, n):     return [mg("mul", "add", lnq="ln")] * n
    def _muladd(self, n):  return [mg("mul", "add")] * n
    def _mula1(self, n):   return [mg("mula")] * n
    def _mov(self, n):     return [mg("mov")] * n

    def _rand_mixed(self, n: int) -> List[InstrGroup]:
        opts = [
            (lambda: mg("mula", "mula"), 0.20),
            (lambda: mg("mul", "add", lnq="ln"), 0.15),
            (lambda: mg("mula"), 0.10),
            (lambda: mg(ldq="ld"), 0.08),
            (lambda: mg("mula", "mula", ldq="ld"), 0.10),
            (lambda: mg("mul", "add"), 0.12),
            (lambda: mg(lnq="ln"), 0.08),
            (lambda: mg("mov"), 0.05),
            (lambda: InstrGroup(), 0.12),
        ]
        pats = [o[0] for o in opts]; wts = [o[1] for o in opts]
        seq = []
        while len(seq) < n:
            pat = self.rng.choices(pats, weights=wts)[0]
            seq += [pat() for _ in range(self.rng.randint(3, 30))]
        return seq[:n]

    def _ld_burst(self, n: int) -> List[InstrGroup]:
        """LD-EX kernel: LD window → MULA×2 burst, repeating."""
        seq = []; i = 0
        while i < n:
            seq.append(mg(ldq="ld")); i += 1
            for _ in range(min(9, n - i)): seq.append(InstrGroup()); i += 1
            for _ in range(min(8, n - i)): seq.append(mg("mula", "mula")); i += 1
        return seq[:n]

    def _serial_mula(self, n: int) -> List[InstrGroup]:
        """Serial MULA dependency chain: one MULA every 5 cycles."""
        seq = []; i = 0
        while i < n:
            seq.append(mg("mula")); i += 1
            for _ in range(min(4, n - i)): seq.append(InstrGroup()); i += 1
        return seq[:n]

    def _alternating(self, n: int, chunk: int = 50) -> List[InstrGroup]:
        """Alternate MULA×2 and MAL every chunk cycles."""
        seq = []; i = 0; flip = True
        while i < n:
            c = min(chunk, n - i)
            seq += (self._mula2(c) if flip else self._mal(c))
            flip = not flip; i += c
        return seq[:n]

    # ── Task builder ──────────────────────────────────────────────────
    def _task(self, functions, gap: int = 80,
              notice: int = None) -> (List[InstrGroup], int):
        """Build a task: advance notice + functions with inter-function gaps.

        Args:
            functions: list of (generator_fn, cycle_count) tuples
            gap: idle cycles between functions (default 80)
            notice: advance notice cycles (default NOTICE=300)

        Returns:
            (seq, task_body_start) — seq includes notice + body; task_body_start
            is the index within seq where the first function begins.
        """
        if notice is None:
            notice = self.NOTICE
        seq = []
        # Advance notice: idle cycles with countdown signal
        for i in range(notice):
            g = InstrGroup()
            g.task_notice = notice - i
            seq.append(g)
        body_start = len(seq)
        # Functions with inter-function gaps
        for j, (gen, n) in enumerate(functions):
            seq += gen(n)
            if j < len(functions) - 1:
                seq += self._idle(gap)
        return seq, body_start

    # ══════════════════════════════════════════════════════════════════
    # Benchmarks (task/function model)
    # ══════════════════════════════════════════════════════════════════

    def _bm1(self, cy):
        """Single task: 3 MULA×2 functions, 300cy notice, 80cy gaps."""
        func_n = max(100, (cy - self.NOTICE - 160) // 3)
        seq, _ = self._task([
            (self._mula2, func_n),
            (self._mula2, func_n),
            (self._mula2, func_n),
        ], gap=80)
        return seq[:cy]

    def _bm2(self, cy):
        """Single task: 3 MAL functions (max load), 300cy notice, 80cy gaps."""
        func_n = max(100, (cy - self.NOTICE - 160) // 3)
        seq, _ = self._task([
            (self._mal, func_n),
            (self._mal, func_n),
            (self._mal, func_n),
        ], gap=80)
        return seq[:cy]

    def _bm3(self, cy):
        """Single task: 4 LD-burst functions, 300cy notice, 80cy gaps."""
        func_n = max(80, (cy - self.NOTICE - 240) // 4)
        seq, _ = self._task([
            (self._ld_burst, func_n),
            (self._ld_burst, func_n),
            (self._ld_burst, func_n),
            (self._ld_burst, func_n),
        ], gap=80)
        return seq[:cy]

    def _bm4(self, cy):
        """Single task: 4 serial-MULA functions, 300cy notice, 80cy gaps."""
        func_n = max(60, (cy - self.NOTICE - 240) // 4)
        seq, _ = self._task([
            (self._serial_mula, func_n),
            (self._serial_mula, func_n),
            (self._serial_mula, func_n),
            (self._serial_mula, func_n),
        ], gap=80)
        return seq[:cy]

    def _bm5(self, cy):
        """Single task: 4 alternating functions, 300cy notice, 80cy gaps."""
        func_n = max(100, (cy - self.NOTICE - 240) // 4)
        seq, _ = self._task([
            (lambda n: self._mula2(n), func_n),
            (lambda n: self._mal(n), func_n),
            (lambda n: self._mula2(n), func_n),
            (lambda n: self._mal(n), func_n),
        ], gap=80)
        return seq[:cy]

    def _bm6(self, cy):
        """Single task: 2 large MAL functions, 300cy notice, 80cy gap."""
        func_n = max(200, (cy - self.NOTICE - 80) // 2)
        seq, _ = self._task([
            (self._mal, func_n),
            (self._mal, func_n),
        ], gap=80)
        return seq[:cy]

    def _bm7(self, cy):
        """Single task: 4 MULA×2 functions w/ recovery gaps, 300cy notice."""
        func_n = max(100, (cy - self.NOTICE - 240) // 4)
        seq, _ = self._task([
            (self._mula2, func_n),
            (self._mula2, func_n),
            (self._mula2, func_n),
            (self._mula2, func_n),
        ], gap=80)
        return seq[:cy]

    def _bm8(self, cy):
        """Dual task: 2×2 random-mixed functions, 300cy notice each."""
        # Task 1
        func_n1 = max(120, (cy // 2 - self.NOTICE - 80) // 2)
        t1, _ = self._task([
            (self._rand_mixed, func_n1),
            (self._rand_mixed, func_n1),
        ], gap=80)
        # Task 2 notice + body
        t2_notice = self.NOTICE
        t2, _ = self._task([
            (self._rand_mixed, func_n1),
            (self._rand_mixed, func_n1),
        ], gap=80, notice=t2_notice)
        # Task 2 notice fires during late Task 1 — overlay notice on tail of t1
        seq = t1[:cy]
        # Add task 2 overlay: notice starts 300cy before task 2
        t2_start = len(t1)
        for i, g in enumerate(t2):
            idx = t2_start + i
            if idx >= cy:
                break
            if idx >= len(seq):
                seq.append(g)
            else:
                # Overlay: task 2 notice may replace idle cycles at end of task 1
                if g.task_notice > 0 and seq[idx].is_empty:
                    seq[idx] = g
                elif g.task_notice > 0:
                    seq[idx].task_notice = max(seq[idx].task_notice, g.task_notice)
                # Non-notice (actual instructions) extend the sequence
                elif not g.is_empty:
                    if idx < len(seq) and seq[idx].is_empty:
                        seq[idx] = g
        while len(seq) < cy:
            seq.append(InstrGroup())
        return seq[:cy]

    def _bm9(self, cy):
        """4 tasks × 2 functions: MULA→MAL→LD→SerialMULA, 300cy notice each.

        Task notices for N+1 fire during the function gap + tail of task N.
        """
        tasks = [
            [("mula2", self._mula2), ("mula2", self._mula2)],
            [("mal", self._mal), ("mal", self._mal)],
            [("ld_burst", self._ld_burst), ("ld_burst", self._ld_burst)],
            [("serial", self._serial_mula), ("serial", self._serial_mula)],
        ]
        return self._build_multitask(cy, tasks)

    def _bm10(self, cy):
        """5 tasks × 2 functions: MAL→Alternating→Random→LD+MULA→MOV, 300cy notice."""
        tasks = [
            [("mal", self._mal), ("mal", self._mal)],
            [("alt", lambda n: self._alternating(n, 50)),
             ("alt", lambda n: self._alternating(n, 50))],
            [("rand", self._rand_mixed), ("rand", self._rand_mixed)],
            [("ld_mula", lambda n: self._ld_burst(n)),
             ("ld_mula", lambda n: self._ld_burst(n))],
            [("mov", self._mov), ("mov", self._mov)],
        ]
        return self._build_multitask(cy, tasks)

    def _build_multitask(self, cy: int, tasks: list) -> List[InstrGroup]:
        """Build a multi-task benchmark with overlapping advance notices.

        Each task: 2 functions + advance notice. The notice for task N+1
        fires during the tail of task N (overlays on instruction idle slots).
        """
        GAP = 80
        seq = []
        task_starts = []  # cycle index where each task body starts

        for task_idx, funcs in enumerate(tasks):
            # Each task: notice, then func1, gap, func2
            func_len = max(80, (cy // len(tasks) - self.NOTICE - GAP) // 2)
            notice_start = len(seq)  # where this task's notice begins
            body_start = notice_start + self.NOTICE

            # Generate notice period
            for i in range(self.NOTICE):
                idx = notice_start + i
                remaining = self.NOTICE - i
                if idx < len(seq):
                    # Overlay: replace idle or add notice to existing
                    if seq[idx].is_empty:
                        seq[idx].task_notice = remaining
                else:
                    g = InstrGroup()
                    g.task_notice = remaining
                    seq.append(g)

            task_starts.append(body_start)

            # Ensure sequence is long enough
            while len(seq) < body_start:
                seq.append(InstrGroup())

            # Function 1
            seq += funcs[0][1](func_len)
            # Inter-function gap
            seq += self._idle(GAP)
            # Function 2
            seq += funcs[1][1](func_len)

        return seq[:cy]


# ══════════════════════════════════════════════════════════════════════
# SimResult
# ══════════════════════════════════════════════════════════════════════
@dataclass
class SimResult:
    name:str; label:str; cycles:int
    voltage_mv:List[float]; token_actual:List[int]; token_ideal:List[int]
    credit:List[int]; fsm_state:List[int]; m5_lock:List[bool]; comp_active:List[bool]
    delta_lim:List[bool]; ld_window:List[bool]; queue_busy:List[bool]
    task_notice:List[int]
    stall_cycles:int; min_voltage_mv:float; max_droop_mv:float
    ipc_efficiency:float; avg_token:float; throttled:bool; version:str


# ══════════════════════════════════════════════════════════════════════
# Simulation Runner
# ══════════════════════════════════════════════════════════════════════
def run_sim(name:str, label:str, stim:List[InstrGroup],
            throttled:bool=True, params:ThrottleParams=None,
            d2i:int=2, i2ex:int=4) -> SimResult:
    pdn=PDNModel(); pipe=PipelineModel(d2i,i2ex)
    ctrl=ThrottleController(params,throttled)
    lv=[];ltok=[];lideal=[];ltask=[];stalls=0;prev_busy=False;prev_actual=0
    version = ("on" if throttled else "off")
    p=params or ThrottleParams()
    N=len(stim)
    prev_cr = 8  # initial credit (max, no limit first cycle)

    for i,grp in enumerate(stim):
        # Push into pipeline (credit is NOT applied at DEC — only resources limit DEC)
        pipe.push_dec(grp)

        # Advance pipeline: apply credit limit at issue stage, then dispatch/issue/shift
        pipe.set_credit(prev_cr)
        issued_tok, early_wu, isu_valid, ex_busy, ld_issued, lnq_idle, heavy_queued = \
            pipe.advance()

        ideal_tok = grp.total_token
        rd_trig = prev_busy and not ex_busy

        cr, dummy = ctrl.step(early_wu, isu_valid, ex_busy, rd_trig, issued_tok,
                               lnq_idle, ld_issued=ld_issued, heavy_queued=heavy_queued,
                               task_notice=grp.task_notice)

        # Track stalls: compare instruction count (prev_cr) to ops offered
        if throttled and len(grp.ops()) > prev_cr:
            stalls += 1

        actual_tok = issued_tok
        dummy_tok = 0
        if throttled and dummy > 0:
            if pipe.exq_idle():
                dummy_tok = min(dummy, TOK["mula"])
                actual_tok = issued_tok + dummy_tok

        if throttled:
            delta = actual_tok - prev_actual
            if delta > p.max_delta_token:
                actual_tok = prev_actual + p.max_delta_token

        v_mv = pdn.step(actual_tok)
        lv.append(v_mv)
        ltok.append(issued_tok)               # IPC tracks real tokens only
        lideal.append(ideal_tok)
        ltask.append(grp.task_notice)
        prev_cr = cr; prev_busy = ex_busy; prev_actual = actual_tok

    min_v=min(lv); max_d=V0_MV-min_v; ideal_s=sum(lideal)
    return SimResult(
        name=name,label=label,cycles=N,
        voltage_mv=lv,token_actual=ltok,token_ideal=lideal,
        credit=ctrl.log_credit,fsm_state=ctrl.log_state,
        m5_lock=ctrl.log_m5,comp_active=ctrl.log_comp,
        delta_lim=ctrl.log_delta_lim,
        ld_window=ctrl.log_ld_window,
        queue_busy=ctrl.log_queue,
        task_notice=ltask,
        stall_cycles=stalls,min_voltage_mv=min_v,max_droop_mv=max_d,
        ipc_efficiency=sum(ltok)/ideal_s if ideal_s>0 else 1.0,
        avg_token=sum(ltok)/len(ltok),throttled=throttled,version=version)


def run_all(cycles: int = 2000) -> dict:
    gen = StimulusGenerator(42)
    p_throttle = make_default_params()
    out = {}
    for bid, bname, desc in StimulusGenerator.BENCHMARKS:
        key = f"{bid}_{bname}";  stim = gen.generate(bid, cycles)
        r_off = run_sim(key, desc, stim, False, None)
        r_on  = run_sim(key, desc, stim, True, p_throttle)
        out[key] = dict(off=r_off, on=r_on, stim=stim, desc=desc, bid=bid)
    return out


if __name__ == "__main__":
    print("=== PDN Calibration (MULA×2 tok=6 step) ===")
    pdn = PDNModel()
    ref = {0: 909, 19: 835, 40: 855, 72: 792, 114: 788, 336: 896, 398: 888}
    for cy in range(420):
        v = pdn.step(6.0)
        if cy in ref:
            print(f"  cy={cy:4d}: {v:6.1f}mV  target={ref[cy]}  err={v-ref[cy]:+.1f}")

    print("\n=== Baseline vs Throttled (extended ramp + PI regulate) ===")
    results = run_all(2000)
    print(f"{'BM':<6} {'Base':>7} {'Throt':>7} {'ΔDrop':>7} "
          f"{'IPC_off':>7} {'IPC_on':>7}  状态")
    print("-" * 70)
    all_ok = True
    for key, v in results.items():
        b = v['off'];  r = v['on']
        delta_droop = b.max_droop_mv - r.max_droop_mv
        ok = delta_droop >= 50
        if not ok:  all_ok = False
        st = f"✓+{delta_droop:.0f}mV" if ok else f"✗+{delta_droop:.0f}mV(需再-{50-delta_droop:.0f})"
        print(f"{v['bid']:<6} {b.max_droop_mv:6.1f}mV {r.max_droop_mv:6.1f}mV "
              f"{delta_droop:+6.1f}mV {b.ipc_efficiency:7.1%} {r.ipc_efficiency:7.1%}  {st}")
    print(f"\n压降改善≥50mV: {'ALL PASS ✓' if all_ok else 'FAIL ✗'}")
