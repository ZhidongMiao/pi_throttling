"""
pipeline.py — Enhanced Vector Processor Pipeline Model
======================================================
Models the microarchitecture per the updated spec:

  DEC → OOO(rename) → SHQ/LDQ → EXQ0/EXQ1/LNQ/STQ → pipeline → WB

Key additions over the minimal D2I/I2EX model:
  - Physical register file (100 phys, 32 arch) with rename & recycling
  - Queue structures with depth backpressure (SHQ=44, LDQ=16, EXQ=16)
  - Per-instruction pipeline depth (LN=14, MULA=9, MUL/ADD=8, MOV=6, LD=10, ST=4)
  - ISU dependency checking via physical registers (oldest-ready-first)
  - DEC bandwidth limit (max 5/cycle)
  - ST instruction support
  - MULA + LN/EXP co-issue constraint enforced at issue time

Issue ports (5 pipelines):
  EXQ0 : MULA | MUL | ADD | MOV
  EXQ1 : MULA | MUL | ADD | MOV
  LNQ  : LN  | EXP
  STQ  : ST
  LDQ  : LD

SHQ (44 entries) feeds EXQ0/EXQ1/LNQ/STQ.  LDQ (16 entries) feeds LD pipeline.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ── Instruction definitions ──────────────────────────────────────────
TOK = {"mula": 3, "ln": 4, "exp": 4, "mul": 2, "add": 2, "mov": 1, "ld": 0, "st": 1, "nop": 0}

# Pipeline depth (cycles from issue to WB complete)
PIPE_DEPTH = {
    "ln": 14, "exp": 14,   # PIPE0..E9→WB
    "mula": 9,              # PIPE0..E4→WB
    "mul": 8, "add": 8,    # PIPE0..E3→WB
    "mov": 6,               # PIPE0..E1→WB
    "ld": 10,               # LD0..LD9→WB
    "st": 4,                # PIPE0..WRITE_MEM
    "nop": 1,
}

# Number of register sources per op
OP_SRC = {"mula": 3, "mul": 2, "add": 2, "exp": 2, "ln": 1, "mov": 1, "ld": 0, "st": 1, "nop": 0}

# Whether op produces a destination register
HAS_DST = {"mula": True, "mul": True, "add": True, "exp": True, "ln": True,
           "mov": True, "ld": True, "st": False, "nop": False}

# ── Microarchitecture parameters ─────────────────────────────────────
ARCH_REGS = 32
PHYS_REGS = 100
RENAME_POOL = PHYS_REGS - ARCH_REGS  # 68 rename registers
SHQ_DEPTH = 44
LDQ_DEPTH = 16
EXQ_DEPTH = 16
MAX_DEC_BW = 5   # max instructions decoded per cycle

# ── Deprecation note ─────────────────────────────────────────────────
# LAT and ROB_SIZE removed — replaced by PIPE_DEPTH and RENAME_POOL.
# LD_LAT removed — LD pipeline depth is PIPE_DEPTH["ld"] = 10.

# ══════════════════════════════════════════════════════════════════════
# InstrGroup
# ══════════════════════════════════════════════════════════════════════
@dataclass
class InstrGroup:
    """One cycle's instruction issue across 5 ports.

    Register operands (optional): when provided, the pipeline model uses these
    arch register numbers for renaming and dependency checking.  When omitted
    (all fields -1/empty), legacy auto-allocation is used for backward compat.
    """
    exq0: Optional[str] = None   # EXQ0: MULA|MUL|ADD|MOV
    exq1: Optional[str] = None   # EXQ1: MULA|MUL|ADD|MOV
    lnq:  Optional[str] = None   # LNQ:  LN|EXP
    ldq:  Optional[str] = None   # LDQ:  LD
    stq:  Optional[str] = None   # STQ:  ST
    task_notice: int = 0

    # Arch register operands per port (dst=-1 means no dst, e.g. ST, NOP)
    # src/dst refer to arch register numbers 0–31
    e0_dst: int = -1;  e0_src: tuple = ()
    e1_dst: int = -1;  e1_src: tuple = ()
    ln_dst: int = -1;  ln_src: tuple = ()
    ld_dst: int = -1;  ld_src: tuple = ()
    st_dst: int = -1;  st_src: tuple = ()

    @property
    def total_token(self) -> int:
        return (TOK.get(self.exq0, 0) + TOK.get(self.exq1, 0) +
                TOK.get(self.lnq, 0) + TOK.get(self.ldq, 0) + TOK.get(self.stq, 0))

    @property
    def has_ld(self) -> bool:
        return self.ldq == "ld"

    @property
    def has_st(self) -> bool:
        return self.stq == "st"

    @property
    def is_empty(self) -> bool:
        return not any([self.exq0, self.exq1, self.lnq, self.ldq, self.stq])

    def ops(self) -> List[Tuple[str, str]]:
        """Return [(port, op), ...] for all non-None ports."""
        result = []
        for port in ('exq0', 'exq1', 'lnq', 'ldq', 'stq'):
            op = getattr(self, port)
            if op is not None:
                result.append((port, op))
        return result


def mg(e0=None, e1=None, lnq=None, ldq=None, stq=None,
       e0_dst=-1, e0_src=(), e1_dst=-1, e1_src=(),
       ln_dst=-1, ln_src=(), ld_dst=-1, ld_src=(), st_dst=-1, st_src=()) -> InstrGroup:
    """Shorthand constructor for InstrGroup with optional arch register operands."""
    return InstrGroup(exq0=e0, exq1=e1, lnq=lnq, ldq=ldq, stq=stq,
                      e0_dst=e0_dst, e0_src=e0_src, e1_dst=e1_dst, e1_src=e1_src,
                      ln_dst=ln_dst, ln_src=ln_src, ld_dst=ld_dst, ld_src=ld_src,
                      st_dst=st_dst, st_src=st_src)


def is_legal_issue(grp: InstrGroup) -> bool:
    """Check MULA / LN/EXP co-issue constraint (at DEC level for backward compat)."""
    has_mula = (grp.exq0 == "mula") or (grp.exq1 == "mula")
    has_trans = grp.lnq in ("ln", "exp")
    return not (has_mula and has_trans)


def enforce_issue_constraint(grp: InstrGroup) -> InstrGroup:
    """If MULA+LN/EXP conflict, squash LNQ (hardware interlock).

    Kept for backward compatibility. The constraint is now also enforced
    inside PipelineModel at issue time.
    """
    if not is_legal_issue(grp):
        return InstrGroup(exq0=grp.exq0, exq1=grp.exq1, lnq=None, ldq=grp.ldq, stq=grp.stq)
    return grp


# ══════════════════════════════════════════════════════════════════════
# MicroOp — a single renamed instruction in the pipeline
# ══════════════════════════════════════════════════════════════════════
@dataclass
class MicroOp:
    op: str
    src_phys: List[int] = field(default_factory=list)   # source phys regs
    dst_phys: int = -1                                    # destination phys reg, -1 if none
    age: int = 0                                          # global sequence number
    token: int = 0                                        # precomputed from TOK
    reads_done: bool = False                              # reader counts released

    @property
    def depth(self) -> int:
        return PIPE_DEPTH[self.op]


# ══════════════════════════════════════════════════════════════════════
# Physical Register File
# ══════════════════════════════════════════════════════════════════════
class PhysRegFile:
    """100 physical registers with rename, reader tracking, and recycling.

    Recycling conditions (per spec):
      1. Arch register has been re-renamed (old phys no longer in arch_map)
      2. Writer instruction completed WB
      3. All readers completed read (reader_count == 0)
    """

    def __init__(self):
        # Free physical registers (initially 32..99, arch regs 0..31 are pre-mapped)
        self.free: List[int] = list(range(PHYS_REGS - 1, ARCH_REGS - 1, -1))
        # Arch → current phys reg mapping
        self.arch_map: List[int] = list(range(ARCH_REGS))
        # Per-phys-reg state
        self.writer_done: List[bool] = [True] * PHYS_REGS
        self.readers: List[int] = [0] * PHYS_REGS
        # Reverse map: which arch reg (if any) maps to this phys reg
        self._phys_to_arch: dict = {i: i for i in range(ARCH_REGS)}

    def free_count(self) -> int:
        return len(self.free)

    def rename(self, arch_dst: int) -> Tuple[int, int]:
        """Allocate new phys reg for arch_dst. Returns (new_phys, old_phys)."""
        old_phys = self.arch_map[arch_dst]
        new_phys = self.free.pop()
        self.arch_map[arch_dst] = new_phys
        self.writer_done[new_phys] = False
        self._phys_to_arch[new_phys] = arch_dst
        # Old phys reg is no longer mapped
        self._phys_to_arch.pop(old_phys, None)
        self._try_recycle(old_phys)
        return new_phys, old_phys

    def add_reader(self, phys: int):
        self.readers[phys] += 1

    def remove_reader(self, phys: int):
        if self.readers[phys] > 0:
            self.readers[phys] -= 1
        self._try_recycle(phys)

    def mark_write_done(self, phys: int):
        self.writer_done[phys] = True
        self._try_recycle(phys)

    def _try_recycle(self, phys: int):
        if phys < ARCH_REGS:        # initial arch regs are never freed
            return
        if phys in self._phys_to_arch:   # still mapped to an arch reg
            return
        if not self.writer_done[phys]:   # writer not done
            return
        if self.readers[phys] > 0:       # still has pending readers
            return
        self.free.append(phys)

    def is_src_ready(self, phys: int) -> bool:
        """Source register is ready if no pending writer (writer_done is True)."""
        return self.writer_done[phys]


# ══════════════════════════════════════════════════════════════════════
# PipelineModel
# ══════════════════════════════════════════════════════════════════════
class PipelineModel:
    """Enhanced OOO pipeline with queue backpressure, register renaming,
    dependency checking, and per-op pipeline depths.

    Provides early_wakeup and ex_busy signals to the throttle controller
    for proactive ramp/warm-window decisions.
    """

    def __init__(self, d2i: int = 2, i2ex: int = 4):
        # d2i, i2ex kept in signature for backward compat; they are not used
        # internally — the new model uses the full microarchitecture instead.
        self.rf = PhysRegFile()
        self.age_counter = 0
        self._credit = 999  # max token allowed per DEC cycle

        # ── Dispatch queues ──────────────────────────────────────────
        self.shq: List[MicroOp] = []   # max 44
        self.ldq: List[MicroOp] = []   # max 16

        # ── Execution queues (waiting for issue) ──────────────────────
        self.exq0_wait: List[MicroOp] = []  # max 16
        self.exq1_wait: List[MicroOp] = []  # max 16
        self.lnq_wait: List[MicroOp] = []   # max 16
        self.stq_wait: List[MicroOp] = []   # max 16

        # ── Pipeline stages per queue (in-flight, stage 0..depth) ──
        # Pipeline length = max_depth + 1 so an instruction can reach
        # stage == depth, where _complete_wb removes it.
        _MAX_DEPTH = max(PIPE_DEPTH.values())
        self.exq0_pipe: List[Optional[MicroOp]] = [None] * (_MAX_DEPTH + 1)
        self.exq1_pipe: List[Optional[MicroOp]] = [None] * (_MAX_DEPTH + 1)
        self.lnq_pipe: List[Optional[MicroOp]]  = [None] * (_MAX_DEPTH + 1)
        self.stq_pipe: List[Optional[MicroOp]]  = [None] * (PIPE_DEPTH["st"] + 1)
        self.ldq_pipe: List[Optional[MicroOp]]  = [None] * (PIPE_DEPTH["ld"] + 1)

        # ── Per-cycle state ──────────────────────────────────────────
        self.issued_token = 0
        self._ld_issued = False
        self._isu_valid = False
        # Stall buffer: ops that couldn't be decoded last cycle
        self._stall_buf: List[Tuple[str, str]] = []

        # ── Backward-compat buffers (kept for has_heavy_in_dec) ──────
        self.dec_buf: List[Optional[InstrGroup]] = [None] * d2i

    # ── Public interface ──────────────────────────────────────────────

    def set_credit(self, limit: int):
        """Set token credit limit for DEC stage."""
        self._credit = limit

    def push_dec(self, grp: InstrGroup) -> int:
        """Try to decode instruction group into SHQ/LDQ. Returns number accepted.

        Pushes to decode buffer (backward compat), then allocates phys regs,
        renames, and dispatches to SHQ or LDQ. Stalls if:
          - Not enough free physical registers
          - SHQ or LDQ is full
          - DEC bandwidth exceeded

        Credit is applied at issue time (advance), not here.
        """
        self.dec_buf = [grp] + self.dec_buf[:-1]

        ops_to_decode = self._stall_buf + grp.ops()
        self._stall_buf = []

        accepted = 0

        for port, op in ops_to_decode:
            if accepted >= MAX_DEC_BW:
                self._stall_buf.append((port, op))
                continue

            need_dst = HAS_DST[op]
            if need_dst and self.rf.free_count() < 1:
                self._stall_buf.append((port, op))
                continue

            if op == "ld":
                if len(self.ldq) >= LDQ_DEPTH:
                    self._stall_buf.append((port, op))
                    continue
            else:
                if len(self.shq) >= SHQ_DEPTH:
                    self._stall_buf.append((port, op))
                    continue

            self._accept(port, op, grp)
            accepted += 1

        return accepted

    def _accept(self, port: str, op: str, grp: InstrGroup = None):
        """Rename and dispatch one instruction into SHQ or LDQ.

        Uses arch register operands from InstrGroup when provided; falls back
        to legacy auto-allocation (dependency-free) otherwise.
        """
        age = self.age_counter
        self.age_counter += 1

        need_dst = HAS_DST[op]
        src_count = OP_SRC[op]

        # Resolve arch register operands from InstrGroup
        if grp is not None:
            port_regs = {
                'exq0': (grp.e0_dst, grp.e0_src),
                'exq1': (grp.e1_dst, grp.e1_src),
                'lnq':  (grp.ln_dst, grp.ln_src),
                'ldq':  (grp.ld_dst, grp.ld_src),
                'stq':  (grp.st_dst, grp.st_src),
            }
            arch_dst, arch_srcs = port_regs.get(port, (-1, ()))
        else:
            arch_dst, arch_srcs = -1, ()

        # Source phys regs — from stimulus arch regs, or legacy safe pool
        if arch_srcs:
            src_phys = []
            for a in arch_srcs:
                phys = self.rf.arch_map[a]
                src_phys.append(phys)
                self.rf.add_reader(phys)
        else:
            # Legacy auto-allocation: always-ready sources (regs 30,31)
            _SRC_REGS = [30, 31]
            src_phys = []
            for s in range(src_count):
                phys = self.rf.arch_map[_SRC_REGS[s % len(_SRC_REGS)]]
                src_phys.append(phys)
                self.rf.add_reader(phys)

        # Destination phys reg — from stimulus or per-op pool
        if need_dst:
            if arch_dst >= 0:
                dst_phys, _ = self.rf.rename(arch_dst)
            else:
                dst_phys, _ = self.rf.rename(_pick_arch_reg(op, age))
        else:
            dst_phys = -1

        mop = MicroOp(op=op, src_phys=src_phys, dst_phys=dst_phys,
                       age=age, token=TOK.get(op, 0))

        if op == "ld":
            self.ldq.append(mop)
        else:
            self.shq.append(mop)

    def advance(self) -> Tuple[int, bool, bool, bool, bool, bool, bool]:
        """Advance pipeline one cycle.

        Returns: (issued_token, early_wakeup, isu_valid, ex_busy,
                  ld_issued, lnq_idle, heavy_queued)
        """
        # 1. Remove completed instructions from end of pipelines
        self._complete_wb()

        # 2. Dispatch SHQ → EXQ0/EXQ1/LNQ/STQ
        self._dispatch_shq()

        # 3. Check deps, pick oldest-ready per queue, check co-issue constraint
        candidates = self._pick_issue_candidates()

        # 4. Issue candidates into pipeline stage 0, respecting credit limit
        self._issue_candidates(candidates)

        # 5. Shift all pipelines forward one stage
        self._shift_pipelines()

        # 6. Decrement reader counts for instructions that passed read stage
        self._release_reads()

        # 7. Compute status signals
        token = self.issued_token
        early_wu = self._has_heavy_inflight()
        isu_valid = self._isu_valid
        ex_busy = self._any_pipe_busy()
        ld_issued = self._ld_issued
        lnq_idle = (not any(self.lnq_pipe) and len(self.lnq_wait) == 0)
        heavy_queued = self._has_heavy_queued()

        return (token, early_wu, isu_valid, ex_busy, ld_issued, lnq_idle, heavy_queued)

    # ── Internal: pipeline completion ─────────────────────────────────

    def _complete_wb(self):
        """Instructions that reached their depth are done. Free resources."""
        for pipe_list in (self.exq0_pipe, self.exq1_pipe, self.lnq_pipe,
                           self.stq_pipe, self.ldq_pipe):
            for stage in range(len(pipe_list)):
                mop = pipe_list[stage]
                if mop is None:
                    continue
                if stage >= mop.depth:   # passed WB
                    pipe_list[stage] = None
                    if mop.dst_phys >= 0:
                        self.rf.mark_write_done(mop.dst_phys)

    # ── Internal: SHQ dispatch ────────────────────────────────────────

    def _dispatch_shq(self):
        """Move ops from SHQ to EXQ wait queues when space available."""
        # Partition SHQ by target queue
        for mop in list(self.shq):
            placed = False
            if mop.op in ("ln", "exp"):
                if len(self.lnq_wait) < EXQ_DEPTH:
                    self.lnq_wait.append(mop)
                    placed = True
            elif mop.op == "st":
                if len(self.stq_wait) < EXQ_DEPTH:
                    self.stq_wait.append(mop)
                    placed = True
            else:  # mula/mul/add/mov → EXQ0 or EXQ1
                # Pick queue with most space, prefer EXQ0 if tied
                e0_space = EXQ_DEPTH - len(self.exq0_wait)
                e1_space = EXQ_DEPTH - len(self.exq1_wait)
                if e0_space >= e1_space and e0_space > 0:
                    self.exq0_wait.append(mop)
                    placed = True
                elif e1_space > 0:
                    self.exq1_wait.append(mop)
                    placed = True

            if placed:
                self.shq.remove(mop)

    # ── Internal: dependency check & issue candidate selection ────────

    def _pick_issue_candidates(self) -> dict:
        """For each queue, find the oldest ready MicroOp.

        Returns dict: queue_name → MicroOp or None.
        """
        def _oldest_ready(wait_list: List[MicroOp]) -> Optional[MicroOp]:
            ready = [m for m in wait_list if self._all_src_ready(m)]
            if not ready:
                return None
            return min(ready, key=lambda m: m.age)

        exq0_cand = _oldest_ready(self.exq0_wait)
        exq1_cand = _oldest_ready(self.exq1_wait)
        lnq_cand  = _oldest_ready(self.lnq_wait)
        stq_cand  = _oldest_ready(self.stq_wait)
        ldq_cand  = _oldest_ready(self.ldq)

        # MULA + LN/EXP co-issue constraint
        has_mula = ((exq0_cand and exq0_cand.op == "mula") or
                    (exq1_cand and exq1_cand.op == "mula"))
        has_trans = lnq_cand and lnq_cand.op in ("ln", "exp")
        if has_mula and has_trans:
            lnq_cand = None

        return {"exq0": exq0_cand, "exq1": exq1_cand, "lnq": lnq_cand,
                "stq": stq_cand, "ldq": ldq_cand}

    def _all_src_ready(self, mop: MicroOp) -> bool:
        for phys in mop.src_phys:
            if not self.rf.is_src_ready(phys):
                return False
        return True

    def _issue_candidates(self, candidates: dict):
        """Issue selected MicroOps into their pipeline stage 0.

        Respects self._credit (max instructions per cycle).  Candidates are
        issued in priority order: EXQ0 > EXQ1 > LNQ > STQ > LDQ.
        """
        self.issued_token = 0
        self._ld_issued = False
        self._isu_valid = False

        pipe_map = {
            "exq0": (self.exq0_wait, self.exq0_pipe),
            "exq1": (self.exq1_wait, self.exq1_pipe),
            "lnq":  (self.lnq_wait,  self.lnq_pipe),
            "stq":  (self.stq_wait,  self.stq_pipe),
            "ldq":  (self.ldq,        self.ldq_pipe),
        }

        # Credit = max instructions to issue this cycle
        remaining = max(0, self._credit)

        for qname in ("exq0", "exq1", "lnq", "stq", "ldq"):
            if remaining <= 0:
                break
            mop = candidates.get(qname)
            if mop is None:
                continue
            wait_list, pipe_list = pipe_map[qname]
            if pipe_list[0] is not None:
                continue

            wait_list.remove(mop)
            pipe_list[0] = mop
            self.issued_token += mop.token
            self._isu_valid = True
            remaining -= 1
            if mop.op == "ld":
                self._ld_issued = True

    # ── Internal: pipeline shift ──────────────────────────────────────

    def _shift_pipelines(self):
        """Shift all pipelines forward one stage (stage N → N+1, N from depth-1 down to 0)."""
        for pipe_list in (self.exq0_pipe, self.exq1_pipe, self.lnq_pipe,
                           self.stq_pipe, self.ldq_pipe):
            for stage in range(len(pipe_list) - 1, 0, -1):
                if pipe_list[stage] is None and pipe_list[stage - 1] is not None:
                    pipe_list[stage] = pipe_list[stage - 1]
                    pipe_list[stage - 1] = None

    def _release_reads(self):
        """Decrement reader counts for instructions that have passed RD_RF1.

        Read-done stage per op type (after this stage, reads are complete):
          compute (MULA/MUL/ADD/MOV/LN/EXP): stage 2 (after RD_RF1)
          ST: stage 2 (after RD_RF1, before WRITE_MEM)
          LD: stage 0 (LD0, no register read stage)
        """
        READ_DONE = {"ln": 2, "exp": 2, "mula": 2, "mul": 2, "add": 2, "mov": 2,
                      "st": 2, "ld": 0, "nop": 0}

        for pipe_list in (self.exq0_pipe, self.exq1_pipe, self.lnq_pipe,
                           self.stq_pipe, self.ldq_pipe):
            for stage, mop in enumerate(pipe_list):
                if mop is None or mop.reads_done:
                    continue
                rd_done = READ_DONE.get(mop.op, 2)
                if stage > rd_done:   # passed the read stage
                    mop.reads_done = True
                    for phys in mop.src_phys:
                        self.rf.remove_reader(phys)

    # ── Internal: status signals ──────────────────────────────────────

    def _total_inflight_token(self) -> int:
        """Sum token across all instructions in queues + pipelines."""
        total = sum(m.token for m in self.shq)
        total += sum(m.token for m in self.ldq)
        for wl in (self.exq0_wait, self.exq1_wait, self.lnq_wait, self.stq_wait):
            total += sum(m.token for m in wl)
        for pl in (self.exq0_pipe, self.exq1_pipe, self.lnq_pipe,
                    self.stq_pipe, self.ldq_pipe):
            total += sum(m.token for m in pl if m is not None)
        return total

    def _has_heavy_inflight(self) -> bool:
        """Significant aggregate load in the pipeline (>= 6 tokens)."""
        return self._total_inflight_token() >= 6

    def _any_pipe_busy(self) -> bool:
        """Any instruction in any execution pipeline stage."""
        for pipe_list in (self.exq0_pipe, self.exq1_pipe, self.lnq_pipe,
                           self.stq_pipe, self.ldq_pipe):
            if any(m is not None for m in pipe_list):
                return True
        return False

    def _has_heavy_queued(self) -> bool:
        """Significant load queued in SHQ/LDQ/wait-queues (>= 6 tokens)."""
        total = sum(m.token for m in self.shq)
        total += sum(m.token for m in self.ldq)
        for wl in (self.exq0_wait, self.exq1_wait, self.lnq_wait, self.stq_wait):
            total += sum(m.token for m in wl)
        return total >= 6

    # ── Port idle status (for dummy injection gating) ─────────────────

    def exq_idle(self) -> bool:
        """True if EXQ0 or EXQ1 could accept a new instruction immediately."""
        return (len(self.exq0_wait) == 0 and self.exq0_pipe[0] is None) or \
               (len(self.exq1_wait) == 0 and self.exq1_pipe[0] is None)

    # ── Backward-compat methods ───────────────────────────────────────

    def has_heavy_in_dec(self) -> bool:
        """Backward compat: heavy ops in decode buffer or pipeline."""
        for g in self.dec_buf:
            if g is not None and g.total_token >= 5:
                return True
        return self._has_heavy_inflight()

    def ex_busy(self) -> bool:
        return self._any_pipe_busy()

    def issue(self, grp: InstrGroup):
        """Backward compat no-op — advance() handles issue internally."""
        pass

    def reset(self):
        self.__init__()


# ══════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════

# Per-op arch register pools — separates destination registers by op type
# to avoid spurious RAW hazards in synthetic benchmarks.
_ARCH_POOL_BASE = {"mula": 0, "mul": 8, "add": 16, "ln": 20, "exp": 22,
                   "mov": 24, "ld": 26, "st": 28}
_ARCH_POOL_SIZE = {"mula": 8, "mul": 8, "add": 4, "ln": 2, "exp": 2,
                   "mov": 2, "ld": 2, "st": 2}
# Source registers are always drawn from the tail of the arch file (24-31),
# which are never used as destinations by any op.  This guarantees sources
# are always ready (modelling compiler-scheduled, dependency-free SIMD code).
_SRC_ARCH_BASE = 24

def _pick_arch_reg(op: str, salt: int) -> int:
    """Deterministic arch register assignment with separated dst/src pools."""
    pool_base = _ARCH_POOL_BASE.get(op, 0)
    pool_size = _ARCH_POOL_SIZE.get(op, 2)
    return pool_base + (salt % pool_size)
