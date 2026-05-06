# DagBlock: Dependency-Graph-Driven Instruction Generation

Date: 2026-05-06 | Status: approved for implementation

## Motivation

Current primitives (`_mula2`, `_mal`, etc.) source all compute operands from a
const pool (arch regs 28–31) that is never written.  The only cross-instruction
RAW dependency is in `_ld_burst` (LD → MULA src0).  **No benchmark ever
generates an ST instruction.**  The result is a pipeline model that never stalls
on data hazards, producing unrealistically high IPC and masking the throttling
controller's behaviour under true load.

Real code follows a **LD → compute → ST** dataflow:

```
LD r0, [addr_a]     # load operand A
LD r1, [addr_b]     # load operand B
MULA r3, r0, r1, r2 # compute (RAW on r0, r1)
ST  r3, [addr_c]    # store result (RAW on r3)
```

We introduce a declarative dependency-graph abstraction (`DagBlock`) that
models this pattern, replaces const-pool sources with loaded-register sources,
and adds ST instructions that consume compute results.  The scheduler
auto-derives issue spacing from pipeline depths.

---

## Core Abstraction

### DagNode — instruction template

| Field | Type | Description |
|-------|------|-------------|
| `op` | `str` | `mula`, `mul`, `add`, `ln`, `exp`, `mov`, `ld`, `st`, `nop` |
| `port` | `str` | `exq0`, `exq1`, `lnq`, `ldq`, `stq` (derived from op) |
| `token` | `int` | From `TOK` table |
| `depth` | `int` | Pipeline depth from `PIPE_DEPTH` |
| `src_count` | `int` | Number of source operands (from `OP_SRC`) |
| `has_dst` | `bool` | Whether instruction produces a destination register |

Sources are resolved via edges; edges not yet connected default to the const
pool (r28–31) at emit time.

### DagEdge — register dependency

```
DagEdge(producer: DagNode, consumer: DagNode, src_slot: int)
```

- `producer.dst` feeds `consumer.src[src_slot]`.
- Producer MUST have `has_dst == True`.
- A node may have multiple incoming edges (multi-source) and multiple outgoing
  edges (fan-out).  PhysRegFile reader tracking handles this.

### DagBlock — a complete dataflow subgraph

```
DagBlock(name="mula2_kernel")
  ├── nodes: List[DagNode]
  ├── edges: List[DagEdge]
  └── linearize() → List[InstrGroup]
```

`linearize()` runs:

1. **Topological sort** — Kahn's algorithm on (nodes, edges).
2. **Register allocation** — assign arch registers to producer dsts and
   consumer srcs.  Arch regs numbered 0–31.  Source slots without incoming
   edges fall back to const pool (r28–31).  Dst slots without outgoing
   edges allocate from a per-block working pool.
3. **Schedule** — for each node in topo order, compute earliest issue cycle:
   ```
   ready = 0
   for edge in incoming:
       ready = max(ready, edge.producer.issue_cy + edge.producer.depth)
   issue_cy = max(ready, next_free_slot(node.port))
   ```
   where `next_free_slot(port)` tracks the last-used cycle for that queue
   (single-issue-per-queue constraint).
4. **Merge** — nodes sharing the same `issue_cy` are bundled into one
   `InstrGroup`.  The MULA/LN hardware interlock is enforced: if a bundle
   contains both mula and ln/exp, the ln/exp is deferred by one cycle.
5. **Idle padding** — cycles with no scheduled instructions become
   `InstrGroup()` (all ports empty).

Constraints:
- Each issue queue (exq0/exq1/lnq/ldq/stq) can issue at most **1** instruction per cycle.
- Different queues may issue concurrently.
- Producer-consumer spacing = producer pipeline depth (not depth+1 — the
  instruction at stage `depth` in the next cycle completes WB).

---

## Two Scenarios

### Scenario 1 — LD/ST outside the for-loop

```
_preload:  LD×2                           # 2 cycles
           idle × ld_depth                # wait for LD completion
_loop:     for _ in range(iterations):
               compute_only_body          # 3-10 compute instrs, ~8cy
_post:     ST×2                           # 2 cycles
```

- Preload LD dsts → compute body edges.
- Compute body dsts → post ST edges.
- LD/ST are OUTSIDE the loop; loop body is pure compute with true RAW
  dependencies on loaded registers.

### Scenario 2 — LD/ST inside the for-loop

```
_loop:     for _ in range(iterations):
               LD × n                     # 1-2 LD
               idle × ld_depth            # wait
               compute_body               # 3-8 compute instrs
               ST × n                     # 1-2 ST
```

- One `DagBlock` contains LD → compute → ST chain.
- `linearize()` emits the entire block including idle gaps.
- ~20% of instructions are LD/ST (by count), consistent with typical
  load-compute-store kernels.

---

## API surface

```python
b = DagBlock(name="example")

# Factory methods — return DagNode
n = b.mula()   # → EXQ0:MULA, 3 src, has_dst
n = b.mul()    # → EXQ0:MUL, 2 src, has_dst
n = b.add()    # → EXQ0:ADD, 2 src, has_dst
n = b.ln()     # → LNQ:LN, 1 src, has_dst
n = b.ld()     # → LDQ:LD, 0 src, has_dst
n = b.st(src_node=None)  # → STQ:ST, 1 src (from edge), no dst
# Node created on `b.default_port()`, which round-robins EXQ0→EXQ1 for ops
# legal on both ports.  LDQ/LNQ/STQ are port-fixed.

# Edge creation
b.edge(producer, consumer, src_slot=0)

# Linearize
seq: list[InstrGroup] = b.linearize()
```

The `_for_loop(body_gen, body_len, iterations)` helper is unchanged — it
repeats `body_gen(body_len)` `iterations` times.  For scenario 1 the preload
and post blobs sit outside the loop; the body is a `DagBlock` computed once
and replayed.

---

## Register allocation strategy

- **Arch registers 0–23**: dynamic pool shared within a DagBlock.  Producers
  allocate dst from this pool (free list, FIFO recycle within the block).
- **Arch registers 28–31**: const pool, never written.  Used for source slots
  with no incoming edge (e.g. `MULA dst, src0_from_LD, const1, const2`).
- **Arch registers 24–27**: reserved for cross-block edges (scenario 1:
  preload LD dst → loop body src → post ST src).  This ensures the arch
  registers survive across `DagBlock` boundaries.

---

## Benchmark integration

Each of the 10 benchmarks is rewritten so every function contains both
scenario 1 and scenario 2 for-loops (roughly 1:1 ratio), with LD:compute:ST
proportions consistent with real code.

The `_make_func(loops, total_cy)` helper distributes cycles across loops as
before. Each loop body is now one of:

| Type | Content | Body length | LD/ST% |
|------|---------|-------------|--------|
| S1-light | LD×2 → MULA×2 → ST×2 (LD/ST outside loop, pure compute inside) | 8 cy | 0% (in loop) |
| S1-heavy | LD×3 → MAL → ST×3 (LD/ST outside loop) | 5 cy | 0% (in loop) |
| S2-light | LD×1 → MULA(dep)→ ST×1 (inside loop) | ~22 cy | ~10% |
| S2-heavy | LD×2 → MULA×2→ST×2 (inside loop) | ~24 cy | ~17% |

All compute instructions consume loaded registers (with true RAW edges) or
const-pool fallback for the remaining src slots (modelling immediate/constant
operands).

---

## What does NOT change

- `PipelineModel`, `PhysRegFile`, `InstrGroup` — no changes.
- `ThrottleController`, `PDNModel`, `PDNObserver` — no changes.
- `_for_loop()`, `_make_func()`, `_task()`, `_build_multitask()` —
  API unchanged, they just receive `DagBlock`-generated sequences.
- `run_sim()`, `run_all()` — no changes.

---

## Expected effects

- **Lower IPC**: true RAW dependencies force pipeline bubbles.
- **Different token pattern**: LD (0 tok), ST (1 tok) mix with compute
  (2-3 tok), changing PDN excitation.
- **Controller re-tuning likely**: emergency/soft-ceiling thresholds may need
  adjustment for the new workload patterns.
- **BM4 unchanged in spirit**: the serial RAW chain already models true
  dependencies; ST is added but the fundamental character stays.

---

## Self-review

- **Placeholders**: none.
- **Internal consistency**: DagBlock scheduling respects single-issue
  constraint and MULA/LN interlock.  Cross-block registers use fixed pool (24–27).
- **Scope**: focused on instruction generation only — no pipeline/controller/PDN
  changes.
- **Ambiguity**: "round-robin EXQ0/EXQ1" — explicitly: `default_port()` picks
  the port with the earlier free slot, preferring EXQ0 on tie.
