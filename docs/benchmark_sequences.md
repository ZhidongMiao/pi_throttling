# Benchmark Instruction Sequences

> Generated from `StimulusGenerator` at 2000 cycles, seed=42.

## Instruction Set

| Op   | Token | Pipe Depth | Issue Ports |
|------|-------|------------|-------------|
| MULA | 3     | 9          | EXQ0, EXQ1  |
| MUL  | 2     | 8          | EXQ0, EXQ1  |
| ADD  | 2     | 8          | EXQ0, EXQ1  |
| LN   | 4     | 14         | LNQ         |
| EXP  | 4     | 14         | LNQ         |
| MOV  | 1     | 6          | EXQ0, EXQ1  |
| LD   | 0     | 10         | LDQ         |
| ST   | 1     | 4          | STQ         |

**Constraints:** MULA + LN/EXP cannot issue in the same cycle (hardware interlock).

## Register File Architecture

```
Arch Regs: 0–31 (32 registers, ISA-visible)
Phys Regs: 0–99 (100 registers, OOO backend)
Rename Pool: 32–99 (68 rename registers)

Pool layout per primitive:
  Working pool:   base..base+count-1  (dst rotates, may create WAW hazards)
  Const pool:     28–31               (src only, never written → always ready)
```

| RegState | Base | Count | Purpose |
|----------|------|-------|---------|
| DEFAULT | 0 | 8 | MULA/MUL/ADD dst rotation |
| DEFAULT | 28 | 4 | Const src pool (never written) |
| `ChainRegState` | 0 | 4 | RAW chain dst (src0 = prev dst) |

Each primitive creates its own `RegState` instances, isolating register namespaces per instruction stream. Within a stream, dsts rotate through the working pool; srcs always come from the const pool (regs 28–31) unless a RAW chain is explicitly constructed.

## Primitive Generators

| Generator | Instruction Pattern | Token/cy | Register Deps |
|-----------|---------------------|----------|---------------|
| `_mula2`  | EXQ0:MULA + EXQ1:MULA | 6 | EXQ0 pool r0–7, EXQ1 pool r8–15, independent |
| `_mal`    | EXQ0:MUL + EXQ1:ADD + LNQ:LN | 8 | MUL r0–7, ADD r8–11, LN r12–15, independent |
| `_muladd` | EXQ0:MUL + EXQ1:ADD | 4 | MUL r0–7, ADD r8–11, independent |
| `_mula1`  | EXQ0:MULA | 3 | Pool r0–7, independent |
| `_mov`    | EXQ0:MOV | 1 | Pool r24–25, independent |
| `_ld_burst` | LD→idle×9→MULA×2×8 (repeat) | ~2.7 avg | **RAW**: LD dst(r16–17) → MULA src0 |
| `_serial_mula` | MULA→idle×4 (repeat) | 0.6 | **RAW chain**: dst(N) → src0(N+1), pool r0–3 |
| `_alternating` | MULA×2×chunk ↔ MAL×chunk | 6↔8 | Per-phase RegState (fresh each chunk) |
| `_rand_mixed` | Weighted random mix (see §BM8) | ~2.5 avg | 9 RegState pools, shared within pattern |

**Idle:** `_idle(n)` = n cycles of `InstrGroup()` (all ports empty, token=0).

**Dependency types:**
- **Independent:** srcs from const pool (r28–31), dst rotates in working pool — no RAW hazards between consecutive instructions
- **RAW chain:** `ChainRegState` makes src0 = previous instruction's dst — forces pipeline serialization
- **Cross-op RAW:** LD writes to register, subsequent MULA reads it (BM3 `_ld_burst`)

## Task Model

Each task:
- **Advance notice:** 300 idle cycles with `task_notice` countdown (300→1)
- **Functions:** 2–4 functions per task, each `func_n` cycles
- **Function gaps:** 80 idle cycles between functions

For multi-task benchmarks, task N+1's notice overlays on task N's tail (replaces idle slots).

---

## BM1 — mula_steady_state

**单Task×3Func: MULA×2稳态 (independent regs)**

```
func_n = max(100, (2000 - 300 - 160) // 3) = 513

EXQ0 pool r0–7, EXQ1 pool r8–15, const srcs r28–31. No RAW chains.
```

 0..299   notice 300 (idle, task_notice=300→1)
300..812   Func1: MULA×2 ×513  (EXQ0:MULA + EXQ1:MULA)
813..892   gap: idle ×80
893..1405  Func2: MULA×2 ×513
1406..1485 gap: idle ×80
1486..1998 Func3: MULA×2 ×513
1999       idle ×1
```

| Queue | Pattern | Total Instr |
|-------|---------|-------------|
| EXQ0  | idle×300, mula×513, idle×80, mula×513, idle×80, mula×513, idle×1 | 1539 MULA |
| EXQ1  | idle×300, mula×513, idle×80, mula×513, idle×80, mula×513, idle×1 | 1539 MULA |
| LNQ   | idle ×2000 | — |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 3078 instructions, 9234 tokens, max sustained load 6 tok/cy.
**Sim:** IPC 100%→98.0%, Vmin 782→831mV, Δdroop +49mV, throttled droop 78mV ✓.

---

## BM2 — mul_add_ln_steady

**单Task×3Func: MUL+ADD+LN最大负载**

```
func_n = max(100, (2000 - 300 - 160) // 3) = 513

 0..299   notice 300
300..812   Func1: MAL ×513  (EXQ0:MUL + EXQ1:ADD + LNQ:LN)
813..892   gap: idle ×80
893..1405  Func2: MAL ×513
1406..1485 gap: idle ×80
1486..1998 Func3: MAL ×513
1999       idle ×1
```

| Queue | Pattern | Total Instr |
|-------|---------|-------------|
| EXQ0  | idle×300, mul×513, idle×80, mul×513, idle×80, mul×513, idle×1 | 1539 MUL |
| EXQ1  | idle×300, add×513, idle×80, add×513, idle×80, add×513, idle×1 | 1539 ADD |
| LNQ   | idle×300, ln×513,  idle×80, ln×513,  idle×80, ln×513,  idle×1 | 1539 LN |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 4617 instructions, 12312 tokens, max sustained load 8 tok/cy (peak).
**Sim:** IPC 100%→98.4%, Vmin 740→845mV, Δdroop +105mV, throttled droop 64mV ✓.

---

## BM3 — ld_ex_kernel

**单Task×4Func: LD窗口+MULA×2爆发 (LD→MULA RAW)**

```
func_n = max(80, (2000 - 300 - 240) // 4) = 365

Each func: LD-burst pattern repeating, RegState pools MULA0(0,8) MULA1(8,8) LD(16,2)
  LD×1 → idle×9 → MULA×2×8 → ...
  = 18 cycles/block: 1 LD + 9 idle + 8 MULA×2

Register deps (per 18-cycle block):
  LD  dst=r16              ← LD writes to ld pool
  idle ×9                  ← LD depth=10, completes during idle window
  MULA×2×8:
    EXQ0 src0=r16          ← RAW from LD dst (r16), const srcs for rest
    EXQ1 srcs=const pool   ← independent

 0..299   notice 300
300..664   Func1: LD-burst ×365 (20 full blocks = 360cy)
665..744   gap: idle ×80
745..1109  Func2: LD-burst ×365
1110..1189 gap: idle ×80
1190..1554 Func3: LD-burst ×365
1555..1634 gap: idle ×80
1635..1999 Func4: LD-burst ×365
```

| Queue | Pattern (per 18-cycle block) | Total Instr |
|-------|------------------------------|-------------|
| EXQ0  | idle×10, mula×8             | 640 MULA per func (×4) |
| EXQ1  | idle×10, mula×8             | 640 MULA per func (×4) |
| LNQ   | idle ×2000 | — |
| LDQ   | ld×1, idle×17 (×20 blocks)  | 21 LD per func (×4) |
| STQ   | idle ×2000 | — |

**Stats:** 1364 instructions (1280 MULA + 84 LD), 3840 tokens.
**Sim:** IPC 100%→100%, Vmin 848→864mV, Δdroop +17mV, throttled droop 45mV ✓.

---

## BM4 — serial_dependency

**单Task×4Func: 串行MULA依赖链 (RAW chain)**

```
func_n = max(60, (2000 - 300 - 240) // 4) = 365

Each func: one MULA every 5 cycles, ChainRegState(0, 4)
  MULA×1 → idle×4 → MULA×1 → idle×4 → ...
  = 73 MULA per func (365/5)

RAW chain (src0 = previous dst):
  cy300: r0  = mula(r28, r29, r30)   ← first, src0 from const pool
  cy305: r1  = mula(r0,  r31, r28)   ← RAW on r0 (MULA depth=9, issue gap=5 → STALL)
  cy310: r2  = mula(r1,  r29, r30)   ← RAW on r1
  cy315: r3  = mula(r2,  r31, r28)   ← RAW on r2
  cy320: r0  = mula(r3,  r29, r30)   ← pool wraps, RAW on r3
  ...

Pipeline behavior:
  - MULA0 issued @ cy300, completes WB @ cy309 (depth 9)
  - MULA1 decoded @ cy305, src=r0 not ready until cy309 → waits in EXQ
  - MULA1 issues @ cy310 (5 cycles of bubble)
  - IPC drops to ~79% (was 100% without register deps)

 0..299   notice 300
300..664   Func1: MULA/idle×73 blocks
665..744   gap: idle ×80
745..1109  Func2: MULA/idle×73 blocks
1110..1189 gap: idle ×80
1190..1554 Func3: MULA/idle×73 blocks
1555..1634 gap: idle ×80
1635..1999 Func4: MULA/idle×73 blocks
```

| Queue | Pattern | Total Instr |
|-------|---------|-------------|
| EXQ0  | (mula×1, idle×4)×73 per func, chained r0→r1→r2→r3→r0 | 73 MULA per func (×4 = 292) |
| EXQ1  | idle ×2000 | — |
| LNQ   | idle ×2000 | — |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 292 instructions, 876 tokens. IPC ~79% due to RAW chain serialization. Tests pipeline dependency stalls under throttle.
**Sim:** IPC 89%→89%, Vmin 902→874mV, Δdroop −28mV, throttled droop 35mV ✓.

---

## BM5 — mula_vs_mul_add_ln

**单Task×4Func: MULA×2↔MAL交替，80cy间隔**

```
func_n = max(100, (2000 - 300 - 240) // 4) = 365

 0..299   notice 300
300..664   Func1: MULA×2 ×365
665..744   gap: idle ×80
745..1109  Func2: MAL ×365
1110..1189 gap: idle ×80
1190..1554 Func3: MULA×2 ×365
1555..1634 gap: idle ×80
1635..1999 Func4: MAL ×365
```

| Queue | Pattern | Total Instr |
|-------|---------|-------------|
| EXQ0  | idle×300, mula×365, idle×80, mul×365, idle×80, mula×365, idle×80, mul×365 | 730 MULA + 730 MUL |
| EXQ1  | idle×300, mula×365, idle×80, add×365, idle×80, mula×365, idle×80, add×365 | 730 MULA + 730 ADD |
| LNQ   | idle×745, ln×365, idle×525, ln×365 | 730 LN |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 3650 instructions, 10220 tokens. Tests load-type switching (MULA×2 ↔ MAL).
**Sim:** IPC 100%→92.6%, Vmin 773→834mV, Δdroop +62mV, throttled droop 75mV ✓.

---

## BM6 — ln_dominated

**单Task×2Func: LN主导持续负载**

```
func_n = max(200, (2000 - 300 - 80) // 2) = 810

 0..299   notice 300
300..1109 Func1: MAL ×810
1110..1189 gap: idle ×80
1190..1999 Func2: MAL ×810
```

| Queue | Pattern | Total Instr |
|-------|---------|-------------|
| EXQ0  | idle×300, mul×810, idle×80, mul×810 | 1620 MUL |
| EXQ1  | idle×300, add×810, idle×80, add×810 | 1620 ADD |
| LNQ   | idle×300, ln×810,  idle×80, ln×810  | 1620 LN |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 4860 instructions, 12960 tokens. Longest sustained max-load (810 cycles). Tests PI regulator under prolonged stress.
**Sim:** IPC 100%→92.9%, Vmin 740→845mV, Δdroop +105mV, throttled droop 64mV ✓.

---

## BM7 — sw_resonance

**单Task×4Func: SW谐振 — MULA×2 365cy on / 80cy off**

```
func_n = max(100, (2000 - 300 - 240) // 4) = 365

Same pattern as BM1 (all MULA×2), tests resonance behaviour
at different burst/recovery cadences (4 bursts × 365cy).
```

| Queue | Pattern | Total Instr |
|-------|---------|-------------|
| EXQ0  | idle×300, mula×365, idle×80, mula×365, idle×80, mula×365, idle×80, mula×365, idle×1 | 1460 MULA |
| EXQ1  | same as EXQ0 | 1460 MULA |
| LNQ   | idle ×2000 | — |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 2920 instructions, 8760 tokens. Tests on/off resonance — designed to excite PDN ringing.
**Sim:** IPC 100%→99.6%, Vmin 782→832mV, Δdroop +50mV, throttled droop 77mV ✓.

---

## BM8 — ooo_mixed

**双Task×2Func: OOO混合负载，每Task 300cy通知**

```
func_n1 = max(120, (1000 - 300 - 80) // 2) = 310

Task1:
  0..299   Task1 notice 300
300..609   Task1 Func1: rand_mixed ×310
610..689   gap: idle ×80
690..999   Task1 Func2: rand_mixed ×310

Task2 (notice overlays on Task1 tail):
  ~700..999 Task2 notice (overlays idle slots of Task1)
1000..1309  Task2 Func1: rand_mixed ×310
1310..1389  gap: idle ×80
1390..1699  Task2 Func2: rand_mixed ×310
1700..1999  idle ×300
```

**Rand_mixed distribution (seed=42):**

| Pattern | Weight | Token |
|---------|--------|-------|
| MULA×2 | 20% | 6 |
| MUL+ADD+LN | 15% | 8 |
| MULA | 10% | 3 |
| LD | 8% | 0 |
| MULA×2+LD | 10% | 6 |
| MUL+ADD | 12% | 4 |
| LN | 8% | 4 |
| MOV | 5% | 1 |
| Idle | 12% | 0 |

Each pattern run for 3-30 cycles (random). Weighted average ~2.5 tok/cy.

**Stats:** 2006 instructions, 4950 tokens. Tests random mixed OOO workload with pipelining.
**Sim:** IPC 100%→99.8%, Vmin 758→840mV, Δdroop +82mV, throttled droop 69mV ✓.

---

## BM9 — multi_task_4

**4Task×2Func: MULA→MAL→LD→串行MULA，每Task 300cy通知**

```
func_n = max(80, (2000/4 - 300 - 80) // 2) = max(80, 60//2) = 80

Task1 (MULA×2):
  0..299   notice 300
300..379   Func1: MULA×2 ×80
380..459   gap: idle ×80
460..539   Func2: MULA×2 ×80

Task2 (MAL) — notice overlays late Task1:
  ~240..539 notice (overlays)
540..619   Func1: MAL ×80
620..699   gap: idle ×80
700..779   Func2: MAL ×80

Task3 (LD-burst) — notice overlays late Task2:
  ~480..779 notice (overlays)
780..859   Func1: LD-burst ×80
860..939   gap: idle ×80
940..1019  Func2: LD-burst ×80

Task4 (Serial MULA) — notice overlays late Task3:
  ~720..1019 notice (overlays)
1020..1099 Func1: Serial MULA ×80
1100..1179 gap: idle ×80
1180..1259 Func2: Serial MULA ×80
1260..1999 idle ×740
```

| Queue | Total Instr |
|-------|-------------|
| EXQ0  | 160 MULA (Task1) + 160 MUL (Task2) + 72 MULA (Task3 LD-burst) + 32 MULA (Task4) = 424 |
| EXQ1  | 160 MULA (Task1) + 160 ADD (Task2) + 72 MULA (Task3) = 392 |
| LNQ   | 160 LN (Task2) |
| LDQ   | 10 LD (Task3) |
| STQ   | — |

**Stats:** 954 instructions, 2672 tokens. Tests 4-way task pipeline with heterogeneous task types.
**Sim:** IPC 99.2%→99.2%, Vmin 752→849mV, Δdroop +97mV, throttled droop 60mV ✓.

---

## BM10 — multi_task_5

**5Task×2Func: MAL→交替→随机→LD+MULA→MOV，每Task 300cy通知**

```
func_n = max(80, (2000/5 - 300 - 80) // 2) = max(80, 10) = 80

Task1 (MAL):        0..299 notice, 300..459 Func1+Func2
Task2 (Alternating): ~160..539 notice overlay, 540..699 Func1+Func2
Task3 (Random):      ~320..699 notice overlay, 700..859 Func1+Func2
Task4 (LD-burst):    ~480..859 notice overlay, 860..1019 Func1+Func2
Task5 (MOV):         ~640..1019 notice overlay, 1020..1179 Func1+Func2
                    1180..1999 idle ×820
```

| Queue | Total Instr |
|-------|-------------|
| EXQ0  | 160 MUL (T1) + 50 MULA+30 MUL (T2) + rand(T3) + 72 MULA (T4) + 36 MOV (T5) |
| EXQ1  | 160 ADD (T1) + 50 MULA+30 ADD (T2) + rand(T3) + 72 MULA (T4) |
| LNQ   | 160 LN (T1) + 30 LN (T2) + rand(T3) |
| LDQ   | rand(T3) + 10 LD (T4) |
| STQ   | rand(T3) |

**Stats:** 1215 instructions, 3262 tokens. Tests 5-way task pipeline with 5 different task types.
**Sim:** IPC 100%→100%, Vmin 748→853mV, Δdroop +105mV, throttled droop 57mV ✓.

---

## Summary

| BM  | Tasks | Fn | Pattern | Deps | Instr | Tokens | Tok/cy | Peak | IPC(off→on) | Vmin(off→on) | Δdroop |
|-----|-------|----|---------|------|-------|--------|--------|------|-------------|---------------|--------|
| BM1 | 1     | 3  | MULA×2 steady | None | 3078 | 9234 | 4.62 | 6 | 100%→98.0% | 782→831mV | **+49mV** |
| BM2 | 1     | 3  | MAX load | None | 4617 | 12312 | 6.16 | 8 | 100%→98.4% | 740→845mV | **+105mV** |
| BM3 | 1     | 4  | LD-burst | LD→MULA RAW | 1364 | 3840 | 1.92 | 6 | 100%→100% | 848→864mV | +17mV |
| BM4 | 1     | 4  | Serial MULA | **RAW chain** | 292 | 876 | 0.44 | 3 | 89%→89% | 902→874mV | −28mV |
| BM5 | 1     | 4  | MULA↔MAL交替 | None (per-phase) | 3650 | 10220 | 5.11 | 8 | 100%→92.6% | 773→834mV | **+62mV** |
| BM6 | 1     | 2  | LN主导长跑 | None | 4860 | 12960 | 6.48 | 8 | 100%→92.9% | 740→845mV | **+105mV** |
| BM7 | 1     | 4  | SW谐振 | None | 2920 | 8760 | 4.38 | 6 | 100%→99.6% | 782→832mV | **+50mV** |
| BM8 | 2     | 2×2 | OOO混合 | Mixed | 2006 | 4950 | 2.48 | 8 | 100%→99.8% | 758→840mV | **+82mV** |
| BM9 | 4     | 2×4 | 4路流水线 | T4: chain | 954 | 2672 | 1.34 | 8 | 99.2%→99.2% | 752→849mV | **+97mV** |
| BM10| 5     | 2×5 | 5路流水线 | Per-task | 1215 | 3262 | 1.63 | 8 | 100%→100% | 748→853mV | **+105mV** |

**Deps:** "None" = independent (const-pool srcs, no RAW). BM4 is the only benchmark with a strict RAW chain per instruction.

### Key Observations

- **10/10 benchmarks achieve droop < 80mV** (goal met). Worst-case throttled droop: BM1 at 78mV.
- **IPC improvement vs old v3:** BM2 +8.5pp (89.9→98.4%), BM6 +8.0pp (84.9→92.9%), BM7 +5.2pp (94.4→99.6%).
- **BM4 RAW chain:** baseline IPC 89% confirms register dependency stalls in pipeline model. Throttled droop 35mV (controller dummy injection on naturally-serialized pipeline — minor, well within limit).
- **Protection strategy:** Soft ceiling at 40mV observer droop caps credit→2 as primary governor; emergency brake at 65mV provides hard floor. PI target 68mV is aspirational — reached only during low-load/recovery periods for IPC maximisation.
