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
- **For-loops per function:** 2–5 loops, each iterating 10–50× with body 5–30 cycles
  - `_for_loop(body_gen, body_len, iterations)` — repeats `body_gen(body_len)` `iterations` times
  - `_make_func(loops, total_cy)` — distributes iterations across loops to fill `total_cy`
  - Each loop iteration creates fresh `RegState`, modelling independent loop bodies

For multi-task benchmarks, task N+1's notice overlays on task N's tail (replaces idle slots).

---

## BM1 — mula_steady_state

**单Task×3Func: MULA×2稳态 (independent regs), 每Func 3个for-loop**

```
func_n = max(100, (2000 - 300 - 160) // 3) = 513

Each func: 3 for-loops of _mula2(body_len=8), ~21 iterations each → ~504cy total
  [MULA×2 ×8] ×21  →  [MULA×2 ×8] ×21  →  [MULA×2 ×8] ×22

0..299   notice 300 (idle, task_notice=300→1)
300..812   Func1: MULA×2 for-loops (~512cy)
813..892   gap: idle ×80
893..1405  Func2: MULA×2 for-loops (~512cy)
1406..1485 gap: idle ×80
1486..1998 Func3: MULA×2 for-loops (~512cy)
1999       idle ×1
```

| Queue | Pattern | Total Ops |
|-------|---------|-------------|
| EXQ0  | MULA ×1536 | 1536 MULA |
| EXQ1  | MULA ×1536 | 1536 MULA |
| LNQ   | idle ×2000 | — |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 3072 ops, 9216 tokens, max sustained load 6 tok/cy.
**Sim:** IPC 100%→97.7%, Vmin 782→836mV, Δdroop +54mV, throttled droop 73mV ✓.

---

## BM2 — mul_add_ln_steady

**单Task×3Func: MUL+ADD+LN最大负载, 每Func 3个for-loop**

```
func_n = max(100, (2000 - 300 - 160) // 3) = 513

Each func: 3 for-loops of _mal(body_len=5), ~34 iterations each → ~510cy total
  [MAL ×5] ×34  →  [MAL ×5] ×34  →  [MAL ×5] ×34

0..299   notice 300
300..812   Func1: MAL for-loops (~510cy)
813..892   gap: idle ×80
893..1405  Func2: MAL for-loops (~510cy)
1406..1485 gap: idle ×80
1486..1998 Func3: MAL for-loops (~510cy)
1999       idle ×1
```

| Queue | Pattern | Total Ops |
|-------|---------|-------------|
| EXQ0  | MUL ×1530 | 1530 MUL |
| EXQ1  | ADD ×1530 | 1530 ADD |
| LNQ   | LN ×1530  | 1530 LN |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 4590 ops (1530 each MUL/ADD/LN), 12240 tokens, max sustained load 8 tok/cy.
**Sim:** IPC 100%→98.3%, Vmin 740→849mV, Δdroop +109mV, throttled droop 60mV ✓.

---

## BM3 — ld_ex_kernel

**单Task×4Func: LD窗口+MULA×2爆发 (LD→MULA RAW), 每Func 3个for-loop**

```
func_n = max(80, (2000 - 300 - 240) // 4) = 365

Each func: 3 for-loops of _ld_burst(body_len=18), ~20 iterations each → ~360cy
  LD-burst block = 18cy: LD×1 + idle×9 + MULA×2×8

Register deps (per 18-cycle block):
  LD  dst=r16              ← LD writes to ld pool
  idle ×9                  ← LD depth=10, completes during idle window
  MULA×2×8:
    EXQ0 src0=r16          ← RAW from LD dst (r16), const srcs for rest
    EXQ1 srcs=const pool   ← independent

 0..299   notice 300
300..664   Func1: LD-burst for-loops (~360cy)
665..744   gap: idle ×80
745..1109  Func2: LD-burst for-loops (~360cy)
1110..1189 gap: idle ×80
1190..1554 Func3: LD-burst for-loops (~360cy)
1555..1634 gap: idle ×80
1635..1999 Func4: LD-burst for-loops (~360cy)
```

| Queue | Pattern | Total Ops |
|-------|---------|-------------|
| EXQ0  | MULA ×640 (per func ×4) | 640 MULA |
| EXQ1  | MULA ×640 (per func ×4) | 640 MULA |
| LNQ   | idle ×2000 | — |
| LDQ   | LD ×21 (per func ×4) | 84 LD |
| STQ   | idle ×2000 | — |

**Stats:** 1364 ops (1280 MULA + 84 LD), 3840 tokens.
**Sim:** IPC 100%→100%, Vmin 848→866mV, Δdroop +18mV, throttled droop 43mV ✓.

---

## BM4 — serial_dependency

**单Task×4Func: 串行MULA依赖链 (RAW chain), 每Func 3个for-loop**

```
func_n = max(60, (2000 - 300 - 240) // 4) = 365

Each func: 3 for-loops of _serial_mula(body_len=5), ~24 iterations each → ~360cy
  serial_mula body: MULA×1 + idle×4 = 5cy per iteration
  ChainRegState resets per iteration (fresh RAW chain each loop body)

Pipeline behavior:
  - MULA0 issued @ cy300, completes WB @ cy309 (depth 9)
  - MULA1 decoded @ cy305, src=r0 not ready until cy309 → waits in EXQ
  - MULA1 issues @ cy310 (5 cycles of bubble)
  - IPC ~79% (was 100% without register deps)

 0..299   notice 300
300..664   Func1: serial-MULA for-loops (~360cy)
665..744   gap: idle ×80
745..1109  Func2: serial-MULA for-loops (~360cy)
1110..1189 gap: idle ×80
1190..1554 Func3: serial-MULA for-loops (~360cy)
1555..1634 gap: idle ×80
1635..1999 Func4: serial-MULA for-loops (~360cy)
```

| Queue | Pattern | Total Ops |
|-------|---------|-------------|
| EXQ0  | MULA per func: ~72 (×4 = 292) | 292 MULA |
| EXQ1  | idle ×2000 | — |
| LNQ   | idle ×2000 | — |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 292 ops, 876 tokens. IPC ~100% (dummy injection fills idle bubbles). Tests pipeline dependency stalls under throttle.
**Sim:** IPC 100%→100%, Vmin 896→874mV, Δdroop −22mV, throttled droop 35mV ✓.

---

## BM5 — mula_vs_mul_add_ln

**单Task×4Func: MULA×2↔MAL交替，80cy间隔, 每Func 3个for-loop**

```
func_n = max(100, (2000 - 300 - 240) // 4) = 365

Func1,3: 3 for-loops of _mula2(body_len=8), ~24 iterations each → ~384cy
Func2,4: 3 for-loops of _mal(body_len=5), ~24 iterations each → ~360cy

 0..299   notice 300
300..664   Func1: MULA×2 for-loops (~24×8=192cy, padded)
665..744   gap: idle ×80
745..1109  Func2: MAL for-loops (~24×5=120cy, padded)
1110..1189 gap: idle ×80
1190..1554 Func3: MULA×2 for-loops
1555..1634 gap: idle ×80
1635..1999 Func4: MAL for-loops
```

| Queue | Pattern | Total Ops |
|-------|---------|-------------|
| EXQ0  | MULA ~720 + MUL ~730 | 1440 MULA + 730 MUL |
| EXQ1  | MULA ~720 + ADD ~730 | 1440 MULA + 730 ADD |
| LNQ   | LN ~730 | 730 LN |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 3630 ops, 10160 tokens. Tests load-type switching (MULA×2 ↔ MAL).
**Sim:** IPC 100%→91.6%, Vmin 770→840mV, Δdroop +70mV, throttled droop 70mV ✓.

---

## BM6 — ln_dominated

**单Task×2Func: LN主导持续负载, 每Func 4个for-loop**

```
func_n = max(200, (2000 - 300 - 80) // 2) = 810

Each func: 4 for-loops of _mal(body_len=5), ~40 iterations each → ~800cy total
  [MAL ×5] ×40  ×4 loops

 0..299   notice 300
300..1109 Func1: MAL for-loops (~800cy)
1110..1189 gap: idle ×80
1190..1999 Func2: MAL for-loops (~800cy)
```

| Queue | Pattern | Total Ops |
|-------|---------|-------------|
| EXQ0  | MUL ×1620 | 1620 MUL |
| EXQ1  | ADD ×1620 | 1620 ADD |
| LNQ   | LN ×1620  | 1620 LN |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 4860 ops (1620 each MUL/ADD/LN), 12960 tokens. Longest sustained max-load (800 cy). Tests PI regulator under prolonged stress.
**Sim:** IPC 100%→92.7%, Vmin 740→849mV, Δdroop +109mV, throttled droop 60mV ✓.

---

## BM7 — sw_resonance

**单Task×4Func: SW谐振 — MULA×2 365cy on / 80cy off, 每Func 3个for-loop**

```
func_n = max(100, (2000 - 300 - 240) // 4) = 365

Each func: 3 for-loops of _mula2(body_len=8), ~24 iterations each → ~360cy

4 bursts × ~360cy MULA×2, each separated by 80cy idle.
Tests PDN resonance at different burst/recovery cadences.
```

| Queue | Pattern | Total Ops |
|-------|---------|-------------|
| EXQ0  | MULA ×1440 | 1440 MULA |
| EXQ1  | MULA ×1440 | 1440 MULA |
| LNQ   | idle ×2000 | — |
| LDQ   | idle ×2000 | — |
| STQ   | idle ×2000 | — |

**Stats:** 2880 ops, 8640 tokens. Tests on/off resonance — designed to excite PDN ringing.
**Sim:** IPC 100%→99.0%, Vmin 782→838mV, Δdroop +56mV, throttled droop 71mV ✓.

---

## BM8 — ooo_mixed

**双Task×2Func: OOO混合负载，每Task 300cy通知, 每Func 3个for-loop**

```
func_n1 = max(120, (1000 - 300 - 80) // 2) = 310

Each func: 3 for-loops of _rand_mixed(body_len=10), ~31 iterations each → ~310cy

Task1:
  0..299   Task1 notice 300
300..609   Task1 Func1: rand_mixed for-loops (~310cy)
610..689   gap: idle ×80
690..999   Task1 Func2: rand_mixed for-loops (~310cy)

Task2 (notice overlays on Task1 tail):
  ~700..999 Task2 notice (overlays idle slots of Task1)
1000..1309  Task2 Func1: rand_mixed for-loops (~310cy)
1310..1389  gap: idle ×80
1390..1699  Task2 Func2: rand_mixed for-loops (~310cy)
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

**Stats:** 2089 ops, 4987 tokens. Tests random mixed OOO workload with pipelining.
**Sim:** IPC 100%→97.5%, Vmin 758→834mV, Δdroop +76mV, throttled droop 75mV ✓.

---

## BM9 — multi_task_4

**4Task×2Func: MULA→MAL→LD→串行MULA，每Task 300cy通知, 每Func 2个for-loop**

```
func_n = max(80, (2000/4 - 300 - 80) // 2) = 80

Each func: 2 for-loops, body_len varies by task type → fills ~80cy

Task1 (MULA×2): 2 for-loops of _mula2(body_len=5), ~16 iter each
Task2 (MAL):     2 for-loops of _mal(body_len=5), ~16 iter each
Task3 (LD-burst): 2 for-loops of _ld_burst(body_len=18), ~10 iter each
Task4 (Serial):  2 for-loops of _serial_mula(body_len=5), ~16 iter each

Task1: 0..299 notice, 300..459 Func1+Func2
Task2: ~240..539 notice overlay, 540..699 Func1+Func2
Task3: ~480..779 notice overlay, 780..1019 Func1+Func2
Task4: ~720..1019 notice overlay, 1020..1259 Func1+Func2
          1260..1999 idle ×740
```

| Queue | Total Ops |
|-------|-------------|
| EXQ0  | 160 MULA (T1) + 160 MUL (T2) + 72 MULA (T3) + 32 MULA (T4) = 424 |
| EXQ1  | 160 MULA (T1) + 160 ADD (T2) + 72 MULA (T3) = 392 |
| LNQ   | 160 LN (T2) |
| LDQ   | 10 LD (T3) |
| STQ   | — |

**Stats:** 954 ops, 2672 tokens. Tests 4-way task pipeline with heterogeneous task types.
**Sim:** IPC 100%→100%, Vmin 752→852mV, Δdroop +100mV, throttled droop 57mV ✓.

---

## BM10 — multi_task_5

**5Task×2Func: MAL→交替→随机→LD+MULA→MOV，每Task 300cy通知, 每Func 2个for-loop**

```
func_n = max(80, (2000/5 - 300 - 80) // 2) = 80

Each func: 2 for-loops, body_len varies by task type → fills ~80cy

Task1 (MAL):         2 for-loops of _mal(body_len=5)
Task2 (Alternating): 2 for-loops of _alternating(body_len=20, chunk=50)
Task3 (Random):      2 for-loops of _rand_mixed(body_len=10)
Task4 (LD-burst):    2 for-loops of _ld_burst(body_len=18)
Task5 (MOV):         2 for-loops of _mov(body_len=2)

Task1: 0..299 notice, 300..459 Func1+Func2
Task2: ~160..539 notice overlay, 540..699 Func1+Func2
Task3: ~320..699 notice overlay, 700..859 Func1+Func2
Task4: ~480..859 notice overlay, 860..1019 Func1+Func2
Task5: ~640..1019 notice overlay, 1020..1179 Func1+Func2
      1180..1999 idle ×820
```

| Queue | Total Ops |
|-------|-------------|
| EXQ0  | 160 MUL (T1) + 40 MULA+20 MUL (T2) + rand(T3) + 72 MULA (T4) + 6 MOV (T5) |
| EXQ1  | 160 ADD (T1) + 40 MULA+20 ADD (T2) + rand(T3) + 72 MULA (T4) |
| LNQ   | 160 LN (T1) + 20 LN (T2) + rand(T3) |
| LDQ   | rand(T3) + 14 LD (T4) |
| STQ   | rand(T3) |

**Stats:** 1164 ops, 3191 tokens. Tests 5-way task pipeline with 5 different task types.
**Sim:** IPC 100%→100%, Vmin 748→845mV, Δdroop +97mV, throttled droop 65mV ✓.

---

## Summary

| BM  | Tasks | Fn | Loops | Pattern | Deps | Ops | Tokens | Tok/cy | Peak | IPC(off→on) | Vmin(off→on) | Δdroop |
|-----|-------|----|-------|---------|------|-----|--------|--------|------|-------------|---------------|--------|
| BM1 | 1     | 3  | 3×3   | MULA×2 steady | None | 3072 | 9216 | 4.61 | 6 | 100%→97.7% | 782→836mV | **+54mV** |
| BM2 | 1     | 3  | 3×3   | MAX load | None | 4590 | 12240 | 6.12 | 8 | 100%→98.3% | 740→849mV | **+109mV** |
| BM3 | 1     | 4  | 4×3   | LD-burst | LD→MULA RAW | 1364 | 3840 | 1.92 | 6 | 100%→100% | 848→866mV | +18mV |
| BM4 | 1     | 4  | 4×3   | Serial MULA | **RAW chain** | 292 | 876 | 0.44 | 3 | 100%→100% | 896→874mV | −22mV |
| BM5 | 1     | 4  | 4×3   | MULA↔MAL交替 | None (per-phase) | 3630 | 10160 | 5.08 | 8 | 100%→91.6% | 770→840mV | **+70mV** |
| BM6 | 1     | 2  | 2×4   | LN主导长跑 | None | 4860 | 12960 | 6.48 | 8 | 100%→92.7% | 740→849mV | **+109mV** |
| BM7 | 1     | 4  | 4×3   | SW谐振 | None | 2880 | 8640 | 4.32 | 6 | 100%→99.0% | 782→838mV | **+56mV** |
| BM8 | 2     | 2×2 | 2×2×3 | OOO混合 | Mixed | 2089 | 4987 | 2.49 | 8 | 100%→97.5% | 758→834mV | **+76mV** |
| BM9 | 4     | 2×4 | 4×2×2 | 4路流水线 | T4: chain | 954 | 2672 | 1.34 | 8 | 100%→100% | 752→852mV | **+100mV** |
| BM10| 5     | 2×5 | 5×2×2 | 5路流水线 | Per-task | 1164 | 3191 | 1.60 | 8 | 100%→100% | 748→845mV | **+97mV** |

**Deps:** "None" = independent (const-pool srcs, no RAW). BM4 is the only benchmark with a strict RAW chain per instruction.
**Loops column:** `T×F×L` = tasks × functions-per-task × loops-per-function.

### Key Observations

- **10/10 benchmarks achieve droop < 80mV** (goal met). Worst-case throttled droop: BM8 at 75mV.
- **For-loop structure:** Each function contains 2–5 for-loops, each iterating 10–50× with 5–30 cycle bodies. Loop iterations are independent (fresh RegState per iteration), modelling real-world loop-carried register renaming.
- **Protection strategy:** Soft ceiling at 38mV caps credit→2 as primary governor; emergency brake at 60mV with 50-cycle hold provides hard floor. Predictive rate limiter at 2.0 mV/cy catches rapid droop. PI target 68mV is aspirational — reached during low-load/recovery for IPC maximisation.
- **Credit scaling by workload width:** MULA×2 (2 instrs/cy) is less constrained by credit caps than MAL (3 instrs/cy) — credit=2 allows full MULA×2 throughput. This required lower emergency threshold (60mV) compared to pre-for-loop controller (65mV).
