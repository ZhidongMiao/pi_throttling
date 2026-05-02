# Warm Window Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add warm_window dummy injection to prevent controller leaving REGULATE during short idle gaps, boosting intermittent-workload IPC.

**Architecture:** Single-file change to `pdn_sim3.py`. Add `warm_post_cycles` to `ThrottleParams`, add `warm_window` signal and `warm_post_timer` to `ThrottleController`, unify `f_busy` to use `warm_window`, and extend dummy injection from `ld_window`-only to `warm_window`.

**Tech Stack:** Python 3, numpy (existing)

---

### Task 1: Add warm_post_cycles to ThrottleParams

**Files:**
- Modify: `pdn_sim3.py:230-237`

- [ ] **Step 1: Add parameter**

In `pdn_sim3.py`, after the Queue lookahead block (line 237), insert:

```python
    # ── Warm-window dummy (pre/post busy) ──────────────────────────
    warm_post_cycles: int = 50   # busy→idle 后 dummy 保温周期
```

- [ ] **Step 2: Verify syntax**

Run: `python3 -c "from pdn_sim3 import ThrottleParams; p = ThrottleParams(); print(p.warm_post_cycles)"`
Expected: `50`

---

### Task 2: Add warm_window signal and timer to ThrottleController

**Files:**
- Modify: `pdn_sim3.py:259-278` (_reset), `pdn_sim3.py:322-457` (step)
- Modify: `pdn_sim3.py:245` (class docstring only, optional)

- [ ] **Step 1: Add warm_post_timer to _reset()**

In `_reset()`, after the LD/queue init (line 273), add:

```python
        # warm window
        self.warm_post_timer = 0;  self.prev_f_busy_nat = False
```

Full updated `_reset()` section (lines 272-278):

```python
        # LD / queue
        self.ld_timer = 0;  self.ld_window = False;  self.queue_busy = False
        # warm window
        self.warm_post_timer = 0;  self.prev_f_busy_nat = False
        # logs
```

- [ ] **Step 2: Verify _reset() syntax**

Run: `python3 -c "from pdn_sim3 import ThrottleController; c = ThrottleController(); print(c.warm_post_timer, c.prev_f_busy_nat)"`
Expected: `0 False`

- [ ] **Step 3: Add warm_window computation and post-warm timer in step()**

In `step()`, replace the `f_busy` line (342) and add warm_window + post-warm logic. The old block:

```python
        self.queue_busy = heavy_queued if self.p.queue_lookahead else False
        f_busy = isu_valid or ex_busy or self.ld_window or self.queue_busy
```

Replace with:

```python
        self.queue_busy = heavy_queued if self.p.queue_lookahead else False

        # ── Warm window (pre-warm + post-warm dummy保温) ──────────
        f_busy_nat = isu_valid or ex_busy             # natural busy (no dummy)

        if f_busy_nat:
            self.warm_post_timer = 0                   # reset on real activity
        elif self.warm_post_timer > 0:
            self.warm_post_timer -= 1                  # count down
        elif not f_busy_nat and self.prev_f_busy_nat:
            self.warm_post_timer = self.p.warm_post_cycles   # start post-warm

        self.warm_window = (self.ld_window or
                            early_wu or
                            self.queue_busy or
                            (self.warm_post_timer > 0))
        f_busy = f_busy_nat or self.warm_window
        self.prev_f_busy_nat = f_busy_nat
```

- [ ] **Step 4: Update log for warm_window**

Change the ld_window log line (455) to log `self.warm_window` instead:

```python
        self.log_ld_window.append(self.warm_window)
```

This way the dashboard's "LD Window" trace will show the full warm_window coverage.

- [ ] **Step 5: Verify warm_window in isolation**

Run: `python3 -c "
from pdn_sim3 import ThrottleController, make_default_params
c = ThrottleController(make_default_params(), True)
# Simulate busy→idle transition
cr, d = c.step(False, True, True, False, 8, True, ld_issued=False, heavy_queued=False)
print('step1 f_busy_nat=True, warm_post=', c.warm_post_timer)
cr, d = c.step(False, False, False, False, 0, True, ld_issued=False, heavy_queued=False)
print('step2 f_busy_nat=False, warm_post=', c.warm_post_timer, 'warm_window=', c.warm_window)
print('dummy=', d)
"`

Expected: step1 warm_post=0, step2 warm_post=50, warm_window=True, dummy=3

---

### Task 3: Extend dummy injection to full warm_window

**Files:**
- Modify: `pdn_sim3.py:440-443` (dummy injection block)

- [ ] **Step 1: Change dummy condition from ld_window to warm_window**

Replace:

```python
        # ── Dummy injection (LD-window) ────────────────────────────
        dummy = 0
        if self.p.ld_aware_enabled and self.ld_window:
            dummy = max(dummy, self.p.ld_dummy_token)
```

With:

```python
        # ── Dummy injection (warm_window: LD + pre-warm + post-warm) ─
        dummy = 0
        if self.p.ld_aware_enabled and self.warm_window:
            dummy = max(dummy, self.p.ld_dummy_token)
```

- [ ] **Step 2: Verify dummy fires on warm_window**

Run: `python3 -c "
from pdn_sim3 import ThrottleController, make_default_params
c = ThrottleController(make_default_params(), True)
# Go through busy->idle to trigger warm_post
c.step(False, True, True, False, 8, True)
c.step(False, False, False, False, 0, True)
cr, d = c.step(False, False, False, False, 0, True)
print('warm_post_timer=', c.warm_post_timer, 'dummy=', d)
"`

Expected: warm_post_timer=48 or 49, dummy=3

---

### Task 4: Update RAMPDN→RAMP recovery to use warm_window

**Files:**
- Modify: `pdn_sim3.py:416`

- [ ] **Step 1: Extend RAMPDN recovery trigger**

Replace line 416:

```python
            if (isu_valid or self.ld_window or self.queue_busy) and self.rd_timer > 10:
```

With:

```python
            if (isu_valid or self.warm_window) and self.rd_timer > 10:
```

This ensures warm_window triggers recovery from RAMPDN back to RAMP, not just ld_window/queue_busy.

---

### Task 5: Update REGULATE→RAMPDN gate to use warm_window

**Files:**
- Modify: `pdn_sim3.py:379`

- [ ] **Step 1: Gate RAMPDN entry on warm_window**

Replace line 379:

```python
            if rd_trig and not self.ld_window and not self.queue_busy:
```

With:

```python
            if rd_trig and not self.warm_window:
```

This prevents entering RAMPDN while warm_window is active.

---

### Task 6: Update IDLE→RAMP transition to use warm_window

**Files:**
- Modify: `pdn_sim3.py:349`

- [ ] **Step 1: Use warm_window as IDLE wake-up**

Replace line 349:

```python
            if early_wu or self.queue_busy or self.ld_window:
```

With:

```python
            if self.warm_window:
```

(This is equivalent since warm_window already includes early_wu, queue_busy, ld_window.)

---

### Task 7: Run full simulation and verify results

- [ ] **Step 1: Run simulation**

Run: `source .venv/bin/activate && python3 run_sim.py --cycles 700`
Expected: All benchmarks complete, no crashes.

- [ ] **Step 2: Check droop constraint**

All benchmarks must have max_droop ≤ 51mV (Throttled column).

- [ ] **Step 3: Check IPC improvement for intermittent workloads**

BM7 and BM8 IPC should be ≥ previous baseline (BM7 ≈ 24%, BM8 ≈ 48%).

- [ ] **Step 4: Check steady-state IPC not degraded**

BM1/2/5/6 IPC should be ≥ previous baseline.

- [ ] **Step 5: Check no stuck states**

Run: `python3 -c "
import json
with open('sim_results.json') as f:
    d = json.load(f)
for bm_id, bm in d['data'].items():
    states = set(bm['on']['fsm_state'])
    print(f'{bm_id}: states={states}')
"`

Expected: All benchmarks contain state 2 (REGULATE). No benchmark should spend entire time in a single state.
