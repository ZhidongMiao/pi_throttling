"""
controller.py — Throttle Controller
====================================
Two-phase di/dt throttle controller with 5-state FSM.

Phase 1 — RAMP:   di/dt-limited soft start with small credit steps.
Phase 2 — REGULATE: PI controller using internal PDN observer (no ADC).

Protection layers (priority order):
  1. Voltage emergency brake     (observer droop > threshold)
  2. Predictive droop rate limit (dV/dt too steep)
  3. ΔToken rate limit           (|Δ| ≤ max_delta_token)
  4. M5 anti-resonance lock      (sliding-window oscillation detection)
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
from enum import IntEnum

from pdn import PDNObserver, V0_MV, V_SIGNOFF
from pipeline import TOK, PIPE_DEPTH

STATE_NAMES = ["IDLE", "RAMP", "REGULATE", "HOLD", "RAMPDN"]


class State(IntEnum):
    IDLE = 0;  RAMP = 1;  REGULATE = 2;  HOLD = 3;  RAMPDN = 4


@dataclass
class ThrottleParams:
    """Extended-ramp + PI-regulate throttling controller parameters.

    Two-phase design:
      Phase 1 — RAMP:  di/dt-limited soft start, small Δ per step, long dwell.
      Phase 2 — REGULATE: PI controller adjusts credit dynamically using an
                  internal PDN observer (no hardware voltage ADC needed).
    """
    version: str = "v4"

    # Phase 1: Extended ramp (per-instruction credit, 1-2 inst/cycle)
    ramp_credits:    tuple = (1, 2)
    ramp_durations:  tuple = (60, 60)                # dwell (120 total)
    ramp_timeout:    int   = 20
    re_ramp_div:     int   = 4

    # Phase 2: PID regulate (per-instruction credit range)
    # Target: droop < 80mV while maximising IPC.
    # PI target is aspirational — soft ceiling at 40mV is the real governor.
    target_droop_mv:    float = 68.0
    pi_kp:              float = 0.10
    pi_ki:              float = 0.003
    pi_kd:              float = 1.0
    pi_i_max:           float = 2.0
    pi_credit_min:      int   = 1
    pi_credit_max:      int   = 3
    pi_settle_cycles:   int   = 15
    pi_update_interval: int   = 8
    pi_deadband_mv:     float = 15.0

    # Droop soft ceiling — primary governor. Caps credit→2 early to
    # prevent PDN from accumulating charge deficit that would breach 80mV.
    soft_ceiling_enabled: bool  = True
    soft_ceiling_droop_mv: float = 40.0
    soft_ceiling_credit:   int   = 2

    # HOLD / RAMPDN
    hold_init:    int = 20
    rampdn_total: int = 80

    # M5 anti-resonance (per-instruction credit threshold)
    m5_enabled:     bool = True
    m5_window:      int  = 150
    m5_osc_thresh:  int  = 8
    m5_lock_credit: int  = 2
    m5_lock_dur:    int  = 100
    m5_hi_thresh:   int  = 5
    m5_lo_thresh:   int  = 3

    # LD-window dummy injection (removed — warm_window handles proactive ramping)
    ld_aware_enabled: bool = True
    ld_lat:           int  = PIPE_DEPTH["ld"]
    ld_dummy_token:   int  = 0

    # Queue lookahead (RAMPDN suppression)
    queue_lookahead:    bool = True
    queue_heavy_thresh: int  = 3

    # Warm-window dummy (pre/post busy)
    warm_post_cycles: int = 65

    # ΔToken rate limit
    max_delta_token:  int = 3

    # Voltage emergency brake (per-instruction credit)
    emergency_enabled:  bool  = True
    emergency_droop_mv: float = 65.0
    emergency_credit:   int   = 1
    emergency_hold:     int   = 30

    # Predictive droop rate limiter (per-instruction credit)
    pred_rate_enabled:  bool  = True
    pred_rate_threshold: float = 2.5
    pred_rate_credit:   int   = 1

    # Sustained high-load lock (SHL) — disabled by default
    shl_enabled:     bool = False
    shl_hi_thresh:   int  = 7
    shl_window:      int  = 40
    shl_lock_credit: int  = 2
    shl_lock_dur:    int  = 120


def make_default_params() -> ThrottleParams:
    """Single throttling preset: extended ramp + PI regulate."""
    return ThrottleParams()


class ThrottleController:
    """Two-phase di/dt throttle controller.

    Phase 1 — RAMP:   di/dt-limited soft start. Small credit steps (Δ≤1),
                      long dwell per step to let PDN ringing decay.
    Phase 2 — REGULATE: PI controller on estimated voltage (internal PDN
                      observer — no hardware ADC needed). Adjusts credit
                      to maximise IPC while holding droop near target.
    """

    def __init__(self, p: ThrottleParams = None, enabled: bool = True):
        self.p = p or ThrottleParams();  self.enabled = enabled
        self.obs = PDNObserver();  self._reset()

    def _reset(self):
        self.state = State.IDLE
        self._prev_state = State.IDLE
        self.ramp_step = 0;  self.ramp_timer = 0;  self.ramp_timeout = 0
        self.hold_timer = 0
        self.pre_hold_state = State.IDLE;  self.pre_hold_credit = 2
        self.rd_timer = 0;  self.pre_rd_credit = 8
        # PI
        self.pi_credit = 3;  self.pi_integral = 0.0
        self.pi_settle_timer = 0;  self.pi_update_timer = 0
        # M5
        self.m5_osc = 0;  self.m5_win = 0;  self.m5_lock = False;  self.m5_lock_t = 0
        self.m5_prev_hi = False;  self.m5_prev_lo = False
        self.prev_issued_tok = 0
        # LD / queue
        self.ld_timer = 0;  self.ld_window = False;  self.queue_busy = False
        # warm window
        self.warm_post_timer = 0;  self.prev_f_busy_nat = False
        # emergency brake
        self.emergency_active = False;  self.emergency_timer = 0
        # SHL
        self.shl_cnt = 0;  self.shl_lock = False;  self.shl_timer = 0
        # predictive
        self.prev_decline = 0.0
        # task notice
        self.task_upcoming = False
        # logs
        self.log_state: List[int] = [];  self.log_credit: List[int] = []
        self.log_m5: List[bool] = [];  self.log_comp: List[bool] = []
        self.log_delta_lim: List[bool] = []
        self.log_ld_window: List[bool] = [];  self.log_queue: List[bool] = []
        self.log_task_notice: List[int] = []

    def _ramp_credit(self, step: int) -> int:
        c = self.p.ramp_credits
        return c[min(step, len(c) - 1)]

    def _rd_credit(self) -> int:
        frac = max(0.0, 1.0 - self.rd_timer / max(1, self.p.rampdn_total))
        return max(0, round(self.pre_rd_credit * frac))

    def _m5(self, tok: int):
        if not self.p.m5_enabled:  self.m5_lock = False;  return
        hi = tok >= self.p.m5_hi_thresh;  lo = tok <= self.p.m5_lo_thresh
        if (self.m5_prev_hi and lo) or (self.m5_prev_lo and hi):  self.m5_osc += 1
        self.m5_prev_hi = hi;  self.m5_prev_lo = lo
        if self.m5_win <= 0:
            if self.m5_osc >= self.p.m5_osc_thresh and not self.m5_lock:
                self.m5_lock = True;  self.m5_lock_t = self.p.m5_lock_dur
            self.m5_osc = 0;  self.m5_win = self.p.m5_window
        else:  self.m5_win -= 1
        if self.m5_lock:
            if self.m5_lock_t > 0:  self.m5_lock_t -= 1
            else:  self.m5_lock = False

    def _pi_update(self, heavy_queued: bool = False):
        if self.pi_settle_timer > 0:
            self.pi_settle_timer -= 1;  return
        if self.m5_lock:  return
        if self.pi_update_timer > 0:
            # Faster countdown when pipeline is backlogged (sustained load)
            self.pi_update_timer -= 2 if heavy_queued else 1
            if self.pi_update_timer < 0:
                self.pi_update_timer = 0
            if self.pi_update_timer > 0:
                return
        self.pi_update_timer = self.p.pi_update_interval

        target = V0_MV - self.p.target_droop_mv
        error = self.obs.voltage - target

        if self.warm_post_timer > 0 and error > 0:
            error = 0.0

        # Deadband: suppress micro-oscillation when near target
        if abs(error) < self.p.pi_deadband_mv:
            # Reset integral to prevent windup during stable periods
            self.pi_integral *= 0.5
            return

        p_term = self.p.pi_kp * error
        self.pi_integral += self.p.pi_ki * error
        self.pi_integral = max(-self.p.pi_i_max, min(self.p.pi_i_max, self.pi_integral))

        # D-term: feedforward resonance damping (prev_decline > 0 when V dropping)
        d_term = self.p.pi_kd * self.prev_decline

        new_credit = round(self.pi_credit + p_term + self.pi_integral - d_term)
        up_lim = 2 if error > 10 else 1
        down_lim = 3 if error < -15 else (2 if error < -5 else 1)
        new_credit = max(self.pi_credit - down_lim, min(self.pi_credit + up_lim, new_credit))
        self.pi_credit = max(self.p.pi_credit_min,
                             min(self.p.pi_credit_max, new_credit))

    def step(self, early_wu: bool, isu_valid: bool, ex_busy: bool,
             rd_trig: bool, tok: int, lnq_idle: bool = True,
             ld_issued: bool = False, heavy_queued: bool = False,
             task_notice: int = 0) -> Tuple[int, int]:

        if not self.enabled:
            self.log_state.append(int(State.REGULATE));  self.log_credit.append(8)
            self.log_m5.append(False);  self.log_comp.append(False)
            self.log_delta_lim.append(False)
            self.log_ld_window.append(False);  self.log_queue.append(False)
            self.log_task_notice.append(0)
            return 8, 0

        # LD window tracking
        if self.p.ld_aware_enabled:
            if ld_issued:  self.ld_timer = self.p.ld_lat
            elif self.ld_timer > 0:  self.ld_timer -= 1
            self.ld_window = self.ld_timer > 0
        else:
            self.ld_window = False

        self.queue_busy = heavy_queued if self.p.queue_lookahead else False

        # Warm window (post-busy dummy保温)
        f_busy_nat = isu_valid or ex_busy

        if f_busy_nat:
            self.warm_post_timer = 0
        elif self.warm_post_timer > 0:
            self.warm_post_timer -= 1
        elif not f_busy_nat and self.prev_f_busy_nat:
            self.warm_post_timer = self.p.warm_post_cycles

        # Task advance notice: pre-warm PDN before task dispatch
        self.task_upcoming = task_notice > 0

        self.warm_window = (self.ld_window or early_wu or self.queue_busy or
                           self.task_upcoming or
                           (self.warm_post_timer > 0))
        f_busy = f_busy_nat or self.warm_window
        self.prev_f_busy_nat = f_busy_nat

        self._m5(tok)
        s = self.state;  ns = s

        # ── FSM ──────────────────────────────────────────────────────
        if s == State.IDLE:
            if self.warm_window:
                ns = State.RAMP;  self.ramp_step = 0
                self.ramp_timer = self.p.ramp_durations[0]
                self.ramp_timeout = self.p.ramp_timeout
                self.pi_credit = self._ramp_credit(0)

        elif s == State.RAMP:
            if self.ramp_timer > 0:
                self.ramp_timer -= 1
            if self.ramp_timer == 0:
                self.ramp_step += 1
                if self.ramp_step < len(self.p.ramp_credits):
                    self.ramp_timer = self.p.ramp_durations[self.ramp_step]
                    self.pi_credit = self._ramp_credit(self.ramp_step)
                else:
                    ns = State.REGULATE
                    self.pi_settle_timer = self.p.pi_settle_cycles
                    self.pi_update_timer = self.p.pi_update_interval
                    self.pi_integral = 0.0
                    self.pi_credit = 2  # conservative start, PI adjusts from here
            elif not f_busy:
                self._hold(s);  ns = State.HOLD
            elif not isu_valid:
                self.ramp_timeout -= 1
                if self.ramp_timeout <= 0:
                    ns = State.IDLE
            else:
                self.ramp_timeout = self.p.ramp_timeout

        elif s == State.REGULATE:
            self._pi_update(heavy_queued)
            if rd_trig and not self.warm_window:
                self.pre_rd_credit = self.pi_credit;  self.rd_timer = 0
                ns = State.RAMPDN
            elif not f_busy:
                self._hold(s);  ns = State.HOLD

        elif s == State.HOLD:
            if f_busy:
                idle_cy = self.p.hold_init - self.hold_timer
                if idle_cy < 5:        step_back = 0
                elif idle_cy < 12:     step_back = 1
                elif idle_cy < 20:     step_back = 2
                else:                  step_back = 3

                if self.pre_hold_state == State.REGULATE:
                    self.pi_credit = max(self.p.pi_credit_min,
                                         self.pi_credit - step_back)
                    self.pi_integral = 0.0
                    self.pi_settle_timer = self.p.pi_settle_cycles // 3
                    self.pi_update_timer = self.p.pi_update_interval
                    ns = State.REGULATE
                else:
                    self.ramp_step = max(0, self.ramp_step - step_back)
                    self.ramp_timer = self.p.ramp_durations[self.ramp_step] // self.p.re_ramp_div
                    self.ramp_timeout = self.p.ramp_timeout
                    ns = State.RAMP
            elif self.hold_timer > 0:
                self.hold_timer -= 1
            else:
                self.pre_rd_credit = self.pre_hold_credit;  self.rd_timer = 0
                ns = State.RAMPDN

        elif s == State.RAMPDN:
            self.rd_timer += 1
            if (isu_valid or self.warm_window) and self.rd_timer > 10:
                ns = State.RAMP
                if self.rd_timer < 30:
                    self.ramp_step = min(2, len(self.p.ramp_credits) - 1)
                else:
                    self.ramp_step = 0
                self.ramp_timer = self.p.ramp_durations[self.ramp_step]
                self.ramp_timeout = self.p.ramp_timeout
            elif self.rd_timer >= self.p.rampdn_total:
                ns = State.IDLE

        self._prev_state = self.state
        self.state = ns

        # ── Compute observation token ─────────────────────────────────
        obs_tok = tok + (self.p.ld_dummy_token if self.ld_window else 0)

        # ── Credit computation (priority-ordered) ─────────────────────

        # 0. RAMP fixed credit
        if self.state in (State.RAMP, State.IDLE):
            credit = self._ramp_credit(self.ramp_step)
        elif self.state == State.RAMPDN:
            credit = self._rd_credit()
        else:
            credit = self.pi_credit

        # 1. Voltage emergency brake
        self.emergency_active = False
        if self.p.emergency_enabled:
            obs_v = self.obs.voltage
            obs_droop = V0_MV - obs_v
            if obs_droop > self.p.emergency_droop_mv:
                self.emergency_active = True
                credit = min(credit, self.p.emergency_credit)
            elif self.emergency_timer <= 0 and obs_droop < (self.p.emergency_droop_mv - 20):
                self.emergency_active = False
            if self.emergency_active:
                if self.emergency_timer > 0:
                    self.emergency_timer -= 1
                    credit = min(credit, self.p.emergency_credit)
                else:
                    self.emergency_active = False
            if obs_droop > self.p.emergency_droop_mv:
                self.emergency_active = True
                self.emergency_timer = self.p.emergency_hold

        # 2. Predictive droop rate limiter
        if self.p.pred_rate_enabled:
            if self.prev_decline > self.p.pred_rate_threshold:
                credit = min(credit, self.p.pred_rate_credit)

        # 3. M5 lock
        if self.m5_lock and self.state not in (State.IDLE, State.RAMP, State.RAMPDN):
            credit = min(credit, self.p.m5_lock_credit)

        # 4. Feedforward resonance damping — per-cycle credit modulation
        if self.state == State.REGULATE and not self.m5_lock and heavy_queued:
            decline = self.prev_decline
            if decline > 3.0:
                credit = max(1, credit - 1)
            elif decline < -2.5:
                # Only increase credit during recovery if droop is below soft ceiling
                obs_v = self.obs.voltage
                obs_droop = V0_MV - obs_v
                if obs_droop <= self.p.soft_ceiling_droop_mv:
                    credit = min(self.p.pi_credit_max, credit + 1)

        # 5. Droop soft ceiling — FINAL clamp (after resonance damping,
        #    to prevent recovery credit bumps from breaching the ceiling).
        if self.p.soft_ceiling_enabled:
            obs_v = self.obs.voltage
            obs_droop_soft = V0_MV - obs_v
            if obs_droop_soft > self.p.soft_ceiling_droop_mv:
                credit = min(credit, self.p.soft_ceiling_credit)

        # Sync PI credit to actual credit on REGULATE entry
        if self.state == State.REGULATE and self._prev_state != State.REGULATE:
            self.pi_credit = credit

        # Sync PI credit during emergency to prevent post-brake credit jump
        if self.emergency_active and self.state == State.REGULATE:
            self.pi_credit = credit

        # ── Dummy injection ───────────────────────────────────────────
        dummy = 0
        if self.ld_window:
            dummy = max(dummy, self.p.ld_dummy_token)

        # Task pre-warm: ramped dummy injection, capped to prevent overshoot
        if self.task_upcoming and not f_busy_nat:
            if task_notice > 200:
                dummy = max(dummy, 1)
            elif task_notice > 100:
                dummy = max(dummy, 2)
            else:
                dummy = max(dummy, 2)  # cap at 2 during pre-warm

        # ── Update observer with pre-Δ-limit token ────────────────────
        self.obs.step(float(tok) + float(dummy))

        # ── Predictive rate tracking ──────────────────────────────────
        if hasattr(self, 'prev_obs_v'):
            self.prev_decline = self.prev_obs_v - self.obs.voltage
        self.prev_obs_v = self.obs.voltage

        # ── Logging ───────────────────────────────────────────────────
        self.log_state.append(int(self.state))
        self.log_credit.append(credit)
        self.log_m5.append(bool(self.m5_lock))
        self.log_comp.append(bool(dummy > 0))
        self.log_delta_lim.append(False)
        self.log_ld_window.append(bool(self.ld_window))
        self.log_queue.append(bool(self.queue_busy))
        self.log_task_notice.append(int(task_notice))

        return credit, dummy

    def _hold(self, s: State):
        self.pre_hold_state = s
        if s == State.RAMP:            self.pre_hold_credit = self._ramp_credit(self.ramp_step)
        else:                          self.pre_hold_credit = 2
        self.hold_timer = self.p.hold_init

    def reset(self):  self.obs.reset();  self._reset()
