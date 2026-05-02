# Warm Window — Dummy 预热窗口优化

## 目标

用 dummy 在空闲期"保温"，阻止控制器离开 REGULATE 进入 HOLD/RAMPDN/IDLE，
避免下次 busy 时重新走 320 周期 RAMP，从而提升间歇性负载的 IPC。

约束：所有 benchmark max droop ≤ 50mV（BM8 当前 51mV，允差 ≤1mV）。

## 机制

新增 `warm_window` 信号，覆盖两个阶段：

- **Pre-warm**（提前拉）：`early_wakeup`（decode 有重指令）或 `queue_busy`（队列前方有重指令）活跃时，提前注入 dummy 预载 PDN
- **Post-warm**（延后收）：busy→idle 后启动计时器（50cy），期间保持 dummy 注入，阻止进入 HOLD

`warm_window` = `ld_window OR early_wu OR queue_busy OR (warm_post_timer > 0)`

### 信号时序

```
Cycle:    0    5    10   15   20   25   30   35   40   45   50
Work:     [idle.....][heavy burst........][idle..........][heavy..]
early_wu:           ← 2-4cy ahead
queue_busy:         ← up to 10cy ahead
warm_post:                                    [50cy保温..............]
warm_window: [███████████████████████████████████████████████████████]
f_busy:      [███████████████████████████████████████████████████████]
Controller:  [REGULATE──────────────────────────────────────────────]
Dummy:       [███████████....................|██████████████████████]
Credit:      [~~~~~~~~~6~~~~~~~~~~~~~~~~~~~~~|~~~~~6~~~~~~~~~~~~~~~~]
```

关键效果：两次 heavy burst 之间控制器不离开 REGULATE，credit 保持高位，
第二次 burst 无需重新 RAMP。

## 改动清单

### 1. ThrottleParams — 新增参数

```python
warm_post_cycles: int = 50   # busy→idle 后 dummy 保温周期
```

Dummy token 数复用现有 `ld_dummy_token`（默认 3）。

### 2. ThrottleController — 改动

- 新增 `warm_post_timer` 状态变量
- 新增 `warm_window` 信号：
  `warm_window = ld_window or early_wu or queue_busy or (warm_post_timer > 0)`
- 修改 `f_busy`：`f_busy = isu_valid or ex_busy or warm_window`（用 warm_window 替代原来的 ld_window + queue_busy）
- 修改 dummy 注入条件：`if self.p.ld_aware_enabled and self.warm_window`
- Post-warm timer 逻辑：busy→idle 转换时启动（50cy），有真实活动时归零重置

### 3. 状态转换影响

| 转换 | 影响 |
|---|---|
| REGULATE → HOLD | `f_busy` 因 warm_post 保持 True → 阻止进入 HOLD |
| HOLD → RAMP/REGULATE | 已有逻辑，不变 |
| IDLE → RAMP | 已有 early_wu/queue_busy 触发，不变 |
| RAMPDN → RAMP | 已有 isu_valid/warm_window 触发，不变 |

### 4. 对 `f_busy` 的完整重构

```
f_busy = isu_valid or ex_busy or warm_window
warm_window = ld_window or early_wu or queue_busy or (warm_post_timer > 0)
```

## 不做的

- 不放松 credit 预算约束（dummy 仍在 `avail = max(0, cr - actual_tok)` 内）
- 不改变 RAMP 参数、PI 参数、M5 参数
- 不改动 run_sim.py 的 dummy 注入逻辑

## 验证

```bash
python3 run_sim.py --cycles 700
```

检查点：
1. 所有 benchmark max_droop ≤ 50mV（BM8 ≤52mV 可接受）
2. BM7/BM8（间歇性负载）IPC 相比当前基线提升
3. 稳态负载（BM1/2/5/6）IPC 不退化
4. 无控制器卡在中间状态
