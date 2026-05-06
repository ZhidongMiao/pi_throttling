# SIMD Vec 处理器 di/dt 节流控制仿真平台

## 项目目标

**压降 < 80mV，同时最大化 IPC。**

节流控制器须在所有 10 个 benchmark 场景下将最差电压压降控制在 80mV 以内（V0=909mV 下 Vmin > 829mV），并尽可能减少 IPC 损失。

---

## 文件结构

```
pi_throttling/
├── pdn.py                  # PDN 模型：7阶并联 IIR 滤波器
├── pipeline.py             # OOO 向量处理器流水线 + 寄存器重命名
├── controller.py           # 节流控制器：5 状态 FSM + PI + 4 层保护
├── pdn_sim3.py             # 仿真引擎：激励生成、模拟运行、结果汇总
├── run_sim.py              # CLI 入口：运行仿真 → sim_results.json
├── build_standalone.py     # 打包脚本：生成单文件 dashboard_standalone.html
├── dashboard.html          # 交互式仪表盘（Chart.js）
├── sim_results.json        # 仿真数据（gitignored，由 run_sim.py 生成）
├── docs/
│   └── benchmark_sequences.md  # Benchmark 指令序列详细文档
└── README.md
```

---

## 快速开始

```bash
# 创建并激活虚拟环境
python3 -m venv .venv && source .venv/bin/activate

# 安装依赖
pip install numpy scipy

# 运行仿真并导出 JSON
python3 run_sim.py --cycles 2000

# 启动本地服务器查看仪表盘
python3 -m http.server 8080
# → http://localhost:8080/dashboard.html

# 或生成单文件 HTML（无需服务器）
python3 build_standalone.py --cycles 2000
# → 打开 dashboard_standalone.html
```

---

## 架构

| 模块 | 职责 |
|------|------|
| **`pdn.py`** | 7 阶并联 IIR 滤波器，3 模态 PDN 物理模型（package + board + VRM），Ts=0.625ns 零阶保持离散化。`PDNModel`（钳位输出）和 `PDNObserver`（非钳位，用于控制器反馈） |
| **`pipeline.py`** | OOO 向量处理器：`InstrGroup`（5 发射端口指令束 + 架构寄存器操作数），`PhysRegFile`（100 物理 / 32 架构寄存器，重命名 + 读跟踪），`PipelineModel`（DEC→rename→SHQ/LDQ→EXQ0/EXQ1/LNQ/STQ→pipeline→WB），建模 RAW/WAW/WAR 依赖 |
| **`controller.py`** | 节流控制器：5 状态 FSM（`IDLE→RAMP→REGULATE→HOLD→RAMPDN`）+ PI 调节器 + 4 层保护，使用内部 PDN 观测器反馈 |
| **`pdn_sim3.py`** | 仿真运行器：`StimulusGenerator`（10 个 BM + for-loop 结构），`RegState`/`ChainRegState` 寄存器分配器，`run_sim()`，`run_all()` |

---

## 控制器参数 (v4)

| 保护层 | 阈值 | 动作 |
|--------|------|------|
| PI 调节 | 目标 68mV 压降 | Credit 1–3 动态调节（Kp=0.10, Ki=0.003, Kd=1.0） |
| 软上限 | 38mV 压降 | Credit ≤ 2 |
| 预测速率限制 | 2.0 mV/cy 下降 | Credit ≤ 1 |
| 紧急刹车 | 60mV 压降 | Credit ≤ 1，保持 50 周期 |

核心设计：PI 目标是期望值（68mV），**软上限 38mV** 是实际主控——提前限制 Credit 防止 PDN 电荷赤字累积。软上限（38mV）与紧急刹车（60mV）之间 22mV 的阶梯裕量提供逐级升级。

---

## Benchmark 场景

10 个 benchmark（BM1–BM10）覆盖稳态、最大负载、LD-burst、串行 RAW 链、交替、谐振、随机混合和多任务流水线。

每个 function 由 2–5 个 for-loop 组成，每个 loop 循环 10–50 次，body 为 5–30 周期的指令序列。

| BM  | 场景 | 压降(节流) | IPC | 验证目标 |
|-----|------|-----------|-----|---------|
| BM1 | MULA×2 稳态 | 73mV | 97.7% | 持续中等负载 |
| BM2 | MUL+ADD+LN 最大负载 | 60mV | 98.3% | 最高负载稳定性 |
| BM3 | LD-burst + MULA×2 | 43mV | 100% | LD→MULA RAW 依赖 |
| BM4 | 串行 MULA 依赖链 | 35mV | 100% | RAW 链流水线停顿 |
| BM5 | MULA×2 ↔ MAL 交替 | 70mV | 91.6% | 负载类型切换 |
| BM6 | LN 主导长跑 | 60mV | 92.7% | PI 调节器长期压力 |
| BM7 | SW 谐振 | 71mV | 99.0% | PDN 谐振激励 |
| BM8 | OOO 混合 | 75mV | 97.5% | 随机混合负载 |
| BM9 | 4 路任务流水线 | 57mV | 100% | 多任务异构 |
| BM10 | 5 路任务流水线 | 65mV | 100% | 5 种任务类型 |

全部 10/10 通过 droop < 80mV。详见 `docs/benchmark_sequences.md`。

---

## 仪表盘功能

打开 `dashboard_standalone.html` 后：

- **左侧列表**：点击切换 benchmark
- **模式切换**：Throttled / Baseline / Overlay
- **KPI 卡片**：最大压降 / 最低电压 / IPC 效率 / Stall 周期
- **电压曲线**：PDN 电压时序 + Sign-off 参考线
- **Token & Credit 图**：实际发射 vs 理想 vs Credit 上限
- **FSM 状态图**：状态机状态 + M5 锁定 + 补偿窗口
- **指令时间线**：Verdi 风格每发射端口指令波形
- **PDN 校准曲线**：IIR 模型 vs 实测参考点

---

## 关键参数

- `FREQ_GHZ = 1.6`, `V0_MV = 909`, `V_SIGNOFF = 675`（234 mV 裕量）
- Token 表：`mula=3, ln=4, exp=4, mul=2, add=2, mov=1, ld=0, st=1`
- 流水线深度：`ln/exp=14, mula=9, ld=10, mul/add=8, mov=6, st=4`
- 寄存器文件：32 架构寄存器，100 物理寄存器（68 重命名池）
- 发射约束：每队列每周期最多 1 条指令，不同队列可并发
