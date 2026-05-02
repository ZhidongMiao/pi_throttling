# SIMD Vec 处理器 di/dt 节流控制仿真平台

## 项目简介

本项目为 SIMD Vec 处理器的 **di/dt 瞬态电流节流控制方案**提供完整的 Python 仿真平台，
并通过交互式 HTML 仪表盘展示仿真结果。

仿真内容覆盖：
- PDN（电源交付网络）多阶谐振物理模型
- 完整 7 状态节流控制器 FSM
- 8 类 benchmark 激励场景
- 有 / 无节流控制的电压响应对比

---

## 文件结构

```
didt_throttle_sim/
├── pdn_sim3.py            # 仿真引擎（核心，唯一 source of truth）
├── run_sim.py             # 仿真入口：运行所有 benchmark，导出 JSON
├── build_standalone.py    # 打包脚本：生成单文件可直接打开的 HTML
├── dashboard.html         # 仪表盘模板（需配合 sim_results.json 使用）
├── sim_results.json       # 仿真结果数据（由 run_sim.py 生成）
├── dashboard_standalone.html  # 单文件版仪表盘（内嵌数据，双击即可打开）
└── README.md              # 本文档
```

---

## 快速开始

### 方式一：直接打开（推荐）

`dashboard_standalone.html` 已内嵌仿真数据，**直接双击在浏览器中打开**即可，无需安装任何依赖或启动服务器。

> ⚠ 需要联网加载 Chart.js CDN（`cdnjs.cloudflare.com`）。离线使用见下方说明。

---

### 方式二：修改参数后重新仿真

```bash
# 1. 安装 Python 依赖（仅需一次）
pip install numpy scipy

# 2. 修改 pdn_sim3.py 中的参数（状态机、benchmark 等）

# 3. 重新打包（自动运行仿真并生成单文件 HTML）
python3 build_standalone.py --cycles 700

# 4. 打开生成的 dashboard_standalone.html
```

也可以分步操作：

```bash
# 只运行仿真，导出 JSON
python3 run_sim.py --cycles 700

# 启动本地 HTTP Server，使用分离的 dashboard.html
python3 -m http.server 8080
# 浏览器访问 http://localhost:8080/dashboard.html
```

---

## 模型说明

### PDN 物理模型（`pdn_sim3.py → PDNModel`）

基于**线性叠加原理**，用三阶阻尼振荡器的步进响应卷积计算任意时变负载下的 PDN 电压：

```
V[k] = V0 - Σ_{j=0}^{M-1} ΔLoad[k-j] × F[j]
```

其中 `F[n]` 为归一化步进响应函数，通过 `scipy.optimize.differential_evolution`
拟合到以下实测数据（Token=20 阶跃，1.6 GHz，loss < 1e-20）：

| 时刻 (ps) | 实测 (mV) | 模型误差 |
|-----------|-----------|---------|
| 500 (cy=0)  | 909 | < 0.1 mV |
| 512 (cy≈19) | 835 | < 0.1 mV |
| 525 (cy≈40) | 855 | < 0.1 mV |
| 545 (cy≈72) | 792 | < 0.1 mV |
| 571 (cy≈114)| 788 | < 0.1 mV |
| 710 (cy≈336)| 896 | < 0.1 mV |
| 749 (cy≈398)| 888 | < 0.1 mV |

---

### 节流控制器（`ThrottleController`）

完整实现方案文档 V0.4 中的 7 状态 FSM：

```
IDLE → RAMP1(Credit=10) → RAMP2(Credit=18) → RAMP3(Credit=26) → FULL(Credit=32)
                              ↕ HOLD（气泡容忍）
                              ↕ RAMPDN（6阶软着陆）
```

附加机制：
- **Look-ahead**：DEC 阶段侦测高功耗指令，提前 D2I 个周期触发唤醒
- **M5 反谐振过滤**：滑动窗口震荡计数，检测 Token 周期性震荡，锁定 Credit 上限
- **补偿窗口**（可选）：8cy 滑动均值 + 5cy 持续确认，向 EXQ2 注入 Dummy 平滑 −di/dt

关键参数（`ThrottleParams`）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ramp1_dur` | 20 cy | RAMP1 持续时间，对齐 1st droop 恢复期 |
| `ramp2_dur` | 40 cy | RAMP2 持续时间，跨过 2nd droop 谷底 |
| `ramp3_dur` | 40 cy | RAMP3 持续时间，跨过最深叠加点 |
| `hold_init` | 15 cy | Hold Timer 初值（L1 miss 典型延迟，待填） |
| `m5_window` | 150 cy | M5 检测滑动窗口 |
| `m5_osc_thresh` | 4 | M5 震荡次数触发阈值 |
| `m5_lock_credit` | 15 | M5 锁定后的 Credit 上限 |
| `comp_enabled` | True | 是否启用补偿窗口 |

---

### Benchmark 场景

| ID  | 场景名 | 验证目标 |
|-----|--------|---------|
| BM1 | Idle → 满载 → Idle | Ramp-up / Ramp-down 基本流程 |
| BM2 | 持续 mula×2 | FULL 态稳定性，Ramp-up 爬升效果 |
| BM3 | mula×2 ↔ mul+add 振荡（ΔToken=6） | 小幅切换，低风险谐振场景 |
| BM4 | mula×2 ↔ mov 周期切换（ΔToken=15，周期≈38cy） | 大幅切换，M5 反谐振压力测试 |
| BM5 | L1 miss 气泡（短 10cy + 长 40cy） | Hold Timer 两种场景验证 |
| BM6 | 软件 19cy on/off（与 PDN 谐振对齐） | 最大构造性干涉压力测试 |
| BM7 | mula×2 → mov 快速回切（双向冲击） | −di/dt + +di/dt 叠加，补偿窗口验证 |
| BM8 | 随机混合 workload | 综合稳定性，长期运行验证 |

---

## 仪表盘功能

打开 `dashboard_standalone.html` 后：

- **左侧 Benchmark 列表**：点击切换查看不同场景
- **顶部模式切换**：
  - `Throttled`：有节流控制的仿真结果
  - `Baseline`：无节流控制（纯 PDN 响应）
  - `Overlay`：两者叠加对比
- **KPI 条**：最大压降 / 最低电压 / IPC 效率 / 平均 Token / Stall 数
- **电压响应曲线**：PDN 电压时序，含 Sign-off 参考线
- **Token & Credit 图**：实际发射 Token vs 理想值 vs Credit 上限
- **FSM State 图**：状态机状态 + M5 锁定区间 + 补偿窗口激活区间
- **PDN 校准曲线**：模型拟合 vs 实测参考点验证
- **全 Benchmark 汇总表**：点击行切换到对应 benchmark

---

## 离线使用（不需要 CDN）

如果需要在无网络环境中使用，可以把 Chart.js 下载到本地：

```bash
curl -o chartjs.min.js https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js
```

然后修改 `dashboard.html` 中的：
```html
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
```
改为：
```html
<script src="chartjs.min.js"></script>
```

再重新运行 `python3 build_standalone.py`，或手动把 chartjs.min.js 内容内嵌到 HTML 中。

---

## 关联文档

`didt_complete_v04.docx` — 完整方案文档，包含：
- 处理器微架构基础与功耗权重定义
- PDN 危机诊断与实测数据
- Guardband 裕量分解与降压目标
- 完整状态机 RTL 伪代码
- 运行中指令切换的电流补偿策略
- 闭环安全保护（Droop Detector）
- 实现路线图与风险登记
- 全部待填参数占位符汇总

---

## 版本历史

| 版本 | 说明 |
|------|------|
| V0.1 | 初始 PDN 模型，基础状态机 |
| V0.2 | 参数占位版，补充 IR drop / PVT / aging 分项 |
| V0.3 | 状态机细化，Look-ahead 原理，波形示意 |
| V0.4 | 补偿窗口（滑动均值修正），完整合并版 |
| V0.5 | Python 仿真平台 + 交互式仪表盘 |
