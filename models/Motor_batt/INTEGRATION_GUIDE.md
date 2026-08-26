# PMSM_Drive 接入飞行仿真 —— GUI 集成指引

把 `PMSM_Drive_ForFlight.slx` 里的 **`PMSM_Drive`** 子系统，接进 `VTOLDynamics` 的
`Force and Moments/Propulsion`，**替换掉 `Detailed` 子系统**（保留飞行的 4 个 PID 速度环）。

---

## 0. 交付物与接口

**`PMSM_Drive` 子系统**（`PMSM_Drive_ForFlight.slx` 内）——全 Simulink，无 Simscape：

| 端口 | 方向 | 宽度 | 含义 |
|---|---|---|---|
| `trqR`    | IN  | 4×1 | 4 台电机的转矩请求（来自飞行 PID） |
| `Drag_tq` | IN  | 4×1 | 4 个旋翼阻力扭矩（来自 Rotor Assembly） |
| `Mot_RPM` | OUT | 4×1 | 4 台电机转速 **[rpm]** |
| `Iabc`    | OUT | 12×1 | 三相电流（每台 3 相，`reshape(x,3,4)` 每列一台）|
| `Idq`     | OUT | 8×1 | id/iq（每台 2 个）|
| `Idc`     | OUT | 4×1 | 各电机 DC 电流 |

内部：`trqR → iq_ref=trqR/Kt`（旁路速度环）→ 动态 iq 限幅 → 电流 PI → 平均值逆变器
（解耦+d轴优先限幅）→ PMSM dq 电气 → **自带惯量 Jm 积分出转速** → Mot_RPM。Vdc 固定 780V。

---

## 1. 参数：startup 里加载

`PMSM_Drive` 的所有块按名字引用工作区变量。在 `startup.m` 里（`load_vtol_dynamics_7000lb`
之后）加一行：

```matlab
pmsm_batt_testbench_data;   % 定义 p,psim,Rs,Ld,Lq,Kt,Bm,Imax,Kp_i,Ki_i,Jm_flight,Vdc_fixed,k_drag
```

这些和 Evolito D500 参数一致。`Jm_flight=0.08`（= 飞行模型 Inertia 值），`Vdc_fixed=780`
（充电态母线，见脚本注释）。

---

## 2. 飞行模型现有连线（替换前）

```
speed cmd → Sum → [PID Controller ×4] → Sum4-7 ┐
                                                ├→ [Detailed]  ┐(Simscape, 慢)
Rotor#_Drag_tq (From36-39) ─────────────────────┘             │
                                                              → bus Mot_RPM_1..4
                                                                → BusSelector → Gain4(1/RPMMAX)
                                                                → Mot_RPM_OUT_1..4 → Rotor Assembly.N
```

**信号映射（要接的三组）：**

| 飞行模型信号 | ↔ | PMSM_Drive 端口 |
|---|---|---|
| Sum4, Sum5, Sum6, Sum7（4 个 trqR 标量）| → Mux → | `trqR` (4×1) |
| Rotor1..4_Drag_tq（From36-39，4 标量）| → Mux → | `Drag_tq` (4×1) |
| `Mot_RPM` (4×1, rpm) | → Demux → | 原 Gain4(1/RPMMAX) 输入端 → Mot_RPM_OUT_x → Rotor Assembly.N |

---

## 3. GUI 步骤（在 VTOLDynamics 里）

1. **拷入子系统**：打开 `PMSM_Drive_ForFlight.slx`，把 `PMSM_Drive` 子系统复制到
   `Force and Moments/Propulsion` 里（放在 Detailed 旁边）。
2. **断开并删除/旁路 `Detailed`**（连同它里面的 Simscape 内容）。
3. **接输入**：
   - 加一个 `Mux`(4)：`Sum4,Sum5,Sum6,Sum7` → Mux → `PMSM_Drive/trqR`。
   - 加一个 `Mux`(4)：4 个 `Rotor#_Drag_tq`（可用 From `Rotor1_Drag_tq`…）→ Mux → `PMSM_Drive/Drag_tq`。
4. **接输出**：`PMSM_Drive/Mot_RPM` → `Demux`(4) → 4 路，分别接到原来 `Gain4`(1/RPMMAX) 的输入
   （即写 `Mot_RPM_1..4` 那 4 个信号处），保持下游 `→Rotor Assembly.N` 不变。
   - 单位已对齐：我们输出 rpm，原 Gain4=1/RPMMAX 正好把 rpm 归一化。
5. **顶层 powergui**：Detailed 旁路后若模型里再无 Simscape 块，可删 `powergui`（去 Simscape 提速）；
   若还有其它 Simscape 用途则保留。
6. **记录电流**：`PMSM_Drive/Iabc`、`Idq`、`Idc` 各接一个 `To Workspace`（Timeseries）。

---

## 4. 验证（片段）

1. 先跑**短片段**（如起飞 3–5s，`set_param(bdroot,'StopTime','5')`），别直接全程（20µs→全程 10+ 小时）。
2. 期望：
   - 4 台转速跟随各自 PID 指令（差动正常）；
   - 三相电流 ~900–1000A 峰值，与**同段重构法**电流吻合；
   - 推力/高度与纯系统级 P4 一致（未加电气限制时）。
3. 全程三相电流波形仍用**重构法**（秒级）拿；详细闭环只跑关键片段。

---

## 5. 注意事项

- **基础步长**：详细模型需 ~20µs（电频率 ~1150Hz）。集成后整个飞行会被拖到 20µs → 慢。仅片段用。
- **PID 可能要微调**：飞行 PID（`motorctrl.p=0.4, i=0.001`）原是给 Simscape 电驱调的；我们的
  plant 动态不同，起飞可能需要重调速度环增益。
- **Vdc=780 是充电态**：标称 720V 在此电机下悬停电压不够（会限幅、丢差动）。接 SA88 电池后
  Vdc 随 SOC 变化，低 SOC 时会再次受限——那时才需要弱磁。
- **想接 SA88 电池**：把 `PMSM_Drive` 里各电机 `Idc` 求和 → SA88 电池块 → Vdc 广播回各电机
  （替换固定 Vdc_fixed 常数）。参考 `PMSM_SA88_4Motor.slx` 的 Battery 子系统。
