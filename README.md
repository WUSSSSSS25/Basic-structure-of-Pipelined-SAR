# 24-bit 三级 Pipelined-SAR ADC Simulink 模型

基于MATLAB 行为级代码（`SAR_FUN.m / SAR_FUN_R.m / RA_FUN.m / PipeSAR_test_3stage.m`）移植的 Simulink 模型，并预留了数字校准接口。

## 架构

```
Vin_p/Vin_n ──► SAR_Stage1 (8b, 冗余) ──► RA1 (G≈2^7=128) ──► SAR_Stage2 (8b, 冗余)
                     │D1[8]                                        │D2[8]
                     ▼                                             ▼
              ┌──────────────────── Digital_Backend ◄──── D3[10] ◄── SAR_Stage3 (10b)
              │  Digital_Calibration (算法接口)                   ▲
              │  Dout = D1·W1 + D2·W2 + D3·W3 − OFS              |
              └──► Dout / Vout                          RA2 (G≈2^7) ◄── 余差2
```

- 位分配：N1+N2+N3−2 = 8+8+10−2 = **24 bit**，级间各 1-bit 冗余（W1/W2、W2/W3 位权各重叠一位）。
- 每个仿真步（Ts = 1/fs）完成一次完整的流水线转换（行为级，与原 for 循环逐采样点等价）。
- 各级 SAR 为全差分 Vcm-Based 时序，含电容失配、kT/C 噪声、比较器失调/噪声；级间放大器含有限增益（Av）、有限 GBW 建立误差、失调与噪声 —— 全部与原 `SAR_FUN_R / SAR_FUN / RA_FUN` 公式一致。

## 文件清单

| 文件 | 作用 |
|---|---|
| `PipeSAR_params.m` | 全部参数初始化（24-bit 配置），加载到 base workspace |
| `build_PipeSAR24_model.m` | **程序化生成** `PipeSAR24.slx` 模型（含全部 MATLAB Function 块代码与连线） |
| `run_PipeSAR24_sim.m` | 一键：参数 → 建模 → 仿真 → FFT 动态测试 → 保存原始码 |
| `My_Digital_Calibration.m` | 离线数字校准算法模板（默认 = 理想重组） |
| `Dynamic_test.m / Static_test.m` | 沿用你原有的动态/静态测试函数 |

## 快速开始

在 MATLAB 中将本文件夹设为当前目录，运行：

```matlab
run_PipeSAR24_sim
```

首次运行会自动生成 `PipeSAR24.slx`。之后也可分步执行：

```matlab
PipeSAR_params;            % 修改参数后重新加载
build_PipeSAR24_model;     % 仅在需要重建模型时
out = sim('PipeSAR24');
```

## 数字校准接口（两种接入方式）

**方式一：在线（模型内）。** 打开 `PipeSAR24/Digital_Backend/Digital_Calibration`，函数签名为：

```matlab
function [Dout, Vout] = fcn(D1, D2, D3, W1, W2, W3, OFS, Kv)
```

- `D1 [1x8]`、`D2 [1x8]`、`D3 [1x10]`：各级原始码（MSB first），每个仿真步更新一次；
- `W1/W2/W3/OFS/Kv` 为 Parameter 作用域，直接来自 workspace —— 最简单的权重类校准只需改写 workspace 中的 `W1/W2/W3/OFS`，无需改模型。改写后若用一键脚本重新仿真，需先设 `use_calibrated_W = 1`，否则 `run_PipeSAR24_sim` 会把权重重置回理想值；
- 若算法需要状态（如 LMS 迭代），在该函数中使用 `persistent` 变量即可逐采样点在线更新。

**方式二：离线（推荐先用这个开发算法）。** 仿真已通过 To Workspace 记录：

- `sim_D1 [num×8]`、`sim_D2 [num×8]`、`sim_D3 [num×10]`：原始码
- `sim_Vres1p / sim_Vres1n`：第一级余差电压（便于基于余差的校准与调试）
- `sim_Vres2p / sim_Vres2n`：第二级余差电压
- `sim_Dout / sim_Vout`：模型内后端输出（对照用）

把算法写进 `My_Digital_Calibration.m`，`run_PipeSAR24_sim.m` 会自动调用并对校准后的码做 FFT 动态测试（SNDR/SFDR/ENOB）。原始数据同时保存到 `PipeSAR24_rawdata.mat`。

## 验证校准效果的建议流程

1. 先全理想跑通（默认参数下离线重组与模型后端输出差值应为 0）；
2. 注入非理想因素制造校准目标。电容失配由 `PipeSAR_params.m` 中的
   `mismatch_seed` 固定随机种子（默认 1），保证多次运行得到同一组失配，
   便于在相同条件下迭代校准算法；设为 `[]` 可恢复每次随机。例如：
   - `sigmaCu1 = 0.001`（第一级电容失配）；
   - `Av = 60 ~ 80`（级间增益误差：闭环增益误差 ≈ 1/(β·Av)，β = 2^(1−N1)，24-bit 下即使 Av=120 dB 也有约 1.3e-4 的增益误差，会把 ENOB 压到 ~13 bit —— 这正是数字校准要修的）；
3. 用你的算法校准后对比 `Dynamic_test` 的 SNDR/ENOB 提升。

## 注意事项

- 24-bit 量级下 `Dout` 最大约 2^24，double 精度够用；
- 静态测试（直方图法 DNL/INL）需要远多于 2^24 的采样点才有统计意义，`run_PipeSAR24_sim.m` 中默认关闭（`do_static = 0`）；
- `randn` 在 MATLAB Function 块内每步独立产生噪声，温度 `T=0` 时 kT/C 噪声自动关闭（与原脚本默认一致）；
- 修改位数分配（N1/N2/N3）后需重新运行 `build_PipeSAR24_model`，因为各级位宽在生成的块代码中是定长的（便于代码生成与定尺寸信号）。
- 生成模型脚本基于 Stateflow API（`sfroot` 查找 `Stateflow.EMChart` 并写入 `Script`、把参数数据作用域设为 `Parameter`）。
