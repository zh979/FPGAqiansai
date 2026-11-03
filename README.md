# laser_weeder_zynq — 基于 ZYNQ-7020 的智能激光除草系统

单片 ZYNQ-7020 上完成 **采集 → CNN 推理 → 跟踪 → 云台瞄准 → 激光照射** 的完整闭环：
PL 内是 HLS 综合出的 128 路并行卷积加速器，PS 内是预处理、后处理、卡尔曼跟踪与
激光安全状态机，STM32 只做 UART→CAN 转换与物理兜底，不参与任何判断。

> 本目录按 2026 全国大学生嵌入式芯片与系统设计竞赛 AMD 赛道「自主选题赛道（初级组）」
> 推荐结构组织，目录名与文件名全部为英文。

---

## 1. 核心指标（全部为板上实测）

| 项目 | 数值 | 出处 |
|---|---|---|
| 目标检测 | YOLOv4-tiny，416×416，2 类（杂草 / 作物） | 训练与验证集 |
| 整网推理 | 429 ms/帧 | `data/board_run_log.txt`，1191 次推理统计 |
| 单层加速比 | 106×（同一帧第 1 层卷积：PL 8.26 ms vs ARM 874 ms） | `b` 键基准程序 |
| 端到端 | 459 ms/拍 ≈ 2.2 fps | 串口时间戳 |
| 瞄准中位误差 | 11 px = 5.9 mm（前馈开）；16.1 mm（前馈关） | 动态打靶对照实验 |
| 事件驱动 | 1770 拍中推理 1191 次，省 33% | 作业实拍日志 |
| PL 资源 | DSP 220/220（100%）、LUT 82%、BRAM 77% | Vivado 实现报告 |
| 时序 | WNS +0.189 ns @ 120 MHz | Vivado 时序报告 |
| 功耗 | 整片 3.28 W（动态 3.05 + 静态 0.22） | Vivado Power 报告 |
| 数值格式 | Q7.9 定点（缩放 512），128 路并行乘累加 | `src/hls/` |

---

## 2. 器件与工具链

- **器件**：`xc7z020clg400-2`（Zynq-7000 SoC，双 Cortex-A9 @667 MHz + Artix-7 可编程逻辑）
- **开发板**：领航者 ZYNQ-7020 开发板 + 自制底板（DVP 相机口、HDMI、CAN、激光使能 IO）
- **工具**：Vivado / Vitis **2019.2**（HLS 顶层 `conv`，目标时钟 5.5 ns，见 `build/hls_build.tcl`）
- **PS 侧**：Xilinx Standalone（裸机），C/C++，无操作系统
- **模型侧**：PyTorch 训练 + `src/tools/export_weights.py` 导出 BN 折叠后的定点权重

### 关于工具版本

比赛的推荐版本是 Vivado/Vitis 2026.1（或 2025.2），本作品在 **2019.2** 上开发与实测，
按选题指南 3.3.3.1 的规定在此说明版本差异并保证脚本可复现。
工程只使用跨版本稳定的部分：HLS C 源码 + IP Catalog 导出 + Block Design + 标准约束文件，
没有依赖 2019.2 的任何专有特性。

全文所有资源、时序与功耗数字都是 2019.2 下的实测结果，换版本重新综合后需重新测量，
不能直接沿用。

---

## 3. 快速上板（无需综合）

`build/hw/` 提供了预构建产物，**不需要安装 Vitis HLS，也不需要跑综合实现**：

| 文件 | 用途 |
|---|---|
| `design_1_wrapper.xsa` | 硬件平台，Vitis 建 Platform Project 时直接选它 |
| `design_1_wrapper.bit` | 比特流，Vivado Hardware Manager 可直接下载 |

两者内嵌的是同一次实现的结果，与 §1 指标表中的资源、时序、功耗数字一一对应。

### 3.1 准备 SD 卡

把 `src/tools/export_weights.py` 导出的权重目录拷到 SD 卡根目录，
连同 `data/yolo_anchors.txt` 与类别表。板子设为 SD 启动。

### 3.2 建平台与应用（Vitis 2019.2）

1. New Platform Project → 选 `build/hw/design_1_wrapper.xsa`
2. 处理器 `ps7_cortexa9_0`，OS `standalone`，语言 **C++**
3. Build Platform
4. New Application Project → 基于该平台 → 模板 Empty Application (C++)
5. 把 `src/ps/` 下全部内容复制进应用的 `src/`，
   其中 `drivers/` 是厂商驱动，`lscript.ld` 覆盖默认链接脚本
6. Build，下载 ELF

运行模式由 `src/ps/main.cpp` 的 `LOOP_MODE` 宏选择，默认 `4`（除草作业）。

### 3.3 只想看硬件跑起来

不建 Vitis 工程也可以：Vivado → Open Hardware Manager → Program Device，
选 `build/hw/design_1_wrapper.bit`。此时 PL 侧相机与显示通路会工作，
但没有 PS 程序，不会有检测框与激光动作。

### 3.4 上板验证的三个检查点

1. **画面通路**：HDMI 应显示相机实时画面与检测框；
2. **加速器**：按 `b` 键跑同层基准，串口打印 PL 与 ARM 两次耗时（应约 8.26 ms 与 874 ms）；
3. **作业节拍**：静置时占用率掉到 0、推理计数停止增长；传送带开启后恢复。

串口 115200 8N1。

---

## 4. 目录结构

```
laser_weeder_zynq/
├── README.md            本文件：作品简介 + 复现步骤
├── src/
│   ├── hls/             PL 加速器（HLS C）：conv / pwconv / std_conv / maxpool / upsample
│   ├── ps/              PS 侧 C/C++：主循环、预处理、后处理、跟踪、激光状态机、云台、标定
│   ├── stm32/           STM32 从机：UART→CAN 帧转换、看门狗、硬上限
│   └── tools/           训练侧工具：BN 折叠、权重量化导出、网络定义
├── sim/                 HLS 测试台与宿主机上的对照校验脚本（C-sim 基准 + Python 复核）
├── build/               构建脚本与预构建产物
│   ├── hw/              硬件平台 XSA + 比特流（可直接烧录，无需综合）
│   ├── reports/         Vivado 综合/实现/功耗原始报告
│   ├── ip_repo/         四个自定义 IP（conv 加速器 + 相机/显示/时钟）
│   ├── constrs/         引脚与时钟约束
│   └── *.tcl            HLS 综合、块设计重建、XSA 导出脚本
├── board/               上板运行说明、引脚与接线、串口输出样例
├── data/                锚框、类别表、实测日志、板上导出的特征图样本
├── skill/               大模型协作沉淀的技能包（见 skill/README.md）
├── report/              设计报告与《大模型协作记录》
└── LICENSE              MIT
```

## 5. 从零构建

§3 已能跑起来，本节供需要修改硬件或核对复现性时使用。

### 5.1 环境

| 组件 | 版本 |
|---|---|
| Vivado / Vitis / Vitis HLS | 2019.2（新版见 §2） |
| Python（训练与校验） | 3.8+，依赖见 `src/tools/requirements.txt` |
| 串口工具 | 任意，115200 8N1 |

硬件：

| 部件 | 型号 / 参数 | 接口 |
|---|---|---|
| 主控 | 领航者 ZYNQ-7020（xc7z020clg400-2） | — |
| 相机 | OV5640，800×480 @40 fps | DVP 并口 → PL |
| 显示 | HDMI，800×480 实时画面 + 检测框 | PL 显示控制器 |
| 云台 | 双轴步进，Emm_V5 闭环驱动器 ×2 | CAN |
| 激光 | 650 nm，<1 mW | STM32 GPIO |
| 协处理 | STM32（UART→CAN + 看门狗） | UART 115200 |
| 台面 | 传送带 33 mm/s，作业面 430×258 mm | — |

引脚分配见 `board/ps_pins.txt`（PS 侧 MIO）与 `board/uart_pins.txt`，
PL 侧见 `build/constrs/lcd.xdc`。

### 5.2 生成加速器 IP

```bash
cd build
vitis_hls -f hls_build.tcl          # 综合 conv 顶层，导出 IP
```

产物已附在 `build/ip_repo/conv_v0_0/`，跳过本步也可以。

### 5.3 重建 Vivado 工程

```tcl
create_project acc ./acc -part xc7z020clg400-2
set_property ip_repo_paths ./ip_repo [current_project]
update_ip_catalog
source ./design_1_bd.tcl
make_wrapper -files [get_files design_1.bd] -top
add_files -fileset constrs_1 ./constrs/lcd.xdc
launch_runs impl_1 -to_step write_bitstream -jobs 4
```

`design_1_bd.tcl` 会重建全部 12 类 IP。Zynq PS 的 DDR 型号、SD 与 UART 分配
需按实际板子核对，不同批次的 DDR 颗粒可能不同。

完成后用 `source ./export_xsa.tcl` 导出平台，或走
File → Export → Export Hardware → Include bitstream。

综合实现后应确认 WNS ≥ 0、资源占用与 §1 表一致。
**重跑实现后布线种子不同，WNS 与资源会有小幅变化，此时 §1 的数字需重新采集。**

### 5.4 重新训练与导出权重

```bash
python src/tools/export_weights.py                 # 读取 logs/best_epoch_weights.pth
python src/tools/export_weights.py <ckpt> <out_dir>
```

脚本会做 BN 折叠、检查检测头通道数是否为 `(类别数+5)*3`、列出权重清单与总大小。
量化到 Q7.9（缩放 512）后与 `data/yolo_anchors.txt`、类别表一起放入 SD 卡。

### 5.5 对照实验

- 前馈开/关对比：`src/ps/track.cpp` 中 `TRACK_LEAD` 置 0.54 s / 0 各跑 9 趟以上；
- 事件驱动：观察 `data/board_run_log.txt` 同样的 1770 拍里推理次数从 1770 降到 1191。

---

## 6. 数据与报告

- `data/board_run_log.txt`：除草作业实拍串口日志（1770 拍）
- `data/L5_*.bin`、`data/boxes.json`：板上导出的特征图与检测框样本
- `report/`：设计报告；`report/llm_collab_log.md`：大模型协作记录
- `skill/`：可复用的技能包，**不绑定本题目**，换一块板卡也能用

---

## 7. 许可

MIT，见 `LICENSE`。第三方组件（YOLOv4-tiny 训练框架）保留其原始许可，见 `src/tools/`。
