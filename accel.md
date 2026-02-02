# 0. Image Processing
- Input: 8x8
- Output: 6x6
- Kernel: 3x3

# 1. Eyeriss
**Ref**: Chen, Yu-Hsin, Joel Emer, and Vivienne Sze. "Eyeriss: A spatial architecture for energy-efficient dataflow for convolutional neural networks." 

## 1.1 Energy Caculation
### 1.1.1 Energy Per Inference
E_eyeriss = E_comp + E_DRAM + E_GLB + E_RF
#### comp: computation
E_comp = 324 * e_MAC 
- MAC: multiply-and-accumulate
- 324: total MAC in 8x8 CNN
(Output size * kernel size = 36 * 9 = 324)

let e_MAC = 1.5 pJ
E_comp = 324 * 1.5p = 486p = 0.486 nJ

#### DRAM: memory access
- Read input: 64
- Read Weight: 9
- Write output: 36
- Let it `e_DRAM` = 200 pJ (200pJ for each access)
- Total DRAM Energy: E_DRAM = (64 + 9 + 36) * 200p = 21800p = 21.8 nJ

#### GLB: Global Buffer (Large on-chip SRAM)
- Similar to DRAM, read input/weight, write output
- Let it consumes 10x more than RF 
- Total GLB Energy: E_GLB = (64 + 9 + 36) * 10p = 1090p = 1.09 nJ

#### RF: Register File Access
- Weight Reads: 324
- Input Reads:  324
- Psum Reads/Writes: 324
- Use baseline, 1 pJ per 16-bit access (standard for 65nm technology)
- Total RF Energy: E_RF = 324 * 3 * 1p = 972p = 0.972 nJ

**Hence**: `E_eyeriss` = 0.486n + 21.8n + 1.09n + 0.972n = 24.35 nJ

### 1.1.2 Init Energy
- Instruction Config: ~0.2 nJ
- Weight Loading: 9 * `e_DRAM` = 1.8 nJ
- Input Priming: (8 + 3) * `e_DRAM` = 2.2 nJ
(Input Priming: load the first row of input, into the front 3 PEs)

**Hence**: `E_init` = 0.2 + 1.8 + 2.2 = 4.2 nJ 

## 1.2 `delay_init`
### Instruction Overheaad
The accelerator receives a configuration packet (8x8 inputs and 3x3 weights).
Usually **10 ~ 50** clock cycles to decode and set the internal state machines.

### Weight Staging
Load weights into RF of each PE (Processing element).
**9** clock cycles

### Input Staging
Input image flow from GLB to PE with pipeline process.
**5 ~ 15** clock cycles

**Total cycle**: 25 + 9 + 10 = 44 clock cycles

Eyeriss operate on 200MHz clock frequency.
T_eyeriss = 1 / 200e+6 = 5 ns
**Hence**: `delay_init` = 44 * 5n = 220 ns

## 1.3 `compute_latency`
Use **Serial PEs**:
In this standard mapping, one PE handles a **row** of the convolution.
**Computing**
- One of output has 6 pixels
- One pixel takes 9 MAC cycles
- So one row takes 6 * 9 = 54 cycles
**Data Loading**
- Load weights: 9 cycles
- Input priming: 11 cycles
- Total: 20 cycles
**Writeback**
Output flow from PEs back into GLB.
Similar to Input Staging. **5 ~ 15** cycles

**Total cycle**: 54 + 20 + 10 = 84 cycles
**Hence**: `compute_latency` = 84 * 5n = 420 ns

## 1.4 Eyeriss's Characterization
We consider Eyeriss consumes the same power for both init and compute processes.
`P_active` = E_eyeriss / compute_latency = 24.35n / 420n = 57 mW
`P_init` = E_init / delay_init = 4.2n / 5m = 
`P_idle` = 15 mW ~ 10% of peak power 

`delay_init` = 
`compute_latency` = 0.42 ns

`energy_active` = P_active * ClockCycle = 57m * 1u = 57 nJ/cycle
`energy_init` = energy_active = 57 nJ/cycle
`energy_idle` = 15 nJ/cycle (estimation)

# 2. FPGA
## 2.1 Configuration Latency
`delay_init`

## 2.2 Inference Latency
`compute_latency`

## 2.3 Energy Per Cycle
`E_init`=
`P_init` = E_init / delay_init
`energy_init` = P_init * CyclePeriod (1 us)

`P_active` = P_static + P_dynamic
`energy_active` = P_active * CyclePeriod
`energy_idle` = P_static * CyclePeriod

# 3. MOUSE

The performanc values for MOUSE are derived from its underlying **Computational RAM (CRAM)** architecture, which uses spintronic Magnetic Tunnel Junctions (MTJs) to perform logic directly in memory.
The metrics for an 8x8 image inference are calculated based on the energy cost of individual MTJ switching and the parallelism of CRAM array.

## 3.1 Energy Per Inference
**~1.5 uJ - 4.2 uJ**
This is derived by summing the energy of the primitive logic operations required for a Binary Neural Network (BNN).

# 4. Eyerissv2

**Eyeriss v2** is designed specifically for "compact" and "sparse" networks. 
According to original research (Chen et al., 2019), Eyeriss v2 is approximately **2.5x more energy-efficient** than Eyeriss v1 for MobileNet-scale workloads.

`delay_init` = 5 ms
`delay_recover` = 10 ms
`compute_latency` = 165 ns
