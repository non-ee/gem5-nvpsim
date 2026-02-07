|         Metric         | MOUSE (Projected MTJ, SHE)  | Eyeriss (16-bit fixed-point)  |
| :--------------------: | :-------------------------: | :---------------------------: |
|    Compute Latency     |           ~50 µs            |           ~0.03 µs            |
|     Compute Energy     |           ~30 pJ            |            ~600 pJ            |
| Initialization Latency |   ~4.48 µs (FRAM load 8B)   |  ~81.76 µs (FRAM load 146B)   |
| Initialization Energy  |   ~3.2 nJ (FRAM load 8B)    |   ~58.4 nJ (FRAM load 146B)   |
|    Restart Latency     |       0 µs (no FRAM)        |  ~81.76 µs (Similar to Init)  |
|     Restart Energy     |       0 µs (no FRAM)        |  ~58.4 nJ (Similar to Init)   |
|  Interruption Safety   | Yes - resumes transparently | No - must re-initialize fully |

## Eyeriss with FRAM Main Memory – Partially Non-Volatile but Compute-Volatile

Eyeriss is a spatial CNN accelerator with a volatile compute fabric. While its original design assumes continuous power, we consider a modified version with FRAM as main memory. Although weights are permanently stored in non-volatile FRAM, all working data—including weight copies loaded into PE registers, input activations in buffers, and partial sums—resides in volatile SRAM. A power interruption loses this entire computational state. Restart therefore requires reloading both weights (from FRAM) and input tiles (from FRAM or recomputed from earlier layers) into SRAM. For an 8×8 convolution with 3×3 kernels, reloading ~146B from FRAM (560 ns/byte, 0.4 nJ/byte) takes ~81.76 µs and ~58.4 nJ, plus recomputation (~0.03 µs, ~600 pJ). Each interruption incurs this FRAM access overhead, making Eyeriss poorly suited for frequently intermittent environments despite its non-volatile backing store.

References:
Chen, Y., Krishna, T., Emer, J., & Sze, V. (2016). Eyeriss: An Energy-Efficient Reconfigurable Accelerator for Deep Convolutional Neural Networks. IEEE International Solid-State Circuits Conference (ISSCC).

## MOUSE (Projected MTJ, SHE) – Fully Non-Volatile and Interruption-Safe Design

MOUSE is architecturally designed for intermittent operation, implementing both storage and computation directly within non-volatile magnetic tunnel junction (MTJ) crossbars. Unlike Eyeriss's volatile SRAM fabric, MOUSE preserves all architectural state—weights, activations, and partial sums—across power loss. Initialization occurs only once at system startup: input data (8 bytes for an 8×8 binary image) loads from FRAM into the MTJ tiles, incurring ~4.48 µs latency and ~3.2 nJ energy (560 ns/byte, 0.4 nJ/byte FRAM access). Crucially, subsequent power interruptions trigger no restart process. Upon power restoration, MOUSE immediately resumes the exact in-progress operation—completing a logic gate or write as if uninterrupted—with effectively zero restart latency/energy and no data reload. This is enabled by idempotent bitwise operations and non-volatile compute-in-memory logic that guarantee forward progress even under arbitrary power cycling, making MOUSE uniquely suited for energy-harvesting environments.

Reference:
Resch, S., Khatamifard, S. K., Chowdhury, Z. I., et al. (2020). MOUSE: Inference In Non-volatile Memory for Energy Harvesting Applications. 2020 53rd Annual IEEE/ACM International Symposium on Microarchitecture (MICRO).
