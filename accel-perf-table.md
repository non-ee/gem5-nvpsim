|            Metric             |          MOUSE (Projected MTJ, SHE)           |            Eyeriss (16-bit fixed-point)            |
| :---------------------------: | :-------------------------------------------: | :------------------------------------------------: |
|         Architecture          |    Fully non-volatile PIM; compute-in-MTJ     | Volatile systolic array; non-volatile FRAM weights |
|        Compute Latency        |                    ~50 µs                     |                      ~0.03 µs                      |
|        Compute Energy         |                    ~30 pJ                     |                      ~600 pJ                       |
|    Initialization Latency     |            ~4.48 µs (FRAM load 8B)            |             ~81.76 µs (FRAM load 146B)             |
|     Initialization Energy     |            ~3.2 nJ (FRAM load 8B)             |             ~58.4 nJ (FRAM load 146B)              |
|  Restart After Interruption   | **No FRAM access**; resumes last op instantly |            **Full FRAM reload** (146B)             |
|        Restart Latency        |                0 µs (no FRAM)                 |            ~81.76 µs (Similar to Init)             |
|        Restart Energy         |                0 µs (no FRAM)                 |             ~58.4 nJ (Similar to Init)             |
|      Interruption Safety      |          Yes - resumes transparently          |           No - must re-initialize fully            |
| Energy-Harvesting Suitability |  Excellent – makes progress per energy burst  |              Poor - high restart cost              |

## MOUSE (Projected MTJ, SHE) – Fully Non-Volatile and Interruption-Safe Design

MOUSE is architected from the ground up as a fully non-volatile in-memory accelerator, where both data storage and computational state reside in non-volatile magnetic tunnel junctions (MTJs). This ensures that all intermediate results and architectural state are preserved across power loss. Importantly, under the assumption that restart equals initialization, MOUSE’s behavior diverges: it only requires a single initialization at system startup, during which input data (e.g., 8 bytes for an 8×8 binary image) is loaded from FRAM into the MTJ tiles. This initialization incurs a latency of ~4.48 µs and energy of ~3.2 nJ, based on FRAM access characteristics of 560 ns/byte and 0.4 nJ/byte. After that, interruptions do not trigger any restart process. Instead, MOUSE immediately resumes execution of the previously interrupted operation upon power restoration, completing the in-progress logic gate or write operation as if no interruption occurred. This results in effectively zero restart latency and near-zero restart energy, aside from the marginal overhead of re-stabilizing peripheral circuitry. No weight or input reload is required, and no checkpointed instruction is re-issued—the system simply continues from the exact point of interruption. MOUSE’s idempotent operations and non-volatile compute-in-memory architecture guarantee correctness and forward progress even under unpredictable power cycling, making it uniquely suited for energy-harvesting applications.

Reference:
Resch, S., Khatamifard, S. K., Chowdhury, Z. I., et al. (2020). MOUSE: Inference In Non-volatile Memory for Energy Harvesting Applications. 2020 53rd Annual IEEE/ACM International Symposium on Microarchitecture (MICRO).

## Eyeriss with FRAM Main Memory – Partially Non-Volatile but Compute-Volatile

Eyeriss is a spatial CNN accelerator originally designed for continuous-power operation, but if integrated with FRAM (Ferroelectric RAM) as main memory, its weights become non-volatile and do not require reloading after power loss. However, Eyeriss’s computation fabric—including the systolic array of processing elements (PEs), activation buffers, and partial sum registers—is implemented in volatile SRAM. Under the assumption that restart equals initialization, this means that any interruption before output writeback loses all in-progress computational state, necessitating a full re-initialization: reloading weights (~18 bytes) and inputs (~128 bytes) from FRAM into on-chip SRAM. Given FRAM access latency of 560 ns/byte and energy of 0.4 nJ/byte, this re-initialization requires ~81.76 µs and ~58.4 nJ for an 8×8 convolution with 3×3 kernels, followed by recomputation from the first operation (~0.03 µs, ~350–600 pJ). Thus, each interruption forces Eyeriss to incur substantial FRAM access overhead, despite non-volatile weight storage. The volatile compute pipeline fundamentally limits Eyeriss’s suitability for intermittent environments, as frequent power outages lead to repeated, costly re-initialization and energy waste.

References:
Chen, Y., Krishna, T., Emer, J., & Sze, V. (2016). Eyeriss: An Energy-Efficient Reconfigurable Accelerator for Deep Convolutional Neural Networks. IEEE International Solid-State Circuits Conference (ISSCC).
