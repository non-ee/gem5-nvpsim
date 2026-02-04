|            Metric             |         MOUSE (Projected MTJ, SHE)          |            Eyeriss (16-bit fixed-point)            |
| :---------------------------: | :-----------------------------------------: | :------------------------------------------------: |
|         Architecture          |   Fully non-volatile PIM; compute-in-MTJ    | Volatile systolic array; non-volatile FRAM weights |
|        Compute Latency        |                  ~10–50 µs                  |                      ~0.03 µs                      |
|        Compute Energy         |                  ~25–30 pJ                  |                    ~350–600 pJ                     |
|   Initialization at Startup   |  ~0.1-1 µs, ~1-5 pJ (sensor load + PC set)  | ~1-5 µs, ~50-200 pJ (weight/input load from FRAM)  |
|  Restart After Interruption   |  **Not re-init**; resumes last instruction  |             **Full re-init** required              |
|    Restart Latency/Energy     |      0 µs, 0 pJ (no restart required)       |    ~1-5 µs, ~50-200 pJ (FRAM load + recompute)     |
|      Interruption Safety      | Yes – instruction-level idempotent recovery | No – recompute from scratch on every interruption  |
| Energy-Harvesting Suitability | Excellent – makes progress per energy burst |  Poor – repeated re-initialization wastes energy   |

## MOUSE (Projected MTJ, SHE) – Fully Non-Volatile and Interruption-Safe Design

MOUSE is architected from the ground up as a fully non-volatile in-memory accelerator, where both data storage and computational state reside in non-volatile magnetic tunnel junctions (MTJs). This ensures that all intermediate results and architectural state are preserved across power loss. Importantly, under the assumption that restart equals initialization, MOUSE’s behavior diverges: it only requires a single initialization at system startup. After that, interruptions do not trigger any restart process. Instead, MOUSE immediately resumes execution of the previously interrupted operation upon power restoration, completing the in-progress logic gate or write operation as if no interruption occurred. This results in effectively zero restart latency and near-zero restart energy, aside from the marginal overhead of re-stabilizing peripheral circuitry. No weight or input reload is required, and no checkpointed instruction is re-issued—the system simply continues from the exact point of interruption. MOUSE’s idempotent operations and non-volatile compute-in-memory architecture guarantee correctness and forward progress even under unpredictable power cycling, making it uniquely suited for energy-harvesting applications.

Reference:
Resch, S., Khatamifard, S. K., Chowdhury, Z. I., et al. (2020). MOUSE: Inference In Non-volatile Memory for Energy Harvesting Applications. 2020 53rd Annual IEEE/ACM International Symposium on Microarchitecture (MICRO).

## Eyeriss with FRAM Main Memory – Partially Non-Volatile but Compute-Volatile

Eyeriss is a spatial CNN accelerator originally designed for continuous-power operation, but if integrated with FRAM (Ferroelectric RAM) as main memory, its weights become non-volatile and do not require reloading after power loss. However, Eyeriss’s computation fabric—including the systolic array of processing elements (PEs), activation buffers, and partial sum registers—is implemented in volatile SRAM. Under the assumption that restart equals initialization, this means that any interruption before output writeback loses all in-progress computational state, necessitating a full re-initialization: reloading weights and inputs from FRAM into on-chip SRAM, followed by recomputation from the first operation. For an 8×8 convolution with 3×3 kernels, this results in a restart latency of ~1–5 µs (dominated by FRAM access) and a restart energy of ~50–200 pJ (for data transfer) plus the full compute energy (~350–600 pJ). While FRAM reduces weight retention overhead, the volatile compute pipeline forces Eyeriss to re-initialize on every interruption, fundamentally limiting its suitability for intermittent environments, as frequent power outages lead to repeated re-initialization and energy waste.

References:
Chen, Y., Krishna, T., Emer, J., & Sze, V. (2016). Eyeriss: An Energy-Efficient Reconfigurable Accelerator for Deep Convolutional Neural Networks. IEEE International Solid-State Circuits Conference (ISSCC).
