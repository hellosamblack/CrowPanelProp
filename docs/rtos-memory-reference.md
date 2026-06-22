# RTOS & Memory Reference — ESP-IDF 6.x / ESP32-P4

Source pages (esp-idf stable/esp32p4):
- `api-reference/system/freertos_idf`
- `api-reference/system/freertos_additions`
- `api-reference/system/mm_sync`
- `api-reference/system/mem_alloc`
- `api-guides/memory-types`

---

## 1. FreeRTOS — SMP / Dual-Core

The ESP32-P4 runs IDF FreeRTOS in **SMP mode**: two cores (Core 0 / PRO_CPU, Core 1 / APP_CPU) with a shared scheduler. Key differences from vanilla FreeRTOS:

- **Stack sizes are in bytes**, not words.
- Each core gets its own idle task (pinned).
- `vTaskSuspendAll()` only suspends scheduling on the **calling core** — not mutual exclusion.
- Disabling interrupts is **not** sufficient for mutual exclusion; use spinlock critical sections.

### Task creation

```c
// Pinned to a specific core (preferred for deterministic behavior)
xTaskCreatePinnedToCore(fn, "name", STACK_BYTES, arg, priority, &handle, core_id);
//   core_id: 0, 1, or tskNO_AFFINITY

// With explicit heap capability for the stack
xTaskCreateWithCaps(fn, "name", STACK_BYTES, arg, priority, &handle, MALLOC_CAP_SPIRAM);
vTaskDeleteWithCaps(handle); // must pair with the above
```

### Core affinity

| Value | Meaning |
|---|---|
| `0` | Pinned to Core 0 (PRO_CPU) |
| `1` | Pinned to Core 1 (APP_CPU) |
| `tskNO_AFFINITY` | Runs on either core |

Query at runtime: `xTaskGetCoreID(handle)`, `xTaskGetCurrentTaskHandleForCore(core)`.

### FPU / float

Using `float` in a task **auto-pins it to the current core** (lazy FPU context switch). `double` uses software emulation — no HW acceleration on P4.

### Periodic / timed tasks

```c
vTaskDelay(pdMS_TO_TICKS(20));          // relative delay
xTaskDelayUntil(&last_wake, period);    // fixed-period loop (no drift)
```

### Stack high-water mark

```c
UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL); // bytes remaining (min ever seen)
```

Useful for sizing; call from the task itself after a representative run.

---

## 2. Critical Sections & Mutexes

### Spinlock critical sections (SMP-safe)

```c
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// From a task
taskENTER_CRITICAL(&s_lock);
// ... protected code (keep short) ...
taskEXIT_CRITICAL(&s_lock);

// From an ISR
taskENTER_CRITICAL_ISR(&s_lock);
taskEXIT_CRITICAL_ISR(&s_lock);
```

Spinlock critical sections disable interrupts on the calling core **and** spin-wait if the other core holds the lock. Keep the protected region as short as possible.

### FreeRTOS mutexes (task-context only)

```c
SemaphoreHandle_t m = xSemaphoreCreateMutex();
xSemaphoreTake(m, portMAX_DELAY);
xSemaphoreGive(m);

// With heap capability
SemaphoreHandle_t m = xSemaphoreCreateMutexWithCaps(MALLOC_CAP_INTERNAL);
vSemaphoreDeleteWithCaps(m);
```

Mutexes support priority inheritance; spinlock critical sections do not. Use mutexes for longer-held locks between tasks. **Never take a mutex from an ISR.**

### Recursive mutex

```c
SemaphoreHandle_t m = xSemaphoreCreateRecursiveMutex();
xSemaphoreTakeRecursive(m, portMAX_DELAY);
xSemaphoreGiveRecursive(m);
```

### Binary / counting semaphores

```c
SemaphoreHandle_t s = xSemaphoreCreateBinary();
xSemaphoreGiveFromISR(s, &higher_prio_task_woken);
portYIELD_FROM_ISR(higher_prio_task_woken);
```

---

## 3. Queues

```c
QueueHandle_t q = xQueueCreate(depth, sizeof(MyMsg_t));

// Send (from task or ISR)
xQueueSend(q, &msg, portMAX_DELAY);
xQueueSendFromISR(q, &msg, &woken);

// Receive
xQueueReceive(q, &msg, portMAX_DELAY);

// With capability
QueueHandle_t q = xQueueCreateWithCaps(depth, sizeof(MyMsg_t), MALLOC_CAP_INTERNAL);
vQueueDeleteWithCaps(q);
```

---

## 4. Event Groups

```c
EventGroupHandle_t eg = xEventGroupCreate();

// Set bits (from ISR use xEventGroupSetBitsFromISR)
xEventGroupSetBits(eg, BIT0 | BIT1);

// Wait for any or all bits
EventBits_t bits = xEventGroupWaitBits(eg, BIT0 | BIT1,
    pdTRUE,         // clear on exit
    pdFALSE,        // wait for ANY (pdTRUE = wait for ALL)
    portMAX_DELAY);

// With capability
EventGroupHandle_t eg = xEventGroupCreateWithCaps(MALLOC_CAP_INTERNAL);
vEventGroupDeleteWithCaps(eg);
```

---

## 5. Ring Buffers (ESP-IDF addition)

Three types:

| Type | Behavior |
|---|---|
| `RINGBUF_TYPE_NOSPLIT` | Items stored contiguously; supports acquire/complete deferred write |
| `RINGBUF_TYPE_ALLOWSPLIT` | Items may split across the wrap; use `xRingbufferReceiveSplit()` |
| `RINGBUF_TYPE_BYTEBUF` | Raw byte stream, no item boundaries |

```c
RingbufHandle_t rb = xRingbufferCreate(SIZE, RINGBUF_TYPE_NOSPLIT);
// or with heap caps:
RingbufHandle_t rb = xRingbufferCreateWithCaps(SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);

// Send
xRingbufferSend(rb, &item, sizeof(item), portMAX_DELAY);
xRingbufferSendFromISR(rb, &item, sizeof(item), &woken);

// Deferred write (NOSPLIT only) — zero-copy into ring buffer
void *slot;
xRingbufferSendAcquire(rb, &slot, sizeof(item), portMAX_DELAY);
memcpy(slot, &item, sizeof(item));
xRingbufferSendComplete(rb, slot);

// Receive
size_t sz;
void *p = xRingbufferReceive(rb, &sz, portMAX_DELAY);
// ... use p ...
vRingbufferReturnItem(rb, p);

// Byte buffer
size_t sz;
void *p = xRingbufferReceiveUpTo(rb, &sz, portMAX_DELAY, MAX_BYTES);
vRingbufferReturnItem(rb, p);
```

Ring buffers integrate with `xQueueSet` for multiplexed waits across multiple sources.

---

## 6. Memory Allocation

### heap_caps_malloc

```c
#include "esp_heap_caps.h"

void *p = heap_caps_malloc(size, caps);
void *p = heap_caps_calloc(n, size, caps);
void *p = heap_caps_realloc(p, size, caps);
void *p = heap_caps_aligned_alloc(alignment, size, caps);
heap_caps_free(p);
```

Standard `malloc()` / `free()` use `MALLOC_CAP_DEFAULT` internally — same as calling `heap_caps_malloc_default()`.

### Capability flags

| Flag | Meaning |
|---|---|
| `MALLOC_CAP_INTERNAL` | Internal SRAM only; safe when cache is disabled |
| `MALLOC_CAP_SPIRAM` | External PSRAM (32 MB on this board) |
| `MALLOC_CAP_DMA` | DMA-accessible; excludes PSRAM unless combined with `MALLOC_CAP_SPIRAM` |
| `MALLOC_CAP_8BIT` | Byte-accessible (all DRAM heaps) |
| `MALLOC_CAP_32BIT` | 32-bit aligned access only (IRAM) |
| `MALLOC_CAP_IRAM_8BIT` | IRAM with unaligned access support |
| `MALLOC_CAP_SIMD` | 16-byte aligned; SIMD-instruction compatible |
| `MALLOC_CAP_DEFAULT` | General purpose (same as `malloc`) |

### Common patterns

```c
// Large buffer in PSRAM (LVGL heap, audio buffers, image assets)
void *buf = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);

// DMA buffer (I2S, SPI) — must be in internal DRAM
void *dma = heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

// DMA + PSRAM (if DMA engine supports external memory)
void *dma_ext = heap_caps_aligned_alloc(alignment, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);

// Stack-declared DMA buffer (discouraged; if unavoidable)
DMA_ATTR uint8_t buf[SIZE]; // static / global
WORD_ALIGNED_ATTR uint8_t stack_buf[SIZE]; // stack — risky
```

### Heap info

```c
size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
size_t free_psram    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
size_t largest       = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
size_t low_water     = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);

multi_heap_info_t info;
heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);
// info.total_free_bytes, info.minimum_free_bytes, info.largest_free_block, etc.
```

---

## 7. Memory Types on ESP32-P4

| Region | Access | DMA | Cache-backed | Notes |
|---|---|---|---|---|
| DRAM (internal SRAM) | Byte | Yes | No | Default heap; safe for DMA, ISR, critical sections |
| IRAM | 32-bit (or 8-bit via `IRAM_ATTR`) | No | No | ISR handlers with `ESP_INTR_FLAG_IRAM`; time-critical code |
| D/IRAM | Both | Partial | No | Dual-purpose; allocation reduces both pools |
| IROM (flash) | Read-only | No | Yes (MMU) | Default code location; cache miss on first access |
| DROM (flash) | Read-only | No | Yes (MMU) | Constant data (`const`) |
| PSRAM (SPIRAM) | Byte | Via cache | Yes (MMU) | 32 MB external; must cache-sync before/after DMA |
| SPM (Scratchpad) | Byte | No | No | Near-core deterministic: 1 cycle (no parity) or 4 cycles |
| RTC FAST | Byte | No | No | Survives light sleep; added to heap by default |

**Key rule for DMA + PSRAM:** DMA bypasses the cache. You must call `esp_cache_msync()` before and/or after DMA operations on PSRAM-backed buffers (see §8).

---

## 8. Cache / Memory Synchronization

Required any time DMA and CPU access the same buffer, especially in PSRAM.

```c
#include "esp_cache.h"
// CMakeLists.txt: PRIV_REQUIRES esp_mm
```

### esp_cache_msync

```c
esp_err_t esp_cache_msync(void *addr, size_t size, int flags);
```

| Scenario | Flags | Timing |
|---|---|---|
| CPU wrote buffer → DMA will read | `ESP_CACHE_MSYNC_FLAG_DIR_C2M` | Call **before** starting DMA |
| DMA wrote buffer → CPU will read | `ESP_CACHE_MSYNC_FLAG_DIR_M2C` | Call **after** DMA completes |
| Default (no direction flag) | C2M (writeback) | — |

Extra flags:
- `ESP_CACHE_MSYNC_FLAG_TYPE_DATA` — data region (default)
- `ESP_CACHE_MSYNC_FLAG_TYPE_INST` — instruction region
- `ESP_CACHE_MSYNC_FLAG_INVALIDATE` — also invalidate cache after writeback
- `ESP_CACHE_MSYNC_FLAG_UNALIGNED` — skip alignment check (risk of corrupting adjacent data)

Addresses and sizes **must be cache-line aligned**. Use:

```c
size_t line = esp_cache_get_line_size_by_addr(addr); // 0 = not cacheable
```

Return codes: `ESP_OK`, `ESP_ERR_INVALID_ARG` (misaligned or not cacheable), `ESP_ERR_NOT_SUPPORTED`.

**Do not call during flash operations** (NVS, OTA, `esp_flash_*`) unless XIP-from-PSRAM is configured.

### Typical DMA pattern

```c
// Allocate in PSRAM
uint8_t *buf = heap_caps_aligned_alloc(cache_line_size, BUF_SIZE, MALLOC_CAP_SPIRAM);

// CPU fills buffer
fill_buffer(buf, BUF_SIZE);

// Writeback cache → PSRAM so DMA sees the data
esp_cache_msync(buf, BUF_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

// Start DMA transfer (reads from PSRAM)
start_dma_read(buf, BUF_SIZE);
wait_dma_done();

// DMA wrote result into buf — invalidate CPU cache so we read fresh data
esp_cache_msync(buf, BUF_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

// CPU reads result
process_buffer(buf, BUF_SIZE);
```

---

## 9. Relevant Patterns in This Project

### LVGL lock (from `prop_ui.c`)
`lvgl_port_lock()` / `lvgl_port_unlock()` wraps an `esp_lcd` mutex — this is **not** an SMP spinlock critical section. It is safe for task-context use but must never be held while calling DMA or flash APIs.

### PSRAM-backed LVGL heap (`lv_port_mem.c`)
All `lv_malloc` / `lv_free` route to `heap_caps_malloc(MALLOC_CAP_SPIRAM)`. The LVGL draw buffers are also in PSRAM. The esp_lcd / DPI panel driver handles framebuffer sync; direct pixel writes (as in `prop_fx.c`) bypass LVGL's draw pipeline and write into the DPI framebuffer directly — the `/screenshot` handler does `Cache_Invalidate_Addr` before reading the framebuffer for this reason.

### Audio DMA (I2S — `prop_audio.c`, `prop_mic.c`)
I2S DMA buffers must be in internal DRAM (`MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`). The `bsp_audio` BSP handles this. If you ever move audio staging buffers to PSRAM, add `esp_cache_msync` C2M before handing the pointer to the I2S driver.

### Task priorities (approximate, confirm in code)
Higher number = higher priority in FreeRTOS.

| Task | Suggested priority | Core |
|---|---|---|
| LVGL timer (esp_lvgl_port) | High (e.g. 4–5) | 0 or unpinned |
| `prop_engine` (10 Hz state machine) | Medium (3) | unpinned |
| `prop_net` / WiFi | Medium (3–4) | unpinned |
| `prop_audio` synthesis | High (5) | 1 (avoid competing with LVGL) |
| `prop_mic` PDM capture | Medium-high (4) | 1 |
| `prop_ble` scan | Low-medium (2) | unpinned |
| `prop_api` HTTP | Low (2) | unpinned |
