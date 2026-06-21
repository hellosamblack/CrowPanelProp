/* Custom LVGL allocator — route every lv_malloc() to PSRAM.
 *
 * Why: LVGL 9 allocates line/shape anti-alias mask buffers via lv_malloc *during
 * drawing* (v8 used a separate reusable buffer pool). On instrument screens the
 * waveform's skewed lines need a ~1 KB mask, but the builtin heap is capped at 32 KB
 * of INTERNAL RAM — it cannot be raised because esp_hosted's SDIO DMA mempool needs
 * that internal RAM (raising CONFIG_LV_MEM_SIZE_KILOBYTES boot-loops). The pool runs
 * dry, lv_malloc returns NULL, and v9 doesn't null-check -> lv_memset(NULL,...) faults
 * (Store access fault in draw_line_skew).
 *
 * Routing LVGL's heap to the 32 MB PSRAM removes the ceiling entirely and leaves
 * internal RAM untouched for esp_hosted/LWIP. The display draw buffers are allocated
 * separately by esp_lvgl_port (already PSRAM via buff_spiram), so this only affects
 * LVGL objects + transient draw masks. Selected via CONFIG_LV_USE_CUSTOM_MALLOC. */
#include "sdkconfig.h"
#include "lvgl.h"
/* Key off the Kconfig bool directly (from sdkconfig.h): LVGL's lv_conf_kconfig.h bridges
 * CONFIG_LV_USE_CUSTOM_MALLOC -> LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM, so when this is
 * set LVGL leaves lv_*_core undefined and expects us to provide them. */
#if defined(CONFIG_LV_USE_CUSTOM_MALLOC)

#include "esp_heap_caps.h"
#include <stddef.h>

#define LV_PSRAM_CAPS (MALLOC_CAP_SPIRAM  | MALLOC_CAP_8BIT)
#define LV_SRAM_CAPS  (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

/* Small allocations (transient draw-mask buffers LVGL 9 allocates per-line during
 * software rendering, typically ≤ 512 B) go to internal SRAM first for lower latency.
 * Larger allocations (widget trees, image caches) go straight to PSRAM.
 * PSRAM fallback keeps us safe if internal RAM runs low. */
#define LV_SRAM_THRESHOLD 512

void lv_mem_init(void) { }
void lv_mem_deinit(void) { }

lv_mem_pool_t lv_mem_add_pool(void *mem, size_t bytes)
{
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;   /* single PSRAM heap; explicit pools not used */
}

void lv_mem_remove_pool(lv_mem_pool_t pool) { LV_UNUSED(pool); }

void *lv_malloc_core(size_t size)
{
    if (size <= LV_SRAM_THRESHOLD) {
        void *p = heap_caps_malloc(size, LV_SRAM_CAPS);
        if (p) return p;
    }
    return heap_caps_malloc(size, LV_PSRAM_CAPS);
}

void *lv_realloc_core(void *p, size_t new_size) { return heap_caps_realloc(p, new_size, LV_PSRAM_CAPS); }

void lv_free_core(void *p) { heap_caps_free(p); }

void lv_mem_monitor_core(lv_mem_monitor_t *mon_p)
{
    if (!mon_p) return;
    lv_memzero(mon_p, sizeof(*mon_p));
    mon_p->free_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
}

lv_result_t lv_mem_test_core(void) { return LV_RESULT_OK; }

#endif /*LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM*/
