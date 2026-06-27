/* prop_ppa_spike — offline PPA blend A8 latency / correctness spike.
 *
 * Phase B, step 1: allocates scratch PSRAM buffers, registers a PPA blend
 * client, runs ppa_do_blend (A8 foreground / fixed black) vs an equivalent
 * software src-over loop, logs timing + correctness data, then frees
 * everything. Never touches the live framebuffer or any LVGL path.
 *
 * Trigger: POST /cmd {"cmd":"fx","ppaspike":true}
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/* Run the one-shot PPA blend spike measurement. Logs results to serial and
 * returns. All scratch buffers and PPA client are freed/unregistered before
 * this function returns. Must NOT be called under the LVGL lock. */
void prop_ppa_spike_run(void);

#ifdef __cplusplus
}
#endif
