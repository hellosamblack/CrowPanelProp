/*
 * prop_csi_cfg_list.h — the single canonical list of runtime-configurable
 * CSI / ESPectre settings, as an X-macro. Both the P4 host (prop_coproc.c) and
 * the C6 slave (prop_csi_slave.c) #include this to generate their settings
 * table / config store, so the two stay in lockstep. Adding a setting = ONE
 * line here (in BOTH copies — main/include and c6_slave/slave/main, byte-identical).
 *
 *   PROP_CSI_CFG(key, type, deflt, lo, hi, desc, opts)
 *     key   : ESPectre's exact YAML name (or a prop-specific name). <PROP_CSI_KEYLEN.
 *     type  : 'I' int (stepper)         'F' float ×1000 (stepper)
 *             'B' bool (OFF/ON)         'E' enum, value = option index
 *             'M' bitmask, bit i = 1<<i (toggle buttons)
 *             'T' threshold mode: -1 auto, -2 min, else manual ×1000
 *     deflt : default value (floats pre-scaled ×1000), within [lo,hi]
 *     lo,hi : inclusive range (steppers + enum index bounds)
 *     desc  : short human description (CSI SETTINGS editor)
 *     opts  : "|"-separated button labels for B/E/M; "" for steppers/threshold
 *
 * Floats are ×1000 on the wire/NVS; the slave divides by 1000 feeding ESPectre.
 * The slave keys off the NAME (ignores type/opts), so type changes are host-only.
 */

/* clang-format off */
#ifndef PROP_CSI_CFG
#define PROP_CSI_CFG(key, type, deflt, lo, hi, desc, opts)
#endif

/* prop-specific: which PPDU types yield CSI frames (bitmask). */
PROP_CSI_CFG(acquire,            'M', 0x03,  0,     7,     "Wi-Fi frame types captured for CSI", "BEACON|HT20|HT40")

/* ESPectre — detection / segmentation */
PROP_CSI_CFG(detection_algorithm,'E', 0,     0,     1,     "Detector engine", "MVS|ML")
PROP_CSI_CFG(segmentation_threshold,'T', -1, -2,    10000, "Motion threshold (AUTO learns it; MANUAL = fixed)", "")
PROP_CSI_CFG(segmentation_window_size,'I', 100, 10, 200,   "Moving-variance window, packets", "")
PROP_CSI_CFG(evaluation_interval,'I', 25,    1,     1000,  "Packets between motion evaluations", "")
PROP_CSI_CFG(motion_on_hits,     'I', 3,     1,     20,    "Consecutive over-threshold hits to enter MOTION", "")
PROP_CSI_CFG(motion_off_hits,    'I', 3,     1,     20,    "Consecutive under-threshold hits to leave MOTION", "")
PROP_CSI_CFG(gain_lock,          'E', 0,     0,     2,     "AGC/FFT gain lock mode", "AUTO|ON|OFF")

/* ESPectre — filters */
PROP_CSI_CFG(lowpass_enabled,    'B', 0,     0,     1,     "Low-pass smoothing of the movement signal", "")
PROP_CSI_CFG(lowpass_cutoff,     'F', 11000, 5000,  20000, "Low-pass cutoff, Hz", "")
PROP_CSI_CFG(hampel_enabled,     'B', 1,     0,     1,     "Hampel outlier (turbulence-spike) filter", "")
PROP_CSI_CFG(hampel_window,      'I', 7,     3,     11,    "Hampel filter window, samples", "")
PROP_CSI_CFG(hampel_threshold,   'F', 5000,  1000,  10000, "Hampel sensitivity (MAD multiplier)", "")

/* ESPectre — traffic generator (keeps a CSI stream flowing on an idle link) */
PROP_CSI_CFG(traffic_generator_mode,'E', 0,  0,     1,     "Self-traffic protocol", "PING|DNS")
PROP_CSI_CFG(traffic_generator_rate,'I', 100, 0,    1000,  "Self-traffic rate to gateway, packets/sec", "")
PROP_CSI_CFG(publish_interval,   'I', 100,   1,     1000,  "Packets between verdict updates", "")

#undef PROP_CSI_CFG
/* clang-format on */
