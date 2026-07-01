/* prop_ui - cassette-futurism LVGL readout, driven by prop_engine state. */
#include "prop_ui.h"
#include "prop_engine.h"
#include "prop_net.h"
#include "prop_settings.h"
#include "prop_fx.h"
#include "prop_mic.h"
#include "prop_audio.h"
#include "prop_ble.h"
#include "prop_csi.h"
#include "prop_ftm.h"
#include "prop_calib.h"
#include "prop_coproc.h"
#include "prop_motion.h"
#include "prop_imu.h"
#include "prop_track.h"
#include "prop_aux_radar.h"
#include "prop_content.h"
#include "bsp_io.h"
#include "bsp_aio.h"
#include "bsp_illuminate.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/temperature_sensor.h"
#include "lvgl.h"

/* Palette (COL_*), fonts (FONT_*) and reusable components live in the design kit —
 * single source of truth. See prop_kit.h. */
#include "prop_kit.h"

#define UI_TAG "PROP_UI"
#define SCAN_MAX 20

#define SCREEN_W 1024           /* physical panel width */
#define RAIL_W   76             /* persistent icon nav rail (left edge, all screens) */
/* SCAN_W is the CONTENT width (right of the rail). All panels/readouts lay out
 * relative to this, so redefining it here insets every screen with no per-panel edits. */
#define SCAN_W (SCREEN_W - RAIL_W)
#define SCAN_TRACK_Y 330        /* taller, lower trace - the hero waveform */
#define SCAN_TRACK_H 160

static lv_obj_t *s_status_label;
static lv_obj_t *s_channel_label;
static lv_obj_t *s_link_label;
static lv_obj_t *s_ip_label;        /* STA IP, dim, under the LINK line */
static lv_obj_t *s_scan_track;
static lv_obj_t *s_scan_blip;
/* The recorder trail is split into segments, each an lv_line indexing into the
 * shared point buffer. Only one new sample lands per tick, so re-rendering just
 * the segment(s) whose slice changed cuts per-frame redraw area ~WAVE_SEGS-fold
 * (the whole 950x160 track redraw was the 72 ms/frame bottleneck). */
#define WAVE_SEGS    10
#define WAVE_SEG_LEN (PROP_WAVE_SAMPLES / WAVE_SEGS)   /* 16 samples per segment */
static lv_obj_t  *s_wave_seg[WAVE_SEGS];            /* segment lines across the track */
static lv_point_precise_t s_wave_pts[PROP_WAVE_SAMPLES];    /* persistent points buffer for lv_line */
static lv_coord_t s_wave_shadow[PROP_WAVE_SAMPLES]; /* last-rendered y, to detect changed segments */
static lv_color_t s_wave_color;                     /* current trace tint (recolor only on change) */

/* Change-trackers so the scanner readout only invalidates widgets that actually
 * change each frame (the big headline re-rasterizing every tick was the cost). */
static lv_coord_t s_last_blip_x = -9999, s_last_marker_x = -9999;
static uint16_t s_last_blip_col = 0xFFFF, s_last_headline_col = 0xFFFF, s_last_marker_col = 0xFFFF;
static int s_last_punch = -1, s_last_sens = -1;

/* Sample count for segment k (segments share a boundary sample so they join). */
static int wave_seg_count(int k)
{
    int start = k * WAVE_SEG_LEN;
    int end = (k == WAVE_SEGS - 1) ? (PROP_WAVE_SAMPLES - 1) : (start + WAVE_SEG_LEN);
    return end - start + 1;
}

/* Channel tuning gauge: a frequency band with a marker at the tuned position. */
static lv_obj_t *s_chan_band;       /* the band rectangle (for inner width) */
static lv_obj_t *s_chan_marker;     /* bright marker, moved to chan_pos each frame */

/* SENS (receiver gain) meter: a horizontal bar fill + a numeric percent. */
static lv_obj_t *s_sens_fill;
static lv_obj_t *s_sens_val;

/* Signal-strength meter: a "SIG" label + a row of square cells that fill amber
 * with signal strength (empty cells are dim outlines), alongside the LINK line. */
#define SIG_CELLS 5
static lv_obj_t *s_sig_cells[SIG_CELLS];   /* outline squares */
static lv_obj_t *s_sig_fill[SIG_CELLS];    /* amber fill inside each, bottom-up */
static lv_obj_t *s_sig_box;     /* meter container, re-anchored to the LINK label */

/* WiFi setup panel widgets */
static lv_obj_t *s_setup_panel;     /* hidden overlay */
static lv_obj_t *s_ssid_dd;         /* dropdown of discovered SSIDs */
static lv_obj_t *s_pass_ta;
static lv_obj_t *s_show_btn;        /* reveal-password toggle */
static lv_obj_t *s_remember_cb;     /* "Remember" checkbox */
static lv_obj_t *s_forget_btn;      /* shown only when the saved net is selected */
static lv_obj_t *s_keyboard;
static lv_obj_t *s_setup_status;

/* Scan results shared between the scan task and the UI. */
static prop_ap_t s_aps[SCAN_MAX];
static volatile bool s_scanning;
static bool s_connect_pending;      /* awaiting connect result for status text */
static bool s_pass_shown;

/* ---- Setup/instrument screens: lazily built, ONE alive at a time ---------
 * LVGL's heap (LV_MEM, 32 KB) cannot grow - esp_hosted's SDIO DMA mempool needs
 * the internal RAM, so a bigger LV_MEM boot-loops ("HS_MP: mempool ... no mem").
 * Therefore panels are NOT all pre-built: exactly one is alive at a time, built
 * on navigation and torn down on leave. This caps LVGL usage at "main screen +
 * one panel" regardless of how many panel types exist (and scales to new ones). */

typedef enum {
    PK_NONE = 0,   /* no panel: the bare SCANNER readout on the root screen */
    PK_HOME,       /* the in-world console (default landing screen) */
    PK_MENU, PK_WIFI, PK_DISPLAY, PK_AUDIO, PK_LEDS, PK_ABOUT, PK_VITALS,
    PK_SCAN, PK_SPECTRUM, PK_RFBAND, PK_BLE, PK_CSI, PK_CSICFG, PK_CSISET, PK_MOTION,
    PK_DIRCAL,        /* travel-direction calibration (opened from the SCANNER apex dot) */
    PK_MINIMAP,       /* north-up dead-reckoning map (breadcrumb path + radar marks) */
    PK_RANGE,         /* WiFi FTM ranging table (per-BSSID distance-to-AP) */
    PK_INSTRUMENTS,   /* submenu: SCANNER / SIGNAL SCAN / SPECTRUM / VITALS */
    PK_SENSORS,       /* submenu: RF BAND / CONTACTS / SIGNAL ENV / CSI CONFIG / SCANNER / MINIMAP / RANGE */
    PK_ARCHIVE,    /* data-archive browser (tabs = sections) */
    PK_ARTICLE,    /* a single archive entry */
    PK_CASSETTE,   /* cassette deck (stub) */
    PK_INSIGHTS,   /* insight engine (stub) */
    PK_IO,         /* I/O bench overview (grid of pin boxes) */
    PK_IO_PIN,     /* I/O bench single-pin config page (s_io_pin) */
} panel_kind_t;

static lv_obj_t *s_root;          /* the CONTENT container (panels are built on it) */
static lv_obj_t *s_rail_strip;    /* persistent icon nav rail on the real screen (left edge) */
static bool s_ui_ready;           /* true once boot build is done - gates screen transitions */
static lv_obj_t *s_cur_panel;     /* the one live setup panel, or NULL (main) */
static panel_kind_t s_cur_kind;

/* Live value readouts updated by panels / the observer - valid ONLY while the
 * owning panel is the current one (NULLed on teardown). */
static lv_obj_t *s_disp_bright_val, *s_audio_vol_val;
static lv_obj_t *s_fx_scan_val, *s_fx_phos_val, *s_fx_vign_val, *s_fx_refr_val;
static lv_obj_t *s_about_ip, *s_about_uptime;

/* VITALS instrument live readouts (valid only while PK_VITALS is current). */
static lv_obj_t *s_vit_temp, *s_vit_ram, *s_vit_uptime, *s_vit_cell;
static lv_obj_t *s_vit_temp_bar, *s_vit_ram_bar, *s_vit_cell_bar;
static lv_obj_t *s_vit_up_net, *s_vit_up_ap, *s_vit_up_link;   /* UPLINK dossier rows */
/* IMU motion sensor rows */
static lv_obj_t *s_vit_yaw, *s_vit_pitch, *s_vit_roll;
static lv_obj_t *s_vit_yaw_bar, *s_vit_pitch_bar, *s_vit_roll_bar;
static lv_obj_t *s_vit_accel;
static lv_obj_t *s_vit_imu_temp;   /* MPU-6050 die temperature */
static lv_obj_t *s_vit_steps;      /* pedometer step count */

/* SIGNAL SCAN instrument (valid only while PK_SCAN is current). */
static lv_obj_t *s_sig_list, *s_sig_status;
static volatile bool s_sig_scanning;

/* SPECTRUM instrument (valid only while PK_SPECTRUM is current). */
static lv_obj_t *s_spec_bars[PROP_MIC_BANDS];
static lv_obj_t *s_spec_db, *s_spec_db_bar, *s_spec_status;
static lv_obj_t *s_spec_src_label;
static lv_obj_t *s_spec_axis[5];    /* x-axis frequency labels */
static float s_spec_decay[PROP_MIC_BANDS];   /* peak-hold / slow decay (UI-side) */
static int8_t s_spec_band[PROP_MIC_BANDS];   /* -1=unset; last color band per bar (0=MUTE,1=AMBER,2=ALERT) */

/* RF BAND instrument (2.4 GHz channel occupancy; valid only while PK_RFBAND is current). */
#define RF_CHANNELS 13                      /* 2.4 GHz channels 1..13 */
static lv_obj_t *s_rf_bars[RF_CHANNELS];
static lv_obj_t *s_rf_status;
static uint8_t s_rf_chan[PROP_NET_CHAN_SLOTS]; /* cached histogram (filled by the scan task) */
static float s_rf_decay[RF_CHANNELS];          /* UI-side rise/decay ballistics */
static int8_t s_rf_band[RF_CHANNELS];          /* -1=unset; last color band per bar (0=MUTE,1=AMBER,2=ALERT) */
static volatile bool s_rf_scanning;

/* BLE CONTACT SIGNATURES instrument (valid only while PK_BLE is current). */
static lv_obj_t *s_ble_summary;
static lv_obj_t *s_ble_list;

typedef struct { lv_obj_t *tag, *dbm, *mac, *dist, *uuid, *tx, *fill; } ble_row_t;
static ble_row_t s_ble_rows[PROP_BLE_MAX];

/* RANGE (WiFi FTM) instrument (valid only while PK_RANGE is current). */
static lv_obj_t *s_range_summary;
static lv_obj_t *s_range_list;

typedef struct { lv_obj_t *tag, *rssi, *mac, *dist, *status, *fill; } range_row_t;
static range_row_t s_range_rows[PROP_FTM_TABLE_MAX];

/* SIGNAL ENVIRONMENT (CSI) instrument (valid only while PK_CSI is current). */
static lv_obj_t *s_csi_bars[PROP_CSI_BINS];
static int8_t s_csi_band[PROP_CSI_BINS];     /* -1=unset; last color band per bar (0=MUTE,1=AMBER,2=ALERT) */
static lv_obj_t *s_csi_status;
static lv_obj_t *s_csi_motion;   /* big MOTION / IDLE state readout */
static lv_obj_t *s_csi_move;     /* movement-vs-threshold numeric readout */
static lv_obj_t *s_csi_rf;       /* turbulence + receiver gain readout */
static lv_obj_t *s_csi_rssi;     /* link RSSI (upper-right) */
static lv_obj_t *s_csi_fp_cells[64]; /* NBVI subcarrier fingerprint (lit cells) */
static lv_obj_t *s_csi_geiger_lbl; /* SPECTRE GEIGER button label */

/* MOTION SCAN instrument (valid only while PK_MOTION is current). */
static lv_obj_t *s_motion_tgt_label;    /* merged headline: "N TARGETS DETECTED" / "NO TARGETS" */
static lv_obj_t *s_motion_blips[3];     /* target blips (amber circles) */
static lv_obj_t *s_motion_tdist[3];     /* T1/T2/T3 distance labels (large) */
static lv_obj_t *s_motion_tvel[3];      /* T1/T2/T3 velocity labels */
static lv_obj_t *s_motion_tarrow[3];    /* T1/T2/T3 bearing arrow lines */
#define PULSE_BAND  9      /* (PULSE_BAND-1)*PULSE_WIDTH = radial band thickness in px */
#define PULSE_WIDTH 4      /* -> 32 px thick band; also sets the apex->rim wrap distance */
#define PULSE_PEAK_OPA 150 /* leading-edge opacity; kept well below the target blips (255)
                            * so the sweep never reads brighter than an actual contact */
#define PULSE_WAVE_STEPS 5     /* radial terrace steps -> the geometric banded look */
#define PULSE_WAVE_AMP   4.0f  /* px scallop amplitude rippling the leading edge */
#define PULSE_WAVE_FREQ  46.0f /* scallop frequency (multiplies the horizontal slope) */
#define PULSE_AMBER_RGB  0xE0B000u /* COL_AMBER as 0xRRGGBB for direct ARGB8888 writes */
/* The pulse is hand-painted into its own ARGB8888 canvas each tick (mirrors prop_fx) —
 * per-pixel alpha gives the wavy/terraced texture lv_draw_arc can't express, and costs
 * zero AA-mask PSRAM allocs. Only the tight band bbox is cleared/painted/invalidated. */
static lv_obj_t *s_motion_pulse;     /* the pulse canvas (child of rbox) */
static uint32_t *s_pulse_buf;        /* ARGB8888 buffer (PSRAM); NULL when torn down */
static int       s_pulse_lead_px;    /* current leading-edge radius (px) */
static int       s_pulse_phase;      /* travelling-wave phase, advances per tick */
static lv_area_t s_pulse_prev_local; /* last frame's band bbox, canvas-local coords */
static bool      s_pulse_prev_valid; /* false until the first paint */
static float     s_pulse_sin[256];   /* scallop sin LUT (filled in build_motion_panel) */
static lv_obj_t *s_motion_gdial[3];     /* YAW/PITCH/ROLL gimbal needle lines */
static lv_obj_t *s_motion_gval[3];      /* per-dial value labels */
static lv_obj_t *s_motion_ble_box[3];   /* 3 strongest BLE contact boxes */
static lv_obj_t *s_motion_ble_name[3];  /* contact name (top, truncated) */
static lv_obj_t *s_motion_ble_dist[3];  /* contact distance (large, metres) */
static lv_obj_t *s_motion_ble_dbm[3];   /* contact RSSI (small, corner) */
/* All-sensor dashboard widgets (live inside the radar box, reference style). */
#define MOTION_FLAG_COUNT 5
static lv_obj_t *s_motion_flags[MOTION_FLAG_COUNT];   /* MOV IMU MIC BLE NET */
#define MOTION_BAR_COUNT 6
static lv_obj_t *s_motion_bars[MOTION_BAR_COUNT];     /* CSI RF MIC SIG ACC FRQ analog bars */
static lv_obj_t *s_motion_bar_lbls[MOTION_BAR_COUNT];
static lv_obj_t *s_motion_dir_q[4];     /* N/E/S/W direction-ring quadrant arcs */
static lv_obj_t *s_motion_dir_dot;      /* operator dot at the ring centre (fan apex) */
/* Animated radar scale: one pixels-per-metre value drives arcs, labels, and blips. */
static float     s_motion_ppm;            /* current px per metre (animated in Task 4) */
static lv_obj_t *s_motion_grid_arc[6];    /* major range rings, 1..6 m */
static lv_obj_t *s_motion_grid_lbl[6];    /* "Nm" labels riding the right fan edge */
/* Autorange ladder — full-scale distance (m) at the rim. LD2450 max ~6 m. */
static const float MOTION_LADDER[5] = { 1.0f, 2.0f, 3.0f, 4.5f, 6.0f };
#define MOTION_NLEVEL 5
static int   s_motion_level;        /* committed ladder index */
static int   s_motion_level_pend;   /* candidate index awaiting dwell */
static int   s_motion_dwell;        /* ticks the candidate has persisted */
static float s_motion_ppm_from;     /* slide start ppm */
static float s_motion_ppm_to;       /* slide target ppm */
static int   s_motion_anim_ms;      /* slide elapsed ms; -1 = idle */
static lv_obj_t *s_motion_grid_minor[2];  /* 0.5 m / 1.5 m rings, shown when scale <= 2 m */
/* Comet trails: per-target ring buffer of recent world samples (mm), re-projected
 * each frame so the tail rescales with autorange. */
#define TRAIL_LEN 8
static lv_obj_t *s_motion_trail[3][TRAIL_LEN];   /* fading dot pool per target */
static int s_trail_x[3][TRAIL_LEN], s_trail_y[3][TRAIL_LEN];  /* world history (mm) */
static int s_trail_n[3];      /* valid samples per slot */
static int s_trail_head[3];   /* ring-buffer head (newest) */
static int s_trail_tick;      /* sample decimation counter */
/* Module-scope point buffers — LVGL keeps a pointer; must outlive the widgets. */
static float s_pulse_lead_m;            /* pulse leading-edge range (m); projected via ppm so it stays locked to the grid */
static lv_point_precise_t s_motion_gpts[3][2];   /* needle points per gimbal dial */
static lv_point_precise_t s_motion_apts[3][2];   /* bearing arrow points per target */
/* Travel-direction ring: gravity estimated as a low-pass of the accelerometer
 * itself (mount/scale-independent) so subtracting it leaves only real motion. */
static float s_motion_grav_lp[3];        /* low-pass gravity estimate (g) */
static bool  s_motion_grav_primed;       /* seeded on first sample after panel open */
/* SCANNER distance-ping (audio): chirps the closest radar target's range. */
static lv_obj_t *s_scan_spk;             /* speaker toggle icon */
static bool      s_scan_ping_en;         /* feedback enabled (persisted in NVS) */
static uint32_t  s_scan_ping_next;       /* tick at which the next chirp may fire */

/* Travel-direction calibration. s_dir_phi rotates the gravity-removed accel vector
 * before the quadrant pick, so the ring's North (top) lines up with the radar
 * boresight ("forward"). Default 0 = identity = legacy behaviour until calibrated.
 * The guided walk flow (PK_DIRCAL) captures one accel accumulator per cardinal. */
static float s_dir_phi;                   /* board->world yaw offset (rad), from NVS */
static lv_obj_t *s_dcal_prompt;           /* big phase instruction word */
static lv_obj_t *s_dcal_sub;              /* secondary line / countdown */
static lv_obj_t *s_dcal_btn_lbl;          /* START / RESTART / DONE button label */
static lv_obj_t *s_dcal_arrow;            /* line arrow pointing the requested way */
static lv_point_precise_t s_dcal_apts[5]; /* arrow polyline (shaft + head; LVGL keeps ptr) */
static int   s_dcal_phase;                /* -1 idle, 0..3 = FWD/BACK/LEFT/RIGHT, 4 done, 5 retry */
static int   s_dcal_ms;                   /* ms elapsed in the current capture phase */
static float s_dcal_acc[4][2];            /* hemisphere-aligned accel accumulator per phase */
static float s_dcal_ref[2];               /* hemisphere reference for the active phase */
static bool  s_dcal_ref_set;
static float s_dcal_grav_lp[3];           /* gravity low-pass during the cal flow */
static bool  s_dcal_grav_primed;
#define DCAL_PHASE_MS 3000                /* capture window per cardinal walk */

/* ---- MINIMAP (PK_MINIMAP): north-up dead-reckoning map ------------------- *
 * The canvas holds the STATIC layer (grid + breadcrumb path + radar marks),
 * repainted only when the data/scale changes (event-driven, not per-frame); the
 * operator dot + heading arrow are widgets glided over it every tick. Same
 * canvas/widget split as the SCANNER, and the same direct-pixel-write rule:
 * ui_observer runs off the LVGL task under the port lock, where the v9 canvas
 * draw pipeline deadlocks, so the path is painted by raw ARGB8888 writes. */
#define MAP_X     8
#define MAP_Y     52
#define MAP_W     932
#define MAP_H     540
#define MAP_PPM_MIN  6.0f     /* px per metre clamp (zoomed out) */
#define MAP_PPM_MAX  90.0f    /* px per metre clamp (zoomed in) */
#define MAP_CRUMB_CAP 512     /* must match prop_track's CRUMB_CAP */
static lv_obj_t *s_map_canvas;            /* ARGB8888 canvas (static map layer) */
static uint32_t *s_map_buf;               /* its PSRAM backing; NULL when torn down */
static lv_obj_t *s_map_op_dot;            /* operator position dot (widget) */
static lv_obj_t *s_map_op_arrow;          /* operator heading arrow (lv_line) */
static lv_point_precise_t s_map_arrow_pts[2];   /* LVGL keeps this pointer */
static lv_obj_t *s_map_info;              /* header status line (hdg / scale / path) */
static lv_obj_t *s_map_msg;               /* centred OFFLINE / WALK-TO-BEGIN notice */
static float s_map_ppm;                   /* committed projection scale (px/m) */
static float s_map_ctr_x, s_map_ctr_y;    /* committed projection centre (world m) */
static bool  s_map_have_fit;              /* false until the first auto-fit commit */
static int   s_map_last_crumbs;           /* dirty tracking: crumb count last drawn */
static uint32_t s_map_marks_sig;          /* dirty tracking: mark-set signature */
/* Observer-side scratch (single-threaded observer; reused each tick — too big for stack). */
static prop_track_crumb_t s_map_crumb_scratch[MAP_CRUMB_CAP];
static prop_track_mark_t  s_map_mark_scratch[PROP_TRACK_MAX_MARKS];

/* SCANNER (PK_MOTION) is a full-screen page (no header/back). The left radar box
 * fills the panel height; the right column of cells matches that height. */
#define RADAR_X   8
#define RADAR_Y   8
#define RADAR_W   456
#define RADAR_H   584

/* Fan radar geometry (inside the radar box). Apex raised off the base so the travel
 * ring + operator dot clear the bottom analog bars; the wedge is narrowed below the
 * LD2450's true ±60° FOV so the width-limited radius grows and the fan fills more of
 * the space above it (half-width = R*sin(half) <= RADAR_W/2 => R <= RADAR_W/(2 sin)). */
#define FAN_APEX_X   228      /* RADAR_W / 2 */
#define FAN_APEX_Y   430      /* raised: ring/dot now clear the bottom bars */
#define FAN_R        296      /* outer radius (width-limited at ±50°: 456/(2 sin50)) */
#define FAN_HALF_DEG  50      /* display half-angle (< LD2450 ±60° FOV — see note) */
#define FAN_LEFT_DEG  (270 - FAN_HALF_DEG)   /* LVGL: 270 = up */
#define FAN_RIGHT_DEG (270 + FAN_HALF_DEG)

/* SCANNER right-column geometry — three cells per row, clearing the radar box on
 * the left and the panel's right border. */
#define MR_X0    472                              /* right column left edge */
#define MR_BOX_W 148                              /* per-box / cell width */
#define MR_PITCH 154                              /* column-to-column pitch */
#define MR_BOX_H 170                              /* per-cell height (3 rows fill the panel) */
#define MR_ROW0  8                                /* gimbals row top */
#define MR_ROW1  194                              /* targets row top */
#define MR_ROW2  402                              /* BLE row top (header rides just above) */

/* CSI CONFIG / auto-calibration panel (valid only while PK_CSICFG is current). */
static lv_obj_t *s_cfg_motion;   /* live MOTION / IDLE */
static lv_obj_t *s_cfg_move;     /* live movement / threshold readout */
static lv_obj_t *s_cfg_meter;    /* movement-vs-threshold bar fill */
static lv_obj_t *s_cfg_phase;    /* big calibration phase / countdown */
static lv_obj_t *s_cfg_btn_lbl;  /* CALIBRATE / CANCEL button label */

/* CSI SETTINGS editor (PK_CSISET): paginated, button-row controls. */
#define MAX_CSI_SET 24
#define CSISET_PER_PAGE 3
static lv_obj_t *s_set_vallbl[MAX_CSI_SET];
static lv_obj_t *s_set_content;   /* container holding the current page's rows */
static lv_obj_t *s_set_pagelbl;   /* "page x/N" */
static int       s_set_page;

/* I/O BENCH (PK_IO overview grid + PK_IO_PIN single-pin config). One panel alive at a
 * time; these widget pointers belong to whichever IO panel is current and are NULLed on
 * teardown. s_io_pin selects which pin the config page edits. */
#define IO_MAX 24
/* Overview grid (PK_IO): per-pin box widgets. */
static lv_obj_t *s_io_box[IO_MAX];     /* box container */
static lv_obj_t *s_io_val[IO_MAX];     /* big pin-number label */
static lv_obj_t *s_io_sub[IO_MAX];     /* state text under the number */
static lv_obj_t *s_io_fill[IO_MAX];    /* analog fill rectangle */
static int       s_io_last[IO_MAX];    /* last-painted state key, to skip redundant repaints */
/* Single-pin config page (PK_IO_PIN): live widgets for the one pin being edited. */
static int       s_io_pin;             /* index of the pin shown by PK_IO_PIN */
static lv_obj_t *s_pin_read;           /* digital level / analog raw+% readout label */
static lv_obj_t *s_pin_fill;           /* analog-in meter fill */
static lv_obj_t *s_pin_edges;          /* interrupt edge-count label */
static lv_obj_t *s_pin_aout;           /* analog-out readout label */
static lv_obj_t *s_pin_slider;         /* analog-out slider */
static lv_obj_t *s_pin_entry;          /* analog-out numeric entry */
static char s_io_err[64];              /* one-shot error banner for the config page */

/* Overview grid geometry: 17 pins as boxes, 6 across, 3 rows, no scroll. Sized to
 * fill the content area below the title (SCAN_W wide, ~600 tall). */
#define IO_COLS  6
#define IO_GAP   10
#define IO_BOX_W 143
#define IO_BOX_H 150

/* Builders + lifecycle (defined below). */
static lv_obj_t *build_home_panel(lv_obj_t *parent);
static lv_obj_t *build_menu_panel(lv_obj_t *parent);
static lv_obj_t *build_wifi_panel(lv_obj_t *parent);
static lv_obj_t *build_display_panel(lv_obj_t *parent);
static lv_obj_t *build_audio_panel(lv_obj_t *parent);
static lv_obj_t *build_leds_panel(lv_obj_t *parent);
static lv_obj_t *build_about_panel(lv_obj_t *parent);
static lv_obj_t *build_vitals_panel(lv_obj_t *parent);
static lv_obj_t *build_signal_panel(lv_obj_t *parent);
static lv_obj_t *build_spectrum_panel(lv_obj_t *parent);
static lv_obj_t *build_rfband_panel(lv_obj_t *parent);
static lv_obj_t *build_ble_panel(lv_obj_t *parent);
static lv_obj_t *build_csi_panel(lv_obj_t *parent);
static lv_obj_t *build_csicfg_panel(lv_obj_t *parent);
static lv_obj_t *build_csiset_panel(lv_obj_t *parent);
static lv_obj_t *build_archive_panel(lv_obj_t *parent);
static lv_obj_t *build_article_panel(lv_obj_t *parent);
static lv_obj_t *build_cassette_panel(lv_obj_t *parent);
static lv_obj_t *build_insights_panel(lv_obj_t *parent);
static lv_obj_t *build_instruments_panel(lv_obj_t *parent);
static lv_obj_t *build_sensors_panel(lv_obj_t *parent);
static lv_obj_t *build_io_panel(lv_obj_t *parent);
static lv_obj_t *build_io_pin_panel(lv_obj_t *parent);
static lv_obj_t *build_motion_panel(lv_obj_t *parent);
static lv_obj_t *build_dircal_panel(lv_obj_t *parent);
static lv_obj_t *build_minimap_panel(lv_obj_t *parent);
static lv_obj_t *build_range_panel(lv_obj_t *parent);
static void wifi_panel_opened(void);
static void start_signal_scan(void);
static void start_rfband_scan(void);
static void set_rail_highlight(void);   /* persistent rail; defined with the rail code */

/* ---- Function rail + dial/tab navigation ---------------------------------
 * The author's control model: a SELECTOR dial moves between top-level FUNCTIONS,
 * a PRESS opens one, switches act as TABS (used inside the ARCHIVE). The physical
 * knobs aren't wired yet, so these are driven from the web portal (/cmd "input")
 * today; bsp_io routes the real controls to the same prop_ui_input() later.
 *
 * The rail is the console's hero: ARCHIVE first (the device IS a data archive),
 * the scanner demoted to one entry among the instruments. */
/* Icon ids for the persistent rail glyphs (drawn from primitives, no font assets). */
typedef enum {
    IC_HOME, IC_ARCHIVE, IC_SCANNER, IC_VITALS, IC_SIGNAL,
    IC_SPECTRUM, IC_RFBAND, IC_CONTACTS, IC_SIGENV,
    IC_INSTRUMENTS, IC_SENSORS,
    IC_CASSETTE, IC_INSIGHTS, IC_SETUP,
} icon_id_t;

static const struct {
    const char *label;
    panel_kind_t kind;
    icon_id_t    icon;
} s_rail[] = {
    { "CONSOLE",     PK_HOME,        IC_HOME        },
    { "ARCHIVE",     PK_ARCHIVE,     IC_ARCHIVE     },
    { "INSTRUMENTS", PK_INSTRUMENTS, IC_INSTRUMENTS },   /* SCANNER/SIGNAL SCAN/SPECTRUM/VITALS */
    { "SENSORS",     PK_SENSORS,     IC_SENSORS     },   /* RF BAND/CONTACTS/SIGNAL ENV (C6 radio) */
    { "CASSETTE",    PK_CASSETTE,    IC_CASSETTE    },
    { "INSIGHTS",    PK_INSIGHTS,    IC_INSIGHTS    },
    { "SETUP",       PK_MENU,        IC_SETUP       },
};
#define RAIL_COUNT ((int)(sizeof(s_rail) / sizeof(s_rail[0])))

static int s_rail_sel;                 /* highlighted function on the console */
static int s_archive_section;          /* current ARCHIVE tab (0..prop_section_count-1) */
static int s_archive_entry;            /* selected entry within the section */

/* Console (PK_HOME) live readouts - valid only while PK_HOME is the live panel. */
static lv_obj_t *s_home_clock, *s_home_temp, *s_home_link;
static lv_obj_t *s_rail_btns[RAIL_COUNT];   /* rail rows, for dial highlighting */

/* ARCHIVE browser widgets - valid only while PK_ARCHIVE is live. */
static lv_obj_t *s_arch_tabs[8];        /* section tab buttons (>= prop_section_count) */
static lv_obj_t *s_arch_rows[16];       /* entry rows, for dial highlighting */
static int s_arch_row_count;
static lv_obj_t *s_article_scroll;      /* PK_ARTICLE body scroll container (dial scrolls it) */

/* Tear down the live panel and invalidate every per-panel widget pointer, so
 * async users (the WiFi scan task, the observer) never touch freed objects. */
static void close_panel(void)
{
    if (s_cur_panel) {
        lv_obj_del(s_cur_panel);   /* frees the panel + all child widgets */
        s_cur_panel = NULL;
    }
    s_cur_kind = PK_NONE;
    s_setup_panel = NULL; s_ssid_dd = NULL; s_pass_ta = NULL; s_show_btn = NULL;
    s_remember_cb = NULL; s_forget_btn = NULL; s_keyboard = NULL; s_setup_status = NULL;
    s_disp_bright_val = NULL; s_audio_vol_val = NULL;
    s_fx_scan_val = s_fx_phos_val = s_fx_vign_val = s_fx_refr_val = NULL;
    s_about_ip = NULL; s_about_uptime = NULL;
    s_vit_temp = NULL; s_vit_ram = NULL; s_vit_uptime = NULL; s_vit_cell = NULL;
    s_vit_temp_bar = NULL; s_vit_ram_bar = NULL; s_vit_cell_bar = NULL;
    s_vit_up_net = NULL; s_vit_up_ap = NULL; s_vit_up_link = NULL;
    s_vit_yaw = NULL; s_vit_pitch = NULL; s_vit_roll = NULL;
    s_vit_yaw_bar = NULL; s_vit_pitch_bar = NULL; s_vit_roll_bar = NULL;
    s_vit_accel = NULL; s_vit_imu_temp = NULL; s_vit_steps = NULL;
    s_sig_list = NULL; s_sig_status = NULL;
    s_spec_db = NULL; s_spec_db_bar = NULL; s_spec_status = NULL;
    s_spec_src_label = NULL;
    for (int i = 0; i < 5; i++) s_spec_axis[i] = NULL;
    s_rf_status = NULL;
    for (int i = 0; i < RF_CHANNELS; i++) s_rf_bars[i] = NULL;
    s_ble_summary = NULL; s_ble_list = NULL;
    memset(s_ble_rows, 0, sizeof(s_ble_rows));
    s_range_summary = NULL; s_range_list = NULL;
    memset(s_range_rows, 0, sizeof(s_range_rows));
    s_csi_status = NULL;
    s_csi_motion = NULL;
    s_csi_move = NULL;
    s_csi_rf = NULL;
    s_csi_rssi = NULL;
    for (int i = 0; i < 64; i++) s_csi_fp_cells[i] = NULL;
    s_csi_geiger_lbl = NULL;
    s_cfg_motion = NULL;
    s_cfg_move = NULL;
    s_cfg_meter = NULL;
    s_cfg_phase = NULL;
    s_cfg_btn_lbl = NULL;
    s_set_content = NULL;
    s_set_pagelbl = NULL;
    for (int i = 0; i < MAX_CSI_SET; i++) s_set_vallbl[i] = NULL;
    for (int i = 0; i < PROP_CSI_BINS; i++) s_csi_bars[i] = NULL;
    s_home_clock = NULL; s_home_temp = NULL; s_home_link = NULL;
    /* s_rail_btns are the persistent rail cells on the real screen - NOT children
     * of the torn-down panel, so they survive close_panel and are never nulled. */
    for (int i = 0; i < (int)(sizeof(s_arch_tabs) / sizeof(s_arch_tabs[0])); i++) s_arch_tabs[i] = NULL;
    for (int i = 0; i < (int)(sizeof(s_arch_rows) / sizeof(s_arch_rows[0])); i++) s_arch_rows[i] = NULL;
    s_arch_row_count = 0;
    s_article_scroll = NULL;
    for (int i = 0; i < IO_MAX; i++) {
        s_io_box[i] = NULL; s_io_val[i] = NULL; s_io_sub[i] = NULL;
        s_io_fill[i] = NULL; s_io_last[i] = -1;
    }
    s_pin_read = NULL; s_pin_fill = NULL; s_pin_edges = NULL;
    s_pin_aout = NULL; s_pin_slider = NULL; s_pin_entry = NULL;
    s_connect_pending = false;
    s_scan_spk = NULL;
    s_motion_tgt_label = NULL;
    s_dcal_prompt = NULL; s_dcal_sub = NULL; s_dcal_btn_lbl = NULL; s_dcal_arrow = NULL;
    for (int i = 0; i < 3; i++) {
        s_motion_blips[i]  = NULL;
        s_motion_tdist[i]  = NULL;
        s_motion_tvel[i]   = NULL;
        s_motion_tarrow[i] = NULL;
    }
    s_motion_pulse     = NULL;   /* canvas object already freed with the panel (line ~441) */
    if (s_pulse_buf) { free(s_pulse_buf); s_pulse_buf = NULL; }   /* its PSRAM backing is ours */
    s_pulse_prev_valid = false;
    for (int i = 0; i < 3; i++) { s_motion_gdial[i] = NULL; s_motion_gval[i] = NULL; }
    for (int i = 0; i < 3; i++) {
        s_motion_ble_box[i]  = NULL; s_motion_ble_name[i] = NULL;
        s_motion_ble_dist[i] = NULL; s_motion_ble_dbm[i]  = NULL;
    }
    for (int i = 0; i < MOTION_FLAG_COUNT; i++) s_motion_flags[i] = NULL;
    for (int i = 0; i < MOTION_BAR_COUNT; i++) { s_motion_bars[i] = NULL; s_motion_bar_lbls[i] = NULL; }
    for (int i = 0; i < 4; i++) s_motion_dir_q[i] = NULL;
    s_motion_dir_dot = NULL;
    for (int i = 0; i < 6; i++) { s_motion_grid_arc[i] = NULL; s_motion_grid_lbl[i] = NULL; }
    for (int i = 0; i < 2; i++) s_motion_grid_minor[i] = NULL;
    for (int i = 0; i < 3; i++) {
        s_trail_n[i] = 0; s_trail_head[i] = 0;
        for (int k = 0; k < TRAIL_LEN; k++) s_motion_trail[i][k] = NULL;
    }
    s_trail_tick = 0;
    s_motion_anim_ms = -1; s_motion_dwell = 0;
    /* MINIMAP: widgets freed with the panel; its canvas PSRAM backing is ours. */
    s_map_canvas = NULL; s_map_op_dot = NULL; s_map_op_arrow = NULL;
    s_map_info = NULL; s_map_msg = NULL;
    if (s_map_buf) { free(s_map_buf); s_map_buf = NULL; }
    s_map_have_fit = false; s_map_last_crumbs = -1; s_map_marks_sig = 0;
}

/* Light the rail cell for the function `kind` belongs to. Sub-panels map to their
 * top-level function (SETUP sub-screens -> SETUP, an article -> ARCHIVE). Runs for
 * every open including PK_NONE (the SCANNER readout), so it precedes the switch. */
static void rail_sync(panel_kind_t kind)
{
    panel_kind_t want = kind;
    switch (kind) {
        case PK_WIFI: case PK_DISPLAY: case PK_AUDIO:
        case PK_LEDS: case PK_ABOUT: case PK_MENU:
        case PK_IO: case PK_IO_PIN: want = PK_MENU; break;
        case PK_ARTICLE: want = PK_ARCHIVE; break;
        /* Instruments + sensors live under their group; the bare scanner readout
         * (PK_NONE) belongs to INSTRUMENTS too. */
        case PK_NONE: case PK_SCAN: case PK_SPECTRUM: case PK_VITALS:
            want = PK_INSTRUMENTS; break;
        case PK_RFBAND: case PK_BLE: case PK_CSI: case PK_CSICFG: case PK_CSISET: case PK_MOTION:
        case PK_DIRCAL: case PK_MINIMAP: case PK_RANGE:
            want = PK_SENSORS; break;
        default: break;
    }
    for (int i = 0; i < RAIL_COUNT; i++) {
        if (s_rail[i].kind == want) { s_rail_sel = i; break; }
    }
    if (s_rail_strip) {
        set_rail_highlight();
        /* Hero instruments (SCANNER readout, SPECTRUM) stay uncluttered on camera:
         * the rail recedes to a dim spine. Any other screen shows it at full strength. */
        bool hero = (kind == PK_NONE || kind == PK_SPECTRUM);
        lv_obj_set_style_opa(s_rail_strip, hero ? 110 : LV_OPA_COVER, 0);
    }
}

/* Switch to a panel (or PK_NONE for the main screen). */
/* Logical nesting depth of a panel, for the screen-change clack pitch (deeper = higher).
 * 0 = console root, 1 = top-level rail panels / submenus, 2 = leaves under a submenu,
 * 3 = a page reached from a leaf (the I/O single-pin config). */
static int panel_depth(panel_kind_t kind)
{
    switch (kind) {
        case PK_NONE: case PK_HOME:
            return 0;
        case PK_MENU: case PK_INSTRUMENTS: case PK_SENSORS:
        case PK_ARCHIVE: case PK_CASSETTE: case PK_INSIGHTS:
            return 1;
        case PK_IO_PIN:
            return 3;
        default:
            return 2;   /* WIFI/DISPLAY/AUDIO/LEDS/ABOUT/IO + the instrument/sensor leaves + ARTICLE */
    }
}

static void open_panel(panel_kind_t kind)
{
    close_panel();
    rail_sync(kind);
    switch (kind) {
        case PK_HOME:    s_cur_panel = build_home_panel(s_root); break;
        case PK_MENU:    s_cur_panel = build_menu_panel(s_root); break;
        case PK_WIFI:    s_cur_panel = build_wifi_panel(s_root); break;
        case PK_DISPLAY: s_cur_panel = build_display_panel(s_root); break;
        case PK_AUDIO:   s_cur_panel = build_audio_panel(s_root); break;
        case PK_LEDS:    s_cur_panel = build_leds_panel(s_root); break;
        case PK_ABOUT:   s_cur_panel = build_about_panel(s_root); break;
        case PK_VITALS:  s_cur_panel = build_vitals_panel(s_root); break;
        case PK_SCAN:    s_cur_panel = build_signal_panel(s_root); break;
        case PK_SPECTRUM: s_cur_panel = build_spectrum_panel(s_root); break;
        case PK_RFBAND:  s_cur_panel = build_rfband_panel(s_root); break;
        case PK_BLE:     s_cur_panel = build_ble_panel(s_root); break;
        case PK_CSI:     s_cur_panel = build_csi_panel(s_root); break;
        case PK_CSICFG:  s_cur_panel = build_csicfg_panel(s_root); break;
        case PK_CSISET:  s_cur_panel = build_csiset_panel(s_root); break;
        case PK_INSTRUMENTS: s_cur_panel = build_instruments_panel(s_root); break;
        case PK_SENSORS:     s_cur_panel = build_sensors_panel(s_root); break;
        case PK_ARCHIVE: s_cur_panel = build_archive_panel(s_root); break;
        case PK_ARTICLE: s_cur_panel = build_article_panel(s_root); break;
        case PK_CASSETTE: s_cur_panel = build_cassette_panel(s_root); break;
        case PK_INSIGHTS: s_cur_panel = build_insights_panel(s_root); break;
        case PK_IO:       s_cur_panel = build_io_panel(s_root); break;
        case PK_IO_PIN:   s_cur_panel = build_io_pin_panel(s_root); break;
        case PK_MOTION:   s_cur_panel = build_motion_panel(s_root); break;
        case PK_DIRCAL:   s_cur_panel = build_dircal_panel(s_root); break;
        case PK_MINIMAP:  s_cur_panel = build_minimap_panel(s_root); break;
        case PK_RANGE:    s_cur_panel = build_range_panel(s_root); break;
        default: break;    /* PK_NONE: no panel - the SCANNER readout shows through */
    }
    s_cur_kind = kind;
    if (kind == PK_WIFI) {
        wifi_panel_opened();
    } else if (kind == PK_SCAN) {
        start_signal_scan();   /* auto-scan on open */
    } else if (kind == PK_RFBAND) {
        start_rfband_scan();   /* auto-scan on open */
    }
    /* Mask the swap with the configured channel-change transition (skipped during
     * the initial boot build; the overlay was created above so the next render
     * shows it over the already-swapped screen). */
    if (s_ui_ready) {
        prop_fx_transition_play();
        /* Mechanical clack on every screen change, pitched up the deeper we navigate
         * (+4 semitones per nesting level) so descending into menus rises in tone. */
        prop_audio_play_pitched(PA_SCREEN, panel_depth(kind) * 4);
    }
}

/* Cassette-futurism styling helpers: amber-on-black, square corners. */
/* Thin wrappers over the kit so existing call sites stay terse; the kit owns the look. */
static void style_btn(lv_obj_t *b)   { kit_style_btn(b); }
static void style_field(lv_obj_t *f) { kit_style_field(f); }

static void style_keyboard(lv_obj_t *kb)
{
    lv_obj_set_style_bg_color(kb, COL_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(kb, COL_AMBER, LV_PART_MAIN);
    lv_obj_set_style_border_width(kb, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(kb, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(kb, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(kb, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(kb, COL_PANEL_ITEM, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(kb, COL_DIM, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(kb, COL_AMBER, LV_PART_ITEMS);
    lv_obj_set_style_border_color(kb, COL_DIM, LV_PART_ITEMS);
    lv_obj_set_style_border_width(kb, 1, LV_PART_ITEMS);
    lv_obj_set_style_radius(kb, 0, LV_PART_ITEMS);
    lv_obj_set_style_text_font(kb, FONT_HEAD, LV_PART_ITEMS);
}

/* FORGET only makes sense when the selected SSID is the saved one. */
static void update_forget_visibility(void)
{
    char sel[33], saved[33];
    lv_dropdown_get_selected_str(s_ssid_dd, sel, sizeof(sel));
    prop_settings_get_str("sta_ssid", saved, sizeof(saved), "");
    if (saved[0] && strcmp(sel, saved) == 0) {
        lv_obj_clear_flag(s_forget_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_forget_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

/* Background task: blocking WiFi scan off the LVGL thread, then populate the
 * dropdown under the LVGL lock. Mirrors the factory Setting app's scan task. */
static void scan_task(void *arg)
{
    (void)arg;
    int n = prop_net_scan(s_aps, SCAN_MAX);

    /* Build a newline-separated option list (LVGL dropdown format). */
    char opts[SCAN_MAX * 34];
    opts[0] = '\0';
    if (n > 0) {
        size_t off = 0;
        for (int i = 0; i < n; i++) {
            off += snprintf(opts + off, sizeof(opts) - off, "%s%s",
                            i ? "\n" : "", s_aps[i].ssid);
            if (off >= sizeof(opts)) {
                break;
            }
        }
    }

    if (lvgl_port_lock(200)) {
        /* The WiFi panel may have been torn down while we scanned; only touch its
         * widgets if it is still the live panel. */
        if (s_cur_kind == PK_WIFI && s_ssid_dd) {
            if (n > 0) {
                lv_dropdown_set_options(s_ssid_dd, opts);
                lv_label_set_text_fmt(s_setup_status, "found %d network%s", n, n == 1 ? "" : "s");
            } else if (n == 0) {
                lv_dropdown_set_options(s_ssid_dd, "(no networks found)");
                lv_label_set_text(s_setup_status, "no networks - tap SCAN");
            } else {
                lv_dropdown_set_options(s_ssid_dd, "(scan failed)");
                lv_label_set_text(s_setup_status, "scan failed (WiFi/C6?)");
            }
            update_forget_visibility();
        }
        lvgl_port_unlock();
    }
    s_scanning = false;
    vTaskDelete(NULL);
}

static void start_scan(void)
{
    if (s_scanning) {
        return;   /* one scan at a time */
    }
    s_scanning = true;
    lv_dropdown_set_options(s_ssid_dd, "(scanning...)");
    lv_label_set_text(s_setup_status, "SCANNING...");
    if (xTaskCreatePinnedToCore(scan_task, "wifi_scan", 4096, NULL, 4, NULL, 0) != pdPASS) {
        s_scanning = false;
        lv_label_set_text(s_setup_status, "scan busy");
    }
}

/* Finish opening the freshly-built WiFi panel: the builder already set default
 * field state, so just hint a saved password and kick off the auto-scan. */
static void wifi_panel_opened(void)
{
    char saved_pass[65];
    prop_settings_get_str("sta_pass", saved_pass, sizeof(saved_pass), "");
    if (saved_pass[0]) {
        lv_textarea_set_placeholder_text(s_pass_ta, "[ saved ]");
    }
    s_pass_shown = false;
    start_scan();   /* auto-scan on open so the list is ready */
}

/* Navigation: build-on-enter, tear-down-on-leave (see open_panel/close_panel). */
static void open_menu_cb(lv_event_t *e)    { (void)e; open_panel(PK_MENU); }
static void back_to_home_cb(lv_event_t *e) { (void)e; open_panel(PK_HOME); }
static void close_setup_cb(lv_event_t *e)  { (void)e; open_panel(PK_HOME); }
static void back_to_menu_cb(lv_event_t *e) { (void)e; open_panel(PK_MENU); }
static void back_to_instruments_cb(lv_event_t *e) { (void)e; open_panel(PK_INSTRUMENTS); }
static void back_to_sensors_cb(lv_event_t *e)     { (void)e; open_panel(PK_SENSORS); }
static void menu_open_cb(lv_event_t *e)    { open_panel((panel_kind_t)(intptr_t)lv_event_get_user_data(e)); }
static void setup_scan_cb(lv_event_t *e)   { (void)e; start_scan(); }
static void ssid_changed_cb(lv_event_t *e) { (void)e; update_forget_visibility(); }

/* Reveal / hide the typed password. */
static void setup_show_cb(lv_event_t *e)
{
    (void)e;
    s_pass_shown = !s_pass_shown;
    lv_textarea_set_password_mode(s_pass_ta, !s_pass_shown);
    lv_label_set_text(lv_obj_get_child(s_show_btn, 0),
                      s_pass_shown ? LV_SYMBOL_EYE_CLOSE : LV_SYMBOL_EYE_OPEN);
}

static void setup_forget_cb(lv_event_t *e)
{
    (void)e;
    prop_net_forget();
    s_connect_pending = false;
    lv_obj_add_flag(s_forget_btn, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_setup_status, "Network forgotten");
}

/* Focusing the password field reveals the keyboard aimed at it. */
static void ta_focus_cb(lv_event_t *e)
{
    lv_keyboard_set_textarea(s_keyboard, lv_event_get_target(e));
    lv_obj_clear_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void kb_done_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void setup_connect_cb(lv_event_t *e)
{
    (void)e;
    char ssid[33];
    lv_dropdown_get_selected_str(s_ssid_dd, ssid, sizeof(ssid));
    if (ssid[0] == '\0' || ssid[0] == '(') {   /* placeholder entry */
        lv_label_set_text(s_setup_status, "Pick a network (SCAN)");
        return;
    }
    const char *pass = lv_textarea_get_text(s_pass_ta);
    char pass_buf[65];
    /* No new password typed: reuse the saved one when reconnecting to the saved
     * network (this is what the "[ saved ]" placeholder promises). */
    if (pass[0] == '\0') {
        char saved_ssid[33];
        prop_settings_get_str("sta_ssid", saved_ssid, sizeof(saved_ssid), "");
        if (saved_ssid[0] && strcmp(ssid, saved_ssid) == 0) {
            prop_settings_get_str("sta_pass", pass_buf, sizeof(pass_buf), "");
            pass = pass_buf;
        }
    }
    bool remember = lv_obj_has_state(s_remember_cb, LV_STATE_CHECKED);

    esp_err_t err = prop_net_set_sta_credentials(ssid, pass, remember);
    if (err == ESP_OK) {
        s_connect_pending = true;   /* ui_observer fills in success/failure */
        lv_label_set_text(s_setup_status, "Connecting...");
        lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_label_set_text(s_setup_status, "Connect error");
    }
}

/* Make a small labelled, themed button. */
static lv_obj_t *make_btn(lv_obj_t *parent, const char *text, lv_coord_t w,
                          lv_align_t align, lv_coord_t x, lv_coord_t y,
                          lv_event_cb_t cb)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, w, 50);
    lv_obj_align(b, align, x, y);
    style_btn(b);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return b;
}

/* Create a registered full-screen setup panel with a themed border, a centered
 * title, and a top-left BACK button wired to back_cb. */
static lv_obj_t *make_panel(lv_obj_t *parent, const char *title, lv_event_cb_t back_cb)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_size(p, SCAN_W, 600);
    lv_obj_align(p, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(p, COL_BG, 0);
    lv_obj_set_style_border_color(p, COL_AMBER, 0);
    lv_obj_set_style_border_width(p, 2, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);

    make_btn(p, LV_SYMBOL_LEFT " BACK", 130, LV_ALIGN_TOP_LEFT, 20, 12, back_cb);

    lv_obj_t *t = lv_label_create(p);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_color(t, COL_AMBER, 0);
    lv_obj_set_style_text_font(t, FONT_HEAD, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 18);
    return p;
}

static lv_obj_t *build_wifi_panel(lv_obj_t *parent)
{
    s_setup_panel = make_panel(parent, "WI-FI", back_to_menu_cb);

    /* Content starts below the header (BACK button + title occupy ~y12..62). */
    /* Row 1: SCAN + SSID dropdown (left) ............ FORGET + CONNECT (right) */
    make_btn(s_setup_panel, "SCAN", 110, LV_ALIGN_TOP_LEFT, 30, 84, setup_scan_cb);

    s_ssid_dd = lv_dropdown_create(s_setup_panel);
    lv_dropdown_set_options(s_ssid_dd, "(tap SCAN)");
    lv_obj_set_width(s_ssid_dd, 340);
    lv_obj_align(s_ssid_dd, LV_ALIGN_TOP_LEFT, 150, 84);
    style_field(s_ssid_dd);
    lv_obj_add_event_cb(s_ssid_dd, ssid_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    make_btn(s_setup_panel, "CONNECT", 150, LV_ALIGN_TOP_RIGHT, -30, 84, setup_connect_cb);
    s_forget_btn = make_btn(s_setup_panel, "FORGET", 150, LV_ALIGN_TOP_RIGHT, -190, 84, setup_forget_cb);
    lv_obj_add_flag(s_forget_btn, LV_OBJ_FLAG_HIDDEN);

    /* Row 2: PASS + field + reveal (left). (BACK in the header replaces CANCEL.) */
    lv_obj_t *pass_lbl = lv_label_create(s_setup_panel);
    lv_label_set_text(pass_lbl, "PASS");
    lv_obj_set_style_text_color(pass_lbl, COL_AMBER, 0);
    lv_obj_align(pass_lbl, LV_ALIGN_TOP_LEFT, 30, 156);

    s_pass_ta = lv_textarea_create(s_setup_panel);
    lv_textarea_set_one_line(s_pass_ta, true);
    lv_textarea_set_password_mode(s_pass_ta, true);
    lv_textarea_set_placeholder_text(s_pass_ta, "password");
    lv_obj_set_width(s_pass_ta, 300);
    lv_obj_align(s_pass_ta, LV_ALIGN_TOP_LEFT, 100, 144);
    style_field(s_pass_ta);
    lv_obj_add_event_cb(s_pass_ta, ta_focus_cb, LV_EVENT_FOCUSED, NULL);

    s_show_btn = make_btn(s_setup_panel, LV_SYMBOL_EYE_OPEN, 70, LV_ALIGN_TOP_LEFT, 410, 144, setup_show_cb);

    /* Row 3: Remember checkbox + status */
    s_remember_cb = lv_checkbox_create(s_setup_panel);
    lv_checkbox_set_text(s_remember_cb, "Remember this network");
    lv_obj_set_style_text_color(s_remember_cb, COL_AMBER, 0);
    lv_obj_align(s_remember_cb, LV_ALIGN_TOP_LEFT, 30, 212);
    lv_obj_add_state(s_remember_cb, LV_STATE_CHECKED);
    /* Theme the tick box amber instead of the default blue. */
    lv_obj_set_style_border_color(s_remember_cb, COL_DIM, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_remember_cb, COL_PANEL_ITEM, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_remember_cb, COL_AMBER, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(s_remember_cb, COL_AMBER, LV_PART_INDICATOR | LV_STATE_CHECKED);

    s_setup_status = lv_label_create(s_setup_panel);
    lv_label_set_text(s_setup_status, "");
    lv_obj_set_style_text_color(s_setup_status, COL_AMBER, 0);
    lv_obj_align(s_setup_status, LV_ALIGN_TOP_LEFT, 360, 214);

    /* Big themed on-screen keyboard, hidden until the password field is focused. */
    s_keyboard = lv_keyboard_create(s_setup_panel);
    lv_obj_set_size(s_keyboard, SCAN_W, 320);
    lv_obj_align(s_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    style_keyboard(s_keyboard);
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_keyboard, kb_done_cb, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(s_keyboard, kb_done_cb, LV_EVENT_CANCEL, NULL);
    return s_setup_panel;
}

/* ---- Real SETUP panels (replace the stubs) ------------------------------- */

/* A left-aligned amber section label. */
static lv_obj_t *panel_label(lv_obj_t *p, const char *txt, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *l = lv_label_create(p);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, COL_AMBER, 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, x, y);
    return l;
}

/* DISPLAY: backlight brightness + CRT effects toggle/intensity. */
static void disp_bright_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    set_lcd_blight(v);
    prop_settings_set_u32("brightness", v);
    lv_label_set_text_fmt(s_disp_bright_val, "%d%%", v);
}
static void fx_toggle_cb(lv_event_t *e)
{
    prop_fx_set_enabled(lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED));
}
static void fx_scan_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_scanlines(v);
    lv_label_set_text_fmt(s_fx_scan_val, "%d%%", v);
}
static void fx_phos_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_phosphor(v);
    lv_label_set_text_fmt(s_fx_phos_val, "%d%%", v);
}
static void fx_vign_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_vignette(v);
    lv_label_set_text_fmt(s_fx_vign_val, "%d%%", v);
}
static void fx_refr_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_refresh(v);
    lv_label_set_text_fmt(s_fx_refr_val, "%d%%", v);
}
static void fx_trans_cb(lv_event_t *e)
{
    uint32_t v = lv_dropdown_get_selected(lv_event_get_target(e));
    prop_settings_set_u32("fx_trans", v);   /* 0 off, 1 snow, 2 roll, 3 collapse, 4 snow+collapse */
}
static void fps_toggle_cb(lv_event_t *e)
{
    prop_ui_set_fps(lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED));
}

static lv_obj_t *build_display_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "DISPLAY", back_to_menu_cb);
    lv_obj_t *b = kit_body(p);   /* flex column, scrolls if the rows overflow */

    uint32_t bl = 80;
    prop_settings_get_u32("brightness", &bl, 80);
    s_disp_bright_val = kit_slider_row(b, "BACKLIGHT", 5, 100, bl, disp_bright_cb);

    kit_switch_row(b, "CRT EFFECTS", prop_fx_enabled(), fx_toggle_cb, NULL);
    s_fx_scan_val = kit_slider_row(b, "SCANLINES",     0, 100, prop_fx_scanlines(), fx_scan_cb);
    s_fx_phos_val = kit_slider_row(b, "PHOSPHOR",      0, 100, prop_fx_phosphor(),  fx_phos_cb);
    s_fx_vign_val = kit_slider_row(b, "VIGNETTE",      0, 100, prop_fx_vignette(),  fx_vign_cb);
    s_fx_refr_val = kit_slider_row(b, "REFRESH SWEEP", 0, 100, prop_fx_refresh(),   fx_refr_cb);

    /* Screen-change transition flavor (the "old TV channel change"). */
    lv_obj_t *trow = kit_row(b);
    lv_obj_t *tl = lv_label_create(trow);
    lv_label_set_text(tl, "TRANSITION");
    lv_obj_set_style_text_color(tl, COL_AMBER, 0);
    lv_obj_t *dd = lv_dropdown_create(trow);
    lv_dropdown_set_options(dd, "OFF\nSNOW\nROLL\nCOLLAPSE\nSNOW+COLLAPSE");
    uint32_t tr = 1;
    prop_settings_get_u32("fx_trans", &tr, 1);
    lv_dropdown_set_selected(dd, tr > 4 ? 1 : tr);
    lv_obj_set_width(dd, 360);
    style_field(dd);
    lv_obj_add_event_cb(dd, fx_trans_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* FPS meter HUD toggle (top-right perf readout). */
    uint32_t fps_on = 0;
    prop_settings_get_u32("fps_on", &fps_on, 0);
    kit_switch_row(b, "FPS METER", fps_on != 0, fps_toggle_cb, NULL);
    return p;
}

/* LEDS: discrete lamps (on/off only) - per-lamp toggles + a lamp test. */
static void led_sw_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    prop_engine_set_led(idx, lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED));
}
static void lamp_test_cb(lv_event_t *e)
{
    (void)e;
    for (int i = 0; i < LED_COUNT; i++) {
        prop_engine_set_led(i, true);
    }
}

static lv_obj_t *build_leds_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "LEDS", back_to_menu_cb);
    lv_obj_t *b = kit_body(p);
    for (int i = 0; i < LED_COUNT && i < 3; i++) {
        char name[16];
        strlcpy(name, bsp_io_led_name((prop_led_t)i), sizeof(name));
        for (char *c = name; *c; c++) *c = (char)toupper((unsigned char)*c);
        kit_switch_row(b, name, bsp_io_led_get((prop_led_t)i), led_sw_cb, (void *)(intptr_t)i);
    }
    lv_obj_t *test = lv_btn_create(b);
    lv_obj_set_size(test, 200, 52);
    kit_style_btn(test);
    lv_obj_add_event_cb(test, lamp_test_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *tl = lv_label_create(test);
    lv_label_set_text(tl, "LAMP TEST");
    lv_obj_center(tl);

    lv_obj_t *note = lv_label_create(b);
    lv_label_set_text(note, "Lamps are on/off (no brightness). Scene animation resumes control.");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    return p;
}

/* AUDIO: master volume + mute for the synthesized feedback tones (persisted in NVS;
 * prop_audio reads these per event, so changes take effect on the next sound). */
static void audio_vol_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_settings_set_u32("audio_vol", v);
    lv_label_set_text_fmt(s_audio_vol_val, "%d%%", v);
    /* The slider's own PA_SLIDER tick plays at the new volume, auditioning the level. */
}
static void audio_mute_cb(lv_event_t *e)
{
    prop_settings_set_u32("audio_mute",
                          lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED) ? 1 : 0);
}
static void audio_test_cb(lv_event_t *e)
{
    (void)e;
    prop_audio_play(PA_SIGNAL);
}

static lv_obj_t *build_audio_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "AUDIO", back_to_menu_cb);
    lv_obj_t *b = kit_body(p);
    uint32_t vol = 60, mute = 0;
    prop_settings_get_u32("audio_vol", &vol, 60);
    prop_settings_get_u32("audio_mute", &mute, 0);

    s_audio_vol_val = kit_slider_row(b, "VOLUME", 0, 100, vol, audio_vol_cb);
    kit_switch_row(b, "MUTE", mute != 0, audio_mute_cb, NULL);

    lv_obj_t *test = lv_btn_create(b);
    lv_obj_set_size(test, 200, 52);
    kit_style_btn(test);
    lv_obj_add_event_cb(test, audio_test_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *tl = lv_label_create(test);
    lv_label_set_text(tl, "TEST TONE");
    lv_obj_center(tl);

    lv_obj_t *note = lv_label_create(b);
    lv_label_set_text(note, prop_audio_available()
                            ? "Synth feedback active over the speaker amp."
                            : "Audio output offline.");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    return p;
}

/* ABOUT: firmware identity + live IP / uptime (refreshed by the observer). */
static lv_obj_t *build_about_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "ABOUT", back_to_menu_cb);
    const esp_app_desc_t *app = esp_app_get_description();

    /* Identity + live telemetry as a flex card of key/value rows (LVGL 9 design kit:
     * rows self-stack with no manual y offsets; kit_info_row returns the value label
     * for the observer to update). */
    lv_obj_t *card = kit_card(p, SCAN_W - 100, LV_SIZE_CONTENT);
    lv_obj_align(card, LV_ALIGN_TOP_MID, 0, 92);

    kit_info_row(card, "UNIT", "COMM // SCANNER UNIT-7");
    kit_info_row(card, "FIRMWARE", app->version);
    char build[48];
    snprintf(build, sizeof(build), "%s  %s", app->date, app->time);
    kit_info_row(card, "BUILD", build);
    kit_info_row(card, "IDF", app->idf_ver);
    s_about_ip = kit_info_row(card, "IP", "...");
    s_about_uptime = kit_info_row(card, "UPTIME", "00:00:00");
    return p;
}

/* ---- VITALS instrument --------------------------------------------------- */

/* Read the P4's internal temperature sensor (installed once, lazily). Returns a
 * sentinel below -500 if the sensor is unavailable. */
static float read_core_temp(void)
{
    static temperature_sensor_handle_t ts;
    static bool tried, ok;
    if (!tried) {
        tried = true;
        temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
        if (temperature_sensor_install(&cfg, &ts) == ESP_OK &&
            temperature_sensor_enable(ts) == ESP_OK) {
            ok = true;
        }
    }
    float c = 0;
    if (ok && temperature_sensor_get_celsius(ts, &c) == ESP_OK) {
        return c;
    }
    return -1000.0f;
}

/* A non-interactive horizontal readout bar (square, amber fill). Returns the fill
 * object; drive it with set_meter(). */
/* Positioned wrapper over kit_meter (the kit builds the square track + amber fill). */
static lv_obj_t *make_meter_bar(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w)
{
    lv_obj_t *fill = kit_meter(parent, w);
    lv_obj_align(lv_obj_get_parent(fill), LV_ALIGN_TOP_LEFT, x, y);
    return fill;
}

static void set_meter(lv_obj_t *fill, int pct, lv_color_t col) { kit_set_meter(fill, pct, col); }

/* A fixed-width flex column for multi-column panel bodies. */
static lv_obj_t *vit_column(lv_obj_t *parent)
{
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_remove_style_all(c);
    lv_obj_set_width(c, LV_PCT(48));
    lv_obj_set_height(c, LV_SIZE_CONTENT);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(c, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(c, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(c, 14, 0);
    return c;
}

static lv_obj_t *build_vitals_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "VITALS", back_to_instruments_cb);

    /* Two-column body so everything fits on one 1024x600 screen (no scroll). */
    lv_obj_t *b = lv_obj_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, LV_PCT(94), LV_PCT(84));
    lv_obj_align(b, LV_ALIGN_TOP_MID, 0, 72);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(b, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(b, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *colL = vit_column(b);
    lv_obj_t *colR = vit_column(b);

    /* Left column: system vitals + uplink. */
    s_vit_temp   = kit_meter_row(colL, "CORE TEMP", &s_vit_temp_bar);
    s_vit_ram    = kit_meter_row(colL, "FREE RAM",  &s_vit_ram_bar);
    s_vit_cell   = kit_meter_row(colL, "CELL",      &s_vit_cell_bar);
    s_vit_uptime = kit_meter_row(colL, "UPTIME",    NULL);

    lv_obj_t *uphdr = lv_label_create(colL);
    lv_label_set_text(uphdr, "UPLINK");
    lv_obj_set_style_text_color(uphdr, COL_AMBER, 0);
    lv_obj_set_style_text_font(uphdr, FONT_HEAD, 0);
    lv_obj_t *card = kit_card(colL, lv_pct(100), LV_SIZE_CONTENT);
    s_vit_up_net  = kit_info_row(card, "NET",  "--");
    s_vit_up_ap   = kit_info_row(card, "AP",   "--");
    s_vit_up_link = kit_info_row(card, "LINK", "--");

    /* Right column: IMU MOTION (MPU-6500 DMP — absent if sensor not wired). */
    lv_obj_t *imuhdr = lv_label_create(colR);
    lv_label_set_text(imuhdr, "MOTION  [MPU-6500]");
    lv_obj_set_style_text_color(imuhdr, COL_AMBER, 0);
    lv_obj_set_style_text_font(imuhdr, FONT_HEAD, 0);
    s_vit_yaw   = kit_meter_row(colR, "YAW",   &s_vit_yaw_bar);
    s_vit_pitch = kit_meter_row(colR, "PITCH", &s_vit_pitch_bar);
    s_vit_roll  = kit_meter_row(colR, "ROLL",  &s_vit_roll_bar);
    lv_obj_t *accel_card = kit_card(colR, lv_pct(100), LV_SIZE_CONTENT);
    s_vit_accel    = kit_info_row(accel_card, "ACCEL", "--");
    s_vit_imu_temp = kit_info_row(accel_card, "CORE TEMP", "--");
    s_vit_steps    = kit_info_row(accel_card, "ON FOOT", "--");

    return p;
}

/* ---- SIGNAL SCAN instrument (WiFi APs rendered as detected contacts) ------ */

#define SIG_MAX_ROWS 7
#define SIG_ROW_H    62

/* Map RSSI (dBm) to a 0..100 strength for the contact meter. */
static int rssi_to_pct(int rssi)
{
    int pct = (rssi + 90) * 100 / 50;   /* -90 -> 0, -40 -> 100 */
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

/* Background WiFi scan; renders results as "contacts" under the LVGL lock. Mirrors
 * the WiFi-setup scan task but populates the instrument list. */
static void signal_scan_task(void *arg)
{
    (void)arg;
    int n = prop_net_scan(s_aps, SCAN_MAX);

    if (lvgl_port_lock(300)) {
        if (s_cur_kind == PK_SCAN && s_sig_list) {
            lv_obj_clean(s_sig_list);   /* drop any previous contacts */
            if (n < 0) {
                lv_label_set_text(s_sig_status, "SCAN FAILED  (C6 radio?)");
            } else if (n == 0) {
                lv_label_set_text(s_sig_status, "NO CONTACTS IN RANGE");
            } else {
                lv_label_set_text_fmt(s_sig_status, "%d CONTACT%s DETECTED",
                                      n, n == 1 ? "" : "S");
                int show = n > SIG_MAX_ROWS ? SIG_MAX_ROWS : n;
                for (int i = 0; i < show; i++) {
                    int y = i * SIG_ROW_H;
                    const char *ssid = s_aps[i].ssid[0] ? s_aps[i].ssid : "<unnamed source>";

                    lv_obj_t *tag = lv_label_create(s_sig_list);
                    lv_label_set_text_fmt(tag, "%02d  %s%s", i + 1, ssid,
                                          s_aps[i].secured ? "  " LV_SYMBOL_CLOSE : "");
                    lv_obj_set_style_text_color(tag, COL_AMBER, 0);
                    lv_obj_align(tag, LV_ALIGN_TOP_LEFT, 6, y);

                    lv_obj_t *dbm = lv_label_create(s_sig_list);
                    lv_label_set_text_fmt(dbm, "%d dBm", s_aps[i].rssi);
                    lv_obj_set_style_text_color(dbm, COL_MUTE, 0);
                    lv_obj_align(dbm, LV_ALIGN_TOP_RIGHT, -6, y);

                    /* Sub-line: vendor (OUI) // PHY generation // security // channel. */
                    const char *vendor = prop_net_oui_vendor(s_aps[i].bssid);
                    char sub[72];
                    snprintf(sub, sizeof(sub), "%s  //  %s  //  %s  //  CH %d%s",
                             vendor ? vendor : "UNKNOWN OEM",
                             prop_phy_label(s_aps[i].phy), s_aps[i].sec,
                             s_aps[i].channel, s_aps[i].ftm ? "  //  FTM" : "");
                    lv_obj_t *sl = lv_label_create(s_sig_list);
                    lv_label_set_text(sl, sub);
                    lv_obj_set_style_text_color(sl, COL_MUTE, 0);
                    lv_obj_set_style_text_font(sl, FONT_BODY, 0);
                    lv_obj_align(sl, LV_ALIGN_TOP_LEFT, 6, y + 22);

                    int pct = rssi_to_pct(s_aps[i].rssi);
                    lv_obj_t *fill = make_meter_bar(s_sig_list, 6, y + 44, SCAN_W - 180);
                    set_meter(fill, pct, pct < 25 ? COL_DIM : COL_AMBER);
                }
            }
        }
        lvgl_port_unlock();
    }
    s_sig_scanning = false;
    vTaskDelete(NULL);
}

static void start_signal_scan(void)
{
    if (s_sig_scanning || !s_sig_status) {
        return;
    }
    s_sig_scanning = true;
    lv_label_set_text(s_sig_status, "SCANNING SPECTRUM...");
    if (xTaskCreatePinnedToCore(signal_scan_task, "sig_scan", 4096, NULL, 4, NULL, 0) != pdPASS) {
        s_sig_scanning = false;
        lv_label_set_text(s_sig_status, "scan busy");
    }
}

static void signal_rescan_cb(lv_event_t *e) { (void)e; start_signal_scan(); }

static lv_obj_t *build_signal_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "SIGNAL SCAN", back_to_instruments_cb);

    s_sig_status = lv_label_create(p);
    lv_label_set_text(s_sig_status, "SCANNING SPECTRUM...");
    lv_obj_set_style_text_color(s_sig_status, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_sig_status, FONT_HEAD, 0);
    lv_obj_align(s_sig_status, LV_ALIGN_TOP_LEFT, 30, 84);

    make_btn(p, "RESCAN", 150, LV_ALIGN_TOP_RIGHT, -30, 78, signal_rescan_cb);

    /* Contact list container (rows are added by the scan task). */
    s_sig_list = lv_obj_create(p);
    lv_obj_remove_style_all(s_sig_list);
    lv_obj_set_size(s_sig_list, SCAN_W - 80, SIG_MAX_ROWS * SIG_ROW_H);
    lv_obj_align(s_sig_list, LV_ALIGN_TOP_LEFT, 40, 140);
    lv_obj_clear_flag(s_sig_list, LV_OBJ_FLAG_SCROLLABLE);
    return p;
}

/* ---- SPECTRUM instrument (live FFT bars + dB meter) ----------------------- */

#define SPEC_BW      30     /* bar width */
#define SPEC_GAP     8      /* gap between bars */
#define SPEC_X0      52     /* left margin */
#define SPEC_BASE    118    /* baseline offset from panel bottom */
#define SPEC_MAXH    312    /* full-scale bar height */

/* X-axis ticks: a target frequency (snapped to the nearest bar) plus an
 * optional note name shown on a second line. The bars are log-spaced, so we
 * pick round/musical targets and label whichever bar lands closest. */
typedef struct { int hz; const char *note; } spec_tick_t;

/* MIC (16 kHz / 8 kHz Nyquist): octaves of concert A — even one-octave log
 * spacing, with the note on a second line. */
static const spec_tick_t s_ticks_mic[5] = {
    {110, "A2"}, {220, "A3"}, {440, "A4"}, {880, "A5"}, {1760, "A6"},
};
/* IO/ADC sources (1 kHz / 500 Hz Nyquist): round numbers. */
static const spec_tick_t s_ticks_adc[5] = {
    {10, NULL}, {50, NULL}, {100, NULL}, {200, NULL}, {400, NULL},
};

static void spec_fmt_freq(char *buf, int n, int hz)
{
    if (hz >= 1000) snprintf(buf, n, "%dk", (hz + 500) / 1000);
    else            snprintf(buf, n, "%d", hz);
}

/* Bar (0..PROP_MIC_BANDS-1) whose center frequency is closest to target_hz,
 * measured in log space to match the log-spaced axis. Uses the current source's
 * sample rate via prop_mic_band_hz(), so call after prop_mic_set_source(). */
static int spec_nearest_band(int target_hz)
{
    int best = 0;
    float bestd = 1e9f;
    float lt = logf((float)target_hz);
    for (int b = 0; b < PROP_MIC_BANDS; b++) {
        int f = prop_mic_band_hz(b);
        if (f <= 0) continue;
        float d = fabsf(logf((float)f) - lt);
        if (d < bestd) { bestd = d; best = b; }
    }
    return best;
}

static void spec_update_axis(spec_src_t src)
{
    static const char *src_names[SPEC_SRC_COUNT] =
        {"MIC", "IO49", "IO50", "IO51", "IO52", "IO53", "IO54"};
    if (s_spec_src_label) {
        lv_label_set_text(s_spec_src_label,
            (src == SPEC_SRC_MIC && !prop_mic_pdm_up())
                ? "MIC (offline)" : src_names[src]);
    }
    if (!s_spec_axis[0]) return;

    const spec_tick_t *ticks = (src == SPEC_SRC_MIC) ? s_ticks_mic : s_ticks_adc;
    for (int i = 0; i < 5; i++) {
        int band = spec_nearest_band(ticks[i].hz);
        char buf[20];
        if (ticks[i].note) {
            /* Mic: exact note frequency on line 1, note name on line 2. */
            snprintf(buf, sizeof(buf), "%d\n%s", ticks[i].hz, ticks[i].note);
        } else {
            spec_fmt_freq(buf, sizeof(buf), ticks[i].hz);
        }
        lv_label_set_text(s_spec_axis[i], buf);
        int bx = SPEC_X0 + band * (SPEC_BW + SPEC_GAP) + SPEC_BW / 2 - 26;
        lv_obj_align(s_spec_axis[i], LV_ALIGN_BOTTOM_LEFT, bx, -SPEC_BASE + 54);
    }
}

static void spec_src_prev_cb(lv_event_t *e)
{
    (void)e;
    spec_src_t src = prop_mic_get_source();
    src = (src == 0) ? (spec_src_t)(SPEC_SRC_COUNT - 1) : (spec_src_t)(src - 1);
    prop_mic_set_source(src);
    spec_update_axis(src);
    for (int i = 0; i < PROP_MIC_BANDS; i++) s_spec_decay[i] = 0.0f;
}

static void spec_src_next_cb(lv_event_t *e)
{
    (void)e;
    spec_src_t src = (spec_src_t)((prop_mic_get_source() + 1) % SPEC_SRC_COUNT);
    prop_mic_set_source(src);
    spec_update_axis(src);
    for (int i = 0; i < PROP_MIC_BANDS; i++) s_spec_decay[i] = 0.0f;
}

static lv_obj_t *build_spectrum_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "SPECTRUM", back_to_instruments_cb);

    if (!prop_mic_available()) {
        s_spec_status = lv_label_create(p);
        lv_label_set_text(s_spec_status, "-- SPECTRUM OFFLINE --");
        lv_obj_set_style_text_color(s_spec_status, COL_DIM, 0);
        lv_obj_set_style_text_font(s_spec_status, FONT_HEAD, 0);
        lv_obj_center(s_spec_status);
        return p;
    }

    /* dB level meter (original position). */
    panel_label(p, "LEVEL", 40, 84);
    s_spec_db = lv_label_create(p);
    lv_label_set_text(s_spec_db, "-- dB");
    lv_obj_set_style_text_color(s_spec_db, COL_MUTE, 0);
    lv_obj_align(s_spec_db, LV_ALIGN_TOP_RIGHT, -40, 84);
    s_spec_db_bar = make_meter_bar(p, 130, 86, SCAN_W - 360);

    /* Baseline the bars stand on. */
    lv_obj_t *base = lv_obj_create(p);
    lv_obj_remove_style_all(base);
    lv_obj_set_size(base, SCAN_W - 2 * SPEC_X0 + SPEC_BW, 2);
    lv_obj_set_style_bg_opa(base, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(base, COL_DIM, 0);
    lv_obj_align(base, LV_ALIGN_BOTTOM_LEFT, SPEC_X0, -SPEC_BASE + 2);

    for (int i = 0; i < PROP_MIC_BANDS; i++) {
        lv_obj_t *b = lv_obj_create(p);
        lv_obj_remove_style_all(b);
        lv_obj_set_size(b, SPEC_BW, 2);
        lv_obj_align(b, LV_ALIGN_BOTTOM_LEFT, SPEC_X0 + i * (SPEC_BW + SPEC_GAP), -SPEC_BASE);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(b, COL_AMBER, 0);
        s_spec_bars[i] = b;
        s_spec_decay[i] = 0.0f;
        s_spec_band[i]  = -1;
    }

    /* X-axis frequency labels (just below baseline). Fixed two-line height so a
     * single-line (IO) and two-line (mic note) label share the same top edge;
     * spec_update_axis() snaps each to its bar and fills the text. */
    for (int i = 0; i < 5; i++) {
        lv_obj_t *al = lv_label_create(p);
        lv_obj_set_style_text_color(al, COL_DIM, 0);
        lv_obj_set_size(al, 52, 44);
        lv_obj_set_style_text_align(al, LV_TEXT_ALIGN_CENTER, 0);
        s_spec_axis[i] = al;
    }

    /* Source selector row at bottom: [<]  SOURCE NAME  [>] in large type. */
    make_btn(p, "<", 80, LV_ALIGN_BOTTOM_LEFT,  SPEC_X0,       -14, spec_src_prev_cb);
    make_btn(p, ">", 80, LV_ALIGN_BOTTOM_RIGHT, -SPEC_X0,      -14, spec_src_next_cb);
    s_spec_src_label = lv_label_create(p);
    lv_obj_set_style_text_color(s_spec_src_label, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_spec_src_label, FONT_HEAD, 0);
    lv_obj_align(s_spec_src_label, LV_ALIGN_BOTTOM_MID, 0, -22);

    spec_update_axis(prop_mic_get_source());
    return p;
}

/* ---- RF BAND instrument (2.4 GHz channel occupancy bar chart) -------------
 * A WiFi-derived "sensor": one bar per channel (1..13), height = how busy/strong
 * that channel is. Pure reuse of the scan path (prop_net_scan_channels) - no new
 * radio config. A background scan fills the cached histogram; the observer renders
 * the bars with the same peak-hold ballistics as the mic SPECTRUM. */

#define RF_BW    42        /* bar width */
#define RF_GAP   20        /* gap between bars */
#define RF_X0    82        /* left margin (13 bars centred in SCAN_W) */
#define RF_BASE  130       /* baseline offset from panel bottom (room for labels) */
#define RF_MAXH  300       /* full-scale bar height */

/* Background channel scan off the LVGL thread; result lands in s_rf_chan and the
 * observer animates it. Mirrors signal_scan_task. */
static void rfband_scan_task(void *arg)
{
    (void)arg;
    uint8_t hist[PROP_NET_CHAN_SLOTS];
    int n = prop_net_scan_channels(hist);

    if (lvgl_port_lock(300)) {
        if (s_cur_kind == PK_RFBAND && s_rf_status) {
            if (n < 0) {
                lv_label_set_text(s_rf_status, "SCAN FAILED  (C6 radio?)");
            } else if (n == 0) {
                memset(s_rf_chan, 0, sizeof(s_rf_chan));
                lv_label_set_text(s_rf_status, "BAND CLEAR  -  NO EMITTERS");
            } else {
                memcpy(s_rf_chan, hist, sizeof(s_rf_chan));
                /* Busiest channel, for the readout. */
                int top = 1, topv = 0;
                for (int ch = 1; ch < PROP_NET_CHAN_SLOTS; ch++) {
                    if (hist[ch] > topv) { topv = hist[ch]; top = ch; }
                }
                lv_label_set_text_fmt(s_rf_status, "%d EMITTER%s  -  PEAK CH %d",
                                      n, n == 1 ? "" : "S", top);
            }
        }
        lvgl_port_unlock();
    }
    s_rf_scanning = false;
    vTaskDelete(NULL);
}

static void start_rfband_scan(void)
{
    if (s_rf_scanning || !s_rf_status) {
        return;
    }
    s_rf_scanning = true;
    lv_label_set_text(s_rf_status, "SCANNING 2.4 GHz...");
    if (xTaskCreatePinnedToCore(rfband_scan_task, "rf_scan", 4096, NULL, 4, NULL, 0) != pdPASS) {
        s_rf_scanning = false;
        lv_label_set_text(s_rf_status, "scan busy");
    }
}

static void rfband_rescan_cb(lv_event_t *e) { (void)e; start_rfband_scan(); }

static lv_obj_t *build_rfband_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "RF BAND", back_to_sensors_cb);

    s_rf_status = lv_label_create(p);
    lv_label_set_text(s_rf_status, "SCANNING 2.4 GHz...");
    lv_obj_set_style_text_color(s_rf_status, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_rf_status, FONT_HEAD, 0);
    lv_obj_align(s_rf_status, LV_ALIGN_TOP_LEFT, 30, 84);

    make_btn(p, "RESCAN", 150, LV_ALIGN_TOP_RIGHT, -30, 78, rfband_rescan_cb);

    /* Baseline the bars stand on. */
    lv_obj_t *base = lv_obj_create(p);
    lv_obj_remove_style_all(base);
    lv_obj_set_size(base, RF_CHANNELS * RF_BW + (RF_CHANNELS - 1) * RF_GAP, 2);
    lv_obj_set_style_bg_opa(base, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(base, COL_DIM, 0);
    lv_obj_align(base, LV_ALIGN_BOTTOM_LEFT, RF_X0, -RF_BASE + 2);

    for (int i = 0; i < RF_CHANNELS; i++) {
        lv_coord_t x = RF_X0 + i * (RF_BW + RF_GAP);
        lv_obj_t *b = lv_obj_create(p);
        lv_obj_remove_style_all(b);
        lv_obj_set_size(b, RF_BW, 2);
        lv_obj_align(b, LV_ALIGN_BOTTOM_LEFT, x, -RF_BASE);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(b, COL_AMBER, 0);
        s_rf_bars[i] = b;
        s_rf_decay[i] = 0.0f;
        s_rf_band[i]  = -1;

        /* Channel number under the baseline. */
        lv_obj_t *cl = lv_label_create(p);
        lv_label_set_text_fmt(cl, "%d", i + 1);
        lv_obj_set_style_text_color(cl, COL_MUTE, 0);
        lv_obj_set_style_text_font(cl, FONT_BODY, 0);
        lv_obj_align(cl, LV_ALIGN_BOTTOM_LEFT, x + RF_BW / 2 - 8, -RF_BASE + 10);
    }

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "2.4 GHz channel occupancy  -  WiFi emitters per channel");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, RF_X0, -30);
    return p;
}

/* ---- CONTACT SIGNATURES instrument (passive BLE scan) ---------------------
 * Nearby BLE advertisers rendered as "contacts": a header summary plus a
 * scrolling list of rows (strength bar + name / Company-ID label). The device
 * set changes slowly, so the observer rebuilds the list throttled (~2.5 Hz). */

#define BLE_ROW_H 88

static lv_obj_t *build_ble_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "CONTACTS", back_to_sensors_cb);

    if (!prop_ble_available()) {
        lv_obj_t *off = lv_label_create(p);
        lv_label_set_text(off, "-- BLE OFFLINE --");
        lv_obj_set_style_text_color(off, COL_DIM, 0);
        lv_obj_set_style_text_font(off, FONT_HEAD, 0);
        lv_obj_center(off);
        return p;
    }

    s_ble_summary = lv_label_create(p);
    lv_label_set_text(s_ble_summary, "LISTENING FOR CONTACTS...");
    lv_obj_set_style_text_color(s_ble_summary, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_ble_summary, FONT_HEAD, 0);
    lv_obj_align(s_ble_summary, LV_ALIGN_TOP_LEFT, 30, 84);

    /* Scrolling contact list — rows are pre-built and reused (no teardown per tick). */
    s_ble_list = lv_obj_create(p);
    lv_obj_remove_style_all(s_ble_list);
    lv_obj_set_size(s_ble_list, SCAN_W - 60, 600 - 140);
    lv_obj_align(s_ble_list, LV_ALIGN_TOP_LEFT, 30, 132);
    lv_obj_set_style_pad_all(s_ble_list, 0, 0);
    lv_obj_set_scroll_dir(s_ble_list, LV_DIR_VER);

    for (int i = 0; i < PROP_BLE_MAX; i++) {
        int y = i * BLE_ROW_H;
        ble_row_t *r = &s_ble_rows[i];
        /* Line 1: identity (left) + RSSI (right). */
        r->tag  = lv_label_create(s_ble_list);
        lv_obj_set_style_text_color(r->tag, COL_AMBER, 0);
        lv_obj_align(r->tag, LV_ALIGN_TOP_LEFT, 6, y);
        r->dbm  = lv_label_create(s_ble_list);
        lv_obj_set_style_text_color(r->dbm, COL_MUTE, 0);
        lv_obj_align(r->dbm, LV_ALIGN_TOP_RIGHT, -6, y);
        /* Line 2: full MAC (left) + distance (right). */
        r->mac  = lv_label_create(s_ble_list);
        lv_obj_set_style_text_color(r->mac, COL_MUTE, 0);
        lv_obj_set_style_text_font(r->mac, FONT_BODY, 0);
        lv_obj_align(r->mac, LV_ALIGN_TOP_LEFT, 6, y + 22);
        r->dist = lv_label_create(s_ble_list);
        lv_obj_set_style_text_color(r->dist, COL_AMBER, 0);
        lv_obj_set_style_text_font(r->dist, FONT_BODY, 0);
        lv_obj_align(r->dist, LV_ALIGN_TOP_RIGHT, -6, y + 22);
        /* Line 3: service UUID (left) + advertised/corrected TX power (right). */
        r->uuid = lv_label_create(s_ble_list);
        lv_obj_set_style_text_color(r->uuid, COL_MUTE, 0);
        lv_obj_set_style_text_font(r->uuid, FONT_BODY, 0);
        lv_obj_align(r->uuid, LV_ALIGN_TOP_LEFT, 6, y + 44);
        r->tx   = lv_label_create(s_ble_list);
        lv_obj_set_style_text_color(r->tx, COL_MUTE, 0);
        lv_obj_set_style_text_font(r->tx, FONT_BODY, 0);
        lv_obj_align(r->tx, LV_ALIGN_TOP_RIGHT, -6, y + 44);
        r->fill = make_meter_bar(s_ble_list, 6, y + 68, SCAN_W - 60 - 150);
        /* Start all rows hidden; observer reveals them as contacts arrive. */
        lv_obj_add_flag(r->tag,  LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->dbm,  LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->mac,  LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->dist, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->uuid, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->tx,   LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->fill, LV_OBJ_FLAG_HIDDEN);
    }
    return p;
}

/* RSSI -> bar fill. RSSI is in dBm (already a log of power); real contacts cluster
 * in the weak -70..-95 band and read flat on a linear map. A sqrt expansion over
 * the useful window spreads that cluster so rows are visually distinct. */
static int ble_rssi_to_bar(int rssi)
{
    float lin = (rssi + 100.0f) / 65.0f;   /* -100 dBm -> 0, -35 dBm -> 1 */
    if (lin < 0.0f) lin = 0.0f;
    if (lin > 1.0f) lin = 1.0f;
    return (int)(sqrtf(lin) * 100.0f);
}

/* Format a distance estimate compactly ("0.4 m", "12 m"). */
static void ble_fmt_dist(char *out, size_t n, float d)
{
    if (d < 10.0f) snprintf(out, n, "%.1f m", (double)d);
    else           snprintf(out, n, "%d m", (int)(d + 0.5f));
}

/* Best human label for a contact: advertised name, else device class, else
 * manufacturer brand, else the MAC tail. Used by the MOTION SCAN BLE row. */
static void ble_contact_name(char *out, size_t n, const prop_ble_dev_t *d)
{
    const char *brand = prop_ble_company_label(d->company_id);
    const char *klass = prop_ble_appearance_label(d->appearance);
    if      (d->name[0]) snprintf(out, n, "%s", d->name);
    else if (klass)      snprintf(out, n, "%s", klass);
    else if (brand)      snprintf(out, n, "%s DEV", brand);
    else                 snprintf(out, n, "%02X:%02X:%02X", d->mac[3], d->mac[4], d->mac[5]);
}

/* Update the content of pre-built row slot `i` in-place. */
static void ble_update_row(int i, const prop_ble_dev_t *d)
{
    ble_row_t *r = &s_ble_rows[i];
    const char *brand = prop_ble_company_label(d->company_id);
    const char *klass = prop_ble_appearance_label(d->appearance);

    char idbuf[48];
    if (d->name[0]) {
        snprintf(idbuf, sizeof(idbuf), "%02d  %s", i + 1, d->name);
    } else if (klass) {
        snprintf(idbuf, sizeof(idbuf), "%02d  %s", i + 1, klass);
    } else if (brand) {
        snprintf(idbuf, sizeof(idbuf), "%02d  %s DEVICE", i + 1, brand);
    } else {
        snprintf(idbuf, sizeof(idbuf), "%02d  %02X:%02X:%02X",
                 i + 1, d->mac[3], d->mac[4], d->mac[5]);
    }
    lv_label_set_text(r->tag, idbuf);
    lv_label_set_text_fmt(r->dbm, "%d dBm", d->rssi);

    /* Full MAC. */
    char mb[24];
    snprintf(mb, sizeof(mb), "%02X:%02X:%02X:%02X:%02X:%02X",
             d->mac[0], d->mac[1], d->mac[2], d->mac[3], d->mac[4], d->mac[5]);
    lv_label_set_text(r->mac, mb);

    char dbuf[16];
    ble_fmt_dist(dbuf, sizeof(dbuf), prop_ble_distance_m(d->rssi, d->tx_power));
    lv_label_set_text_fmt(r->dist, "~ %s", dbuf);

    /* Advertised service UUID (128/32/16-bit). Stored little-endian; the 128-bit
     * form is printed MSB-first in the canonical 8-4-4-4-12 layout. */
    char ub[52];
    const uint8_t *u = d->uuid;
    if (d->uuid_len == 16) {
        /* Canonical long form, lowercase: 8-4-4-4-12 (bytes stored little-endian). */
        snprintf(ub, sizeof(ub),
            "UUID %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
            u[15], u[14], u[13], u[12], u[11], u[10], u[9], u[8],
            u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);
    } else if (d->uuid_len == 4) {
        uint32_t v = (uint32_t)u[0] | ((uint32_t)u[1] << 8) |
                     ((uint32_t)u[2] << 16) | ((uint32_t)u[3] << 24);
        snprintf(ub, sizeof(ub), "UUID 0x%08lx", (unsigned long)v);
    } else if (d->uuid_len == 2) {
        snprintf(ub, sizeof(ub), "UUID 0x%04x", (uint16_t)u[0] | ((uint16_t)u[1] << 8));
    } else {
        snprintf(ub, sizeof(ub), "UUID  --");
    }
    lv_label_set_text(r->uuid, ub);

    /* TX power: advertised value, plus the corrected value we range with when the
     * advertised one is absent or implausible. */
    char tb[40];
    int8_t corr = prop_ble_calib_txpower(d->tx_power);
    if (d->tx_power == PROP_BLE_TXPWR_NONE) {
        snprintf(tb, sizeof(tb), "TX --  (use %d dBm)", corr);
    } else if (corr != d->tx_power) {
        snprintf(tb, sizeof(tb), "TX %d -> %d dBm", d->tx_power, corr);
    } else {
        snprintf(tb, sizeof(tb), "TX %d dBm", d->tx_power);
    }
    lv_label_set_text(r->tx, tb);

    int pct = ble_rssi_to_bar(d->rssi);
    set_meter(r->fill, pct, pct < 25 ? COL_DIM : COL_AMBER);
}

/* ---- RANGE instrument (WiFi 802.11mc FTM ranging) --------------------------
 * Distance-to-AP for every FTM-capable BSSID the C6 has scanned, driven by
 * prop_ftm's background probe+backoff engine. Most environments have zero or
 * few FTM-capable APs (see prop_ftm.h) — the table filters to capable-only
 * (prop_ftm_get_table sorts capable-first), so a quiet environment shows the
 * empty-state message rather than a long list of APs that will never range. */

#define RANGE_ROW_H 88

static lv_obj_t *build_range_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "RANGE", back_to_sensors_cb);

    if (!prop_ftm_available()) {
        lv_obj_t *off = lv_label_create(p);
        lv_label_set_text(off, "-- FTM RANGING OFFLINE --");
        lv_obj_set_style_text_color(off, COL_DIM, 0);
        lv_obj_set_style_text_font(off, FONT_HEAD, 0);
        lv_obj_center(off);
        return p;
    }

    s_range_summary = lv_label_create(p);
    lv_label_set_text(s_range_summary, "SCANNING FOR FTM-CAPABLE APs...");
    lv_obj_set_style_text_color(s_range_summary, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_range_summary, FONT_HEAD, 0);
    lv_obj_align(s_range_summary, LV_ALIGN_TOP_LEFT, 30, 84);

    /* Scrolling range list — rows are pre-built and reused (no teardown per tick). */
    s_range_list = lv_obj_create(p);
    lv_obj_remove_style_all(s_range_list);
    lv_obj_set_size(s_range_list, SCAN_W - 60, 600 - 140);
    lv_obj_align(s_range_list, LV_ALIGN_TOP_LEFT, 30, 132);
    lv_obj_set_style_pad_all(s_range_list, 0, 0);
    lv_obj_set_scroll_dir(s_range_list, LV_DIR_VER);

    for (int i = 0; i < PROP_FTM_TABLE_MAX; i++) {
        int y = i * RANGE_ROW_H;
        range_row_t *r = &s_range_rows[i];
        /* Line 1: SSID/hidden (left) + RSSI (right). */
        r->tag    = lv_label_create(s_range_list);
        lv_obj_set_style_text_color(r->tag, COL_AMBER, 0);
        lv_obj_align(r->tag, LV_ALIGN_TOP_LEFT, 6, y);
        r->rssi   = lv_label_create(s_range_list);
        lv_obj_set_style_text_color(r->rssi, COL_MUTE, 0);
        lv_obj_align(r->rssi, LV_ALIGN_TOP_RIGHT, -6, y);
        /* Line 2: BSSID + vendor (left) + distance (right). */
        r->mac    = lv_label_create(s_range_list);
        lv_obj_set_style_text_color(r->mac, COL_MUTE, 0);
        lv_obj_set_style_text_font(r->mac, FONT_BODY, 0);
        lv_obj_align(r->mac, LV_ALIGN_TOP_LEFT, 6, y + 22);
        r->dist   = lv_label_create(s_range_list);
        lv_obj_set_style_text_color(r->dist, COL_AMBER, 0);
        lv_obj_set_style_text_font(r->dist, FONT_BODY, 0);
        lv_obj_align(r->dist, LV_ALIGN_TOP_RIGHT, -6, y + 22);
        /* Line 3: status tag. */
        r->status = lv_label_create(s_range_list);
        lv_obj_set_style_text_color(r->status, COL_MUTE, 0);
        lv_obj_set_style_text_font(r->status, FONT_BODY, 0);
        lv_obj_align(r->status, LV_ALIGN_TOP_LEFT, 6, y + 44);
        r->fill   = make_meter_bar(s_range_list, 6, y + 68, SCAN_W - 60 - 150);
        /* Start all rows hidden; observer reveals them as the table fills in. */
        lv_obj_add_flag(r->tag,    LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->rssi,   LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->mac,    LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->dist,   LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r->fill,   LV_OBJ_FLAG_HIDDEN);
    }
    return p;
}

/* Status word (+ remaining backoff, where relevant) for entry `e`. */
static void range_fmt_status(char *out, size_t n, const prop_ftm_entry_t *e, uint32_t now)
{
    switch (e->status) {
        case PROP_FTM_ENTRY_OK:        snprintf(out, n, "LIVE"); break;
        case PROP_FTM_ENTRY_BUSY:      snprintf(out, n, "RADIO BUSY - RETRYING"); break;
        case PROP_FTM_ENTRY_NEVER:     snprintf(out, n, "PENDING FIRST PROBE"); break;
        case PROP_FTM_ENTRY_NO_REPLY:
        case PROP_FTM_ENTRY_TIMEOUT:
        default: {
            const char *why = (e->status == PROP_FTM_ENTRY_TIMEOUT) ? "TIMEOUT" : "NO REPLY";
            if (e->next_probe_ms > now) {
                uint32_t s = (e->next_probe_ms - now) / 1000;
                snprintf(out, n, "%s - RETRY IN %lus", why, (unsigned long)s);
            } else {
                snprintf(out, n, "%s - RETRYING", why);
            }
            break;
        }
    }
}

/* Update the content of pre-built row slot `i` in-place. */
static void range_update_row(int i, const prop_ftm_entry_t *e, uint32_t now)
{
    range_row_t *r = &s_range_rows[i];

    char idbuf[48];
    snprintf(idbuf, sizeof(idbuf), "%02d  %s", i + 1, e->ssid[0] ? e->ssid : "(hidden)");
    lv_label_set_text(r->tag, idbuf);
    lv_label_set_text_fmt(r->rssi, "%d dBm", e->rssi);

    const char *vendor = prop_net_oui_vendor(e->bssid);
    char mb[48];
    snprintf(mb, sizeof(mb), "%02X:%02X:%02X:%02X:%02X:%02X  %s",
             e->bssid[0], e->bssid[1], e->bssid[2], e->bssid[3], e->bssid[4], e->bssid[5],
             vendor ? vendor : "");
    lv_label_set_text(r->mac, mb);

    if (e->last_dist_cm >= 0) {
        float m = e->last_dist_cm / 100.0f;
        lv_label_set_text_fmt(r->dist, m < 10.0f ? "~ %.1f m" : "~ %.0f m", (double)m);
    } else {
        lv_label_set_text(r->dist, "--");
    }

    char sb[40];
    range_fmt_status(sb, sizeof(sb), e, now);
    lv_label_set_text(r->status, sb);

    int pct = ble_rssi_to_bar(e->rssi);
    lv_color_t col = (e->status == PROP_FTM_ENTRY_OK) ? COL_AMBER
                    : (pct < 25 ? COL_DIM : COL_MUTE);
    set_meter(r->fill, pct, col);
}

/* ---- SIGNAL ENVIRONMENT instrument (WiFi CSI, or synthetic fallback) -------
 * Per-subcarrier channel amplitude as a dense bar field - the RF "texture" of the
 * room. Driven by prop_csi, which serves real CSI when frames are arriving and a
 * synthetic RSSI-variance trace otherwise; the header reports which is live. */

#define CSI_BW   20
#define CSI_GAP  8
#define CSI_X0   30
#define CSI_BASE 118
#define CSI_MAXH 230   /* bar tops reach y~252 — clear of the readout text above */

static void csi_rebaseline_cb(lv_event_t *e) { (void)e; prop_calib_reset(); }
static void csi_cog_cb(lv_event_t *e)        { (void)e; open_panel(PK_CSISET); }
static void csi_geiger_cb(lv_event_t *e)     { (void)e; prop_csi_set_geiger(!prop_csi_geiger()); }

static lv_obj_t *build_csi_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "SIGNAL ENV", back_to_sensors_cb);

    /* Settings cog (top-right corner) → CSI SETTINGS. */
    lv_obj_t *cog = make_btn(p, LV_SYMBOL_SETTINGS, 60, LV_ALIGN_TOP_RIGHT, -20, 12, csi_cog_cb);
    (void)cog;

    s_csi_status = lv_label_create(p);
    lv_label_set_text(s_csi_status, "ACQUIRING CHANNEL...");
    lv_obj_set_style_text_color(s_csi_status, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_csi_status, FONT_HEAD, 0);
    lv_obj_align(s_csi_status, LV_ALIGN_TOP_LEFT, 30, 84);

    /* Big MOTION / IDLE state — the headline readout (camera-legible: bold head
     * font, amber for idle, alert red for motion). */
    s_csi_motion = lv_label_create(p);
    lv_label_set_text(s_csi_motion, "IDLE");
    lv_obj_set_style_text_color(s_csi_motion, COL_MUTE, 0);
    lv_obj_set_style_text_font(s_csi_motion, FONT_HEAD, 0);
    lv_obj_align(s_csi_motion, LV_ALIGN_TOP_LEFT, 30, 120);

    /* Movement metric vs detection threshold. */
    s_csi_move = lv_label_create(p);
    lv_label_set_text(s_csi_move, "MOVEMENT --- / THR ---");
    lv_obj_set_style_text_color(s_csi_move, COL_MUTE, 0);
    lv_obj_align(s_csi_move, LV_ALIGN_TOP_LEFT, 30, 152);

    /* Extra RF datapoints: raw turbulence + receiver gain state. */
    s_csi_rf = lv_label_create(p);
    lv_label_set_text(s_csi_rf, "TURB ---  GAIN ---");
    lv_obj_set_style_text_color(s_csi_rf, COL_DIM, 0);
    lv_obj_align(s_csi_rf, LV_ALIGN_TOP_LEFT, 30, 178);

    /* ---- upper-right quadrant: actions + RF fingerprint ---- */
    make_btn(p, "RE-BASELINE", 240, LV_ALIGN_TOP_RIGHT, -30, 78, csi_rebaseline_cb);
    lv_obj_t *gb = make_btn(p, "SPECTRE GEIGER", 240, LV_ALIGN_TOP_RIGHT, -30, 134, csi_geiger_cb);
    s_csi_geiger_lbl = lv_obj_get_child(gb, 0);

    s_csi_rssi = lv_label_create(p);
    lv_label_set_text(s_csi_rssi, "LINK ---");
    lv_obj_set_style_text_color(s_csi_rssi, COL_MUTE, 0);
    lv_obj_align(s_csi_rssi, LV_ALIGN_TOP_RIGHT, -30, 196);

    /* NBVI subcarrier fingerprint — a 64-cell strip; the selected band lights up. */
    lv_obj_t *fpt = lv_label_create(p);
    lv_label_set_text(fpt, "NBVI BAND");
    lv_obj_set_style_text_color(fpt, COL_DIM, 0);
    lv_obj_align(fpt, LV_ALIGN_TOP_LEFT, 30, 206);
    for (int i = 0; i < 64; i++) {
        lv_obj_t *c = lv_obj_create(p);
        lv_obj_remove_style_all(c);
        lv_obj_set_size(c, 4, 16);
        lv_obj_align(c, LV_ALIGN_TOP_LEFT, 30 + i * 6, 228);
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(c, COL_DIM, 0);
        s_csi_fp_cells[i] = c;
    }

    lv_obj_t *base = lv_obj_create(p);
    lv_obj_remove_style_all(base);
    lv_obj_set_size(base, PROP_CSI_BINS * CSI_BW + (PROP_CSI_BINS - 1) * CSI_GAP, 2);
    lv_obj_set_style_bg_opa(base, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(base, COL_DIM, 0);
    lv_obj_align(base, LV_ALIGN_BOTTOM_LEFT, CSI_X0, -CSI_BASE + 2);

    for (int i = 0; i < PROP_CSI_BINS; i++) {
        lv_obj_t *b = lv_obj_create(p);
        lv_obj_remove_style_all(b);
        lv_obj_set_size(b, CSI_BW, 2);
        lv_obj_align(b, LV_ALIGN_BOTTOM_LEFT, CSI_X0 + i * (CSI_BW + CSI_GAP), -CSI_BASE);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(b, COL_AMBER, 0);
        s_csi_bars[i] = b;
        s_csi_band[i]  = -1;
    }

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "WiFi CSI motion variance  -  history (newest right)");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, CSI_X0, -30);
    return p;
}

/* ---- CSI CONFIG + continuous auto-calibration (PK_CSICFG) ----------------- */
static void csicfg_reset_cb(lv_event_t *e) { (void)e; prop_calib_reset(); }
static void csicfg_auto_cb(lv_event_t *e)  { (void)e; prop_calib_set_auto(!prop_calib_auto()); }
static void csicfg_settings_cb(lv_event_t *e) { (void)e; open_panel(PK_CSISET); }

static lv_obj_t *build_csicfg_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "CSI CONFIG", back_to_sensors_cb);

    /* Live feedback: motion state + movement vs threshold + meter. */
    s_cfg_motion = lv_label_create(p);
    lv_label_set_text(s_cfg_motion, "IDLE");
    lv_obj_set_style_text_color(s_cfg_motion, COL_MUTE, 0);
    lv_obj_set_style_text_font(s_cfg_motion, FONT_HEAD, 0);
    lv_obj_align(s_cfg_motion, LV_ALIGN_TOP_LEFT, 30, 78);

    s_cfg_move = lv_label_create(p);
    lv_label_set_text(s_cfg_move, "MOVEMENT --- / THR ---");
    lv_obj_set_style_text_color(s_cfg_move, COL_MUTE, 0);
    lv_obj_align(s_cfg_move, LV_ALIGN_TOP_LEFT, 30, 114);

    s_cfg_meter = kit_meter(p, SCAN_W - 320);
    lv_obj_align(lv_obj_get_parent(s_cfg_meter), LV_ALIGN_TOP_LEFT, 30, 144);

    /* Adaptive status line (learning / adapting / baseline). */
    s_cfg_phase = lv_label_create(p);
    lv_label_set_text(s_cfg_phase, "AUTO-ADAPTING");
    lv_obj_set_style_text_color(s_cfg_phase, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_cfg_phase, FONT_HEAD, 0);
    lv_obj_align(s_cfg_phase, LV_ALIGN_TOP_LEFT, 30, 206);

    lv_obj_t *hint = lv_label_create(p);
    lv_label_set_text(hint,
        "AUTO threshold learns this room's quiet baseline continuously and\n"
        "keeps the motion threshold just above it - no leaving the room.\n"
        "RESET re-learns fast after you move the unit.");
    lv_obj_set_style_text_color(hint, COL_MUTE, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 30, 248);

    lv_obj_t *abtn = make_btn(p, "AUTO: ON", 200, LV_ALIGN_BOTTOM_LEFT, 30, -40, csicfg_auto_cb);
    s_cfg_btn_lbl = lv_obj_get_child(abtn, 0);
    make_btn(p, "RESET BASELINE", 220, LV_ALIGN_BOTTOM_LEFT, 244, -40, csicfg_reset_cb);
    make_btn(p, "SETTINGS " LV_SYMBOL_RIGHT, 200, LV_ALIGN_BOTTOM_LEFT, 478, -40, csicfg_settings_cb);
    return p;
}

/* ---- CSI SETTINGS editor (PK_CSISET): paginated, button-row controls -------
 * Every ESPectre/CSI setting with a short description, controlled by button rows
 * (enum/bitmask/bool select), threshold-mode (AUTO/MIN/MANUAL + stepper), or a
 * [-]/[+] stepper for ranges. Paginated with PREV/NEXT buttons (no swipe). */
static void back_to_csicfg_cb(lv_event_t *e) { (void)e; open_panel(PK_CSICFG); }
static void render_csiset_page(void);

static void set_fmt_value(char *buf, size_t n, char type, int32_t val)
{
    if (type == 'T') {
        if (val == -1)      { snprintf(buf, n, "AUTO"); }
        else if (val == -2) { snprintf(buf, n, "MIN"); }
        else                { snprintf(buf, n, "%ld.%02ld", (long)val / 1000, ((long)val % 1000) / 10); }
    } else if (type == 'F') { snprintf(buf, n, "%ld.%02ld", (long)val / 1000, ((long)val % 1000) / 10); }
    else if (type == 'B')   { snprintf(buf, n, "%s", val ? "ON" : "OFF"); }
    else                    { snprintf(buf, n, "%ld", (long)val); }
}

/* Clamp + apply a setting, then re-render the page (refreshes highlights/value). */
static void csiset_set(int idx, int32_t val)
{
    const char *key; int32_t lo, hi;
    if (!prop_coproc_csi_describe(idx, &key, NULL, NULL, &lo, &hi, NULL, NULL)) { return; }
    if (val < lo) { val = lo; }
    if (val > hi) { val = hi; }
    prop_coproc_csi_set(key, val);
    render_csiset_page();
}

static int32_t csiset_step(int32_t lo, int32_t hi)
{
    int32_t s = (hi - lo) / 20;
    return s < 1 ? 1 : s;
}

/* ud = (setting_index << 8) | sub */
static void csiset_opt_cb(lv_event_t *e)
{
    int ud = (int)(intptr_t)lv_event_get_user_data(e), idx = ud >> 8, opt = ud & 0xff;
    char type; int32_t val;
    prop_coproc_csi_describe(idx, NULL, &val, &type, NULL, NULL, NULL, NULL);
    if (type == 'M')      { val ^= (1 << opt); }   /* bitmask: toggle this bit */
    else                  { val = opt; }           /* enum / bool: select */
    csiset_set(idx, val);
}
static void csiset_thr_cb(lv_event_t *e)
{
    int ud = (int)(intptr_t)lv_event_get_user_data(e), idx = ud >> 8, mode = ud & 0xff;
    int32_t val;
    prop_coproc_csi_describe(idx, NULL, &val, NULL, NULL, NULL, NULL, NULL);
    /* AUTO here == the continuous adaptive controller (same as CSI CONFIG's AUTO).
     * MIN/MANUAL turn it off so the chosen fixed threshold sticks. */
    if (mode == 0) {                            /* AUTO */
        prop_calib_set_auto(true);
        render_csiset_page();
    } else if (mode == 1) {                      /* MIN */
        prop_calib_set_auto(false);
        csiset_set(idx, -2);
    } else {                                     /* MANUAL (keep/seed 2.0) */
        prop_calib_set_auto(false);
        csiset_set(idx, val >= 0 ? val : 2000);
    }
}
static void csiset_step_cb(lv_event_t *e)
{
    int ud = (int)(intptr_t)lv_event_get_user_data(e), idx = ud >> 8, dir = ud & 0xff;
    char type; int32_t val, lo, hi;
    prop_coproc_csi_describe(idx, NULL, &val, &type, &lo, &hi, NULL, NULL);
    int32_t slo = (type == 'T') ? 0 : lo;          /* threshold manual steps 0..hi */
    int32_t step = csiset_step(slo, hi);
    val = (dir ? val + step : val - step);
    if (val < slo) { val = slo; }
    if (val > hi)  { val = hi; }
    csiset_set(idx, val);
}
static void csiset_page_cb(lv_event_t *e)
{
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    int n = prop_coproc_csi_count(), pages = (n + CSISET_PER_PAGE - 1) / CSISET_PER_PAGE;
    s_set_page += dir;
    if (s_set_page < 0) { s_set_page = 0; }
    if (s_set_page >= pages) { s_set_page = pages - 1; }
    render_csiset_page();
}

/* A themed pill button; filled when selected. ud = (idx<<8)|sub. */
static void csiset_pill(lv_obj_t *row, const char *txt, bool sel, lv_event_cb_t cb, int ud)
{
    lv_obj_t *b = lv_btn_create(row);
    lv_obj_set_height(b, 38);
    style_btn(b);
    lv_obj_set_style_pad_hor(b, 14, 0);
    if (sel) {
        lv_obj_set_style_bg_color(b, COL_AMBER, 0);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    }
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, (void *)(intptr_t)ud);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, sel ? COL_BG : COL_AMBER, 0);
    lv_obj_center(l);
}

static void render_csiset_page(void)
{
    if (!s_set_content) { return; }
    lv_obj_clean(s_set_content);
    for (int i = 0; i < MAX_CSI_SET; i++) { s_set_vallbl[i] = NULL; }

    int n = prop_coproc_csi_count();
    int pages = (n + CSISET_PER_PAGE - 1) / CSISET_PER_PAGE;
    int start = s_set_page * CSISET_PER_PAGE;

    for (int i = start; i < start + CSISET_PER_PAGE && i < n; i++) {
        const char *key, *desc, *opts; char type; int32_t val, lo, hi;
        if (!prop_coproc_csi_describe(i, &key, &val, &type, &lo, &hi, &desc, &opts)) { continue; }

        lv_obj_t *card = kit_card(s_set_content, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(card, 10, 0);
        lv_obj_set_style_pad_row(card, 6, 0);

        /* header: NAME ......... VALUE */
        lv_obj_t *hdr = lv_obj_create(card);
        lv_obj_remove_style_all(hdr);
        lv_obj_set_size(hdr, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_t *name = lv_label_create(hdr);
        lv_label_set_text(name, key);
        lv_obj_set_style_text_color(name, COL_AMBER, 0);
        lv_obj_t *vlbl = lv_label_create(hdr);
        char vb[16]; set_fmt_value(vb, sizeof(vb), type, val);
        if (type == 'T' && prop_calib_auto()) { snprintf(vb, sizeof(vb), "AUTO"); }
        lv_label_set_text(vlbl, vb);
        lv_obj_set_style_text_color(vlbl, COL_MUTE, 0);
        s_set_vallbl[i] = vlbl;

        /* control row */
        lv_obj_t *ctl = lv_obj_create(card);
        lv_obj_remove_style_all(ctl);
        lv_obj_set_size(ctl, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(ctl, LV_FLEX_FLOW_ROW_WRAP);
        lv_obj_set_style_pad_column(ctl, 8, 0);
        lv_obj_set_style_pad_row(ctl, 6, 0);

        if (type == 'B') {
            csiset_pill(ctl, "OFF", val == 0, csiset_opt_cb, (i << 8) | 0);
            csiset_pill(ctl, "ON",  val != 0, csiset_opt_cb, (i << 8) | 1);
        } else if (type == 'E' || type == 'M') {
            /* split opts on '|' */
            char buf[64]; strncpy(buf, opts ? opts : "", sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
            int k = 0; char *save = NULL;
            for (char *tok = strtok_r(buf, "|", &save); tok && k < 8; tok = strtok_r(NULL, "|", &save), k++) {
                bool sel = (type == 'M') ? (val & (1 << k)) != 0 : (val == k);
                csiset_pill(ctl, tok, sel, csiset_opt_cb, (i << 8) | k);
            }
        } else if (type == 'T') {
            bool autoon = prop_calib_auto();
            csiset_pill(ctl, "AUTO",   autoon,                 csiset_thr_cb, (i << 8) | 0);
            csiset_pill(ctl, "MIN",    !autoon && val == -2,   csiset_thr_cb, (i << 8) | 1);
            csiset_pill(ctl, "MANUAL", !autoon && val >= 0,    csiset_thr_cb, (i << 8) | 2);
            if (!autoon && val >= 0) {
                csiset_pill(ctl, "-", false, csiset_step_cb, (i << 8) | 0);
                csiset_pill(ctl, "+", false, csiset_step_cb, (i << 8) | 1);
            }
        } else {   /* I / F stepper */
            csiset_pill(ctl, LV_SYMBOL_MINUS, false, csiset_step_cb, (i << 8) | 0);
            csiset_pill(ctl, LV_SYMBOL_PLUS,  false, csiset_step_cb, (i << 8) | 1);
        }

        lv_obj_t *dl = lv_label_create(card);
        lv_label_set_text(dl, desc);
        lv_label_set_long_mode(dl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(dl, lv_pct(100));
        lv_obj_set_style_text_color(dl, COL_MUTE, 0);
    }

    if (s_set_pagelbl) {
        lv_label_set_text_fmt(s_set_pagelbl, "PAGE %d/%d", s_set_page + 1, pages);
    }
}

static lv_obj_t *build_csiset_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "CSI SETTINGS", back_to_csicfg_cb);
    s_set_page = 0;

    /* page nav (button-scrolled, not swipe) */
    lv_obj_t *prev = lv_btn_create(p);
    style_btn(prev); lv_obj_set_size(prev, 150, 50);
    lv_obj_align(prev, LV_ALIGN_BOTTOM_LEFT, 30, -18);
    lv_obj_add_event_cb(prev, csiset_page_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);
    lv_obj_t *pl = lv_label_create(prev); lv_label_set_text(pl, LV_SYMBOL_LEFT " PREV"); lv_obj_center(pl);

    lv_obj_t *next = lv_btn_create(p);
    style_btn(next); lv_obj_set_size(next, 150, 50);
    lv_obj_align(next, LV_ALIGN_BOTTOM_RIGHT, -30, -18);
    lv_obj_add_event_cb(next, csiset_page_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);
    lv_obj_t *nl = lv_label_create(next); lv_label_set_text(nl, "NEXT " LV_SYMBOL_RIGHT); lv_obj_center(nl);

    s_set_pagelbl = lv_label_create(p);
    lv_obj_set_style_text_color(s_set_pagelbl, COL_AMBER, 0);
    lv_obj_align(s_set_pagelbl, LV_ALIGN_BOTTOM_MID, 0, -32);

    /* scrollable content holding the current page's cards */
    s_set_content = lv_obj_create(p);
    lv_obj_remove_style_all(s_set_content);
    lv_obj_set_size(s_set_content, SCAN_W - 60, 430);
    lv_obj_align(s_set_content, LV_ALIGN_TOP_MID, 0, 66);
    lv_obj_set_flex_flow(s_set_content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_set_content, 10, 0);
    lv_obj_clear_flag(s_set_content, LV_OBJ_FLAG_SCROLLABLE);

    render_csiset_page();
    return p;
}

/* ---- Console home (PK_HOME): the in-world multi-data interface ------------
 * The default landing screen. Shows the device identity + a data-sponge status
 * strip (clock/date/temp/intake) and the function rail the SELECTOR drives. */

/* ---- Persistent icon rail glyphs (drawn from LVGL primitives, no font assets,
 * mirroring the article-visual approach). Each glyph is a set of small filled/
 * outlined boxes and polylines parented to a rail cell; recolouring iterates the
 * cell's children, so highlight is a colour swap with no rebuild. */

#define RAIL_CELL_H (600 / RAIL_COUNT)

/* lv_line keeps a pointer to its points (no copy) - each polyline needs a live
 * buffer. The rail is built once, so one buffer per glyph is enough. */
static lv_point_precise_t s_ic_scan[9];
static lv_point_precise_t s_ic_vit[6];
static lv_point_precise_t s_ic_sig[3][3];
static lv_point_precise_t s_ic_ins[2][2];
static lv_point_precise_t s_ic_ble[6];
static lv_point_precise_t s_ic_csi[3][8];
static lv_point_precise_t s_ic_instr[2];
static lv_point_precise_t s_ic_sens[3][3];

static lv_obj_t *ic_box(lv_obj_t *cell, int x, int y, int w, int h, lv_color_t col, bool fill, bool circle)
{
    lv_obj_t *o = lv_obj_create(cell);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_set_pos(o, x, y);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    if (fill) { lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0); lv_obj_set_style_bg_color(o, col, 0); }
    else { lv_obj_set_style_border_width(o, 2, 0); lv_obj_set_style_border_color(o, col, 0); }
    if (circle) lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    return o;
}

static lv_obj_t *ic_poly(lv_obj_t *cell, lv_point_precise_t *pts, int n, lv_color_t col)
{
    lv_obj_t *l = lv_line_create(cell);
    lv_line_set_points(l, pts, n);
    lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_line_color(l, col, 0);
    lv_obj_set_style_line_width(l, 3, 0);
    lv_obj_set_style_line_rounded(l, true, 0);
    return l;
}

/* Draw glyph `id` into `cell` (RAIL_W x RAIL_CELL_H), centred, in colour `col`. */
static void draw_icon(lv_obj_t *cell, icon_id_t id, lv_color_t col)
{
    switch (id) {
    case IC_HOME: {                                  /* house: roof + body */
        static lv_point_precise_t roof[3] = {{22,30},{38,16},{54,30}};
        ic_poly(cell, roof, 3, col);
        ic_box(cell, 26, 30, 24, 18, col, false, false);
        break; }
    case IC_ARCHIVE:                                 /* stacked bars / ledger */
        ic_box(cell, 24, 22, 28, 5, col, true, false);
        ic_box(cell, 24, 31, 28, 5, col, true, false);
        ic_box(cell, 24, 40, 28, 5, col, true, false);
        break;
    case IC_SCANNER: {                               /* sine sweep */
        int xs[9] = {22,26,30,34,38,42,46,50,54};
        int ys[9] = {33,26,33,40,33,26,33,40,33};
        for (int k = 0; k < 9; k++) { s_ic_scan[k].x = xs[k]; s_ic_scan[k].y = ys[k]; }
        ic_poly(cell, s_ic_scan, 9, col);
        break; }
    case IC_VITALS: {                                /* heartbeat trace */
        int xs[6] = {20,30,34,40,46,56};
        int ys[6] = {33,33,22,44,33,33};
        for (int k = 0; k < 6; k++) { s_ic_vit[k].x = xs[k]; s_ic_vit[k].y = ys[k]; }
        ic_poly(cell, s_ic_vit, 6, col);
        break; }
    case IC_SIGNAL: {                                /* radiating chevrons + source dot */
        int rx[3] = {8,14,20};
        for (int c = 0; c < 3; c++) {
            int r = rx[c];
            s_ic_sig[c][0].x = 38 - r; s_ic_sig[c][0].y = 44 + (3 - c) * 0;
            s_ic_sig[c][1].x = 38;     s_ic_sig[c][1].y = 44 - r;
            s_ic_sig[c][2].x = 38 + r; s_ic_sig[c][2].y = 44;
            s_ic_sig[c][0].y = 44; s_ic_sig[c][2].y = 44;
            ic_poly(cell, s_ic_sig[c], 3, col);
        }
        ic_box(cell, 35, 46, 6, 6, col, true, true);
        break; }
    case IC_SPECTRUM: {                              /* bar analyzer */
        int xs[4] = {22,32,42,52};
        int hs[4] = {14,26,18,28};
        for (int k = 0; k < 4; k++) ic_box(cell, xs[k], 48 - hs[k], 6, hs[k], col, true, false);
        break; }
    case IC_RFBAND: {                                /* channel-occupancy bars on a baseline */
        int xs[5] = {21,29,37,45,53};
        int hs[5] = {12,24,16,28,14};
        for (int k = 0; k < 5; k++) ic_box(cell, xs[k], 47 - hs[k], 5, hs[k], col, true, false);
        ic_box(cell, 19, 47, 40, 2, col, true, false);
        break; }
    case IC_CONTACTS: {                              /* bluetooth rune (nearby contacts) */
        int xs[6] = {30,46,38,38,46,30};
        int ys[6] = {40,24,16,48,40,24};
        for (int k = 0; k < 6; k++) { s_ic_ble[k].x = xs[k]; s_ic_ble[k].y = ys[k]; }
        ic_poly(cell, s_ic_ble, 6, col);
        break; }
    case IC_SIGENV: {                                /* signal-environment waterfall (stacked waves) */
        for (int d = 0; d < 3; d++) {
            int baseY = 24 + d * 9;
            for (int i = 0; i < 8; i++) {
                s_ic_csi[d][i].x = 20 + i * 5;
                s_ic_csi[d][i].y = baseY + (int)(sinf(i * 1.1f + d) * 3.0f);
            }
            ic_poly(cell, s_ic_csi[d], 8, col);
        }
        break; }
    case IC_INSTRUMENTS: {                            /* gauge: dial outline + needle + hub */
        ic_box(cell, 22, 20, 32, 32, col, false, true);
        s_ic_instr[0].x = 38; s_ic_instr[0].y = 36;
        s_ic_instr[1].x = 50; s_ic_instr[1].y = 24;
        ic_poly(cell, s_ic_instr, 2, col);
        ic_box(cell, 35, 33, 6, 6, col, true, true);
        break; }
    case IC_SENSORS: {                                /* radiating chevrons + source dot (radio sensing) */
        int rx[3] = {8,14,20};
        for (int c = 0; c < 3; c++) {
            int r = rx[c];
            s_ic_sens[c][0].x = 38 - r; s_ic_sens[c][0].y = 44;
            s_ic_sens[c][1].x = 38;     s_ic_sens[c][1].y = 44 - r;
            s_ic_sens[c][2].x = 38 + r; s_ic_sens[c][2].y = 44;
            ic_poly(cell, s_ic_sens[c], 3, col);
        }
        ic_box(cell, 35, 46, 6, 6, col, true, true);
        break; }
    case IC_CASSETTE:                                /* shell + two reels */
        ic_box(cell, 22, 24, 32, 18, col, false, false);
        ic_box(cell, 28, 30, 6, 6, col, true, true);
        ic_box(cell, 44, 30, 6, 6, col, true, true);
        break;
    case IC_INSIGHTS: {                              /* node graph */
        s_ic_ins[0][0].x = 31; s_ic_ins[0][0].y = 27; s_ic_ins[0][1].x = 50; s_ic_ins[0][1].y = 31;
        s_ic_ins[1][0].x = 31; s_ic_ins[1][0].y = 27; s_ic_ins[1][1].x = 37; s_ic_ins[1][1].y = 46;
        ic_poly(cell, s_ic_ins[0], 2, col);
        ic_poly(cell, s_ic_ins[1], 2, col);
        ic_box(cell, 28, 24, 6, 6, col, true, true);
        ic_box(cell, 47, 28, 6, 6, col, true, true);
        ic_box(cell, 34, 43, 6, 6, col, true, true);
        break; }
    case IC_SETUP:                                   /* sliders */
        ic_box(cell, 24, 25, 28, 3, col, true, false);
        ic_box(cell, 24, 34, 28, 3, col, true, false);
        ic_box(cell, 24, 43, 28, 3, col, true, false);
        ic_box(cell, 30, 22, 7, 9, col, true, false);
        ic_box(cell, 44, 31, 7, 9, col, true, false);
        ic_box(cell, 36, 40, 7, 9, col, true, false);
        break;
    }
}

/* Recolour every primitive in a rail cell (bg/line/border all set, harmlessly). */
static void recolor_cell(lv_obj_t *cell, lv_color_t col)
{
    uint32_t n = lv_obj_get_child_cnt(cell);
    for (uint32_t k = 0; k < n; k++) {
        lv_obj_t *ch = lv_obj_get_child(cell, k);
        lv_obj_set_style_bg_color(ch, col, 0);
        lv_obj_set_style_line_color(ch, col, 0);
        lv_obj_set_style_border_color(ch, col, 0);
    }
}

/* Light the cursor cell (glyph inverts to dark on an amber fill); others sit as
 * muted glyphs on transparent - camera-legible, matches the old rail. */
static void set_rail_highlight(void)
{
    for (int i = 0; i < RAIL_COUNT; i++) {
        if (!s_rail_btns[i]) {
            continue;
        }
        bool sel = (i == s_rail_sel);
        lv_obj_set_style_bg_opa(s_rail_btns[i], sel ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_color(s_rail_btns[i], COL_AMBER, 0);
        recolor_cell(s_rail_btns[i], sel ? COL_BG : COL_MUTE);
    }
}

static void rail_click_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    s_rail_sel = i;
    open_panel(s_rail[i].kind);
}

/* Build the persistent rail on the real screen (left of the content container). */
static void build_rail(lv_obj_t *screen)
{
    s_rail_strip = lv_obj_create(screen);
    lv_obj_remove_style_all(s_rail_strip);
    lv_obj_set_size(s_rail_strip, RAIL_W, 600);
    lv_obj_set_pos(s_rail_strip, 0, 0);
    lv_obj_set_style_bg_opa(s_rail_strip, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_rail_strip, lv_color_hex(0x080806), 0);
    lv_obj_set_style_border_side(s_rail_strip, LV_BORDER_SIDE_RIGHT, 0);
    lv_obj_set_style_border_color(s_rail_strip, COL_DIM, 0);
    lv_obj_set_style_border_width(s_rail_strip, 2, 0);
    lv_obj_clear_flag(s_rail_strip, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < RAIL_COUNT; i++) {
        lv_obj_t *cell = lv_obj_create(s_rail_strip);
        lv_obj_remove_style_all(cell);
        lv_obj_set_size(cell, RAIL_W, RAIL_CELL_H);
        lv_obj_set_pos(cell, 0, i * RAIL_CELL_H);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(cell, rail_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        draw_icon(cell, s_rail[i].icon, COL_MUTE);
        s_rail_btns[i] = cell;
    }
    set_rail_highlight();
}

/* A top-of-console status cell: caption above an (updatable) value. */
static lv_obj_t *status_cell(lv_obj_t *p, const char *cap, lv_coord_t x)
{
    lv_obj_t *c = lv_label_create(p);
    lv_label_set_text(c, cap);
    lv_obj_set_style_text_color(c, COL_MUTE, 0);
    lv_obj_set_style_text_font(c, FONT_BODY, 0);
    lv_obj_align(c, LV_ALIGN_TOP_LEFT, x, 96);

    lv_obj_t *v = lv_label_create(p);
    lv_label_set_text(v, "--");
    lv_obj_set_style_text_color(v, COL_AMBER, 0);
    lv_obj_set_style_text_font(v, FONT_HEAD, 0);
    lv_obj_align(v, LV_ALIGN_TOP_LEFT, x, 116);
    return v;
}

static lv_obj_t *build_home_panel(lv_obj_t *parent)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_size(p, SCAN_W, 600);
    lv_obj_align(p, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(p, COL_BG, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(p);
    lv_label_set_text(t, "ARMADA MULTI-DATA INTERFACE");
    lv_obj_set_style_text_color(t, COL_AMBER, 0);
    lv_obj_set_style_text_font(t, FONT_HEAD, 0);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 24, 16);

    lv_obj_t *sub = lv_label_create(p);
    lv_label_set_text(sub, "FIELD UNIT 7   //   OPER: ZARRAH");
    lv_obj_set_style_text_color(sub, COL_MUTE, 0);
    lv_obj_set_style_text_font(sub, FONT_BODY, 0);
    lv_obj_align(sub, LV_ALIGN_TOP_LEFT, 24, 52);

    lv_obj_t *rule = lv_obj_create(p);
    lv_obj_set_size(rule, SCAN_W - 48, 3);
    lv_obj_set_style_bg_color(rule, COL_DIM, 0);
    lv_obj_set_style_border_width(rule, 0, 0);
    lv_obj_align(rule, LV_ALIGN_TOP_MID, 0, 84);

    /* Data-sponge status strip (live values filled by the observer). */
    s_home_clock = status_cell(p, "CLOCK", 24);
    lv_obj_t *date = status_cell(p, "DATE", 230);
    lv_label_set_text(date, prop_date_stamp);   /* static placeholder (no RTC yet) */
    s_home_temp  = status_cell(p, "CORE TEMP", 600);
    s_home_link  = status_cell(p, "INTAKE", 790);

    /* The function rail now lives in the persistent left strip; the console body
     * stays an uncluttered identity card. */
    lv_obj_t *prompt = lv_label_create(p);
    lv_label_set_text(prompt, "DATA SPONGE LINKED  //  SELECT A FUNCTION FROM THE RAIL");
    lv_obj_set_style_text_color(prompt, COL_MUTE, 0);
    lv_obj_set_style_text_font(prompt, FONT_HEAD, 0);
    lv_obj_align(prompt, LV_ALIGN_LEFT_MID, 24, 0);

    lv_obj_t *hint = lv_label_create(p);
    lv_label_set_text(hint, "SELECTOR  rotate rail  -  PRESS  open  -  SWITCH  archive tabs");
    lv_obj_set_style_text_color(hint, COL_MUTE, 0);
    lv_obj_set_style_text_font(hint, FONT_BODY, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -16);
    return p;
}

/* ---- ARCHIVE browser (PK_ARCHIVE): tabbed sections of entries ------------- */

static int clamp_section(int n)
{
    if (n < 0) return 0;
    if (n >= prop_section_count) return prop_section_count - 1;
    return n;
}

static void set_arch_row_highlight(void)
{
    for (int i = 0; i < s_arch_row_count; i++) {
        if (!s_arch_rows[i]) {
            continue;
        }
        bool sel = (i == s_archive_entry);
        lv_obj_set_style_bg_color(s_arch_rows[i], sel ? COL_AMBER : COL_PANEL_ITEM, 0);
        lv_obj_set_style_border_color(s_arch_rows[i], sel ? COL_AMBER : COL_DIM, 0);
        lv_obj_t *lbl = lv_obj_get_child(s_arch_rows[i], 0);
        if (lbl) {
            lv_obj_set_style_text_color(lbl, sel ? COL_BG : COL_AMBER, 0);
        }
    }
}

static void arch_tab_cb(lv_event_t *e)
{
    s_archive_section = clamp_section((int)(intptr_t)lv_event_get_user_data(e));
    s_archive_entry = 0;
    open_panel(PK_ARCHIVE);   /* rebuild for the new section (one panel alive) */
}

static void arch_row_cb(lv_event_t *e)
{
    s_archive_entry = (int)(intptr_t)lv_event_get_user_data(e);
    open_panel(PK_ARTICLE);
}

static lv_obj_t *build_archive_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "ARCHIVE", back_to_home_cb);
    s_archive_section = clamp_section(s_archive_section);

    /* Section tabs (the author's "switches = tabs"), below the header row. */
    for (int i = 0; i < prop_section_count && i < (int)(sizeof(s_arch_tabs) / sizeof(s_arch_tabs[0])); i++) {
        bool sel = (i == s_archive_section);
        lv_obj_t *b = lv_btn_create(p);
        lv_obj_set_size(b, 150, 40);
        lv_obj_align(b, LV_ALIGN_TOP_LEFT, 40 + i * 160, 70);
        style_btn(b);
        lv_obj_set_style_bg_color(b, sel ? COL_AMBER : COL_PANEL_ITEM, 0);
        lv_obj_add_event_cb(b, arch_tab_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *l = lv_label_create(b);
        lv_label_set_text(l, prop_sections[i].name);
        lv_obj_set_style_text_color(l, sel ? COL_BG : COL_AMBER, 0);
        lv_obj_center(l);
        s_arch_tabs[i] = b;
    }

    /* Entry rows for the active section. */
    const prop_section_t *sec = &prop_sections[s_archive_section];
    s_arch_row_count = sec->count;
    if (s_arch_row_count > (int)(sizeof(s_arch_rows) / sizeof(s_arch_rows[0]))) {
        s_arch_row_count = (int)(sizeof(s_arch_rows) / sizeof(s_arch_rows[0]));
    }
    if (s_archive_entry >= s_arch_row_count) {
        s_archive_entry = 0;
    }
    for (int i = 0; i < s_arch_row_count; i++) {
        lv_obj_t *b = lv_btn_create(p);
        lv_obj_set_size(b, SCAN_W - 80, 52);
        lv_obj_align(b, LV_ALIGN_TOP_LEFT, 40, 128 + i * 60);
        style_btn(b);
        lv_obj_add_event_cb(b, arch_row_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *l = lv_label_create(b);
        lv_label_set_text(l, sec->entries[i].title);
        lv_obj_set_style_text_font(l, FONT_HEAD, 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 18, 0);
        s_arch_rows[i] = b;
    }
    set_arch_row_highlight();
    return p;
}

static void back_to_archive_cb(lv_event_t *e) { (void)e; open_panel(PK_ARCHIVE); }

/* ---- Article visuals: one distinct, asset-free motif per section type -------
 * Every visual is drawn from LVGL primitives in the amber-on-black style kit, so
 * each section reads as its own kind of page (thermal trace / nav chart / bio-scan
 * dossier / field plate) without shipping any image data. Pseudo-values come from
 * the entry title so each page looks individual; the author edits text only and
 * the visuals follow. The visual band sits at y116..284; the body starts at y312. */

#define VIS_Y      116
#define VIS_H      168
#define VIS_BODY_Y 312

static lv_point_precise_t s_vis_curve[64];   /* climate trace (lv_line needs a live buffer) */
static lv_point_precise_t s_vis_route[5];    /* map: survey trail through region markers */
static lv_point_precise_t s_vis_dune[3][20]; /* map: dune-field contour texture */
static lv_point_precise_t s_vis_ridge[12];   /* map: escarpment ridge line */

static uint32_t title_hash(const char *s)
{
    uint32_t h = 2166136261u;
    while (s && *s) {
        h = (h ^ (uint8_t)*s++) * 16777619u;
    }
    return h;
}

/* A framed inset box - the common visual container. */
static lv_obj_t *visual_frame(lv_obj_t *p, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *b = lv_obj_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, w, h);
    lv_obj_align(b, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(b, COL_PANEL_ITEM, 0);
    lv_obj_set_style_border_color(b, COL_DIM, 0);
    lv_obj_set_style_border_width(b, 2, 0);
    lv_obj_set_style_radius(b, 0, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    return b;
}

/* A thin filled bar: grid lines, crosshairs, cactus segments. */
static lv_obj_t *vbar(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                      lv_coord_t w, lv_coord_t h, lv_color_t col)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_align(o, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o, col, 0);
    return o;
}

/* A circle (outline or filled), centred at (cx,cy). */
static lv_obj_t *vcircle(lv_obj_t *parent, lv_coord_t cx, lv_coord_t cy,
                         lv_coord_t r, lv_color_t col, bool fill)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, 2 * r, 2 * r);
    lv_obj_align(o, LV_ALIGN_TOP_LEFT, cx - r, cy - r);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    if (fill) {
        lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(o, col, 0);
    } else {
        lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(o, col, 0);
        lv_obj_set_style_border_width(o, 2, 0);
    }
    return o;
}

static lv_obj_t *vis_caption(lv_obj_t *p, const char *txt)
{
    lv_obj_t *l = lv_label_create(p);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, COL_MUTE, 0);
    lv_obj_set_style_text_font(l, FONT_BODY, 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 30, 94);
    return l;
}

/* CLIMATE - a diurnal thermal trace (oscilloscope/recorder character). */
static void vis_climate(lv_obj_t *p, const prop_entry_t *en)
{
    vis_caption(p, "THERMAL CYCLE  //  24H DIURNAL");
    lv_obj_t *box = visual_frame(p, 30, VIS_Y, SCAN_W - 60, VIS_H);
    int iw = SCAN_W - 60 - 4, ih = VIS_H - 4;

    for (int i = 1; i < 8; i++) vbar(box, i * iw / 8, 0, 1, ih, COL_DIM);
    for (int i = 1; i < 4; i++) vbar(box, 0, i * ih / 4, iw, 1, COL_DIM);

    uint32_t h = title_hash(en->title);
    float amp = 0.62f + (h % 26) / 100.0f;
    int lo = 2 + (int)(h % 7);
    int hi = 42 + (int)((h >> 3) % 9);
    int n = (int)(sizeof(s_vis_curve) / sizeof(s_vis_curve[0]));
    for (int i = 0; i < n; i++) {
        float u = (float)i / (n - 1);                       /* 0..1 across the day */
        float t = -cosf((u - 0.22f) * 6.2831853f);          /* min ~dawn, peak ~afternoon */
        t = t * 0.5f + 0.5f;
        t = t * amp + (1.0f - amp) * 0.15f;
        s_vis_curve[i].x = (lv_coord_t)(2 + u * (iw - 1));
        s_vis_curve[i].y = (lv_coord_t)(2 + (1.0f - t) * (ih - 1));
    }
    lv_obj_t *line = lv_line_create(box);
    lv_obj_set_style_line_color(line, COL_AMBER, 0);
    lv_obj_set_style_line_width(line, 2, 0);
    lv_line_set_points(line, s_vis_curve, n);

    lv_obj_t *hl = lv_label_create(p);
    lv_label_set_text_fmt(hl, "HI %d C   LO %02d C", hi, lo);
    lv_obj_set_style_text_color(hl, COL_MUTE, 0);
    lv_obj_set_style_text_font(hl, FONT_BODY, 0);
    lv_obj_align(hl, LV_ALIGN_TOP_RIGHT, -34, 94);

    const char *ax[3] = { "NIGHT", "NOON", "NIGHT" };
    lv_align_t aa[3] = { LV_ALIGN_TOP_LEFT, LV_ALIGN_TOP_MID, LV_ALIGN_TOP_RIGHT };
    lv_coord_t ax_x[3] = { 34, 0, -34 };
    for (int i = 0; i < 3; i++) {
        lv_obj_t *l = lv_label_create(p);
        lv_label_set_text(l, ax[i]);
        lv_obj_set_style_text_color(l, COL_DIM, 0);
        lv_obj_set_style_text_font(l, FONT_BODY, 0);
        lv_obj_align(l, aa[i], ax_x[i], VIS_Y + VIS_H + 2);
    }
}

/* MAP - a survey map of the territory: a faint graticule, dune-field contours, the
 * escarpment ridge, labelled region markers linked by a trail, a compass rose and
 * a scale bar, with the current region marked. Drawn from primitives, no assets. */
static void vis_map(lv_obj_t *p, int entry, const prop_entry_t *en)
{
    vis_caption(p, "SECTOR CHART  //  SURVEY MAP");
    lv_obj_t *box = visual_frame(p, 30, VIS_Y, SCAN_W - 60, VIS_H);
    int iw = SCAN_W - 60 - 4, ih = VIS_H - 4;

    /* Lat/long graticule (faint). */
    for (int i = 1; i < 8; i++) vbar(box, i * iw / 8, 0, 1, ih, COL_DIM);
    for (int i = 1; i < 4; i++) vbar(box, 0, i * ih / 4, iw, 1, COL_DIM);

    /* Dune fields: stacked wavy contour lines (terrain texture) in the west. */
    int dn = (int)(sizeof(s_vis_dune[0]) / sizeof(s_vis_dune[0][0]));
    for (int d = 0; d < 3; d++) {
        int baseY = (int)((0.30f + d * 0.16f) * ih);
        for (int i = 0; i < dn; i++) {
            s_vis_dune[d][i].x = (lv_coord_t)(iw * 0.04f + (float)i / (dn - 1) * iw * 0.50f);
            s_vis_dune[d][i].y = (lv_coord_t)(baseY + sinf(i * 0.7f + d * 1.3f) * 5.0f);
        }
        lv_obj_t *du = lv_line_create(box);
        lv_obj_set_style_line_color(du, COL_DIM, 0);
        lv_obj_set_style_line_width(du, 2, 0);
        lv_line_set_points(du, s_vis_dune[d], dn);
    }

    /* The escarpment: a bold ridge line down the eastern edge. */
    int rn = (int)(sizeof(s_vis_ridge) / sizeof(s_vis_ridge[0]));
    for (int i = 0; i < rn; i++) {
        s_vis_ridge[i].x = (lv_coord_t)(iw * 0.74f + sinf(i * 1.1f) * 12.0f + (i & 1 ? 5 : -5));
        s_vis_ridge[i].y = (lv_coord_t)((float)i / (rn - 1) * (ih - 6) + 3);
    }
    lv_obj_t *ridge = lv_line_create(box);
    lv_obj_set_style_line_color(ridge, COL_MUTE, 0);
    lv_obj_set_style_line_width(ridge, 3, 0);
    lv_line_set_points(ridge, s_vis_ridge, rn);

    /* Region markers (one per MAP entry), linked by a dim survey trail. */
    static const float mx[4] = { 0.16f, 0.42f, 0.80f, 0.30f };
    static const float my[4] = { 0.42f, 0.72f, 0.45f, 0.80f };
    int nmark = prop_sections[1].count;
    if (nmark > 4) nmark = 4;
    for (int i = 0; i < nmark; i++) {
        s_vis_route[i].x = (lv_coord_t)(mx[i] * (iw - 1));
        s_vis_route[i].y = (lv_coord_t)(my[i] * (ih - 1));
    }
    if (nmark > 1) {
        lv_obj_t *trail = lv_line_create(box);
        lv_obj_set_style_line_color(trail, COL_DIM, 0);
        lv_obj_set_style_line_width(trail, 1, 0);
        lv_line_set_points(trail, s_vis_route, nmark);
    }

    int here = nmark ? (((entry % nmark) + nmark) % nmark) : 0;
    for (int i = 0; i < nmark; i++) {
        lv_coord_t cx = s_vis_route[i].x, cy = s_vis_route[i].y;
        bool act = (i == here);
        vbar(box, cx - (act ? 5 : 4), cy - (act ? 5 : 4),
             act ? 10 : 8, act ? 10 : 8, act ? COL_AMBER : COL_MUTE);
        lv_obj_t *nb = lv_label_create(box);
        lv_label_set_text_fmt(nb, "%d", i + 1);
        lv_obj_set_style_text_color(nb, act ? COL_AMBER : COL_MUTE, 0);
        lv_obj_set_style_text_font(nb, FONT_BODY, 0);
        lv_obj_align(nb, LV_ALIGN_TOP_LEFT, cx + 8, cy - 8);
        if (act) {
            vbar(box, cx, 0, 1, ih, COL_AMBER);     /* you-are-here crosshair */
            vbar(box, 0, cy, iw, 1, COL_AMBER);
        }
    }

    /* Compass rose (top-right inside the frame). */
    int compx = iw - 42, compy = 32;
    vbar(box, compx, compy - 16, 2, 32, COL_MUTE);
    vbar(box, compx - 16, compy, 32, 2, COL_MUTE);
    vbar(box, compx - 1, compy - 22, 2, 8, COL_AMBER);   /* north tip */
    lv_obj_t *nlbl = lv_label_create(box);
    lv_label_set_text(nlbl, "N");
    lv_obj_set_style_text_color(nlbl, COL_AMBER, 0);
    lv_obj_set_style_text_font(nlbl, FONT_BODY, 0);
    lv_obj_align(nlbl, LV_ALIGN_TOP_LEFT, compx - 5, compy - 42);

    /* Scale bar (bottom-left inside the frame). */
    vbar(box, 16, ih - 18, 96, 2, COL_MUTE);
    vbar(box, 16, ih - 22, 2, 8, COL_MUTE);
    vbar(box, 64, ih - 21, 2, 6, COL_MUTE);
    vbar(box, 112, ih - 22, 2, 8, COL_MUTE);
    lv_obj_t *sclbl = lv_label_create(box);
    lv_label_set_text(sclbl, "0       100 KM");
    lv_obj_set_style_text_color(sclbl, COL_DIM, 0);
    lv_obj_set_style_text_font(sclbl, FONT_BODY, 0);
    lv_obj_align(sclbl, LV_ALIGN_TOP_LEFT, 14, ih - 36);

    /* Active region name + grid reference. */
    lv_obj_t *co = lv_label_create(p);
    lv_label_set_text_fmt(co, "%s   //   GRID %02d.%d / %02d.%d", en->title,
                          (int)(mx[here] * 40), (here * 7) % 10,
                          (int)(my[here] * 40), (here * 3) % 10);
    lv_obj_set_style_text_color(co, COL_MUTE, 0);
    lv_obj_set_style_text_font(co, FONT_BODY, 0);
    lv_obj_align(co, LV_ALIGN_TOP_RIGHT, -34, 94);
}

/* WILDLIFE - a bio-scan dossier: a reticle portrait + a stat block. */
static void vis_wildlife(lv_obj_t *p, const prop_entry_t *en)
{
    vis_caption(p, "SPECIMEN DOSSIER  //  BIO-SCAN");
    lv_obj_t *pf = visual_frame(p, 30, VIS_Y, VIS_H, VIS_H);   /* square portrait */
    int c = VIS_H / 2 - 2;
    vbar(pf, c, 6, 1, VIS_H - 16, COL_DIM);                    /* reticle crosshair */
    vbar(pf, 6, c, VIS_H - 16, 1, COL_DIM);
    vcircle(pf, c, c, 58, COL_DIM, false);
    vcircle(pf, c, c, 40, COL_DIM, false);
    vcircle(pf, c - 16, c - 6, 9, COL_AMBER, true);            /* nocturnal "eyes" */
    vcircle(pf, c + 16, c - 6, 9, COL_AMBER, true);

    uint32_t h = title_hash(en->title);
    const char *labels[4] = { "SIZE", "THREAT", "ACTIVITY", "WATER NEED" };
    int vals[4] = {
        10 + (int)(h % 80),
        (int)((h >> 3) % 95),
        30 + (int)((h >> 6) % 70),
        5 + (int)((h >> 9) % 45),
    };
    int x0 = 30 + VIS_H + 30;
    for (int i = 0; i < 4; i++) {
        lv_coord_t y = VIS_Y + 6 + i * 40;
        lv_obj_t *l = lv_label_create(p);
        lv_label_set_text(l, labels[i]);
        lv_obj_set_style_text_color(l, COL_AMBER, 0);
        lv_obj_set_style_text_font(l, FONT_BODY, 0);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, x0, y);
        lv_obj_t *bar = make_meter_bar(p, x0 + 150, y, SCAN_W - 30 - (x0 + 150));
        set_meter(bar, vals[i], vals[i] > 75 ? COL_ALERT : COL_AMBER);
    }
}

/* PLANTS - a field-guide plate: a drawn cactus + spec tags. */
static void vis_plants(lv_obj_t *p, const prop_entry_t *en)
{
    vis_caption(p, "FIELD PLATE  //  FLORA SPECIMEN");
    lv_obj_t *pl = visual_frame(p, 30, VIS_Y, VIS_H, VIS_H);

    uint32_t h = title_hash(en->title);
    vbar(pl, 14, VIS_H - 22, VIS_H - 32, 2, COL_DIM);          /* ground line */
    vbar(pl, 74, 46, 14, VIS_H - 70, COL_AMBER);              /* trunk */
    if (h & 1) {                                              /* left arm */
        vbar(pl, 48, 92, 28, 12, COL_AMBER);
        vbar(pl, 48, 64, 12, 40, COL_AMBER);
    }
    if (h & 2) {                                             /* right arm */
        vbar(pl, 86, 78, 26, 12, COL_AMBER);
        vbar(pl, 100, 50, 12, 40, COL_AMBER);
    }
    if (h & 4) {
        vcircle(pl, 81, 44, 7, COL_AMBER, true);             /* bloom / fruit */
    }

    const char *tags[3] = { "HEIGHT", "WATER", "BLOOM" };
    char vbuf[3][20];
    snprintf(vbuf[0], sizeof(vbuf[0]), "%d - %d m", 1 + (int)(h % 4), 6 + (int)((h >> 2) % 10));
    snprintf(vbuf[1], sizeof(vbuf[1]), "%s", (h & 8) ? "VERY LOW" : "LOW");
    snprintf(vbuf[2], sizeof(vbuf[2]), "%s", (h & 4) ? "NIGHT" : "SEASONAL");
    int x0 = 30 + VIS_H + 30;
    for (int i = 0; i < 3; i++) {
        lv_coord_t y = VIS_Y + 8 + i * 50;
        lv_obj_t *chip = lv_obj_create(p);
        lv_obj_remove_style_all(chip);
        lv_obj_set_size(chip, SCAN_W - 30 - x0, 38);
        lv_obj_align(chip, LV_ALIGN_TOP_LEFT, x0, y);
        lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(chip, COL_PANEL_ITEM, 0);
        lv_obj_set_style_border_color(chip, COL_DIM, 0);
        lv_obj_set_style_border_width(chip, 1, 0);
        lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *lt = lv_label_create(chip);
        lv_label_set_text(lt, tags[i]);
        lv_obj_set_style_text_color(lt, COL_MUTE, 0);
        lv_obj_align(lt, LV_ALIGN_LEFT_MID, 12, 0);
        lv_obj_t *lv = lv_label_create(chip);
        lv_label_set_text(lv, vbuf[i]);
        lv_obj_set_style_text_color(lv, COL_AMBER, 0);
        lv_obj_align(lv, LV_ALIGN_RIGHT_MID, -12, 0);
    }
}

static lv_obj_t *build_article_panel(lv_obj_t *parent)
{
    int section = clamp_section(s_archive_section);
    const prop_section_t *sec = &prop_sections[section];
    int idx = s_archive_entry;
    if (idx < 0 || idx >= sec->count) {
        idx = 0;
    }
    const prop_entry_t *en = &sec->entries[idx];

    lv_obj_t *p = make_panel(parent, sec->name, back_to_archive_cb);

    lv_obj_t *t = lv_label_create(p);
    lv_label_set_text(t, en->title);
    lv_obj_set_style_text_color(t, COL_AMBER, 0);
    lv_obj_set_style_text_font(t, FONT_HEAD, 0);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 30, 64);

    /* Distinct visual per section type (index order matches prop_sections). */
    switch (section) {
        case 0: vis_climate(p, en); break;
        case 1: vis_map(p, idx, en); break;
        case 2: vis_wildlife(p, en); break;
        case 3: vis_plants(p, en); break;
        default: break;
    }

    /* Scrollable body container so long entries page under the SELECTOR dial. */
    s_article_scroll = lv_obj_create(p);
    lv_obj_set_size(s_article_scroll, SCAN_W - 60, 600 - VIS_BODY_Y - 16);
    lv_obj_align(s_article_scroll, LV_ALIGN_TOP_LEFT, 30, VIS_BODY_Y);
    lv_obj_set_style_bg_opa(s_article_scroll, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_article_scroll, 0, 0);
    lv_obj_set_style_pad_all(s_article_scroll, 0, 0);
    lv_obj_set_scroll_dir(s_article_scroll, LV_DIR_VER);

    lv_obj_t *body = lv_label_create(s_article_scroll);
    lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(body, SCAN_W - 100);
    lv_label_set_text(body, en->body);
    lv_obj_set_style_text_color(body, COL_AMBER, 0);
    lv_obj_set_style_text_font(body, FONT_BODY, 0);
    lv_obj_align(body, LV_ALIGN_TOP_LEFT, 0, 0);
    return p;
}

/* ---- CASSETTE deck (PK_CASSETTE): stubbed transport ----------------------- */
static lv_obj_t *build_cassette_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "CASSETTE", back_to_home_cb);

    /* Two reels + a counter, purely decorative for now. */
    for (int i = 0; i < 2; i++) {
        lv_obj_t *reel = lv_obj_create(p);
        lv_obj_set_size(reel, 120, 120);
        lv_obj_align(reel, LV_ALIGN_TOP_MID, i ? 170 : -170, 120);
        lv_obj_set_style_radius(reel, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(reel, COL_PANEL_ITEM, 0);
        lv_obj_set_style_border_color(reel, COL_AMBER, 0);
        lv_obj_set_style_border_width(reel, 3, 0);
    }
    lv_obj_t *ctr = lv_label_create(p);
    lv_label_set_text(ctr, "0 0 0 0");
    lv_obj_set_style_text_color(ctr, COL_AMBER, 0);
    lv_obj_set_style_text_font(ctr, FONT_HEAD, 0);
    lv_obj_align(ctr, LV_ALIGN_TOP_MID, 0, 150);

    const char *xport[] = { LV_SYMBOL_PREV, LV_SYMBOL_PLAY, LV_SYMBOL_STOP, LV_SYMBOL_NEXT };
    for (int i = 0; i < 4; i++) {
        make_btn(p, xport[i], 90, LV_ALIGN_TOP_MID, (i - 2) * 100 + 50, 300, back_to_home_cb);
    }

    lv_obj_t *st = lv_label_create(p);
    lv_label_set_text(st, "NO CASSETTE LOADED");
    lv_obj_set_style_text_color(st, COL_MUTE, 0);
    lv_obj_set_style_text_font(st, FONT_HEAD, 0);
    lv_obj_align(st, LV_ALIGN_BOTTOM_MID, 0, -70);

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "Transport idle - playback not yet wired to the audio stage.");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_MID, 0, -34);
    return p;
}

/* ---- INSIGHTS engine (PK_INSIGHTS): stub ---------------------------------- */
static lv_obj_t *build_insights_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "INSIGHTS", back_to_home_cb);

    lv_obj_t *hd = lv_label_create(p);
    lv_label_set_text(hd, "INSIGHT ENGINE  //  IDLE");
    lv_obj_set_style_text_color(hd, COL_AMBER, 0);
    lv_obj_set_style_text_font(hd, FONT_HEAD, 0);
    lv_obj_align(hd, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "Programmed correlations across the archive will surface here.");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_CENTER, 0, 24);
    return p;
}

/* Global SETUP menu (its own lazily-built panel). Rows open sub-panels by kind. */
/* ======================= I/O BENCH (PK_IO / PK_IO_PIN) ===================
 * Overview is a one-page grid of pin boxes (number inside, coloured by state).
 * Tapping a box opens a single-pin config page exposing the full GPIO option set
 * (mode, pull-up/down, drive strength, edge interrupt) plus level/analog controls.
 * bsp_aio owns all the hardware + NVS persistence; this is a pure view over it. */

static void back_to_io_cb(lv_event_t *e) { (void)e; open_panel(PK_IO); }

static const char *io_mode_name(aio_mode_t m)
{
    switch (m) {
        case AIO_DIGITAL_IN:  return "DIGITAL IN";
        case AIO_DIGITAL_OUT: return "DIGITAL OUT";
        case AIO_ANALOG_IN:   return "ANALOG IN";
        case AIO_ANALOG_OUT:  return "ANALOG OUT";
        default:              return "?";
    }
}

/* Candidate modes a pin can take, in dropdown order (ANALOG IN only on ADC pins). */
static aio_mode_t io_candidate(int pin, int sel)
{
    int n = 0;
    for (int m = 0; m < AIO_MODE_COUNT; m++) {
        if (m == AIO_ANALOG_IN && !bsp_aio_info(pin)->adc_ok) continue;
        if (n == sel) return (aio_mode_t)m;
        n++;
    }
    return AIO_DIGITAL_IN;
}
static int io_sel_of(int pin, aio_mode_t mode)
{
    int n = 0;
    for (int m = 0; m < AIO_MODE_COUNT; m++) {
        if (m == AIO_ANALOG_IN && !bsp_aio_info(pin)->adc_ok) continue;
        if (m == (int)mode) return n;
        n++;
    }
    return 0;
}

static void io_aout_text(int idx, char *buf, size_t n)
{
    int pct = bsp_aio_get_aout(idx);
    if (bsp_aio_get_volts_pref(idx)) snprintf(buf, n, "%.2f V", (double)bsp_aio_volts(pct));
    else                             snprintf(buf, n, "%d%%", pct);
}

static lv_obj_t *io_btn(lv_obj_t *parent, const char *txt, lv_coord_t w, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, w, 50);
    style_btn(b);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_center(l);
    return b;
}

/* ---- overview box paint ---- */

/* Tapping a box opens its single-pin config page. */
static void io_box_tap_cb(lv_event_t *e)
{
    s_io_pin = (int)(intptr_t)lv_event_get_user_data(e);
    open_panel(PK_IO_PIN);
}

/* A small int capturing everything io_box_paint draws, so the observer repaints a box
 * only when its state actually changes. */
static int io_box_key(int i)
{
    aio_mode_t m = bsp_aio_get_mode(i);
    int v = 0;
    if (m == AIO_ANALOG_IN)       { int raw = 0, pct = 0; if (bsp_aio_read_ain(i, &raw, &pct) == ESP_OK) v = pct; }
    else if (m == AIO_ANALOG_OUT) { v = bsp_aio_get_aout(i); }
    else                          { v = (bsp_aio_read_level(i) > 0); }   /* DI / DO */
    return (int)m * 1000 + v;
}

/* Paint one overview box: border by direction (thin/muted input, thick/amber output,
 * hairline dim when disabled), fill/colour by value. Safe from build and the observer. */
static void io_box_paint(int i)
{
    lv_obj_t *box = s_io_box[i];
    if (!box || !s_io_val[i] || !s_io_sub[i] || !s_io_fill[i]) return;
    aio_mode_t m = bsp_aio_get_mode(i);
    bool analog = (m == AIO_ANALOG_IN || m == AIO_ANALOG_OUT);
    bool out = aio_is_output(m) || m == AIO_ANALOG_OUT;

    /* Border encodes direction: thin muted = input, thick amber = output. */
    lv_obj_set_style_border_color(box, out ? COL_AMBER : COL_MUTE, 0);
    lv_obj_set_style_border_width(box, out ? 4 : 2, 0);
    lv_obj_add_flag(s_io_fill[i], LV_OBJ_FLAG_HIDDEN);

    if (analog) {
        int pct = io_box_key(i) % 1000;
        lv_obj_set_style_bg_color(box, COL_PANEL_ITEM, 0);
        lv_obj_clear_flag(s_io_fill[i], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(s_io_fill[i], COL_DIM, 0);
        int inner = IO_BOX_H - 2 * (out ? 4 : 2);
        lv_obj_set_height(s_io_fill[i], pct * inner / 100);
        lv_obj_align(s_io_fill[i], LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_text_color(s_io_val[i], COL_AMBER, 0);
        char sub[16];
        if (m == AIO_ANALOG_OUT && bsp_aio_get_volts_pref(i))
            snprintf(sub, sizeof(sub), "%.2fV", (double)bsp_aio_volts(pct));
        else
            snprintf(sub, sizeof(sub), "%d%%", pct);
        lv_label_set_text(s_io_sub[i], sub);
        lv_obj_set_style_text_color(s_io_sub[i], COL_AMBER, 0);
    } else {   /* a plain digital GPIO mode */
        int lvl = bsp_aio_read_level(i) > 0;
        lv_obj_set_style_bg_color(box, lvl ? COL_AMBER : COL_PANEL_ITEM, 0);
        lv_color_t fg = lvl ? COL_BG : COL_MUTE;   /* invert text when the box lights up */
        lv_obj_set_style_text_color(s_io_val[i], fg, 0);
        lv_label_set_text(s_io_sub[i], lvl ? "HI" : "LO");
        lv_obj_set_style_text_color(s_io_sub[i], fg, 0);
    }
}

/* ---- single-pin config-page callbacks (operate on s_io_pin) ---- */
static void io_rebuild_async(void *u) { (void)u; if (s_cur_kind == PK_IO_PIN) open_panel(PK_IO_PIN); }

static void io_mode_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    aio_mode_t m = io_candidate(pin, lv_dropdown_get_selected(lv_event_get_target(e)));
    esp_err_t err = bsp_aio_set_mode(pin, m);
    if (err != ESP_OK) {
        snprintf(s_io_err, sizeof(s_io_err), "%s",
                 err == ESP_ERR_NO_MEM ? "no free PWM channel" : esp_err_to_name(err));
    }
    lv_async_call(io_rebuild_async, NULL);   /* mode change restructures the page */
}

static void io_pull_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_pull(pin, (aio_pull_t)lv_dropdown_get_selected(lv_event_get_target(e)));
}
static void io_filter_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_filter(pin, lv_dropdown_get_selected(lv_event_get_target(e)));
}
static void io_drive_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_drive(pin, lv_dropdown_get_selected(lv_event_get_target(e)));
}
static void io_irq_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_irq(pin, (aio_irq_t)lv_dropdown_get_selected(lv_event_get_target(e)));
    lv_async_call(io_rebuild_async, NULL);   /* show/hide the edge-count row */
}
static void io_edges_reset_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_reset_edges(pin);
    if (s_pin_edges) lv_label_set_text(s_pin_edges, "0");
}
static void io_od_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_od(pin, lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED));
}
static void io_atten_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_atten(pin, lv_dropdown_get_selected(lv_event_get_target(e)));
}
static void io_freq_cb(lv_event_t *e)
{
    bsp_aio_set_freq(lv_dropdown_get_selected(lv_event_get_target(e)));
}
static void io_dout_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_dout(pin, lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED));
}
static void io_volts_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    bsp_aio_set_volts_pref(pin, lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED));
    if (s_pin_aout) { char b[24]; io_aout_text(pin, b, sizeof(b)); lv_label_set_text(s_pin_aout, b); }
}

/* Apply a new analog-out duty and keep slider / entry / readout in sync. */
static void io_aout_apply(int pin, int pct, bool sync_entry)
{
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    bsp_aio_set_aout(pin, pct);
    if (s_pin_slider) lv_slider_set_value(s_pin_slider, pct, LV_ANIM_OFF);
    if (sync_entry && s_pin_entry) { char b[8]; snprintf(b, sizeof(b), "%d", pct); lv_textarea_set_text(s_pin_entry, b); }
    if (s_pin_aout) { char b[24]; io_aout_text(pin, b, sizeof(b)); lv_label_set_text(s_pin_aout, b); }
}
static void io_slider_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    io_aout_apply(pin, lv_slider_get_value(lv_event_get_target(e)), true);
}
static void io_entry_cb(lv_event_t *e)
{
    int pin = (int)(intptr_t)lv_event_get_user_data(e);
    io_aout_apply(pin, atoi(lv_textarea_get_text(lv_event_get_target(e))), false);
}
static void io_step_cb(lv_event_t *e)
{
    intptr_t p = (intptr_t)lv_event_get_user_data(e);
    int pin = (int)(p >> 1);
    io_aout_apply(pin, bsp_aio_get_aout(pin) + ((p & 1) ? 5 : -5), true);
}

/* A muted one-line explanation under a setting (sits tight to its control). */
static void io_cap(lv_obj_t *b, const char *txt)
{
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_width(l, LV_PCT(100));
    lv_obj_set_style_text_color(l, COL_MUTE, 0);
    lv_obj_set_style_text_opa(l, LV_OPA_70, 0);
}

/* A tight setting block: control + caption with a small gap, so a setting reads as one
 * unit and the body's larger gap separates one setting from the next. */
static lv_obj_t *io_setting(lv_obj_t *b)
{
    lv_obj_t *c = lv_obj_create(b);
    lv_obj_remove_style_all(c);
    lv_obj_set_width(c, LV_PCT(100));
    lv_obj_set_height(c, LV_SIZE_CONTENT);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(c, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(c, 2, 0);
    return c;
}

/* A labelled dropdown row: "LABEL ....... [dropdown]". Returns the dropdown. */
static lv_obj_t *io_dd_row(lv_obj_t *b, const char *label, const char *opts, int sel,
                           lv_event_cb_t cb, int pin)
{
    lv_obj_t *row = kit_row(b);
    lv_obj_t *l = lv_label_create(row);
    lv_label_set_text(l, label);
    lv_obj_set_style_text_color(l, COL_AMBER, 0);
    lv_obj_t *dd = lv_dropdown_create(row);
    lv_dropdown_set_options(dd, opts);
    lv_dropdown_set_selected(dd, sel);
    lv_obj_set_width(dd, 250);
    style_field(dd);
    lv_obj_add_event_cb(dd, cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)pin);
    return dd;
}

/* Setting = control + caption, grouped tightly. */
static lv_obj_t *io_set_dd(lv_obj_t *b, const char *label, const char *opts, int sel,
                           lv_event_cb_t cb, int pin, const char *cap)
{
    lv_obj_t *c = io_setting(b);
    lv_obj_t *dd = io_dd_row(c, label, opts, sel, cb, pin);
    io_cap(c, cap);
    return dd;
}
static lv_obj_t *io_set_sw(lv_obj_t *b, const char *label, bool on, lv_event_cb_t cb,
                           int pin, const char *cap)
{
    lv_obj_t *c = io_setting(b);
    lv_obj_t *sw = kit_switch_row(c, label, on, cb, (void *)(intptr_t)pin);
    io_cap(c, cap);
    return sw;
}
static lv_obj_t *io_set_info(lv_obj_t *b, const char *key, const char *val, const char *cap)
{
    lv_obj_t *c = io_setting(b);
    lv_obj_t *v = kit_info_row(c, key, val);
    io_cap(c, cap);
    return v;
}

/* Overview: every pin as a box on one page. Number inside; border = direction
 * (thin/muted input, thick/amber output); digital boxes light amber when HIGH;
 * analog boxes fill from the bottom in proportion to their value. Tap to configure. */
static lv_obj_t *build_io_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "I/O BENCH", back_to_menu_cb);

    lv_obj_t *grid = lv_obj_create(p);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, IO_COLS * IO_BOX_W + (IO_COLS - 1) * IO_GAP,
                    3 * IO_BOX_H + 2 * IO_GAP);
    lv_obj_align(grid, LV_ALIGN_TOP_MID, 0, 76);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_column(grid, IO_GAP, 0);
    lv_obj_set_style_pad_row(grid, IO_GAP, 0);

    int count = bsp_aio_count();
    for (int i = 0; i < count && i < IO_MAX; i++) {
        const aio_pin_t *pin = bsp_aio_info(i);

        lv_obj_t *box = lv_obj_create(grid);
        lv_obj_set_size(box, IO_BOX_W, IO_BOX_H);
        lv_obj_set_style_radius(box, 0, 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(box, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(box, io_box_tap_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        s_io_box[i] = box;

        lv_obj_t *fill = lv_obj_create(box);   /* analog fill, behind the text */
        lv_obj_remove_style_all(fill);
        lv_obj_set_width(fill, LV_PCT(100));
        lv_obj_set_height(fill, 0);
        lv_obj_align(fill, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_opa(fill, LV_OPA_COVER, 0);
        s_io_fill[i] = fill;

        lv_obj_t *num = lv_label_create(box);
        lv_label_set_text(num, pin->label + 2);   /* strip "IO" prefix */
        lv_obj_set_style_text_font(num, FONT_STATUS, 0);
        lv_obj_align(num, LV_ALIGN_CENTER, 0, -10);
        s_io_val[i] = num;

        lv_obj_t *sub = lv_label_create(box);
        lv_label_set_text(sub, "");
        lv_obj_align(sub, LV_ALIGN_BOTTOM_MID, 0, -8);
        s_io_sub[i] = sub;

        s_io_last[i] = io_box_key(i);
        io_box_paint(i);
    }
    return p;
}

/* Single-pin config page: the full GPIO option set for one pin + its live readout. */
static lv_obj_t *build_io_pin_panel(lv_obj_t *parent)
{
    int i = s_io_pin;
    const aio_pin_t *pin = bsp_aio_info(i);
    aio_mode_t m = bsp_aio_get_mode(i);

    lv_obj_t *p = make_panel(parent, pin->label, back_to_io_cb);
    lv_obj_t *b = kit_body(p);
    /* Tight setting-to-setting gap so every config page fits without scrolling
     * (each setting groups its own control + caption with a 2px gap; see io_setting). */
    lv_obj_set_style_pad_row(b, 8, 0);

    if (s_io_err[0]) {
        lv_obj_t *err = lv_label_create(b);
        lv_label_set_text(err, s_io_err);
        lv_obj_set_style_text_color(err, COL_ALERT, 0);
        s_io_err[0] = '\0';
    }

    /* Build a capability-filtered mode option list. */
    char opts[96]; size_t off = 0; opts[0] = '\0';
    for (int mm = 0; mm < AIO_MODE_COUNT; mm++) {
        if (mm == AIO_ANALOG_IN && !pin->adc_ok) continue;
        off += snprintf(opts + off, sizeof(opts) - off, "%s%s", off ? "\n" : "", io_mode_name((aio_mode_t)mm));
    }
    io_set_dd(b, "MODE", opts, io_sel_of(i, m), io_mode_cb, i,
              pin->adc_ok ? "How this pin behaves (ADC-capable: analog in available)."
                          : "How this pin behaves: read it or drive it.");

    if (m == AIO_DIGITAL_IN) {
        io_set_dd(b, "PULL", "NONE\nPULL-UP\nPULL-DOWN", bsp_aio_get_pull(i), io_pull_cb, i,
                  "Internal resistor that sets the level when nothing drives the pin.");
        io_set_dd(b, "INTERRUPT", "OFF\nRISING\nFALLING\nANY EDGE", bsp_aio_get_irq(i), io_irq_cb, i,
                  "Count signal edges on the pin in the background.");
        if (bsp_aio_get_irq(i) != AIO_IRQ_OFF) {
            char ec[12]; snprintf(ec, sizeof(ec), "%lu", (unsigned long)bsp_aio_get_edges(i));
            s_pin_edges = kit_info_row(b, "EDGES", ec);
            kit_list_row(b, "RESET EDGE COUNT", io_edges_reset_cb, (void *)(intptr_t)i);
        }
        s_pin_read = io_set_info(b, "LEVEL", bsp_aio_read_level(i) > 0 ? "HIGH" : "LOW",
                                 "Live pin reading.");

    } else if (m == AIO_DIGITAL_OUT) {
        io_set_sw(b, "OPEN-DRAIN", bsp_aio_get_od(i), io_od_cb, i,
                  "On: only pulls LOW, releases HIGH (for shared/I2C-style lines).");
        io_set_dd(b, "STRENGTH", "WEAK ~5mA\nMEDIUM ~10mA\nDEFAULT ~20mA\nSTRONG ~40mA",
                  bsp_aio_get_drive(i), io_drive_cb, i, "How hard the pin drives - its max output current.");
        io_set_dd(b, "PULL", "NONE\nPULL-UP\nPULL-DOWN", bsp_aio_get_pull(i), io_pull_cb, i,
                  "Internal resistor; in open-drain it can act as the bus pull-up.");
        io_set_sw(b, "OUTPUT", bsp_aio_get_dout(i), io_dout_cb, i,
                  "The level to drive: on = HIGH, off = LOW.");
        s_pin_read = io_set_info(b, "LEVEL", bsp_aio_read_level(i) > 0 ? "HIGH" : "LOW",
                                 "Actual line, read back from the pin.");

    } else if (m == AIO_ANALOG_IN) {
        io_set_dd(b, "RANGE", "0 dB ~1.1V\n2.5 dB ~1.5V\n6 dB ~2.2V\n12 dB ~3.3V",
                  bsp_aio_get_atten(i), io_atten_cb, i, "Full-scale input voltage (ADC attenuation).");
        io_set_dd(b, "FILTER", "OFF\nLIGHT\nMEDIUM\nHEAVY", bsp_aio_get_filter(i), io_filter_cb, i,
                  "Smooths a noisy signal (IIR low-pass) - more = slower, steadier.");
        lv_obj_t *meter = io_setting(b);
        s_pin_read = kit_meter_row(meter, "INPUT", &s_pin_fill);
        int raw = 0, pct = 0;
        if (bsp_aio_read_ain(i, &raw, &pct) == ESP_OK) {
            char v[28]; snprintf(v, sizeof(v), "%d  %d%%  %.2fV",
                                 raw, pct, (double)(bsp_aio_ain_vmax(i) * raw / 4095.0f));
            lv_label_set_text(s_pin_read, v);
            kit_set_meter(s_pin_fill, pct, COL_AMBER);
        }
        io_cap(meter, "Live reading: raw count, % of range, approx volts.");

    } else {   /* AIO_ANALOG_OUT */
        /* Number keyboard for the duty entry, hidden until the field is focused. */
        s_keyboard = lv_keyboard_create(p);
        lv_keyboard_set_mode(s_keyboard, LV_KEYBOARD_MODE_NUMBER);
        lv_obj_set_size(s_keyboard, SCAN_W, 300);
        lv_obj_align(s_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
        style_keyboard(s_keyboard);
        lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(s_keyboard, kb_done_cb, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(s_keyboard, kb_done_cb, LV_EVENT_CANCEL, NULL);

        /* PWM frequency - one preset shared by every analog-out pin. */
        char fopts[64]; size_t fo = 0; fopts[0] = '\0';
        for (int k = 0; k < AIO_FREQ_COUNT; k++) {
            uint32_t hz = bsp_aio_freq_hz(k);
            if (hz >= 1000) fo += snprintf(fopts + fo, sizeof(fopts) - fo, "%s%u kHz", fo ? "\n" : "", (unsigned)(hz / 1000));
            else            fo += snprintf(fopts + fo, sizeof(fopts) - fo, "%s%u Hz",  fo ? "\n" : "", (unsigned)hz);
        }
        io_set_dd(b, "PWM FREQ", fopts, bsp_aio_get_freq(), io_freq_cb, i,
                  "Switching rate - shared by all analog-out pins.");
        io_set_sw(b, "SHOW VOLTS", bsp_aio_get_volts_pref(i), io_volts_cb, i,
                  "Display the level as nominal volts (duty x 3.3V) instead of %.");

        lv_obj_t *outset = io_setting(b);
        char rd[24]; io_aout_text(i, rd, sizeof(rd));
        s_pin_aout = kit_info_row(outset, "OUTPUT", rd);
        io_cap(outset, "Duty cycle - slider, type a value, or step +/-5%.");

        s_pin_slider = lv_slider_create(b);
        lv_obj_set_width(s_pin_slider, LV_PCT(100));
        lv_slider_set_range(s_pin_slider, 0, 100);
        lv_slider_set_value(s_pin_slider, bsp_aio_get_aout(i), LV_ANIM_OFF);
        kit_style_slider(s_pin_slider);
        lv_obj_add_event_cb(s_pin_slider, io_slider_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);

        lv_obj_t *ctl = kit_row(b);
        io_btn(ctl, LV_SYMBOL_MINUS, 70, io_step_cb, (void *)(intptr_t)(i << 1));
        lv_obj_t *ta = lv_textarea_create(ctl);
        lv_textarea_set_one_line(ta, true);
        lv_textarea_set_accepted_chars(ta, "0123456789");
        lv_textarea_set_max_length(ta, 3);
        char db[8]; snprintf(db, sizeof(db), "%d", bsp_aio_get_aout(i));
        lv_textarea_set_text(ta, db);
        lv_obj_set_width(ta, 140);
        style_field(ta);
        lv_obj_add_event_cb(ta, ta_focus_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta, io_entry_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);
        s_pin_entry = ta;
        io_btn(ctl, LV_SYMBOL_PLUS, 70, io_step_cb, (void *)(intptr_t)((i << 1) | 1));
    }
    return p;
}

static lv_obj_t *build_menu_panel(lv_obj_t *parent)
{
    /* SETUP holds CONFIGURATION only; instruments (VITALS/SCAN/SPECTRUM) live on
     * the console rail. BACK returns to the console. */
    lv_obj_t *m = make_panel(parent, "SETUP", close_setup_cb);
    lv_obj_t *b = kit_body(m);
    kit_list_row(b, "WI-FI",   menu_open_cb, (void *)(intptr_t)PK_WIFI);
    kit_list_row(b, "DISPLAY", menu_open_cb, (void *)(intptr_t)PK_DISPLAY);
    kit_list_row(b, "AUDIO",   menu_open_cb, (void *)(intptr_t)PK_AUDIO);
    kit_list_row(b, "LEDS",    menu_open_cb, (void *)(intptr_t)PK_LEDS);
    kit_list_row(b, "I/O BENCH", menu_open_cb, (void *)(intptr_t)PK_IO);
    kit_list_row(b, "ABOUT",   menu_open_cb, (void *)(intptr_t)PK_ABOUT);
    return m;
}

/* INSTRUMENTS group: the unit's own readouts. Rows open the full-screen instruments
 * (which BACK to here). Keeps the rail uncluttered - see the SENSORS twin below. */
static lv_obj_t *build_instruments_panel(lv_obj_t *parent)
{
    lv_obj_t *m = make_panel(parent, "INSTRUMENTS", back_to_home_cb);
    lv_obj_t *b = kit_body(m);
    kit_list_row(b, "SCANNER",     menu_open_cb, (void *)(intptr_t)PK_NONE);
    kit_list_row(b, "SIGNAL SCAN", menu_open_cb, (void *)(intptr_t)PK_SCAN);
    kit_list_row(b, "SPECTRUM",    menu_open_cb, (void *)(intptr_t)PK_SPECTRUM);
    kit_list_row(b, "VITALS",      menu_open_cb, (void *)(intptr_t)PK_VITALS);
    return m;
}

/* SENSORS group: the C6 radio-environment instruments (RF BAND / CONTACTS / SIGNAL ENV). */
static lv_obj_t *build_sensors_panel(lv_obj_t *parent)
{
    lv_obj_t *m = make_panel(parent, "SENSORS", back_to_home_cb);
    lv_obj_t *b = kit_body(m);
    kit_list_row(b, "RF BAND",    menu_open_cb, (void *)(intptr_t)PK_RFBAND);
    kit_list_row(b, "CONTACTS",   menu_open_cb, (void *)(intptr_t)PK_BLE);
    kit_list_row(b, "SIGNAL ENV", menu_open_cb, (void *)(intptr_t)PK_CSI);
    kit_list_row(b, "CSI CONFIG", menu_open_cb, (void *)(intptr_t)PK_CSICFG);
    kit_list_row(b, "SCANNER", menu_open_cb, (void *)(intptr_t)PK_MOTION);
    kit_list_row(b, "MINIMAP", menu_open_cb, (void *)(intptr_t)PK_MINIMAP);
    kit_list_row(b, "RANGE",   menu_open_cb, (void *)(intptr_t)PK_RANGE);
    return m;
}

/* ---- MOTION SCAN panel — Alien-style pie-slice radar + gimbals + sensors ----
 * Left 440x440: pie-slice (fan) with oscillating sweep, range arcs, target blips
 * within the slice, a 4-quadrant travel-direction ring at the base apex, plus
 * boolean status words and analog sensor bars (all-sensor overlay).
 * Right: target count, three orientation gimbal dials (YAW/PITCH/ROLL),
 * T1/T2/T3 target data, and the 3 strongest BLE contacts. */

/* 3-letter status word: bright amber when active, dim when not (reference style). */
static void set_flag_word(lv_obj_t *lbl, bool active)
{
    if (!lbl) return;
    lv_obj_set_style_text_color(lbl, active ? COL_AMBER : COL_DIM, 0);
}

static inline int clamp_pct(int v)
{
    if (v < 0)   return 0;
    if (v > 100) return 100;
    return v;
}

/* Forward decl — defined later in this file, used by motion_layout_grid below. */
static void label_set_text_cached(lv_obj_t *label, const char *txt);

/* Smallest ladder level keeping the furthest target at <=80% radius; 6 m when idle. */
static int motion_desired_level(float furthest_m, int have_target)
{
    if (!have_target) return MOTION_NLEVEL - 1;
    for (int i = 0; i < MOTION_NLEVEL; i++)
        if (furthest_m <= 0.80f * MOTION_LADDER[i]) return i;
    return MOTION_NLEVEL - 1;
}

/* Map an LD2450 target (mm, sign-agnostic) to a fan screen point at the current
 * scale. Shared by the blips and their comet trails. */
static void motion_target_screen_pos(int x_mm, int y_mm, int *bx, int *by)
{
    const float maxoff = (float)FAN_HALF_DEG * (3.14159265f / 180.0f);
    float fx = (float)x_mm, fy = (float)y_mm;
    float dist = sqrtf(fx * fx + fy * fy);
    float r = (dist / 1000.0f) * s_motion_ppm;
    if (r > (float)FAN_R) r = (float)FAN_R;
    float off = atan2f(fx, fabsf(fy) + 1.0f);
    if (off >  maxoff) off =  maxoff;
    if (off < -maxoff) off = -maxoff;
    float scr = (270.0f * (3.14159265f / 180.0f)) + off;
    *bx = (int)(FAN_APEX_X + cosf(scr) * r);
    *by = (int)(FAN_APEX_Y + sinf(scr) * r);
}

/* Lay out range rings + labels from s_motion_ppm with an adaptive "nice" major
 * step, so 3-5 labelled rings are always on screen — even zoomed in close. Rings
 * slide as ppm animates; only the arc fades near the rim, the label stays legible. */
static void motion_layout_grid(void)
{
    const float d2r = 3.14159265f / 180.0f;
    const float re  = FAN_RIGHT_DEG * d2r;            /* right fan edge for labels */
    float scale_m = (float)FAN_R / s_motion_ppm;      /* full-scale distance at the rim */

    float step = 2.0f;                                /* metres between major rings */
    if      (scale_m <= 1.25f) step = 0.25f;
    else if (scale_m <= 2.5f)  step = 0.5f;
    else if (scale_m <= 5.0f)  step = 1.0f;

    for (int i = 0; i < 6; i++) {
        lv_obj_t *arc = s_motion_grid_arc[i];
        lv_obj_t *lbl = s_motion_grid_lbl[i];
        if (!arc) break;
        float d = step * (i + 1);                     /* this ring's distance (m) */
        int r = (int)(d * s_motion_ppm + 0.5f);
        if (r < 8 || r > FAN_R) {
            lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        /* No rim fade: the outermost in-scale ring sits exactly at FAN_R and must
         * stay solid so it matches its legend label. Rings clip crisply at the rim. */
        lv_obj_set_size(arc, r * 2, r * 2);
        lv_obj_set_pos(arc, FAN_APEX_X - r, FAN_APEX_Y - r);
        lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_HIDDEN);

        /* Clean metre label: "0.25m" / "0.5m" / "1m" / "1.5m" / "2m". */
        char tb[24];
        int mm = (int)(d * 1000.0f + 0.5f);
        if      (mm % 1000 == 0) snprintf(tb, sizeof(tb), "%dm", mm / 1000);
        else if (mm % 100  == 0) snprintf(tb, sizeof(tb), "%d.%dm", mm / 1000, (mm % 1000) / 100);
        else                     snprintf(tb, sizeof(tb), "%d.%02dm", mm / 1000, (mm % 1000) / 10);
        label_set_text_cached(lbl, tb);
        lv_obj_set_pos(lbl, (int)(FAN_APEX_X + cosf(re) * r) + 4,
                            (int)(FAN_APEX_Y + sinf(re) * r) - 10);
        lv_obj_clear_flag(lbl, LV_OBJ_FLAG_HIDDEN);   /* label stays full-opacity (legend) */
    }

    /* Minor rings are superseded by the adaptive major step — keep them hidden. */
    for (int i = 0; i < 2; i++)
        if (s_motion_grid_minor[i]) lv_obj_add_flag(s_motion_grid_minor[i], LV_OBJ_FLAG_HIDDEN);
}

/* Toggle the SCANNER distance-ping audio feedback; persist across reboots. */
static void scan_spk_toggle_cb(lv_event_t *e)
{
    (void)e;
    s_scan_ping_en = !s_scan_ping_en;
    prop_settings_set_u32("scan_ping", s_scan_ping_en ? 1 : 0);
    s_scan_ping_next = 0;
    if (s_scan_spk) {
        lv_label_set_text(s_scan_spk, s_scan_ping_en ? LV_SYMBOL_VOLUME_MAX : LV_SYMBOL_MUTE);
        lv_obj_set_style_text_color(s_scan_spk, s_scan_ping_en ? COL_AMBER : COL_DIM, 0);
    }
    prop_audio_play(PA_BUTTON);
}

/* Tapping the operator dot at the fan apex opens the travel-direction calibration. */
static void dir_dot_cb(lv_event_t *e)
{
    (void)e;
    prop_audio_play(PA_OPEN);
    open_panel(PK_DIRCAL);
}

/* Zero a canvas-local rect of the A8 pulse buffer (clamped to the canvas). */
static void pulse_clear_rect(const lv_area_t *a)
{
    if (!s_pulse_buf) { return; }
    int x0 = a->x1 < 0 ? 0 : a->x1;
    int y0 = a->y1 < 0 ? 0 : a->y1;
    int x1 = a->x2 >= RADAR_W ? RADAR_W - 1 : a->x2;
    int y1 = a->y2 >= RADAR_H ? RADAR_H - 1 : a->y2;
    for (int y = y0; y <= y1; y++) {
        memset(s_pulse_buf + (size_t)y * RADAR_W + x0, 0, (size_t)(x1 - x0 + 1) * 4);
    }
}

/* Paint the wavy Halo band into the ARGB8888 buffer (canvas-local coords). Only pixels
 * inside the fan sector and the [r_in, r_out] annulus are written; the bbox must have
 * been cleared first. The look: a per-pixel radial ramp (bright leading edge -> 0 trail)
 * quantised into PULSE_WAVE_STEPS terraces, with the outer edge scalloped by a sin LUT —
 * this is the geometric "wave" the arc stack only hinted at. Brightness is capped at
 * PULSE_PEAK_OPA so the sweep never out-brights a target blip. */
static void pulse_paint_band(const lv_area_t *bbox, int r_out, int r_in, int phase)
{
    if (!s_pulse_buf || r_out <= r_in) { return; }
    int x0 = bbox->x1 < 0 ? 0 : bbox->x1;
    int y0 = bbox->y1 < 0 ? 0 : bbox->y1;
    int x1 = bbox->x2 >= RADAR_W ? RADAR_W - 1 : bbox->x2;
    int y1 = bbox->y2 >= RADAR_H ? RADAR_H - 1 : bbox->y2;
    const float inv_span = 1.0f / (float)(r_out - r_in);
    const float tan_half = 1.7320508f;          /* tan(60 deg) — fan half-angle */
    for (int y = y0; y <= y1; y++) {
        int dy = y - FAN_APEX_Y;
        if (dy >= 0) { continue; }              /* band is strictly above the apex */
        int neg_dy = -dy;
        uint32_t *row = s_pulse_buf + (size_t)y * RADAR_W;
        for (int x = x0; x <= x1; x++) {
            int dx = x - FAN_APEX_X;
            if (dx > tan_half * neg_dy || -dx > tan_half * neg_dy) { continue; }  /* fan gate */
            float r = sqrtf((float)(dx * dx + dy * dy));
            if (r < (float)r_in) { continue; }
            /* scallop the outer edge: index the sin LUT by horizontal slope + phase */
            float u = (float)dx / (float)neg_dy;                 /* -1.73..1.73 in-fan */
            int li = ((int)(u * PULSE_WAVE_FREQ) + phase) & 255;
            float r_out_eff = (float)r_out + PULSE_WAVE_AMP * s_pulse_sin[li];
            if (r > r_out_eff) { continue; }
            float t = (r - (float)r_in) * inv_span;              /* 0 trail .. 1 lead */
            int step = (int)(t * PULSE_WAVE_STEPS);
            if (step >= PULSE_WAVE_STEPS) { step = PULSE_WAVE_STEPS - 1; }
            int a = PULSE_PEAK_OPA * step / (PULSE_WAVE_STEPS - 1);
            if (r > (float)(FAN_R - 16)) { a = (int)(a * (FAN_R - r) / 16.0f); }  /* rim fade */
            if (a <= 0) { continue; }
            if (a > PULSE_PEAK_OPA) { a = PULSE_PEAK_OPA; }
            row[x] = ((uint32_t)a << 24) | PULSE_AMBER_RGB;
        }
    }
}

static lv_obj_t *build_motion_panel(lv_obj_t *parent)
{
    /* SCANNER: full-screen page — no title header, no BACK button (the nav rail is
     * the only chrome). A bare bordered container fills the whole content area. */
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_size(p, SCAN_W, 600);
    lv_obj_align(p, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(p, COL_BG, 0);
    lv_obj_set_style_border_color(p, COL_AMBER, 0);
    lv_obj_set_style_border_width(p, 2, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);

    /* ---- Left: radar display box (fills panel height) ---- */
    lv_obj_t *rbox = lv_obj_create(p);
    lv_obj_set_size(rbox, RADAR_W, RADAR_H);
    lv_obj_align(rbox, LV_ALIGN_TOP_LEFT, RADAR_X, RADAR_Y);
    lv_obj_set_style_bg_color(rbox, COL_PANEL_ITEM, 0);
    lv_obj_set_style_bg_opa(rbox, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(rbox, COL_DIM, 0);
    lv_obj_set_style_border_width(rbox, 1, 0);
    lv_obj_set_style_radius(rbox, 0, 0);
    lv_obj_set_style_pad_all(rbox, 0, 0);
    lv_obj_clear_flag(rbox, LV_OBJ_FLAG_SCROLLABLE);

    /* Speaker toggle (top-right corner, above the wedge): enables the distance-ping
     * chirp for the closest radar target. */
    s_scan_spk = lv_label_create(rbox);
    lv_label_set_text(s_scan_spk, s_scan_ping_en ? LV_SYMBOL_VOLUME_MAX : LV_SYMBOL_MUTE);
    lv_obj_set_style_text_font(s_scan_spk, FONT_HEAD, 0);
    lv_obj_set_style_text_color(s_scan_spk, s_scan_ping_en ? COL_AMBER : COL_DIM, 0);
    lv_obj_align(s_scan_spk, LV_ALIGN_TOP_RIGHT, -12, 10);
    lv_obj_add_flag(s_scan_spk, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_scan_spk, 18);
    lv_obj_add_event_cb(s_scan_spk, scan_spk_toggle_cb, LV_EVENT_CLICKED, NULL);

    /* Major range-ring pool (1..6 m). Positioned each tick from s_motion_ppm so the
     * rings slide as the scale animates; created hidden. */
    for (int i = 0; i < 6; i++) {
        lv_obj_t *arc = lv_arc_create(rbox);
        lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
        lv_arc_set_rotation(arc, 0);
        lv_arc_set_bg_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_arc_set_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_obj_set_style_arc_width(arc, 1, LV_PART_MAIN);
        lv_obj_set_style_arc_color(arc, COL_DIM, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
        lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
        s_motion_grid_arc[i] = arc;

        lv_obj_t *lbl = lv_label_create(rbox);
        lv_label_set_text(lbl, "");
        lv_obj_set_style_text_font(lbl, FONT_BODY, 0);
        lv_obj_set_style_text_color(lbl, COL_MUTE, 0);   /* camera-legible (style kit) */
        lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
        s_motion_grid_lbl[i] = lbl;
    }
    /* Minor half-metre rings (close-range detail); created hidden, even fainter. */
    for (int i = 0; i < 2; i++) {
        lv_obj_t *arc = lv_arc_create(rbox);
        lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
        lv_arc_set_rotation(arc, 0);
        lv_arc_set_bg_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_arc_set_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_obj_set_style_arc_width(arc, 1, LV_PART_MAIN);
        lv_obj_set_style_arc_color(arc, COL_DIM, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(arc, LV_OPA_40, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
        lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
        s_motion_grid_minor[i] = arc;
    }
    s_motion_grav_primed = false;                 /* reseed gravity low-pass on (re)open */
    s_motion_level = MOTION_NLEVEL - 1;            /* boot at full 6 m, zoom in as targets appear */
    s_motion_level_pend = s_motion_level;
    s_motion_dwell = 0;
    s_motion_ppm = (float)FAN_R / MOTION_LADDER[s_motion_level];
    s_motion_ppm_to = s_motion_ppm;
    s_motion_ppm_from = s_motion_ppm;
    s_motion_anim_ms = -1;

    /* Fan edge lines + boresight (static — never updated). */
    static lv_point_precise_t s_fan_left[2], s_fan_right[2], s_fan_bore[2];
    {
        const float d2r = 3.14159265f / 180.0f;
        float la = FAN_LEFT_DEG * d2r, ra = FAN_RIGHT_DEG * d2r;
        s_fan_left[0].x = FAN_APEX_X; s_fan_left[0].y = FAN_APEX_Y;
        s_fan_left[1].x = (lv_value_precise_t)(FAN_APEX_X + cosf(la) * FAN_R);
        s_fan_left[1].y = (lv_value_precise_t)(FAN_APEX_Y + sinf(la) * FAN_R);
        s_fan_right[0].x = FAN_APEX_X; s_fan_right[0].y = FAN_APEX_Y;
        s_fan_right[1].x = (lv_value_precise_t)(FAN_APEX_X + cosf(ra) * FAN_R);
        s_fan_right[1].y = (lv_value_precise_t)(FAN_APEX_Y + sinf(ra) * FAN_R);
        s_fan_bore[0].x = FAN_APEX_X; s_fan_bore[0].y = FAN_APEX_Y;
        s_fan_bore[1].x = FAN_APEX_X; s_fan_bore[1].y = FAN_APEX_Y - FAN_R;
        lv_point_precise_t *segs[3] = { s_fan_left, s_fan_right, s_fan_bore };
        for (int i = 0; i < 3; i++) {
            lv_obj_t *ln = lv_line_create(rbox);
            lv_line_set_points(ln, segs[i], 2);
            lv_obj_set_style_line_color(ln, COL_DIM, 0);
            lv_obj_set_style_line_width(ln, 1, 0);
            lv_obj_align(ln, LV_ALIGN_TOP_LEFT, 0, 0);
        }
    }

    /* (Boresight quarter-metre ticks removed — grid now drawn dynamically from
     * s_motion_ppm in motion_layout_grid; minor rings added in a later task.) */

    /* (Fixed "1m"/"2m" legend removed — labels now ride the rings in motion_layout_grid.) */

    /* Halo pulse — a hand-painted ARGB8888 canvas (see pulse_paint_band). The observer
     * advances the lead radius, repaints only the tight band bbox, and invalidates only
     * that bbox, so dirty area tracks band thickness (fixed) not the expanding-circle
     * bbox. Per-pixel alpha gives the wavy/terraced look and costs zero AA-mask allocs. */
    for (int i = 0; i < 256; i++) {
        s_pulse_sin[i] = sinf((float)i * (2.0f * 3.14159265f / 256.0f));
    }
    size_t pulse_sz = LV_CANVAS_BUF_SIZE(RADAR_W, RADAR_H, 32, LV_DRAW_BUF_STRIDE_ALIGN);
    s_pulse_buf = heap_caps_malloc(pulse_sz, MALLOC_CAP_SPIRAM);
    if (s_pulse_buf) {
        memset(s_pulse_buf, 0, pulse_sz);
        s_motion_pulse = lv_canvas_create(rbox);
        lv_canvas_set_buffer(s_motion_pulse, s_pulse_buf, RADAR_W, RADAR_H, LV_COLOR_FORMAT_ARGB8888);
        lv_obj_set_pos(s_motion_pulse, 0, 0);
        lv_obj_clear_flag(s_motion_pulse, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    }
    s_pulse_lead_m     = 0.0f;
    s_pulse_lead_px    = 0;
    s_pulse_phase      = 0;
    s_pulse_prev_valid = false;

    /* Comet-tail dot pool (per target), created hidden — drawn behind the blips. */
    for (int i = 0; i < 3; i++) {
        s_trail_n[i] = 0; s_trail_head[i] = 0;
        for (int k = 0; k < TRAIL_LEN; k++) {
            lv_obj_t *d = lv_obj_create(rbox);
            lv_obj_remove_style_all(d);
            lv_obj_set_size(d, 4, 4);
            lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(d, COL_AMBER, 0);
            lv_obj_set_style_bg_opa(d, LV_OPA_TRANSP, 0);
            lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
            s_motion_trail[i][k] = d;
        }
    }
    s_trail_tick = 0;

    /* 3 target blips (amber circles, initially hidden) */
    for (int i = 0; i < 3; i++) {
        s_motion_blips[i] = lv_obj_create(rbox);
        lv_obj_remove_style_all(s_motion_blips[i]);
        lv_obj_set_size(s_motion_blips[i], 12, 12);
        lv_obj_set_style_radius(s_motion_blips[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s_motion_blips[i], COL_AMBER, 0);
        lv_obj_set_style_bg_opa(s_motion_blips[i], LV_OPA_COVER, 0);
        lv_obj_add_flag(s_motion_blips[i], LV_OBJ_FLAG_HIDDEN);
    }

    /* ---- Status above the wedge (the blank space at the top of the radar box) ---- */

    /* Merged headline — count folded into the alert: "NO TARGETS" (mute) when clear,
     * "N TARGET[S] DETECTED" (alert red) when the radar has contacts. Large font. */
    s_motion_tgt_label = lv_label_create(rbox);
    lv_label_set_text(s_motion_tgt_label, "NO TARGETS");
    lv_obj_set_style_text_color(s_motion_tgt_label, COL_MUTE, 0);
    lv_obj_set_style_text_font(s_motion_tgt_label, FONT_STATUS, 0);
    lv_obj_align(s_motion_tgt_label, LV_ALIGN_TOP_MID, 0, 24);

    /* ---- Right column ---- */

    /* Three horizon-style attitude indicators — YAW / PITCH / ROLL (all axes).
     * Each: a box with a fixed centre crosshair + an amber line driven by its
     * axis in the artificial-horizon style (observer updates the line). */
    static const char *const dial_name[3] = { "YAW", "PITCH", "ROLL" };
    /* Crosshair centred at box-local (74, 96) in the 148x170 cell. */
    static lv_point_precise_t s_hz_h[2] = {{8, 96}, {140, 96}};
    static lv_point_precise_t s_hz_v[2] = {{74, 24}, {74, 164}};
    for (int k = 0; k < 3; k++) {
        lv_obj_t *box = lv_obj_create(p);
        lv_obj_set_size(box, MR_BOX_W, MR_BOX_H);
        lv_obj_align(box, LV_ALIGN_TOP_LEFT, MR_X0 + k * MR_PITCH, MR_ROW0);
        lv_obj_set_style_bg_color(box, COL_PANEL_ITEM, 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(box, COL_DIM, 0);
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_radius(box, 0, 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *nm = lv_label_create(box);
        lv_label_set_text(nm, dial_name[k]);
        lv_obj_set_style_text_color(nm, COL_MUTE, 0);
        lv_obj_set_style_text_font(nm, FONT_BODY, 0);
        lv_obj_align(nm, LV_ALIGN_TOP_LEFT, 4, 3);

        lv_obj_t *hc = lv_line_create(box);
        lv_line_set_points(hc, s_hz_h, 2);
        lv_obj_set_style_line_color(hc, COL_DIM, 0);
        lv_obj_set_style_line_width(hc, 1, 0);
        lv_obj_align(hc, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_t *vc = lv_line_create(box);
        lv_line_set_points(vc, s_hz_v, 2);
        lv_obj_set_style_line_color(vc, COL_DIM, 0);
        lv_obj_set_style_line_width(vc, 1, 0);
        lv_obj_align(vc, LV_ALIGN_TOP_LEFT, 0, 0);

        /* moving indicator line (box-local pts; observer updates) */
        s_motion_gpts[k][0].x = 8;   s_motion_gpts[k][0].y = 96;
        s_motion_gpts[k][1].x = 140; s_motion_gpts[k][1].y = 96;
        s_motion_gdial[k] = lv_line_create(box);
        lv_line_set_points(s_motion_gdial[k], s_motion_gpts[k], 2);
        lv_obj_set_style_line_color(s_motion_gdial[k], COL_AMBER, 0);
        lv_obj_set_style_line_width(s_motion_gdial[k], 3, 0);
        lv_obj_align(s_motion_gdial[k], LV_ALIGN_TOP_LEFT, 0, 0);

        s_motion_gval[k] = lv_label_create(box);
        lv_label_set_text(s_motion_gval[k], "0\xc2\xb0");
        lv_obj_set_style_text_color(s_motion_gval[k], COL_AMBER, 0);
        lv_obj_set_style_text_font(s_motion_gval[k], FONT_HEAD, 0);
        lv_obj_align(s_motion_gval[k], LV_ALIGN_BOTTOM_RIGHT, -6, -4);
    }

    /* T1/T2/T3 target readout: distance (large) + velocity (bottom-left) +
     * bearing arrow in a ring (bottom-right) — velocity & ring no longer overlap. */
    static const char *const tn_hdr[3] = { "T1", "T2", "T3" };
    for (int i = 0; i < 3; i++) {
        lv_obj_t *box = lv_obj_create(p);
        lv_obj_set_size(box, MR_BOX_W, MR_BOX_H);
        lv_obj_align(box, LV_ALIGN_TOP_LEFT, MR_X0 + i * MR_PITCH, MR_ROW1);
        lv_obj_set_style_bg_color(box, COL_PANEL_ITEM, 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(box, COL_DIM, 0);
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_radius(box, 0, 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *hdr = lv_label_create(box);
        lv_label_set_text(hdr, tn_hdr[i]);
        lv_obj_set_style_text_color(hdr, COL_MUTE, 0);
        lv_obj_set_style_text_font(hdr, FONT_BODY, 0);
        lv_obj_align(hdr, LV_ALIGN_TOP_LEFT, 6, 4);

        s_motion_tdist[i] = lv_label_create(box);
        lv_label_set_text(s_motion_tdist[i], "--.-m");
        lv_obj_set_style_text_color(s_motion_tdist[i], COL_DIM, 0);
        lv_obj_set_style_text_font(s_motion_tdist[i], FONT_STATUS, 0);
        lv_obj_align(s_motion_tdist[i], LV_ALIGN_LEFT_MID, 6, -6);

        /* Velocity sits in the bottom-left, clear of the bearing ring on the right. */
        s_motion_tvel[i] = lv_label_create(box);
        lv_label_set_text(s_motion_tvel[i], "");
        lv_obj_set_style_text_color(s_motion_tvel[i], COL_DIM, 0);
        lv_obj_set_style_text_font(s_motion_tvel[i], FONT_BODY, 0);
        lv_obj_align(s_motion_tvel[i], LV_ALIGN_BOTTOM_LEFT, 6, -10);

        /* Compass ring around the bearing arrow — bottom-right corner. */
        lv_obj_t *ring = lv_obj_create(box);
        lv_obj_remove_style_all(ring);
        lv_obj_set_size(ring, 42, 42);
        lv_obj_set_pos(ring, 98, 116);   /* centre at box-local (119, 137) */
        lv_obj_set_style_border_color(ring, COL_DIM, 0);
        lv_obj_set_style_border_width(ring, 1, 0);
        lv_obj_set_style_border_opa(ring, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);

        /* Bearing arrow: initially neutral (straight up = forward), hidden */
        s_motion_apts[i][0].x = 119; s_motion_apts[i][0].y = 137;
        s_motion_apts[i][1].x = 119; s_motion_apts[i][1].y = 119;
        s_motion_tarrow[i] = lv_line_create(box);
        lv_line_set_points(s_motion_tarrow[i], s_motion_apts[i], 2);
        lv_obj_set_style_line_color(s_motion_tarrow[i], COL_AMBER, 0);
        lv_obj_set_style_line_width(s_motion_tarrow[i], 3, 0);
        lv_obj_align(s_motion_tarrow[i], LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_add_flag(s_motion_tarrow[i], LV_OBJ_FLAG_HIDDEN);
    }

    /* BLE section header rides just above the contact row. */
    lv_obj_t *blehdr = lv_label_create(p);
    lv_label_set_text(blehdr, "BLE CONTACTS");
    lv_obj_set_style_text_color(blehdr, COL_MUTE, 0);
    lv_obj_set_style_text_font(blehdr, FONT_BODY, 0);
    lv_obj_align(blehdr, LV_ALIGN_TOP_LEFT, MR_X0, MR_ROW2 - 22);

    /* Three strongest BLE contacts — same cell size as the target readout.
     * Each: contact name (top), distance in metres (large), dBm (corner). */
    for (int i = 0; i < 3; i++) {
        lv_obj_t *box = lv_obj_create(p);
        s_motion_ble_box[i] = box;
        lv_obj_set_size(box, MR_BOX_W, MR_BOX_H);
        lv_obj_align(box, LV_ALIGN_TOP_LEFT, MR_X0 + i * MR_PITCH, MR_ROW2);
        lv_obj_set_style_bg_color(box, COL_PANEL_ITEM, 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(box, COL_DIM, 0);
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_radius(box, 0, 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

        /* Contact name — full width, ellipsised if it overflows the box. */
        s_motion_ble_name[i] = lv_label_create(box);
        lv_label_set_long_mode(s_motion_ble_name[i], LV_LABEL_LONG_DOT);
        lv_obj_set_width(s_motion_ble_name[i], MR_BOX_W - 8);
        lv_label_set_text(s_motion_ble_name[i], "----");
        lv_obj_set_style_text_color(s_motion_ble_name[i], COL_DIM, 0);
        lv_obj_set_style_text_font(s_motion_ble_name[i], FONT_BODY, 0);
        lv_obj_align(s_motion_ble_name[i], LV_ALIGN_TOP_LEFT, 4, 4);

        s_motion_ble_dist[i] = lv_label_create(box);
        lv_label_set_text(s_motion_ble_dist[i], "-- m");
        lv_obj_set_style_text_color(s_motion_ble_dist[i], COL_DIM, 0);
        lv_obj_set_style_text_font(s_motion_ble_dist[i], FONT_STATUS, 0);
        lv_obj_align(s_motion_ble_dist[i], LV_ALIGN_LEFT_MID, 6, 0);

        /* RSSI in dBm — small, tucked in the bottom-right corner. */
        s_motion_ble_dbm[i] = lv_label_create(box);
        lv_label_set_text(s_motion_ble_dbm[i], "");
        lv_obj_set_style_text_color(s_motion_ble_dbm[i], COL_MUTE, 0);
        lv_obj_set_style_text_font(s_motion_ble_dbm[i], FONT_BODY, 0);
        lv_obj_align(s_motion_ble_dbm[i], LV_ALIGN_BOTTOM_RIGHT, -4, -4);
    }

    /* ---- All-sensor overlay inside the radar box (reference layout) ---- */

    /* Boolean status words, top-right corner (dim/bright, like "ATT/SUS/DEC"). */
    static const char *const flag_txt[MOTION_FLAG_COUNT] =
        { "MOV", "IMU", "MIC", "BLE", "NET" };
    for (int i = 0; i < MOTION_FLAG_COUNT; i++) {
        s_motion_flags[i] = lv_label_create(rbox);
        lv_label_set_text(s_motion_flags[i], flag_txt[i]);
        lv_obj_set_style_text_font(s_motion_flags[i], FONT_BODY, 0);
        lv_obj_set_style_text_color(s_motion_flags[i], COL_DIM, 0);
        lv_obj_align(s_motion_flags[i], LV_ALIGN_BOTTOM_RIGHT, -8, -8 - (MOTION_FLAG_COUNT - 1 - i) * 20);
    }

    /* Analog sensor bars, bottom-left — two columns, rows aligned, left-justified.
     * Layout: col A = CSI / MIC / ACC, col B = RF / SIG / FRQ (no dividers). FRQ is
     * the loudest audio frequency from the mic FFT — fills the 6th slot, balancing
     * the grid into a clean 3x2. */
    static const char *const bar_txt[MOTION_BAR_COUNT] = { "CSI", "RF", "MIC", "SIG", "ACC", "FRQ" };
    static const uint8_t bar_col[MOTION_BAR_COUNT] = { 0, 1, 0, 1, 0, 1 };
    static const uint8_t bar_row[MOTION_BAR_COUNT] = { 0, 0, 1, 1, 2, 2 };
    for (int i = 0; i < MOTION_BAR_COUNT; i++) {
        int lx = bar_col[i] ? 200 : 8;
        int ly = -10 - (2 - bar_row[i]) * 28;
        s_motion_bar_lbls[i] = lv_label_create(rbox);
        lv_label_set_text(s_motion_bar_lbls[i], bar_txt[i]);
        lv_obj_set_style_text_font(s_motion_bar_lbls[i], FONT_BODY, 0);
        lv_obj_set_style_text_color(s_motion_bar_lbls[i], COL_MUTE, 0);
        lv_obj_align(s_motion_bar_lbls[i], LV_ALIGN_BOTTOM_LEFT, lx, ly);

        s_motion_bars[i] = lv_bar_create(rbox);
        lv_obj_set_size(s_motion_bars[i], 86, 8);
        lv_obj_align_to(s_motion_bars[i], s_motion_bar_lbls[i], LV_ALIGN_OUT_RIGHT_MID, 8, 0);
        lv_bar_set_range(s_motion_bars[i], 0, 100);
        lv_obj_set_style_bg_color(s_motion_bars[i], COL_PANEL_ITEM, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_motion_bars[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_color(s_motion_bars[i], COL_DIM, LV_PART_MAIN);
        lv_obj_set_style_border_width(s_motion_bars[i], 1, LV_PART_MAIN);
        lv_obj_set_style_radius(s_motion_bars[i], 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_motion_bars[i], COL_AMBER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_motion_bars[i], 0, LV_PART_INDICATOR);
        lv_bar_set_value(s_motion_bars[i], 0, LV_ANIM_OFF);
    }

    /* 4-quadrant direction ring around the operator dot at the fan apex (N/E/S/W).
     * Lit quadrant shows the operator's direction of travel. */
    {
        const int cx = FAN_APEX_X, cy = FAN_APEX_Y, rr = 30;
        /* index 0=N(top) 1=E(right) 2=S(bottom) 3=W(left); 5° inset per side. */
        static const int q_start[4] = { 230, 320, 50, 140 };
        for (int i = 0; i < 4; i++) {
            s_motion_dir_q[i] = lv_arc_create(rbox);
            lv_obj_remove_style(s_motion_dir_q[i], NULL, LV_PART_KNOB);
            lv_obj_clear_flag(s_motion_dir_q[i], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_size(s_motion_dir_q[i], rr * 2, rr * 2);
            lv_obj_set_pos(s_motion_dir_q[i], cx - rr, cy - rr);
            lv_arc_set_rotation(s_motion_dir_q[i], 0);
            lv_arc_set_bg_angles(s_motion_dir_q[i], q_start[i], q_start[i] + 80);
            lv_arc_set_angles(s_motion_dir_q[i], q_start[i], q_start[i] + 80);
            lv_obj_set_style_arc_width(s_motion_dir_q[i], 8, LV_PART_MAIN);
            lv_obj_set_style_arc_color(s_motion_dir_q[i], COL_DIM, LV_PART_MAIN);
            lv_obj_set_style_arc_width(s_motion_dir_q[i], 8, LV_PART_INDICATOR);
            lv_obj_set_style_arc_color(s_motion_dir_q[i], COL_DIM, LV_PART_INDICATOR);
        }
        s_motion_dir_dot = lv_obj_create(rbox);
        lv_obj_remove_style_all(s_motion_dir_dot);
        lv_obj_set_size(s_motion_dir_dot, 12, 12);
        lv_obj_set_style_radius(s_motion_dir_dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s_motion_dir_dot, COL_AMBER, 0);
        lv_obj_set_style_bg_opa(s_motion_dir_dot, LV_OPA_COVER, 0);
        lv_obj_set_pos(s_motion_dir_dot, cx - 6, cy - 6);
        /* Tap target → travel-direction calibration. Generous hit area (small dot). */
        lv_obj_add_flag(s_motion_dir_dot, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_ext_click_area(s_motion_dir_dot, 22);
        lv_obj_add_event_cb(s_motion_dir_dot, dir_dot_cb, LV_EVENT_CLICKED, NULL);
    }

    return p;
}

/* ---- Travel-direction calibration (PK_DIRCAL) ----------------------------------
 * A guided four-step walk (forward / backward / left / right) that learns the
 * board->world yaw so the SCANNER travel ring's North lines up with the radar
 * boresight. "Forward" = the direction the radar pulses go. Built as a normal panel
 * (one-alive-at-a-time) rather than an overlay — no perf hit, frees on exit. */

static const char *const DCAL_NAME[4] =
    { "WALK FORWARD", "WALK BACKWARD", "WALK LEFT", "WALK RIGHT" };
#define DCAL_MIN_MAG 1.0f         /* reject a run with too little net motion */

static void dcal_reset_run(void)
{
    s_dcal_phase = -1;
    s_dcal_ms = 0;
    s_dcal_ref_set = false;
    s_dcal_grav_primed = false;
    memset(s_dcal_acc, 0, sizeof(s_dcal_acc));
}

/* Point the on-screen arrow the way the user must walk (forward = up = boresight). */
static void dcal_set_arrow(int phase)
{
    if (!s_dcal_arrow) return;
    const int cx = SCAN_W / 2, cy = 286, L = 64, h = 24;
    float dx = 0.0f, dy = 0.0f;
    switch (phase) {
        case 0: dy = -1.0f; break;   /* forward -> up */
        case 1: dy =  1.0f; break;   /* backward -> down */
        case 2: dx = -1.0f; break;   /* left */
        case 3: dx =  1.0f; break;   /* right */
        default: lv_obj_add_flag(s_dcal_arrow, LV_OBJ_FLAG_HIDDEN); return;
    }
    float px = -dy, py = dx;         /* perpendicular for the arrowhead wings */
    int tipx = cx + (int)(dx * L), tipy = cy + (int)(dy * L);
    int talx = cx - (int)(dx * L), taly = cy - (int)(dy * L);
    s_dcal_apts[0].x = talx;                            s_dcal_apts[0].y = taly;
    s_dcal_apts[1].x = tipx;                            s_dcal_apts[1].y = tipy;
    s_dcal_apts[2].x = tipx - (int)(dx*h) + (int)(px*h); s_dcal_apts[2].y = tipy - (int)(dy*h) + (int)(py*h);
    s_dcal_apts[3].x = tipx;                            s_dcal_apts[3].y = tipy;
    s_dcal_apts[4].x = tipx - (int)(dx*h) - (int)(px*h); s_dcal_apts[4].y = tipy - (int)(dy*h) - (int)(py*h);
    lv_line_set_points(s_dcal_arrow, s_dcal_apts, 5);
    lv_obj_clear_flag(s_dcal_arrow, LV_OBJ_FLAG_HIDDEN);
}

static void back_to_motion_cb(lv_event_t *e) { (void)e; open_panel(PK_MOTION); }

/* Action button: START / CANCEL (while running) / DONE / RETRY. */
static void dcal_action_cb(lv_event_t *e)
{
    (void)e;
    if (s_dcal_phase == 4) {            /* DONE — calibration already saved */
        prop_audio_play(PA_BACK);
        open_panel(PK_MOTION);
        return;
    }
    if (s_dcal_phase >= 0 && s_dcal_phase <= 3) {   /* CANCEL a run in progress */
        dcal_reset_run();
        prop_audio_play(PA_BACK);
        return;
    }
    dcal_reset_run();                  /* idle (-1) or retry (5) -> START */
    s_dcal_phase = 0;
    prop_audio_play(PA_OPEN);
}

static lv_obj_t *build_dircal_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "CALIBRATE DIRECTION", back_to_motion_cb);
    dcal_reset_run();

    s_dcal_prompt = lv_label_create(p);
    lv_label_set_text(s_dcal_prompt, "DIRECTION SETUP");
    lv_obj_set_style_text_color(s_dcal_prompt, COL_MUTE, 0);
    lv_obj_set_style_text_font(s_dcal_prompt, FONT_STATUS, 0);
    lv_obj_align(s_dcal_prompt, LV_ALIGN_TOP_MID, 0, 96);

    /* Direction arrow (line polyline; hidden until a walk phase is active). */
    s_dcal_arrow = lv_line_create(p);
    lv_obj_set_style_line_color(s_dcal_arrow, COL_AMBER, 0);
    lv_obj_set_style_line_width(s_dcal_arrow, 8, 0);
    lv_obj_set_style_line_rounded(s_dcal_arrow, true, 0);
    lv_obj_align(s_dcal_arrow, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(s_dcal_arrow, LV_OBJ_FLAG_HIDDEN);

    s_dcal_sub = lv_label_create(p);
    lv_label_set_text(s_dcal_sub, "PRESS START, THEN WALK AS PROMPTED");
    lv_obj_set_style_text_color(s_dcal_sub, COL_MUTE, 0);
    lv_obj_set_style_text_font(s_dcal_sub, FONT_HEAD, 0);
    lv_obj_align(s_dcal_sub, LV_ALIGN_TOP_MID, 0, 410);

    lv_obj_t *btn = make_btn(p, "START", 220, LV_ALIGN_BOTTOM_MID, 0, -40, dcal_action_cb);
    s_dcal_btn_lbl = lv_obj_get_child(btn, 0);

    return p;
}

/* Signal level is measured in quarter-cells. The gauge deliberately never tops
 * out (cassette-futurism: a perfect reading is suspicious) - SIG_QMAX is one
 * quarter short of completely full. */
#define SIG_CELL  16          /* px per square */
#define SIG_GAP   5           /* px between squares */
#define SIG_LBL_W 42          /* room for the "SIG" label before the cells */
#define SIG_INSET 3           /* gap between the outline and its fill */
#define SIG_QMAX  (SIG_CELLS * 4 - 1)

/* Map RSSI (dBm) to quarter-cells (1..SIG_QMAX). 0 dBm sentinel / no link -> 0. */
static int rssi_to_quarters(int rssi)
{
    if (rssi >= 0) {
        return 0;
    }
    int q = ((rssi + 90) * SIG_QMAX) / 40;   /* -90 dBm -> 0, -50 dBm -> max */
    if (q < 1)        q = 1;
    if (q > SIG_QMAX) q = SIG_QMAX;
    return q;
}

/* ===================== MINIMAP (PK_MINIMAP) ============================== *
 * North-up dead-reckoning map: a breadcrumb path from prop_track's PDR, plus
 * last-known radar marks. The static layer (grid + path + marks) is hand-painted
 * into an ARGB8888 canvas by direct pixel writes (the v9 canvas draw pipeline
 * deadlocks under the port lock the observer holds); the operator dot + heading
 * arrow are widgets glided over it each tick. See build_minimap_panel + the
 * PK_MINIMAP block in ui_observer. */

#define MAP_ARGB(a, rgb) (((uint32_t)(a) << 24) | (uint32_t)(rgb))

/* World (metres) → canvas-local pixels, north-up, from the committed fit. */
static inline void map_world_to_px(float wx, float wy, int *px, int *py)
{
    *px = (int)(MAP_W * 0.5f + (wx - s_map_ctr_x) * s_map_ppm + 0.5f);
    *py = (int)(MAP_H * 0.5f - (wy - s_map_ctr_y) * s_map_ppm + 0.5f);  /* +y = North = up */
}

static inline void map_px(int x, int y, uint32_t argb)
{
    if ((unsigned)x >= (unsigned)MAP_W || (unsigned)y >= (unsigned)MAP_H) return;
    s_map_buf[(size_t)y * MAP_W + x] = argb;
}

static void map_disc(int cx, int cy, int r, uint32_t argb)
{
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx * dx + dy * dy <= r * r) map_px(cx + dx, cy + dy, argb);
}

static void map_ring(int cx, int cy, int r, uint32_t argb)
{
    int ri2 = (r - 1) * (r - 1), ro2 = r * r;
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
            int d2 = dx * dx + dy * dy;
            if (d2 <= ro2 && d2 >= ri2) map_px(cx + dx, cy + dy, argb);
        }
}

/* Bresenham segment (clipped per-pixel by map_px). */
static void map_line(int x0, int y0, int x1, int y1, uint32_t argb)
{
    int dx = x1 - x0; if (dx < 0) dx = -dx;
    int dy = y1 - y0; if (dy < 0) dy = -dy;
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        map_px(x0, y0, argb);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* Nice grid step (m) giving ~8 divisions across the map at the current scale. */
static float map_grid_step(float ppm)
{
    float target = (float)MAP_W / ppm / 8.0f;
    static const float ladder[] = { 0.25f, 0.5f, 1, 2, 5, 10, 20, 50, 100 };
    for (int i = 0; i < (int)(sizeof(ladder) / sizeof(ladder[0])); i++)
        if (ladder[i] >= target) return ladder[i];
    return 100.0f;
}

/* Repaint the static map layer (grid + path + marks). Event-driven — called only
 * when the data or scale changed. Reads the observer scratch arrays. */
static void map_redraw(int nc, int nm)
{
    if (!s_map_buf) return;
    memset(s_map_buf, 0, (size_t)MAP_W * MAP_H * 4);   /* transparent → box bg shows */

    /* World grid (scrolls with the map, reinforcing motion). */
    float step = map_grid_step(s_map_ppm);
    uint32_t gcol = MAP_ARGB(70, 0x6B5300);   /* COL_DIM, faint */
    float halfw = (MAP_W * 0.5f) / s_map_ppm, halfh = (MAP_H * 0.5f) / s_map_ppm;
    int kx0 = (int)floorf((s_map_ctr_x - halfw) / step);
    int kx1 = (int)ceilf((s_map_ctr_x + halfw) / step);
    for (int k = kx0; k <= kx1; k++) {
        int px, py; map_world_to_px((float)k * step, s_map_ctr_y, &px, &py);
        if (px >= 0 && px < MAP_W) for (int y = 0; y < MAP_H; y++) s_map_buf[(size_t)y * MAP_W + px] = gcol;
    }
    int ky0 = (int)floorf((s_map_ctr_y - halfh) / step);
    int ky1 = (int)ceilf((s_map_ctr_y + halfh) / step);
    for (int k = ky0; k <= ky1; k++) {
        int px, py; map_world_to_px(s_map_ctr_x, (float)k * step, &px, &py);
        if (py >= 0 && py < MAP_H) { uint32_t *row = s_map_buf + (size_t)py * MAP_W;
            for (int x = 0; x < MAP_W; x++) row[x] = gcol; }
    }

    /* Breadcrumb path — oldest faint → newest bright (COL_MUTE). */
    int ppx = 0, ppy = 0;
    for (int i = 0; i < nc; i++) {
        int px, py;
        map_world_to_px(s_map_crumb_scratch[i].x, s_map_crumb_scratch[i].y, &px, &py);
        int a = (nc > 1) ? (80 + 150 * i / (nc - 1)) : 230;
        uint32_t c = MAP_ARGB(a, 0xB58A00);
        if (i > 0) map_line(ppx, ppy, px, py, c);
        map_disc(px, py, 2, c);
        ppx = px; ppy = py;
    }

    /* Last-known radar marks — red, fading with age (COL_ALERT). */
    for (int i = 0; i < nm; i++) {
        int px, py;
        map_world_to_px(s_map_mark_scratch[i].x, s_map_mark_scratch[i].y, &px, &py);
        int a = 230 - (int)(s_map_mark_scratch[i].age_ms * 190 / 30000);  /* TTL 30 s */
        if (a < 40) a = 40;
        map_disc(px, py, 4, MAP_ARGB(a, 0xFF3030));
        map_ring(px, py, 8, MAP_ARGB(a / 2, 0xFF3030));
    }

    lv_obj_invalidate(s_map_canvas);
}

static void map_reset_cb(lv_event_t *e)
{
    (void)e;
    prop_track_reset();
    s_map_have_fit = false;
    s_map_last_crumbs = -1;
    s_map_marks_sig = 0;
    prop_audio_play(PA_BUTTON);
}

static lv_obj_t *build_minimap_panel(lv_obj_t *parent)
{
    /* Full-screen page (rail is the chrome) with a thin header for status + RESET. */
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_size(p, SCAN_W, 600);
    lv_obj_align(p, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(p, COL_BG, 0);
    lv_obj_set_style_border_color(p, COL_AMBER, 0);
    lv_obj_set_style_border_width(p, 2, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(p);
    lv_label_set_text(title, "MINIMAP");
    lv_obj_set_style_text_font(title, FONT_HEAD, 0);
    lv_obj_set_style_text_color(title, COL_AMBER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 12);

    /* Honest drift note (gyro-only heading; see prop_track header). */
    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "HEADING: RELATIVE (GYRO)");
    lv_obj_set_style_text_font(note, FONT_BODY, 0);
    lv_obj_set_style_text_color(note, COL_DIM, 0);
    lv_obj_align(note, LV_ALIGN_TOP_LEFT, 150, 20);

    s_map_info = lv_label_create(p);
    lv_label_set_text(s_map_info, "");
    lv_obj_set_style_text_font(s_map_info, FONT_BODY, 0);
    lv_obj_set_style_text_color(s_map_info, COL_MUTE, 0);
    lv_obj_align(s_map_info, LV_ALIGN_TOP_LEFT, 430, 20);

    make_btn(p, "RESET", 110, LV_ALIGN_TOP_RIGHT, -12, 6, map_reset_cb);

    /* Map box. */
    lv_obj_t *box = lv_obj_create(p);
    lv_obj_set_size(box, MAP_W, MAP_H);
    lv_obj_align(box, LV_ALIGN_TOP_LEFT, MAP_X, MAP_Y);
    lv_obj_set_style_bg_color(box, COL_PANEL_ITEM, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(box, COL_DIM, 0);
    lv_obj_set_style_border_width(box, 1, 0);
    lv_obj_set_style_radius(box, 0, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    /* North marker (north-up: always at the top). */
    lv_obj_t *nlbl = lv_label_create(box);
    lv_label_set_text(nlbl, "N");
    lv_obj_set_style_text_font(nlbl, FONT_HEAD, 0);
    lv_obj_set_style_text_color(nlbl, COL_MUTE, 0);
    lv_obj_align(nlbl, LV_ALIGN_TOP_MID, 0, 6);

    /* Static map-layer canvas. */
    size_t sz = LV_CANVAS_BUF_SIZE(MAP_W, MAP_H, 32, LV_DRAW_BUF_STRIDE_ALIGN);
    s_map_buf = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
    if (s_map_buf) {
        memset(s_map_buf, 0, sz);
        s_map_canvas = lv_canvas_create(box);
        lv_canvas_set_buffer(s_map_canvas, s_map_buf, MAP_W, MAP_H, LV_COLOR_FORMAT_ARGB8888);
        lv_obj_set_pos(s_map_canvas, 0, 0);
        lv_obj_clear_flag(s_map_canvas, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    }

    /* Operator heading arrow (under the dot) + dot, glided every tick. */
    s_map_op_arrow = lv_line_create(box);
    lv_obj_set_style_line_color(s_map_op_arrow, COL_AMBER, 0);
    lv_obj_set_style_line_width(s_map_op_arrow, 3, 0);
    lv_obj_add_flag(s_map_op_arrow, LV_OBJ_FLAG_HIDDEN);

    s_map_op_dot = lv_obj_create(box);
    lv_obj_set_size(s_map_op_dot, 14, 14);
    lv_obj_set_style_radius(s_map_op_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_map_op_dot, COL_AMBER, 0);
    lv_obj_set_style_bg_opa(s_map_op_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_map_op_dot, COL_BG, 0);
    lv_obj_set_style_border_width(s_map_op_dot, 2, 0);
    lv_obj_clear_flag(s_map_op_dot, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_map_op_dot, LV_OBJ_FLAG_HIDDEN);

    /* Centred status notice (OFFLINE / WALK-TO-BEGIN). */
    s_map_msg = lv_label_create(box);
    lv_label_set_text(s_map_msg, "");
    lv_obj_set_style_text_font(s_map_msg, FONT_HEAD, 0);
    lv_obj_set_style_text_color(s_map_msg, COL_MUTE, 0);
    lv_obj_align(s_map_msg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_map_msg, LV_OBJ_FLAG_HIDDEN);

    /* Force a fit + redraw on the first observer tick. */
    s_map_have_fit = false;
    s_map_last_crumbs = -1;
    s_map_marks_sig = 0;
    s_map_ppm = 30.0f;
    s_map_ctr_x = s_map_ctr_y = 0.0f;
    return p;
}

/* Build the meter ("SIG" + a row of square cells, each filling left-to-right in
 * 25% steps) as a fixed-size child of `parent` (positioned by the caller). */
static lv_obj_t *build_signal_meter(lv_obj_t *parent)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_remove_style_all(box);
    int w = SIG_LBL_W + SIG_CELLS * SIG_CELL + (SIG_CELLS - 1) * SIG_GAP;
    lv_obj_set_size(box, w, SIG_CELL + 4);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(box);
    lv_label_set_text(lbl, "SIG");
    lv_obj_set_style_text_color(lbl, COL_MUTE, 0);
    lv_obj_set_style_text_font(lbl, FONT_BODY, 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);

    for (int i = 0; i < SIG_CELLS; i++) {
        lv_obj_t *c = lv_obj_create(box);
        lv_obj_remove_style_all(c);
        lv_obj_set_size(c, SIG_CELL, SIG_CELL);
        lv_obj_align(c, LV_ALIGN_LEFT_MID, SIG_LBL_W + i * (SIG_CELL + SIG_GAP), 0);
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(c, COL_BG, 0);        /* empty */
        lv_obj_set_style_border_color(c, COL_DIM, 0);
        lv_obj_set_style_border_width(c, 2, 0);
        lv_obj_set_style_radius(c, 0, 0);
        s_sig_cells[i] = c;

        lv_obj_t *f = lv_obj_create(c);                 /* left-anchored fill */
        lv_obj_remove_style_all(f);
        lv_obj_set_height(f, SIG_CELL - 2 * SIG_INSET);
        lv_obj_align(f, LV_ALIGN_LEFT_MID, SIG_INSET - 2, 0);
        lv_obj_set_style_bg_opa(f, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(f, COL_AMBER, 0);
        lv_obj_set_style_radius(f, 0, 0);
        lv_obj_add_flag(f, LV_OBJ_FLAG_HIDDEN);
        s_sig_fill[i] = f;
    }
    return box;
}

/* Fill the cells to `quarters` total, left-to-right, 25% per step, in `col`. */
static void set_signal_bars(int quarters, lv_color_t col)
{
    int span = SIG_CELL - 2 * SIG_INSET;   /* full inner width */
    for (int i = 0; i < SIG_CELLS; i++) {
        int q = quarters - i * 4;
        if (q < 0) q = 0;
        if (q > 4) q = 4;
        if (q == 0) {
            lv_obj_add_flag(s_sig_fill[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(s_sig_fill[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_width(s_sig_fill[i], q * span / 4);
            lv_obj_set_style_bg_color(s_sig_fill[i], col, 0);
            lv_obj_align(s_sig_fill[i], LV_ALIGN_LEFT_MID, SIG_INSET - 2, 0);
        }
    }
}

/* ---- Channel tuning gauge ---------------------------------------------------
 * A frequency band (100..400 MHz) with minor ticks and a bright marker showing
 * where the receiver is tuned. The waveform below is "the noise at that channel". */
#define CHAN_BAND_W 760
#define CHAN_BAND_H 28
#define CHAN_TICKS  10

static void build_channel_gauge(lv_obj_t *parent, lv_coord_t y)
{
    s_chan_band = lv_obj_create(parent);
    lv_obj_remove_style_all(s_chan_band);
    lv_obj_set_size(s_chan_band, CHAN_BAND_W, CHAN_BAND_H);
    lv_obj_align(s_chan_band, LV_ALIGN_TOP_MID, 0, y);
    lv_obj_set_style_bg_opa(s_chan_band, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_chan_band, COL_PANEL_ITEM, 0);
    lv_obj_set_style_border_color(s_chan_band, COL_DIM, 0);
    lv_obj_set_style_border_width(s_chan_band, 2, 0);
    lv_obj_set_style_radius(s_chan_band, 0, 0);
    lv_obj_clear_flag(s_chan_band, LV_OBJ_FLAG_SCROLLABLE);

    int inner = CHAN_BAND_W - 4;
    for (int i = 0; i <= CHAN_TICKS; i++) {
        lv_obj_t *t = lv_obj_create(s_chan_band);
        lv_obj_remove_style_all(t);
        bool major = (i % 5 == 0);
        lv_obj_set_size(t, major ? 2 : 1, major ? CHAN_BAND_H - 8 : CHAN_BAND_H - 18);
        lv_obj_align(t, LV_ALIGN_LEFT_MID, i * inner / CHAN_TICKS - (i == CHAN_TICKS ? 2 : 0), 0);
        lv_obj_set_style_bg_opa(t, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(t, COL_DIM, 0);
    }

    /* The tuned-position marker (bright vertical bar), moved in the observer. */
    s_chan_marker = lv_obj_create(s_chan_band);
    lv_obj_remove_style_all(s_chan_marker);
    lv_obj_set_size(s_chan_marker, 4, CHAN_BAND_H);
    lv_obj_set_style_bg_opa(s_chan_marker, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_chan_marker, COL_AMBER, 0);
    lv_obj_align(s_chan_marker, LV_ALIGN_LEFT_MID, 0, 0);

    /* "TUNE" caption left of the band; band-edge frequency labels below. */
    lv_obj_t *cap = lv_label_create(parent);
    lv_label_set_text(cap, "TUNE");
    lv_obj_set_style_text_color(cap, COL_MUTE, 0);
    lv_obj_set_style_text_font(cap, FONT_BODY, 0);
    lv_obj_align_to(cap, s_chan_band, LV_ALIGN_OUT_LEFT_MID, -12, 0);

    lv_obj_t *lo = lv_label_create(parent);
    lv_label_set_text(lo, "100");
    lv_obj_set_style_text_color(lo, COL_MUTE, 0);
    lv_obj_set_style_text_font(lo, FONT_BODY, 0);
    lv_obj_align_to(lo, s_chan_band, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    lv_obj_t *hi = lv_label_create(parent);
    lv_label_set_text(hi, "400 MHz");
    lv_obj_set_style_text_color(hi, COL_MUTE, 0);
    lv_obj_set_style_text_font(hi, FONT_BODY, 0);
    lv_obj_align_to(hi, s_chan_band, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 2);
}

/* ---- SENS (receiver gain) meter ---------------------------------------------
 * "SENS [====    ] 65%" - reflects the sensitivity the web slider drives, which
 * scales the live waveform amplitude. */
#define SENS_W 220
#define SENS_H 22

static void build_sens_meter(lv_obj_t *parent, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, "SENS");
    lv_obj_set_style_text_color(lbl, COL_MUTE, 0);
    lv_obj_set_style_text_font(lbl, FONT_BODY, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, x, y);

    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, SENS_W, SENS_H);
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, x, y + 2);
    lv_obj_align(bar, LV_ALIGN_TOP_LEFT, x + 56, y);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(bar, COL_PANEL_ITEM, 0);
    lv_obj_set_style_border_color(bar, COL_DIM, 0);
    lv_obj_set_style_border_width(bar, 2, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    s_sens_fill = lv_obj_create(bar);
    lv_obj_remove_style_all(s_sens_fill);
    lv_obj_set_height(s_sens_fill, SENS_H - 8);
    lv_obj_set_width(s_sens_fill, 0);
    lv_obj_align(s_sens_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(s_sens_fill, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_sens_fill, COL_AMBER, 0);

    s_sens_val = lv_label_create(parent);
    lv_label_set_text(s_sens_val, "--%");
    lv_obj_set_style_text_color(s_sens_val, COL_MUTE, 0);
    lv_obj_set_style_text_font(s_sens_val, FONT_BODY, 0);
    lv_obj_align_to(s_sens_val, bar, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
}

static void build_screen(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, COL_BG, LV_PART_MAIN);
    lv_obj_set_style_text_font(screen, FONT_BODY, 0);   /* Eurostile everywhere (inherited) */

    /* Persistent icon nav rail down the left edge of every screen. */
    build_rail(screen);

    /* Content container to the rail's right: the SCANNER readout builds here and
     * every lazily-built panel is created on it (s_root), so all of them inset
     * past the rail with no per-panel geometry changes. */
    lv_obj_t *scr = lv_obj_create(screen);
    lv_obj_remove_style_all(scr);
    lv_obj_set_size(scr, SCAN_W, 600);
    lv_obj_set_pos(scr, RAIL_W, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    s_root = scr;   /* lazily-built panels are created on this content container */

    /* Title bar (this root screen is the SCANNER instrument; the console home is
     * an overlay panel shown on top of it by default - see the open_panel(PK_HOME)
     * at the end of this function). */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "SCANNER  //  UNIT-7");
    lv_obj_set_style_text_color(title, COL_AMBER, 0);
    lv_obj_set_style_text_font(title, FONT_HEAD, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 24, 20);

    /* A divider line under the title (thin amber bar). */
    lv_obj_t *rule = lv_obj_create(scr);
    lv_obj_set_size(rule, SCAN_W - 48, 3);
    lv_obj_set_style_bg_color(rule, COL_DIM, 0);
    lv_obj_set_style_border_width(rule, 0, 0);
    lv_obj_align(rule, LV_ALIGN_TOP_MID, 0, 64);

    /* LINK indicator (top-right), with the signal meter pinned just to its left
     * (re-anchored in the observer so it tracks the label's changing width) and
     * the IP on the line below. */
    s_link_label = lv_label_create(scr);
    lv_label_set_text(s_link_label, "LINK: ----");
    lv_obj_set_style_text_color(s_link_label, COL_DIM, 0);
    lv_obj_set_style_text_font(s_link_label, FONT_BODY, 0);
    lv_obj_align(s_link_label, LV_ALIGN_TOP_RIGHT, -24, 12);

    s_sig_box = build_signal_meter(scr);
    lv_obj_align_to(s_sig_box, s_link_label, LV_ALIGN_OUT_LEFT_MID, -16, 0);

    /* IP on the second header line - kept above the y=64 divider so it isn't clipped. */
    s_ip_label = lv_label_create(scr);
    lv_label_set_text(s_ip_label, "");
    lv_obj_set_style_text_color(s_ip_label, COL_MUTE, 0);
    lv_obj_set_style_text_font(s_ip_label, FONT_BODY, 0);
    lv_obj_align(s_ip_label, LV_ALIGN_TOP_RIGHT, -24, 38);

    /* Big status line (the hero headline) */
    s_status_label = lv_label_create(scr);
    lv_label_set_text(s_status_label, "STANDBY");
    lv_obj_set_style_text_color(s_status_label, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_status_label, FONT_STATUS, 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 96);

    /* Channel / frequency readout */
    s_channel_label = lv_label_create(scr);
    lv_label_set_text(s_channel_label, "CH -- / --- MHz");
    lv_obj_set_style_text_color(s_channel_label, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_channel_label, FONT_HEAD, 0);
    lv_obj_align(s_channel_label, LV_ALIGN_TOP_MID, 0, 168);

    /* Channel tuning gauge (band + marker) below the readout. */
    build_channel_gauge(scr, 214);

    /* SENS meter just above the waveform, on its left edge. */
    build_sens_meter(scr, 24, SCAN_TRACK_Y - 32);

    /* Scanner track + sweeping blip */
    s_scan_track = lv_obj_create(scr);
    lv_obj_set_size(s_scan_track, SCAN_W - 48, SCAN_TRACK_H);
    lv_obj_set_style_bg_color(s_scan_track, lv_color_hex(0x141008), 0);
    lv_obj_set_style_border_color(s_scan_track, COL_DIM, 0);
    lv_obj_set_style_border_width(s_scan_track, 2, 0);
    lv_obj_set_style_radius(s_scan_track, 0, 0);
    lv_obj_align(s_scan_track, LV_ALIGN_TOP_MID, 0, SCAN_TRACK_Y);
    lv_obj_clear_flag(s_scan_track, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_scan_track, 0, 0);   /* line/blip use the full inner box */

    /* Recorder trail: an oscilloscope/seismograph line the engine fills column by
     * column, split into segments so each tick re-renders only the changed slice.
     * Built before the blip so the bright write-head draws on top. The shared point
     * buffer starts as a flat midline; the observer fills real values. */
    int mid0 = (SCAN_TRACK_H - 4) / 2;
    for (int i = 0; i < PROP_WAVE_SAMPLES; i++) {
        s_wave_pts[i].x = (lv_coord_t)((int)i * (SCAN_W - 48 - 1) / (PROP_WAVE_SAMPLES - 1));
        s_wave_pts[i].y = (lv_coord_t)mid0;
        s_wave_shadow[i] = (lv_coord_t)mid0;
    }
    s_wave_color = COL_AMBER;
    for (int k = 0; k < WAVE_SEGS; k++) {
        lv_obj_t *seg = lv_line_create(s_scan_track);
        lv_obj_set_style_line_color(seg, COL_AMBER, 0);
        lv_obj_set_style_line_width(seg, 2, 0);
        lv_obj_set_style_line_rounded(seg, false, 0);
        lv_obj_align(seg, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_line_set_points(seg, &s_wave_pts[k * WAVE_SEG_LEN], wave_seg_count(k));
        s_wave_seg[k] = seg;
    }

    s_scan_blip = lv_obj_create(s_scan_track);
    lv_obj_set_size(s_scan_blip, 4, SCAN_TRACK_H - 8);   /* thin write-head marker */
    lv_obj_set_style_bg_color(s_scan_blip, COL_AMBER, 0);
    lv_obj_set_style_border_width(s_scan_blip, 0, 0);
    lv_obj_set_style_radius(s_scan_blip, 0, 0);
    lv_obj_align(s_scan_blip, LV_ALIGN_LEFT_MID, 0, 0);

    /* Footer hint */
    lv_obj_t *foot = lv_label_create(scr);
    lv_label_set_text(foot, "AUTHORIZED PERSONNEL ONLY  -  SYS REV 0.1");
    lv_obj_set_style_text_color(foot, COL_MUTE, 0);
    lv_obj_set_style_text_font(foot, FONT_BODY, 0);
    lv_obj_align(foot, LV_ALIGN_BOTTOM_MID, 0, -20);

    /* SETUP button (bottom-right) opens the WiFi config panel. */
    lv_obj_t *setup_btn = lv_btn_create(scr);
    lv_obj_set_size(setup_btn, 110, 44);
    lv_obj_align(setup_btn, LV_ALIGN_BOTTOM_RIGHT, -16, -12);
    lv_obj_set_style_bg_color(setup_btn, lv_color_hex(0x141008), 0);
    lv_obj_set_style_border_color(setup_btn, COL_MUTE, 0);
    lv_obj_set_style_border_width(setup_btn, 1, 0);
    lv_obj_set_style_radius(setup_btn, 0, 0);
    lv_obj_add_event_cb(setup_btn, open_menu_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *setup_lbl = lv_label_create(setup_btn);
    lv_label_set_text(setup_lbl, "SETUP");
    lv_obj_set_style_text_color(setup_lbl, COL_MUTE, 0);
    lv_obj_center(setup_lbl);

    /* HOME button (bottom-left) returns this instrument to the console. */
    lv_obj_t *home_btn = lv_btn_create(scr);
    lv_obj_set_size(home_btn, 130, 44);
    lv_obj_align(home_btn, LV_ALIGN_BOTTOM_LEFT, 16, -12);
    lv_obj_set_style_bg_color(home_btn, lv_color_hex(0x141008), 0);
    lv_obj_set_style_border_color(home_btn, COL_MUTE, 0);
    lv_obj_set_style_border_width(home_btn, 1, 0);
    lv_obj_set_style_radius(home_btn, 0, 0);
    lv_obj_add_event_cb(home_btn, back_to_home_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *home_lbl = lv_label_create(home_btn);
    lv_label_set_text(home_lbl, LV_SYMBOL_LEFT " CONSOLE");
    lv_obj_set_style_text_color(home_lbl, COL_MUTE, 0);
    lv_obj_center(home_lbl);

    /* SETUP menu + sub-screens are built lazily on navigation (open_panel). The
     * SCANNER (PK_MOTION) is the default landing screen on boot. */
    uint32_t ping = 0;
    prop_settings_get_u32("scan_ping", &ping, 0);
    s_scan_ping_en = (ping != 0);

    /* Travel-direction calibration: stored as signed milli-radians (u32 two's-comp). */
    uint32_t phi_mrad = 0;
    prop_settings_get_u32("dir_phi", &phi_mrad, 0);
    s_dir_phi = (float)(int32_t)phi_mrad / 1000.0f;

    open_panel(PK_MOTION);
    s_ui_ready = true;   /* subsequent navigations get the channel-change transition */
}

static const char *link_text(prop_link_t link)
{
    switch (link) {
        case LINK_STA: return "LINK: NET";
        case LINK_AP:  return "LINK: LOCAL";
        default:       return "LINK: ----";
    }
}

/* lv_label_set_text has no dedupe in LVGL v8: it reallocs + invalidates the label
 * on every call, even with identical text. At 20 Hz that's a needless redraw per
 * label per frame. Skip the write when the text hasn't actually changed. */
static void label_set_text_cached(lv_obj_t *label, const char *txt)
{
    const char *cur = lv_label_get_text(label);
    if (!cur || strcmp(cur, txt) != 0) {
        lv_label_set_text(label, txt);
    }
}

/* Observer: runs in engine task context -> must take the LVGL lock. */
static void ui_observer(const prop_state_t *st, void *ctx)
{
    (void)ctx;
    if (!lvgl_port_lock(50)) {
        return;  /* skip this frame rather than block the engine */
    }

    /* The whole SCANNER readout below lives on the root screen. Every other screen
     * is a full-screen OPAQUE panel built on top of it (PK_HOME is the default
     * landing screen), so when one is up this readout is completely obscured.
     * Repainting it at 20 Hz then is pure waste - worse, each invisible
     * lv_line_set_points() invalidation forces LVGL to recomposite that stripe, and
     * with the CRT overlay on, to software-alpha-blend the overlay over it. Skip the
     * entire block unless the bare scanner is the visible screen. */
    if (s_cur_kind == PK_NONE) {
        label_set_text_cached(s_status_label, st->status);
        label_set_text_cached(s_channel_label, st->channel);
        label_set_text_cached(s_link_label, link_text(st->link));
        lv_obj_set_style_text_color(s_link_label, st->link == LINK_DOWN ? COL_DIM : COL_AMBER, 0);
        /* Keep the meter pinned to the (variable-width) LINK label. */
        lv_obj_align_to(s_sig_box, s_link_label, LV_ALIGN_OUT_LEFT_MID, -12, 0);

        /* IP + signal strength alongside the LINK indicator. prop_net_get_rssi() is a
         * cheap cached read (a background task does the SDIO poll), so this stays off
         * the critical path and never stalls the display. */
        if (st->link == LINK_STA) {
            char ip[16];
            prop_net_get_ip(ip, sizeof(ip));
            label_set_text_cached(s_ip_label, ip);
            set_signal_bars(rssi_to_quarters(prop_net_get_rssi()), COL_AMBER);
        } else if (st->link == LINK_AP) {
            label_set_text_cached(s_ip_label, "AP 192.168.4.1");
            set_signal_bars(0, COL_AMBER);
        } else {
            label_set_text_cached(s_ip_label, "");
            set_signal_bars(0, COL_AMBER);
        }

        /* ALERT tints the readout red; everything else is amber phosphor. */
        bool alert = (st->scene == SCENE_ALERT);
        lv_color_t status_col = alert ? COL_ALERT : COL_AMBER;

        /* Plot the engine's recorder trail across the scanner track. The UI owns no
         * waveform logic - it just maps the signed columns to screen coordinates. */
        lv_coord_t cw = lv_obj_get_content_width(s_scan_track);
        lv_coord_t ch = lv_obj_get_content_height(s_scan_track);
        if (cw <= 1) cw = SCAN_W - 52;
        if (ch <= 1) ch = SCAN_TRACK_H - 4;
        int mid = ch / 2;
        int half = ch / 2 - 1;
        for (int i = 0; i < PROP_WAVE_SAMPLES; i++) {
            s_wave_pts[i].x = (lv_coord_t)((int)i * (cw - 1) / (PROP_WAVE_SAMPLES - 1));
            s_wave_pts[i].y = (lv_coord_t)(mid - (st->wave[i] * half) / 100);
        }
        /* Re-render only the segments whose slice actually changed (steady state =
         * one segment; spikes = a few; SENS change = all). This is the optimisation
         * that takes the trace from ~14 fps (full-track redraw) toward 60. */
        bool recolor = !lv_color_eq(s_wave_color, status_col);  /* v9 lv_color_t has no .full */
        for (int k = 0; k < WAVE_SEGS; k++) {
            int start = k * WAVE_SEG_LEN;
            int cnt = wave_seg_count(k);
            bool changed = false;
            for (int i = start; i < start + cnt; i++) {
                if (s_wave_pts[i].y != s_wave_shadow[i]) { changed = true; break; }
            }
            if (changed) {
                lv_line_set_points(s_wave_seg[k], &s_wave_pts[start], cnt);
                for (int i = start; i < start + cnt; i++) s_wave_shadow[i] = s_wave_pts[i].y;
            }
            if (recolor) {
                lv_obj_set_style_line_color(s_wave_seg[k], status_col, 0);
            }
        }
        s_wave_color = status_col;

        /* Every per-frame style/text write below is gated on an actual change. The
         * big STANDBY headline re-rasterizing each tick (it's large AA text) was the
         * real cost behind the ~73 ms frames - far more than the waveform. In steady
         * state only the blip (4 px) and one wave segment now invalidate. */

        /* The bright blip rides the write head; when SIGNAL_ACQUIRED freezes the
         * sweep it parks on the eruption as a peak marker. */
        int head_x = (int)st->wave_head * (cw - 1) / (PROP_WAVE_SAMPLES - 1);
        if (head_x != s_last_blip_x) {
            lv_obj_align(s_scan_blip, LV_ALIGN_LEFT_MID, head_x, 0);
            s_last_blip_x = head_x;
        }
        uint16_t blip_col = lv_color_to_u16(status_col);
        if (blip_col != s_last_blip_col) {
            lv_obj_set_style_bg_color(s_scan_blip, status_col, 0);
            s_last_blip_col = blip_col;
        }

        /* Status punch-in: SIGNAL_ACQUIRED / ALERT slam the headline in big + bright,
         * then settle. The font swap + flash only fire while punching (brief). */
        bool punching = st->scene_tick < 6 && (st->scene == SCENE_SIGNAL_ACQUIRED || alert);
        lv_color_t headline_col = status_col;
        if (punching && (st->scene_tick & 1)) {
            headline_col = alert ? lv_color_hex(0xFF9090) : lv_color_hex(0xFFE060);
        }
        if ((int)punching != s_last_punch) {
            lv_obj_set_style_text_font(s_status_label, punching ? FONT_PUNCH : FONT_STATUS, 0);
            s_last_punch = punching;
        }
        uint16_t hl_col = lv_color_to_u16(headline_col);
        if (hl_col != s_last_headline_col) {
            lv_obj_set_style_text_color(s_status_label, headline_col, 0);
            s_last_headline_col = hl_col;
        }

        /* Channel gauge marker tracks the tuned position (scrambles while SCANNING). */
        lv_coord_t band_w = lv_obj_get_content_width(s_chan_band);
        if (band_w <= 1) band_w = CHAN_BAND_W - 4;
        int marker_x = (int)st->chan_pos * (band_w - 4) / 100;
        if (marker_x != s_last_marker_x) {
            lv_obj_align(s_chan_marker, LV_ALIGN_LEFT_MID, marker_x, 0);
            s_last_marker_x = marker_x;
        }
        uint16_t marker_col = lv_color_to_u16(alert ? COL_ALERT : COL_AMBER);
        if (marker_col != s_last_marker_col) {
            lv_obj_set_style_bg_color(s_chan_marker, alert ? COL_ALERT : COL_AMBER, 0);
            s_last_marker_col = marker_col;
        }

        /* SENS meter reflects the receiver gain the web slider drives. */
        if ((int)st->sensitivity != s_last_sens) {
            lv_coord_t sens_w = lv_obj_get_content_width(lv_obj_get_parent(s_sens_fill));
            if (sens_w <= 1) sens_w = SENS_W - 4;
            lv_obj_set_width(s_sens_fill, (int)st->sensitivity * sens_w / 100);
            lv_label_set_text_fmt(s_sens_val, "%d%%", st->sensitivity);
            s_last_sens = st->sensitivity;
        }
    }

    /* Refresh the console's data-sponge status strip (~2 Hz) while it's live.
     * No RTC/SNTP yet, so CLOCK shows uptime; DATE is the static placeholder. */
    if (s_cur_kind == PK_HOME && s_home_clock && (st->tick % 10 == 0)) {
        uint32_t up = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        lv_label_set_text_fmt(s_home_clock, "%02u:%02u:%02u",
                              (unsigned)(up / 3600), (unsigned)((up / 60) % 60),
                              (unsigned)(up % 60));
        float t = read_core_temp();
        if (t > -500.0f) {
            char fbuf[24];
            snprintf(fbuf, sizeof(fbuf), "%.1f C", t);
            lv_label_set_text(s_home_temp, fbuf);
        } else {
            lv_label_set_text(s_home_temp, "-- C");
        }
        lv_label_set_text(s_home_link, st->link == LINK_STA ? "LINKED" :
                                       (st->link == LINK_AP ? "LOCAL" : "SEEKING"));
        lv_obj_set_style_text_color(s_home_link,
                                    st->link == LINK_DOWN ? COL_MUTE : COL_AMBER, 0);
    }

    /* Refresh the VITALS instrument (~2 Hz) only while it's the live panel. */
    if (s_cur_kind == PK_VITALS && s_vit_temp && (st->tick % 10 == 0)) {
        /* LVGL's printf has no %f support - format floats with stdio snprintf. */
        char fbuf[24];
        float t = read_core_temp();
        if (t > -500.0f) {
            snprintf(fbuf, sizeof(fbuf), "%.1f C", t);
            lv_label_set_text(s_vit_temp, fbuf);
            set_meter(s_vit_temp_bar, (int)(t * 100.0f / 80.0f), t > 60 ? COL_ALERT : COL_AMBER);
        } else {
            lv_label_set_text(s_vit_temp, "-- C");
        }
        size_t free_i = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t tot_i = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
        size_t free_p = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        lv_label_set_text_fmt(s_vit_ram, "%uK int / %uK psram",
                              (unsigned)(free_i / 1024), (unsigned)(free_p / 1024));
        int used = tot_i ? (int)((tot_i - free_i) * 100 / tot_i) : 0;
        set_meter(s_vit_ram_bar, used, used > 85 ? COL_ALERT : COL_AMBER);

        uint32_t up = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        /* Diegetic cell voltage: a gentle wander for on-camera life (no battery ADC). */
        float cell = 3.85f + 0.12f * sinf(up * 0.05f);
        snprintf(fbuf, sizeof(fbuf), "%.2f V", cell);
        lv_label_set_text(s_vit_cell, fbuf);
        set_meter(s_vit_cell_bar, (int)((cell - 3.3f) / (4.2f - 3.3f) * 100.0f), COL_AMBER);
        lv_label_set_text_fmt(s_vit_uptime, "%02u:%02u:%02u",
                              (unsigned)(up / 3600), (unsigned)((up / 60) % 60), (unsigned)(up % 60));

        /* UPLINK dossier (cached connected-AP info). */
        if (s_vit_up_net) {
            prop_uplink_t ul;
            prop_net_get_uplink(&ul);
            if (ul.connected) {
                lv_label_set_text_fmt(s_vit_up_net, "%s  %d dBm", ul.ssid, ul.rssi);
                const char *v = prop_net_oui_vendor(ul.bssid);
                lv_label_set_text_fmt(s_vit_up_ap, "%s  %02X:%02X:%02X",
                                      v ? v : "UNKNOWN OEM", ul.bssid[3], ul.bssid[4], ul.bssid[5]);
                lv_label_set_text_fmt(s_vit_up_link, "%s  -  CH %d  -  %s",
                                      ul.phy[0] ? ul.phy : "--", ul.channel,
                                      ul.country[0] ? ul.country : "--");
            } else {
                lv_label_set_text(s_vit_up_net, "-- NO UPLINK --");
                lv_label_set_text(s_vit_up_ap, "--");
                lv_label_set_text(s_vit_up_link, "--");
            }
        }

        /* IMU — yaw/pitch/roll bars + accel readout. */
        if (s_vit_yaw) {
            prop_imu_data_t imu;
            prop_imu_get_data(&imu);
            if (imu.online && imu.valid) {
                /* Convert radians to degrees; map ±180° to 0..100% bar. */
                float yaw_d   = imu.yaw   * (180.0f / (float)M_PI);
                float pitch_d = imu.pitch * (180.0f / (float)M_PI);
                float roll_d  = imu.roll  * (180.0f / (float)M_PI);
                char fbuf[32];

                snprintf(fbuf, sizeof(fbuf), "%.1f \xc2\xb0", yaw_d);  /* UTF-8 degree sign */
                lv_label_set_text(s_vit_yaw, fbuf);
                int yaw_pct = (int)((yaw_d + 180.0f) * 100.0f / 360.0f);
                set_meter(s_vit_yaw_bar, yaw_pct, COL_AMBER);

                snprintf(fbuf, sizeof(fbuf), "%.1f \xc2\xb0", pitch_d);
                lv_label_set_text(s_vit_pitch, fbuf);
                int pitch_pct = (int)((pitch_d + 90.0f) * 100.0f / 180.0f);
                set_meter(s_vit_pitch_bar, pitch_pct, COL_AMBER);

                snprintf(fbuf, sizeof(fbuf), "%.1f \xc2\xb0", roll_d);
                lv_label_set_text(s_vit_roll, fbuf);
                int roll_pct = (int)((roll_d + 90.0f) * 100.0f / 180.0f);
                set_meter(s_vit_roll_bar, roll_pct, COL_AMBER);

                /* Floats via snprintf — lv_label_set_text_fmt's printf has no %f. */
                if (s_vit_accel) {
                    char ab[40];
                    snprintf(ab, sizeof(ab), "X%+.2fg  Y%+.2fg  Z%+.2fg",
                             (double)imu.ax / 16384.0, (double)imu.ay / 16384.0,
                             (double)imu.az / 16384.0);
                    lv_label_set_text(s_vit_accel, ab);
                }
                if (s_vit_imu_temp) {
                    char tb[20];
                    snprintf(tb, sizeof(tb), "%.1f \xc2\xb0""C", (double)imu.temp_c);
                    lv_label_set_text(s_vit_imu_temp, tb);
                }
                if (s_vit_steps)
                    lv_label_set_text_fmt(s_vit_steps, "%u STEP  ~%um",
                                          (unsigned)imu.step_count,
                                          (unsigned)((imu.step_count * 3) / 4));  /* ~0.75 m/step */
            } else {
                lv_label_set_text(s_vit_yaw,   imu.online ? "-- \xc2\xb0" : "OFFLINE");
                lv_label_set_text(s_vit_pitch, imu.online ? "-- \xc2\xb0" : "OFFLINE");
                lv_label_set_text(s_vit_roll,  imu.online ? "-- \xc2\xb0" : "OFFLINE");
                set_meter(s_vit_yaw_bar,   0, COL_DIM);
                set_meter(s_vit_pitch_bar, 0, COL_DIM);
                set_meter(s_vit_roll_bar,  0, COL_DIM);
                if (s_vit_accel) lv_label_set_text(s_vit_accel, "--");
                if (s_vit_imu_temp) lv_label_set_text(s_vit_imu_temp, "--");
                if (s_vit_steps) lv_label_set_text(s_vit_steps, "--");
            }
        }
    }

    /* Drive the SPECTRUM bars from the mic FFT every frame while it's live. The
     * bars rise instantly and fall with a slow decay (retro analyser ballistics). */
    if (s_cur_kind == PK_SPECTRUM && s_spec_bars[0]) {
        uint8_t bands[PROP_MIC_BANDS];
        prop_mic_get_bands(bands);
        for (int i = 0; i < PROP_MIC_BANDS; i++) {
            float v = (float)bands[i];
            if (v >= s_spec_decay[i]) s_spec_decay[i] = v;
            else s_spec_decay[i] *= 0.80f;
            int pct = (int)s_spec_decay[i];
            int h = 2 + pct * SPEC_MAXH / 100;
            lv_obj_set_height(s_spec_bars[i], h);
            int8_t band = (pct > 85) ? 2 : (pct > 35) ? 1 : 0;
            if (band != s_spec_band[i]) {
                s_spec_band[i] = band;
                lv_obj_set_style_bg_color(s_spec_bars[i],
                                          band == 2 ? COL_ALERT : (band == 1 ? COL_AMBER : COL_MUTE), 0);
            }
        }
        if (st->tick % 4 == 0 && s_spec_db) {
            lv_label_set_text_fmt(s_spec_db, "%d dB", prop_mic_get_db());
            set_meter(s_spec_db_bar, (prop_mic_get_db() + 60) * 100 / 60, COL_AMBER);
        }
    }

    /* Drive the RF BAND bars from the cached channel histogram. The data only
     * changes on a (re)scan, so bars rise to their value and hold; the slow decay
     * gives a smooth settle and lets a fresh scan with weaker channels ease down. */
    if (s_cur_kind == PK_RFBAND && s_rf_bars[0]) {
        for (int i = 0; i < RF_CHANNELS; i++) {
            float v = (float)s_rf_chan[i + 1];   /* s_rf_chan[ch], ch = i+1 */
            if (v >= s_rf_decay[i]) s_rf_decay[i] = v;
            else s_rf_decay[i] *= 0.90f;
            int pct = (int)s_rf_decay[i];
            int h = 2 + pct * RF_MAXH / 100;
            lv_obj_set_height(s_rf_bars[i], h);
            int8_t band = (pct > 70) ? 2 : (pct > 20) ? 1 : 0;
            if (band != s_rf_band[i]) {
                s_rf_band[i] = band;
                lv_obj_set_style_bg_color(s_rf_bars[i],
                                          band == 2 ? COL_ALERT : (band == 1 ? COL_AMBER : COL_MUTE), 0);
            }
        }
    }

    /* Update the BLE contact list in-place (throttled ~2.5 Hz). Pre-built row slots
     * are shown/hidden rather than destroyed and recreated each tick. */
    if (s_cur_kind == PK_BLE && s_ble_list && (st->tick % 8 == 0)) {
        int cnt = 0, named = 0, known = 0;
        int8_t best = 0;
        prop_ble_get_summary(&cnt, &best, &named, &known);
        if (cnt == 0) {
            lv_label_set_text(s_ble_summary, "NO CONTACTS IN RANGE");
        } else {
            char cb[16];
            ble_fmt_dist(cb, sizeof(cb), prop_ble_distance_m(best, PROP_BLE_TXPWR_NONE));
            lv_label_set_text_fmt(s_ble_summary,
                                  "%d CONTACT%s  -  NEAREST ~%s  -  %d KNOWN",
                                  cnt, cnt == 1 ? "" : "S", cb, known);
        }

        prop_ble_dev_t devs[PROP_BLE_MAX];
        int n = prop_ble_get_devices(devs, PROP_BLE_MAX);
        for (int i = 0; i < PROP_BLE_MAX; i++) {
            ble_row_t *r = &s_ble_rows[i];
            if (!r->tag) break;   /* pool not built (BLE offline path) */
            if (i < n) {
                ble_update_row(i, &devs[i]);
                lv_obj_clear_flag(r->tag,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->dbm,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->mac,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->dist, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->uuid, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->tx,   LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->fill, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(r->tag,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->dbm,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->mac,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->dist, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->uuid, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->tx,   LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->fill, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    /* Update the RANGE (FTM) list in-place (throttled ~2.5 Hz), same pre-built
     * row-pool pattern as CONTACTS. Only FTM-capable BSSIDs are shown — the
     * table itself tracks every scanned BSSID, but prop_ftm_get_table sorts
     * capable-first so the display cutoff is just "until capable stops". */
    if (s_cur_kind == PK_RANGE && s_range_list && (st->tick % 8 == 0)) {
        static prop_ftm_entry_t entries[PROP_FTM_TABLE_MAX];   /* off animate_task's tight stack */
        int total = prop_ftm_get_table(entries, PROP_FTM_TABLE_MAX);
        int n = 0;
        while (n < total && entries[n].ftm_capable) {
            n++;
        }

        if (n == 0) {
            lv_label_set_text(s_range_summary, "NO FTM-CAPABLE APs DETECTED");
        } else {
            int ranged = 0;
            for (int i = 0; i < n; i++) {
                if (entries[i].last_dist_cm >= 0) ranged++;
            }
            lv_label_set_text_fmt(s_range_summary, "%d FTM-CAPABLE AP%s  -  %d RANGED",
                                  n, n == 1 ? "" : "S", ranged);
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
        for (int i = 0; i < PROP_FTM_TABLE_MAX; i++) {
            range_row_t *r = &s_range_rows[i];
            if (!r->tag) break;   /* pool not built (FTM offline path) */
            if (i < n) {
                range_update_row(i, &entries[i], now);
                lv_obj_clear_flag(r->tag,    LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->rssi,   LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->mac,    LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->dist,   LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->status, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r->fill,   LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(r->tag,    LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->rssi,   LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->mac,    LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->dist,   LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->status, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r->fill,   LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    /* Drive the SIGNAL ENVIRONMENT bars from the CSI movement-history FIFO. Each
     * bar is a fixed past sample — render col[i] DIRECTLY (no peak-hold decay,
     * which would let historical bars change height after the fact). The FIFO
     * scroll happens at the source (prop_csi.c) when a new C6 sample arrives. */
    if (s_cur_kind == PK_CSI && s_csi_bars[0]) {
        uint8_t col[PROP_CSI_BINS];
        prop_csi_get_column(col);
        for (int i = 0; i < PROP_CSI_BINS; i++) {
            int pct = col[i];
            int h = 2 + pct * CSI_MAXH / 100;
            lv_obj_set_height(s_csi_bars[i], h);
            int8_t band = (pct > 85) ? 2 : (pct > 35) ? 1 : 0;
            if (band != s_csi_band[i]) {
                s_csi_band[i] = band;
                lv_obj_set_style_bg_color(s_csi_bars[i],
                                          band == 2 ? COL_ALERT : (band == 1 ? COL_AMBER : COL_MUTE), 0);
            }
        }
        /* Motion verdict + movement readout (real C6 data when live). */
        bool motion = false; int move_mi = 0, thr_mi = 0;
        bool live = prop_csi_get_motion(&motion, &move_mi, &thr_mi);
        prop_calib_status_t ccs; prop_calib_get(&ccs);
        if (st->tick % 4 == 0) {
            if (s_csi_motion) {
                if (ccs.countdown > 0) {
                    char mb[24]; snprintf(mb, sizeof(mb), "RE-BASELINE %d", ccs.countdown);
                    label_set_text_cached(s_csi_motion, mb);
                    lv_obj_set_style_text_color(s_csi_motion, COL_AMBER, 0);
                } else {
                    label_set_text_cached(s_csi_motion, !live ? "--" : (motion ? "MOTION" : "IDLE"));
                    lv_obj_set_style_text_color(s_csi_motion, motion ? COL_ALERT : COL_MUTE, 0);
                }
            }
            if (s_csi_rssi) {
                int rssi = prop_net_get_rssi();
                char sb[24];
                if (rssi) { snprintf(sb, sizeof(sb), "LINK %d dBm", rssi); }
                else      { snprintf(sb, sizeof(sb), "LINK ---"); }
                label_set_text_cached(s_csi_rssi, sb);
            }
            if (s_csi_geiger_lbl) {
                label_set_text_cached(s_csi_geiger_lbl,
                                      prop_csi_geiger() ? "GEIGER: ON" : "SPECTRE GEIGER");
            }
            if (s_csi_move) {
                char buf[40];
                if (live) {
                    snprintf(buf, sizeof(buf), "MOVEMENT %d / THR %d", move_mi, thr_mi);
                } else {
                    snprintf(buf, sizeof(buf), "MOVEMENT --- / THR ---");
                }
                label_set_text_cached(s_csi_move, buf);
            }
            if (s_csi_rf) {
                int turb = 0, agc = 0, fft = 0; bool gl = false;
                prop_csi_get_rf(&turb, &agc, &fft, &gl);
                char rb[64];
                if (live) {
                    snprintf(rb, sizeof(rb), "TURB %d.%02d   AGC %d  FFT %d   GAIN %s",
                             turb / 1000, (turb % 1000) / 10, agc, fft,
                             gl ? "LOCKED" : "UNLOCKED");
                } else {
                    snprintf(rb, sizeof(rb), "TURB ---  GAIN ---");
                }
                label_set_text_cached(s_csi_rf, rb);
            }
        }
            if (s_csi_status) {
                label_set_text_cached(s_csi_status, live
                                      ? "CSI LIVE  //  ON-C6 MOTION"
                                      : "SYNTHETIC  //  NO C6 FEED");
            }
            if (s_csi_fp_cells[0]) {
                uint8_t sc[12]; prop_csi_get_subcarriers(sc);
                static uint8_t last_sc[12];
                if (memcmp(sc, last_sc, 12) != 0) {   /* re-light only on change */
                    memcpy(last_sc, sc, 12);
                    bool sel[64] = { 0 };
                    for (int k = 0; k < 12; k++) { if (sc[k] < 64) { sel[sc[k]] = true; } }
                    for (int b = 0; b < 64; b++) {
                        if (!s_csi_fp_cells[b]) { continue; }
                        lv_obj_set_height(s_csi_fp_cells[b], sel[b] ? 16 : 5);
                        lv_obj_set_style_bg_color(s_csi_fp_cells[b], sel[b] ? COL_AMBER : COL_DIM, 0);
                    }
                }
            }
    }

    /* CSI CONFIG panel: live feedback + guided calibration phase/countdown. */
    if (s_cur_kind == PK_CSICFG && s_cfg_phase && (st->tick % 4 == 0)) {
        bool motion = false; int move_mi = 0, thr_mi = 0;
        bool live = prop_csi_get_motion(&motion, &move_mi, &thr_mi);
        if (s_cfg_motion) {
            label_set_text_cached(s_cfg_motion, !live ? "--" : (motion ? "MOTION" : "IDLE"));
            lv_obj_set_style_text_color(s_cfg_motion, motion ? COL_ALERT : COL_MUTE, 0);
        }
        if (s_cfg_move) {
            char buf[40];
            if (live) { snprintf(buf, sizeof(buf), "MOVEMENT %d / THR %d", move_mi, thr_mi); }
            else      { snprintf(buf, sizeof(buf), "MOVEMENT --- / THR ---"); }
            label_set_text_cached(s_cfg_move, buf);
        }
        if (s_cfg_meter) {
            int thr = thr_mi > 0 ? thr_mi : 1000;
            kit_set_meter(s_cfg_meter, move_mi * 100 / (thr * 2),
                          motion ? COL_ALERT : COL_AMBER);
        }

        prop_calib_status_t cs;
        prop_calib_get(&cs);
        char b[64];
        if (cs.auto_on && cs.live && cs.fill_pct < 100) {
            snprintf(b, sizeof(b), "LEARNING  %d%%", cs.fill_pct);
        } else if (cs.auto_on && cs.live) {
            snprintf(b, sizeof(b), "AUTO-ADAPTING  base %d", cs.baseline_milli);
        } else {
            snprintf(b, sizeof(b), "%s", cs.message[0] ? cs.message : "AUTO OFF");
        }
        label_set_text_cached(s_cfg_phase, b);
        if (s_cfg_btn_lbl) {
            label_set_text_cached(s_cfg_btn_lbl, cs.auto_on ? "AUTO: ON" : "AUTO: OFF");
        }
    }

    /* Refresh the ABOUT panel's live fields (~1 Hz) only while it's the live panel. */
    if (s_cur_kind == PK_ABOUT && s_about_ip && (st->tick % 20 == 0)) {
        char ip[16];
        prop_net_get_ip(ip, sizeof(ip));
        lv_label_set_text_fmt(s_about_ip, "%s", ip[0] ? ip : "(no link)");
        uint32_t s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        lv_label_set_text_fmt(s_about_uptime, "%02u:%02u:%02u",
                              (unsigned)(s / 3600), (unsigned)((s / 60) % 60),
                              (unsigned)(s % 60));
    }

    /* While a connection attempt is in flight, reflect its real outcome in the
     * setup panel status - but only while the WiFi panel is still the live one. */
    if (s_connect_pending && s_cur_kind == PK_WIFI && s_setup_status) {
        prop_sta_state_t s = prop_net_sta_state();
        if (s == STA_CONNECTED) {
            char ip[16];
            prop_net_get_ip(ip, sizeof(ip));
            lv_label_set_text_fmt(s_setup_status, "Connected  %s", ip);
            s_connect_pending = false;
        } else if (s == STA_FAILED) {
            lv_label_set_text(s_setup_status, "Failed - check password");
            s_connect_pending = false;
        }
    }

    /* I/O BENCH live readouts (~4 Hz). Overview: repaint a box only when its state key
     * changes. Config page: refresh the one pin's level/analog/edge-count readouts. */
    if (s_cur_kind == PK_IO && (st->tick % 5 == 0)) {
        int count = bsp_aio_count();
        for (int i = 0; i < count && i < IO_MAX; i++) {
            if (!s_io_box[i]) continue;
            int key = io_box_key(i);
            if (key != s_io_last[i]) { s_io_last[i] = key; io_box_paint(i); }
        }
    } else if (s_cur_kind == PK_IO_PIN && (st->tick % 5 == 0)) {
        int i = s_io_pin;
        aio_mode_t m = bsp_aio_get_mode(i);
        if (aio_is_digital(m) && s_pin_read) {
            label_set_text_cached(s_pin_read, bsp_aio_read_level(i) > 0 ? "HIGH" : "LOW");
        } else if (m == AIO_ANALOG_IN && s_pin_read) {
            int raw = 0, pct = 0;
            if (bsp_aio_read_ain(i, &raw, &pct) == ESP_OK) {
                char v[28]; snprintf(v, sizeof(v), "%d  %d%%  %.2fV",
                                     raw, pct, (double)(bsp_aio_ain_vmax(i) * raw / 4095.0f));
                label_set_text_cached(s_pin_read, v);
                if (s_pin_fill) kit_set_meter(s_pin_fill, pct, COL_AMBER);
            }
        }
        if (s_pin_edges) {
            char ec[12]; snprintf(ec, sizeof(ec), "%lu", (unsigned long)bsp_aio_get_edges(i));
            label_set_text_cached(s_pin_edges, ec);
        }
    }

    /* Drive the MOTION SCAN panel: radial pulse, target blips, gimbal, aux labels. */
    /* ---- MINIMAP: dead-reckoning map (event-driven canvas + glided operator) ---- */
    if (s_cur_kind == PK_MINIMAP && s_map_canvas) {
        prop_track_pose_t pose;
        prop_track_get_pose(&pose);

        if (!prop_track_available() || !pose.valid) {
            if (s_map_msg) {
                label_set_text_cached(s_map_msg,
                    prop_track_available() ? "WALK TO BEGIN MAPPING" : "DEAD-RECKONING OFFLINE");
                lv_obj_set_style_text_color(s_map_msg,
                    prop_track_available() ? COL_MUTE : COL_ALERT, 0);
                lv_obj_clear_flag(s_map_msg, LV_OBJ_FLAG_HIDDEN);
            }
            if (s_map_op_dot)   lv_obj_add_flag(s_map_op_dot, LV_OBJ_FLAG_HIDDEN);
            if (s_map_op_arrow) lv_obj_add_flag(s_map_op_arrow, LV_OBJ_FLAG_HIDDEN);
        } else {
            if (s_map_msg) lv_obj_add_flag(s_map_msg, LV_OBJ_FLAG_HIDDEN);

            int nc = prop_track_get_crumbs(s_map_crumb_scratch, MAP_CRUMB_CAP);
            int nm = prop_track_get_marks(s_map_mark_scratch, PROP_TRACK_MAX_MARKS);

            /* Auto-fit bounds over path + operator + marks (world metres). */
            float minx = pose.x, maxx = pose.x, miny = pose.y, maxy = pose.y;
            for (int i = 0; i < nc; i++) {
                float x = s_map_crumb_scratch[i].x, y = s_map_crumb_scratch[i].y;
                if (x < minx) minx = x;
                if (x > maxx) maxx = x;
                if (y < miny) miny = y;
                if (y > maxy) maxy = y;
            }
            for (int i = 0; i < nm; i++) {
                float x = s_map_mark_scratch[i].x, y = s_map_mark_scratch[i].y;
                if (x < minx) minx = x;
                if (x > maxx) maxx = x;
                if (y < miny) miny = y;
                if (y > maxy) maxy = y;
            }
            const float margin = 1.5f;                 /* metres of breathing room */
            float spanx = (maxx - minx) + 2.0f * margin;
            float spany = (maxy - miny) + 2.0f * margin;
            if (spanx < 1.0f) spanx = 1.0f;
            if (spany < 1.0f) spany = 1.0f;
            float ppm = (float)MAP_W / spanx;
            float ppmy = (float)MAP_H / spany;
            if (ppmy < ppm) ppm = ppmy;
            if (ppm < MAP_PPM_MIN) ppm = MAP_PPM_MIN;
            if (ppm > MAP_PPM_MAX) ppm = MAP_PPM_MAX;
            float cx = (minx + maxx) * 0.5f, cy = (miny + maxy) * 0.5f;

            /* Hysteresis: only re-fit (→ redraw) on a meaningful change, so the
             * heavy canvas repaint isn't run every 20 Hz frame. */
            bool refit = !s_map_have_fit
                || fabsf(ppm - s_map_ppm) > 0.06f * s_map_ppm
                || fabsf(cx - s_map_ctr_x) * s_map_ppm > 24.0f
                || fabsf(cy - s_map_ctr_y) * s_map_ppm > 24.0f;

            uint32_t sig = (uint32_t)nm;
            for (int i = 0; i < nm; i++) {
                sig = sig * 2654435761u
                    + (uint32_t)(int)(s_map_mark_scratch[i].x * 4.0f)
                    + (uint32_t)(int)(s_map_mark_scratch[i].y * 4.0f) * 31u;
            }
            bool dirty = refit || (nc != s_map_last_crumbs) || (sig != s_map_marks_sig);

            if (refit) { s_map_ppm = ppm; s_map_ctr_x = cx; s_map_ctr_y = cy; s_map_have_fit = true; }
            if (dirty) { map_redraw(nc, nm); s_map_last_crumbs = nc; s_map_marks_sig = sig; }

            /* Operator dot + heading arrow — glide every tick over the static layer. */
            int opx, opy;
            map_world_to_px(pose.x, pose.y, &opx, &opy);
            if (s_map_op_dot) {
                lv_obj_set_pos(s_map_op_dot, opx - 7, opy - 7);
                lv_obj_clear_flag(s_map_op_dot, LV_OBJ_FLAG_HIDDEN);
            }
            if (s_map_op_arrow) {
                const float L = 20.0f;
                s_map_arrow_pts[0].x = opx;
                s_map_arrow_pts[0].y = opy;
                s_map_arrow_pts[1].x = (lv_value_precise_t)(opx + L * sinf(pose.heading));
                s_map_arrow_pts[1].y = (lv_value_precise_t)(opy - L * cosf(pose.heading));
                lv_line_set_points(s_map_op_arrow, s_map_arrow_pts, 2);
                lv_obj_clear_flag(s_map_op_arrow, LV_OBJ_FLAG_HIDDEN);
            }

            /* Header status, throttled ~2 Hz. */
            if (s_map_info && (st->tick % 10 == 0)) {
                float hdg = pose.heading * (180.0f / 3.14159265f);
                hdg = fmodf(hdg, 360.0f); if (hdg < 0.0f) hdg += 360.0f;
                int across = (int)((float)MAP_W / s_map_ppm + 0.5f);
                char tb[64];
                snprintf(tb, sizeof(tb), "HDG %03d   ACROSS %dm   PATH %d",
                         (int)(hdg + 0.5f), across, nc);
                label_set_text_cached(s_map_info, tb);
            }
        }
    }

    if (s_cur_kind == PK_MOTION && s_motion_pulse && s_pulse_buf) {
        /* Wavy Halo pulse: a banded sweep expands apex->rim and repeats. Advance the
         * leading edge in REAL-WORLD range (m), projected through ppm so it stays locked
         * to the rings/blips as the scale animates (~6 px/tick). Then repaint + invalidate
         * only the tight band bbox (previous to clear + current to draw), keeping the
         * dirty area proportional to band thickness, not the expanding circle. */
        s_pulse_lead_m += 6.0f / s_motion_ppm;
        float lead_px = s_pulse_lead_m * s_motion_ppm;
        if (lead_px > (float)(FAN_R + PULSE_BAND * PULSE_WIDTH)) { s_pulse_lead_m = 0.0f; lead_px = 0.0f; }
        s_pulse_lead_px = (int)lead_px;

        const int PAD = 6;                                   /* scallop + slope rounding */
        int r_out = s_pulse_lead_px; if (r_out > FAN_R) { r_out = FAN_R; }
        int r_in  = s_pulse_lead_px - (PULSE_BAND - 1) * PULSE_WIDTH; if (r_in < 0) { r_in = 0; }
        int xext  = (int)(0.8660254f * r_out + 0.999f);      /* |cos210|=|cos330| */
        /* canvas-local band bbox (canvas is at rbox 0,0 so local == rbox coords) */
        lv_area_t cur = {
            .x1 = FAN_APEX_X - xext - PAD,
            .x2 = FAN_APEX_X + xext + PAD,
            .y1 = FAN_APEX_Y - r_out - PAD,                  /* topmost at 270deg */
            .y2 = FAN_APEX_Y - (r_in / 2) + PAD,             /* lowest at 210/330, inner r */
        };

        /* Repaint the buffer: clear previous + current band rects, then paint. */
        if (s_pulse_prev_valid) { pulse_clear_rect(&s_pulse_prev_local); }
        pulse_clear_rect(&cur);
        if (r_out > 0) { pulse_paint_band(&cur, r_out, r_in, s_pulse_phase); }
        s_pulse_phase = (s_pulse_phase + 2) & 255;           /* travelling scallop */

        /* Invalidate the same rects in absolute coords (canvas-local + canvas origin). */
        lv_area_t oc; lv_obj_get_coords(s_motion_pulse, &oc);
        if (s_pulse_prev_valid) {
            lv_area_t pabs = { s_pulse_prev_local.x1 + oc.x1, s_pulse_prev_local.y1 + oc.y1,
                               s_pulse_prev_local.x2 + oc.x1, s_pulse_prev_local.y2 + oc.y1 };
            lv_obj_invalidate_area(s_motion_pulse, &pabs);
        }
        if (r_out > 0) {
            lv_area_t cabs = { cur.x1 + oc.x1, cur.y1 + oc.y1, cur.x2 + oc.x1, cur.y2 + oc.y1 };
            lv_obj_invalidate_area(s_motion_pulse, &cabs);
        }
        s_pulse_prev_local = cur;
        s_pulse_prev_valid = (r_out > 0);

        /* Update target blips from LD2450 cache, placed within the fan. */
        prop_motion_target_t tgts[3];
        int cnt = prop_motion_get_targets(tgts, 3);

        /* --- Autorange: pick a ladder level from the furthest target, debounce,
         * then ease s_motion_ppm toward it. One deliberate zoom per threshold. --- */
        float furthest_m = 0.0f;
        for (int i = 0; i < cnt; i++) {
            float fx = (float)tgts[i].x_mm, fy = (float)tgts[i].y_mm;
            float dm = sqrtf(fx * fx + fy * fy) / 1000.0f;
            if (dm > furthest_m) furthest_m = dm;
        }
        int desired = motion_desired_level(furthest_m, cnt > 0);
        if (desired == s_motion_level) {
            s_motion_dwell = 0;
        } else {
            if (desired != s_motion_level_pend) { s_motion_level_pend = desired; s_motion_dwell = 0; }
            else                                 { s_motion_dwell++; }
            /* 0.4 s dwell for target-driven changes; 3 s before the idle decay to 6 m. */
            int need = (desired == MOTION_NLEVEL - 1 && cnt == 0) ? 60 : 8;
            if (s_motion_dwell >= need) {
                s_motion_level    = desired;
                s_motion_ppm_from = s_motion_ppm;
                s_motion_ppm_to   = (float)FAN_R / MOTION_LADDER[desired];
                s_motion_anim_ms  = 0;
                s_motion_dwell    = 0;
            }
        }
        if (s_motion_anim_ms >= 0) {            /* ~50 ms per observer tick (20 Hz) */
            s_motion_anim_ms += 50;
            float t = s_motion_anim_ms / 500.0f;   /* 500 ms slide */
            if (t >= 1.0f) { t = 1.0f; s_motion_anim_ms = -1; }
            float e = t * t * (3.0f - 2.0f * t);   /* smoothstep ease-in-out */
            s_motion_ppm = s_motion_ppm_from + (s_motion_ppm_to - s_motion_ppm_from) * e;
        }

        motion_layout_grid();

        /* Sample decimation for trails: push ~every 3rd tick => ~1.2 s of history. */
        s_trail_tick++;
        int trail_push = (s_trail_tick % 3 == 0);

        for (int i = 0; i < 3; i++) {
            if (!s_motion_blips[i]) break;
            if (i < cnt) {
                int bx, by;
                motion_target_screen_pos(tgts[i].x_mm, tgts[i].y_mm, &bx, &by);
                lv_obj_set_pos(s_motion_blips[i], bx - 6, by - 6);
                lv_obj_set_style_bg_color(s_motion_blips[i],
                    (tgts[i].speed_mm_s > 2000 || tgts[i].speed_mm_s < -2000)
                        ? COL_ALERT : COL_AMBER, 0);
                lv_obj_clear_flag(s_motion_blips[i], LV_OBJ_FLAG_HIDDEN);

                if (trail_push) {
                    s_trail_head[i] = (s_trail_head[i] + 1) % TRAIL_LEN;
                    s_trail_x[i][s_trail_head[i]] = tgts[i].x_mm;
                    s_trail_y[i][s_trail_head[i]] = tgts[i].y_mm;
                    if (s_trail_n[i] < TRAIL_LEN) s_trail_n[i]++;
                }
            } else {
                lv_obj_add_flag(s_motion_blips[i], LV_OBJ_FLAG_HIDDEN);
                s_trail_n[i] = 0;
            }

            /* Comet trail: k=0 newest (brightest/biggest) -> oldest faint/small.
             * Re-projected each tick so it rescales smoothly during autorange. */
            for (int k = 0; k < TRAIL_LEN; k++) {
                lv_obj_t *d = s_motion_trail[i][k];
                if (!d) break;
                if (k < s_trail_n[i]) {
                    int idx = (s_trail_head[i] - k + TRAIL_LEN) % TRAIL_LEN;
                    int tx, ty;
                    motion_target_screen_pos(s_trail_x[i][idx], s_trail_y[i][idx], &tx, &ty);
                    int sz  = 8 - (6 * k) / (TRAIL_LEN - 1);       /* 8 -> 2 px */
                    int opa = 170 - (170 * k) / (TRAIL_LEN - 1);   /* 170 -> 0 */
                    lv_obj_set_size(d, sz, sz);
                    lv_obj_set_pos(d, tx - sz / 2, ty - sz / 2);
                    lv_obj_set_style_bg_opa(d, (lv_opa_t)opa, 0);
                    lv_obj_clear_flag(d, LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }

        /* Merged headline: count folded into the alert text, alert-red when present. */
        if (s_motion_tgt_label) {
            char cb[40];
            if (cnt > 0)
                snprintf(cb, sizeof(cb), "%d TARGET%s DETECTED", cnt, cnt == 1 ? "" : "S");
            else
                snprintf(cb, sizeof(cb), "NO TARGETS");
            label_set_text_cached(s_motion_tgt_label, cb);
            lv_obj_set_style_text_color(s_motion_tgt_label, cnt > 0 ? COL_ALERT : COL_MUTE, 0);
        }

        /* Update T1/T2/T3 three-column target readout. */
        for (int i = 0; i < 3; i++) {
            if (!s_motion_tdist[i]) break;
            if (i < cnt) {
                float fx = (float)tgts[i].x_mm, fy = (float)tgts[i].y_mm;
                int dm = (int)(sqrtf(fx * fx + fy * fy) / 100.0f + 0.5f); /* tenths of m */
                char dbuf[20];
                snprintf(dbuf, sizeof(dbuf), "%d.%dm", dm / 10, dm % 10);
                label_set_text_cached(s_motion_tdist[i], dbuf);
                lv_obj_set_style_text_color(s_motion_tdist[i], COL_AMBER, 0);

                int spd = (int)tgts[i].speed_mm_s;
                char vbuf[14];
                snprintf(vbuf, sizeof(vbuf), "%+d mm/s", spd);
                label_set_text_cached(s_motion_tvel[i], vbuf);
                lv_obj_set_style_text_color(s_motion_tvel[i],
                    spd > 0 ? COL_AMBER : COL_MUTE, 0);

                /* Bearing arrow — atan2(x, y): ang=0 straight ahead, +π/2 right */
                float ang = atan2f(fx, fy + 0.001f);
                s_motion_apts[i][0].x = 119; s_motion_apts[i][0].y = 137;
                s_motion_apts[i][1].x = 119 + (lv_value_precise_t)(sinf(ang) * 17.0f);
                s_motion_apts[i][1].y = 137 - (lv_value_precise_t)(cosf(ang) * 17.0f);
                lv_line_set_points(s_motion_tarrow[i], s_motion_apts[i], 2);
                lv_obj_clear_flag(s_motion_tarrow[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                label_set_text_cached(s_motion_tdist[i], "--.-m");
                lv_obj_set_style_text_color(s_motion_tdist[i], COL_DIM, 0);
                label_set_text_cached(s_motion_tvel[i], "");
                lv_obj_add_flag(s_motion_tarrow[i], LV_OBJ_FLAG_HIDDEN);
            }
        }

        /* Horizon-style attitude indicators (YAW / PITCH / ROLL). Box-local
         * geometry: 148x170 cell, crosshair centre (74,96). */
        if (s_motion_gdial[0]) {
            float pitch_deg = 0.0f, roll_deg = 0.0f, yaw_deg = 0.0f;
            prop_imu_get_orientation(&pitch_deg, &roll_deg, &yaw_deg);

            /* YAW: heading bug — vertical line slides L/R with heading (±180°). */
            float yw = yaw_deg;
            while (yw > 180.0f) yw -= 360.0f;
            while (yw < -180.0f) yw += 360.0f;
            int yx = 74 + (int)(yw / 180.0f * 62.0f);
            if (yx < 10)  yx = 10;
            if (yx > 138) yx = 138;
            s_motion_gpts[0][0].x = yx; s_motion_gpts[0][0].y = 24;
            s_motion_gpts[0][1].x = yx; s_motion_gpts[0][1].y = 164;
            lv_line_set_points(s_motion_gdial[0], s_motion_gpts[0], 2);

            /* PITCH: horizon climbs/dives — horizontal line shifts vertically. */
            int py = 96 + (int)(pitch_deg * 1.4f);
            if (py < 26)  py = 26;
            if (py > 162) py = 162;
            s_motion_gpts[1][0].x = 8;   s_motion_gpts[1][0].y = py;
            s_motion_gpts[1][1].x = 140; s_motion_gpts[1][1].y = py;
            lv_line_set_points(s_motion_gdial[1], s_motion_gpts[1], 2);

            /* ROLL: horizon tilts — line through centre rotated by roll. */
            float rr = roll_deg * (3.14159265f / 180.0f);
            int dx = (int)(cosf(rr) * 64.0f), dy = (int)(sinf(rr) * 64.0f);
            s_motion_gpts[2][0].x = 74 - dx; s_motion_gpts[2][0].y = 96 + dy;
            s_motion_gpts[2][1].x = 74 + dx; s_motion_gpts[2][1].y = 96 - dy;
            lv_line_set_points(s_motion_gdial[2], s_motion_gpts[2], 2);

            float vals[3] = { yaw_deg, pitch_deg, roll_deg };
            for (int k = 0; k < 3; k++)
                if (s_motion_gval[k])
                    lv_label_set_text_fmt(s_motion_gval[k], "%d\xc2\xb0", (int)vals[k]);
        }

        /* 3 strongest BLE contacts — throttled to 2 Hz (every 10 ticks at 20 Hz). */
        if (st->tick % 10 == 0 && s_motion_ble_box[0]) {
            prop_ble_dev_t bd[3];
            int bn = prop_ble_get_devices(bd, 3);   /* strongest RSSI first */
            for (int i = 0; i < 3; i++) {
                if (i < bn) {
                    char nm[24];
                    ble_contact_name(nm, sizeof(nm), &bd[i]);
                    label_set_text_cached(s_motion_ble_name[i], nm);
                    lv_obj_set_style_text_color(s_motion_ble_name[i], COL_AMBER, 0);

                    char db[16];
                    ble_fmt_dist(db, sizeof(db), prop_ble_distance_m(bd[i].rssi, bd[i].tx_power));
                    label_set_text_cached(s_motion_ble_dist[i], db);
                    lv_obj_set_style_text_color(s_motion_ble_dist[i], COL_AMBER, 0);

                    char rb[16];
                    snprintf(rb, sizeof(rb), "%d dBm", bd[i].rssi);
                    label_set_text_cached(s_motion_ble_dbm[i], rb);
                } else {
                    label_set_text_cached(s_motion_ble_name[i], "----");
                    lv_obj_set_style_text_color(s_motion_ble_name[i], COL_DIM, 0);
                    label_set_text_cached(s_motion_ble_dist[i], "-- m");
                    lv_obj_set_style_text_color(s_motion_ble_dist[i], COL_DIM, 0);
                    label_set_text_cached(s_motion_ble_dbm[i], "");
                }
            }
        }

        /* All-sensor dashboard: boolean status words (dim/bright). */
        if (s_motion_flags[0]) {
            bool csi_motion = false; int mvf = 0, thf = 1;
            prop_csi_get_motion(&csi_motion, &mvf, &thf);
            set_flag_word(s_motion_flags[0], csi_motion);
            set_flag_word(s_motion_flags[1], prop_imu_available());
            set_flag_word(s_motion_flags[2], prop_mic_pdm_up());
            set_flag_word(s_motion_flags[3], prop_ble_available());
            set_flag_word(s_motion_flags[4], prop_net_sta_state() == STA_CONNECTED);
        }

        /* Analog sensor bars (ACC is set below from gravity-removed linear accel). */
        if (s_motion_bars[0]) {
            bool m = false; int mv = 0, th = 1;
            prop_csi_get_motion(&m, &mv, &th);
            int csi_pct = clamp_pct(th > 0 ? (mv * 100 / th) : 0);
            int turb = 0, agc = 0, fftg = 0; bool lk = false;
            prop_csi_get_rf(&turb, &agc, &fftg, &lk);
            /* turbulence (raw spatial std) reads ~6500-7000 milli at idle and rises
             * with RF disturbance; scale so idle sits low-mid with headroom to peak. */
            int turb_pct = clamp_pct(turb / 200);
            int mic_pct = clamp_pct(prop_mic_get_db() + 90);
            int rssi = prop_net_get_rssi();
            int sig_pct = clamp_pct((rssi >= 0) ? 0 : rssi + 100);
            lv_bar_set_value(s_motion_bars[0], csi_pct,  LV_ANIM_OFF);
            lv_bar_set_value(s_motion_bars[1], turb_pct, LV_ANIM_OFF);
            lv_bar_set_value(s_motion_bars[2], mic_pct,  LV_ANIM_OFF);
            lv_bar_set_value(s_motion_bars[3], sig_pct,  LV_ANIM_OFF);

            /* FRQ: loudest audio frequency. Find the strongest mic FFT band and map
             * its centre Hz across the source's range (0..Nyquist) to the bar. Gated
             * on a minimum band magnitude so silence reads empty, not noise-driven. */
            if (s_motion_bars[5]) {
                uint8_t bands[PROP_MIC_BANDS];
                prop_mic_get_bands(bands);
                int peak_b = 0, peak_v = 0;
                for (int b = 0; b < PROP_MIC_BANDS; b++)
                    if (bands[b] > peak_v) { peak_v = bands[b]; peak_b = b; }
                int frq_pct = 0;
                if (peak_v >= 10) {                       /* above the noise floor */
                    int nyq = prop_mic_sample_rate() / 2;
                    if (nyq > 0) frq_pct = clamp_pct(prop_mic_band_hz(peak_b) * 100 / nyq);
                }
                lv_bar_set_value(s_motion_bars[5], frq_pct, LV_ANIM_OFF);
            }
        }

        /* 4-quadrant direction ring: light the quadrant of TRAVEL, not tilt.
         * Gravity is estimated as a slow low-pass of the accelerometer itself, so
         * subtracting it leaves only dynamic (travel) acceleration. This is immune
         * to the DMP quaternion convention/bias that made the old version twitch at
         * rest — when still, the low-pass equals the reading and the residual is ~0. */
        if (s_motion_dir_q[0]) {
            prop_imu_data_t im; prop_imu_get_data(&im);
            float axg = (float)im.ax / 16384.0f;
            float ayg = (float)im.ay / 16384.0f;
            float azg = (float)im.az / 16384.0f;
            if (!s_motion_grav_primed) {
                s_motion_grav_lp[0] = axg; s_motion_grav_lp[1] = ayg; s_motion_grav_lp[2] = azg;
                s_motion_grav_primed = true;
            } else {
                const float a = 0.05f;   /* ~1 s time constant at 20 Hz */
                s_motion_grav_lp[0] += a * (axg - s_motion_grav_lp[0]);
                s_motion_grav_lp[1] += a * (ayg - s_motion_grav_lp[1]);
                s_motion_grav_lp[2] += a * (azg - s_motion_grav_lp[2]);
            }
            float lax = axg - s_motion_grav_lp[0];   /* linear accel (g), gravity removed */
            float lay = ayg - s_motion_grav_lp[1];
            float laz = azg - s_motion_grav_lp[2];
            float lmag = sqrtf(lax * lax + lay * lay);
            int lit = -1;                          /* -1 = not moving (deadband) */
            if (lmag > 0.08f) {
                /* Rotate the board-frame accel by the calibrated yaw so "forward"
                 * (radar boresight) lights North. s_dir_phi == 0 -> legacy identity. */
                float c = cosf(s_dir_phi), s = sinf(s_dir_phi);
                float rx = lax * c - lay * s;
                float ry = lax * s + lay * c;
                float ang = atan2f(ry, rx);        /* -pi..pi */
                if      (ang >= -0.785f && ang <  0.785f) lit = 1;  /* +x -> E */
                else if (ang >=  0.785f && ang <  2.356f) lit = 0;  /* +y -> N */
                else if (ang >= -2.356f && ang < -0.785f) lit = 2;  /* -y -> S */
                else                                      lit = 3;  /* -x -> W */
            }
            for (int i = 0; i < 4; i++) {
                lv_color_t c = (i == lit) ? COL_AMBER : COL_DIM;
                lv_obj_set_style_arc_color(s_motion_dir_q[i], c, LV_PART_MAIN);
                lv_obj_set_style_arc_color(s_motion_dir_q[i], c, LV_PART_INDICATOR);
            }

            /* ACC bar: magnitude of the gravity-removed linear accel — reads ~0 at
             * rest (gravity no longer counts as acceleration) and rises with motion. */
            if (s_motion_bars[4]) {
                float lmag3 = sqrtf(lax * lax + lay * lay + laz * laz);
                lv_bar_set_value(s_motion_bars[4], clamp_pct((int)(lmag3 * 120.0f)), LV_ANIM_OFF);
            }
        }

        /* Distance ping: chirp the closest radar target's range when enabled — pitch
         * and repetition rate both rise as the target gets nearer. Closest target is
         * the smallest range among the live radar targets (not BLE). */
        if (s_scan_ping_en && prop_audio_available()) {
            float best_m = 1e9f;
            for (int i = 0; i < cnt; i++) {
                float fx = (float)tgts[i].x_mm, fy = (float)tgts[i].y_mm;
                float dm = sqrtf(fx * fx + fy * fy) / 1000.0f;
                if (dm < best_m) best_m = dm;
            }
            if (cnt > 0 && best_m < 6.5f) {
                if (st->tick >= s_scan_ping_next) {
                    /* Closer -> higher pitch (capped, mellow) and shorter interval. */
                    float t = (6.0f - best_m) / 6.0f;          /* 0 far .. 1 near */
                    if (t < 0.0f) t = 0.0f;
                    if (t > 1.0f) t = 1.0f;
                    int semis = (int)(t * 7.0f + 0.5f);        /* 0..7 semitones */
                    int interval = (int)(28.0f - t * 22.0f);   /* ~28 (1.4s) .. ~6 (0.3s) ticks */
                    if (interval < 5) interval = 5;
                    prop_audio_play_pitched(PA_PING, semis);
                    s_scan_ping_next = st->tick + interval;
                }
            } else {
                s_scan_ping_next = st->tick;   /* no target — ready to chirp immediately */
            }
        }
    }

    /* Drive the travel-direction calibration flow (PK_DIRCAL). Captures one
     * hemisphere-aligned linear-accel accumulator per cardinal walk, then solves for
     * the board->world yaw and persists it. */
    if (s_cur_kind == PK_DIRCAL && s_dcal_prompt) {
        if (s_dcal_phase >= 0 && s_dcal_phase <= 3) {
            /* Gravity-removed horizontal accel (same low-pass trick as the ring). */
            prop_imu_data_t im; prop_imu_get_data(&im);
            float axg = (float)im.ax / 16384.0f;
            float ayg = (float)im.ay / 16384.0f;
            float azg = (float)im.az / 16384.0f;
            if (!s_dcal_grav_primed) {
                s_dcal_grav_lp[0] = axg; s_dcal_grav_lp[1] = ayg; s_dcal_grav_lp[2] = azg;
                s_dcal_grav_primed = true;
            } else {
                const float a = 0.05f;
                s_dcal_grav_lp[0] += a * (axg - s_dcal_grav_lp[0]);
                s_dcal_grav_lp[1] += a * (ayg - s_dcal_grav_lp[1]);
                s_dcal_grav_lp[2] += a * (azg - s_dcal_grav_lp[2]);
            }
            float lx = axg - s_dcal_grav_lp[0], ly = ayg - s_dcal_grav_lp[1];
            float m = sqrtf(lx * lx + ly * ly);
            if (m > 0.06f) {
                /* Fold push + stop into one direction: flip samples to the hemisphere
                 * of the first significant push so they reinforce instead of cancel. */
                float vx = lx, vy = ly;
                if (!s_dcal_ref_set) {
                    s_dcal_ref[0] = vx; s_dcal_ref[1] = vy; s_dcal_ref_set = true;
                } else if (vx * s_dcal_ref[0] + vy * s_dcal_ref[1] < 0.0f) {
                    vx = -vx; vy = -vy;
                }
                s_dcal_acc[s_dcal_phase][0] += vx;
                s_dcal_acc[s_dcal_phase][1] += vy;
            }

            label_set_text_cached(s_dcal_prompt, DCAL_NAME[s_dcal_phase]);
            lv_obj_set_style_text_color(s_dcal_prompt, COL_AMBER, 0);
            int secs = (DCAL_PHASE_MS - s_dcal_ms + 999) / 1000;
            char sb[40];
            snprintf(sb, sizeof(sb), "CAPTURING...  %d", secs < 0 ? 0 : secs);
            label_set_text_cached(s_dcal_sub, sb);
            dcal_set_arrow(s_dcal_phase);
            if (s_dcal_btn_lbl) label_set_text_cached(s_dcal_btn_lbl, "CANCEL");

            s_dcal_ms += 50;
            if (s_dcal_ms >= DCAL_PHASE_MS) {
                s_dcal_phase++;
                s_dcal_ms = 0;
                s_dcal_ref_set = false;
                prop_audio_play(PA_TAB);
                if (s_dcal_phase == 4) {
                    /* Forward axis = forward-push minus backward-push; reinforce with
                     * the left/right axis rotated +90° (board "east" -> forward). */
                    float fx = s_dcal_acc[0][0] - s_dcal_acc[1][0];
                    float fy = s_dcal_acc[0][1] - s_dcal_acc[1][1];
                    float ex = s_dcal_acc[3][0] - s_dcal_acc[2][0];
                    float ey = s_dcal_acc[3][1] - s_dcal_acc[2][1];
                    float Fx = fx + (-ey);   /* rotate (ex,ey) by +90°: (x,y)->(-y,x) */
                    float Fy = fy + ( ex);
                    if (sqrtf(Fx * Fx + Fy * Fy) < DCAL_MIN_MAG) {
                        s_dcal_phase = 5;    /* not enough motion — offer a retry */
                        prop_audio_play(PA_DENY);
                    } else {
                        float theta = atan2f(Fy, Fx);
                        s_dir_phi = (float)M_PI / 2.0f - theta;   /* forward -> North */
                        while (s_dir_phi >  (float)M_PI) s_dir_phi -= 2.0f * (float)M_PI;
                        while (s_dir_phi < -(float)M_PI) s_dir_phi += 2.0f * (float)M_PI;
                        prop_settings_set_u32("dir_phi",
                            (uint32_t)(int32_t)(s_dir_phi * 1000.0f));
                        prop_track_set_dir_phi(s_dir_phi);   /* path heading tracks the recal */
                        prop_audio_play(PA_SIGNAL);
                    }
                }
            }
        } else if (s_dcal_phase == 4) {
            label_set_text_cached(s_dcal_prompt, "CALIBRATED");
            lv_obj_set_style_text_color(s_dcal_prompt, COL_AMBER, 0);
            char sb[48];
            snprintf(sb, sizeof(sb), "FORWARD OFFSET %d\xc2\xb0", (int)(s_dir_phi * 57.2958f));
            label_set_text_cached(s_dcal_sub, sb);
            if (s_dcal_arrow) lv_obj_add_flag(s_dcal_arrow, LV_OBJ_FLAG_HIDDEN);
            if (s_dcal_btn_lbl) label_set_text_cached(s_dcal_btn_lbl, "DONE");
        } else if (s_dcal_phase == 5) {
            label_set_text_cached(s_dcal_prompt, "NOT ENOUGH MOTION");
            lv_obj_set_style_text_color(s_dcal_prompt, COL_ALERT, 0);
            label_set_text_cached(s_dcal_sub, "WALK A FEW STEPS EACH WAY");
            if (s_dcal_arrow) lv_obj_add_flag(s_dcal_arrow, LV_OBJ_FLAG_HIDDEN);
            if (s_dcal_btn_lbl) label_set_text_cached(s_dcal_btn_lbl, "RETRY");
        } else {   /* idle (-1) */
            label_set_text_cached(s_dcal_prompt, "DIRECTION SETUP");
            lv_obj_set_style_text_color(s_dcal_prompt, COL_MUTE, 0);
            label_set_text_cached(s_dcal_sub, "PRESS START, THEN WALK AS PROMPTED");
            if (s_dcal_arrow) lv_obj_add_flag(s_dcal_arrow, LV_OBJ_FLAG_HIDDEN);
            if (s_dcal_btn_lbl) label_set_text_cached(s_dcal_btn_lbl, "START");
        }
    }

    lvgl_port_unlock();
}

void prop_ui_goto(const char *screen)
{
    if (!screen || !lvgl_port_lock(500)) {
        return;
    }
    if (strcmp(screen, "home") == 0)         open_panel(PK_HOME);
    else if (strcmp(screen, "scanner") == 0) open_panel(PK_NONE);
    else if (strcmp(screen, "menu") == 0)    open_panel(PK_MENU);
    else if (strcmp(screen, "wifi") == 0)    open_panel(PK_WIFI);
    else if (strcmp(screen, "display") == 0) open_panel(PK_DISPLAY);
    else if (strcmp(screen, "audio") == 0)   open_panel(PK_AUDIO);
    else if (strcmp(screen, "leds") == 0)    open_panel(PK_LEDS);
    else if (strcmp(screen, "about") == 0)   open_panel(PK_ABOUT);
    else if (strcmp(screen, "vitals") == 0)  open_panel(PK_VITALS);
    else if (strcmp(screen, "scan") == 0)    open_panel(PK_SCAN);
    else if (strcmp(screen, "spectrum") == 0) open_panel(PK_SPECTRUM);
    else if (strcmp(screen, "rfband") == 0)  open_panel(PK_RFBAND);
    else if (strcmp(screen, "ble") == 0)     open_panel(PK_BLE);
    else if (strcmp(screen, "csi") == 0)     open_panel(PK_CSI);
    else if (strcmp(screen, "csicfg") == 0)  open_panel(PK_CSICFG);
    else if (strcmp(screen, "csiset") == 0)  open_panel(PK_CSISET);
    else if (strcmp(screen, "instruments") == 0) open_panel(PK_INSTRUMENTS);
    else if (strcmp(screen, "sensors") == 0)     open_panel(PK_SENSORS);
    else if (strcmp(screen, "motion") == 0)      open_panel(PK_MOTION);
    else if (strcmp(screen, "dircal") == 0)      open_panel(PK_DIRCAL);
    else if (strcmp(screen, "minimap") == 0)     open_panel(PK_MINIMAP);
    else if (strcmp(screen, "range") == 0)       open_panel(PK_RANGE);
    else if (strcmp(screen, "archive") == 0) open_panel(PK_ARCHIVE);
    else if (strcmp(screen, "cassette") == 0) open_panel(PK_CASSETTE);
    else if (strcmp(screen, "insights") == 0) open_panel(PK_INSIGHTS);
    else if (strcmp(screen, "io") == 0)       open_panel(PK_IO);
    /* Per-pin deep-link for the screenshot loop: "io27" opens the IO27 config page. */
    else if (strncmp(screen, "io", 2) == 0 && isdigit((unsigned char)screen[2])) {
        int gpio = atoi(screen + 2);
        for (int i = 0; i < bsp_aio_count(); i++) {
            if (bsp_aio_info(i)->gpio == gpio) { s_io_pin = i; open_panel(PK_IO_PIN); break; }
        }
    }
    lvgl_port_unlock();
    ESP_LOGI(UI_TAG, "goto screen: %s", screen);
}

const char *prop_ui_current_screen(void)
{
    switch (s_cur_kind) {
        case PK_NONE:        return "scanner";
        case PK_HOME:        return "home";
        case PK_MENU:        return "menu";
        case PK_WIFI:        return "wifi";
        case PK_DISPLAY:     return "display";
        case PK_AUDIO:       return "audio";
        case PK_LEDS:        return "leds";
        case PK_ABOUT:       return "about";
        case PK_VITALS:      return "vitals";
        case PK_SCAN:        return "scan";
        case PK_SPECTRUM:    return "spectrum";
        case PK_RFBAND:      return "rfband";
        case PK_BLE:         return "ble";
        case PK_CSI:         return "csi";
        case PK_CSICFG:      return "csicfg";
        case PK_CSISET:      return "csiset";
        case PK_MOTION:      return "motion";
        case PK_DIRCAL:      return "dircal";
        case PK_MINIMAP:     return "minimap";
        case PK_INSTRUMENTS: return "instruments";
        case PK_SENSORS:     return "sensors";
        case PK_ARCHIVE:     return "archive";
        case PK_ARTICLE:     return "article";
        case PK_CASSETTE:    return "cassette";
        case PK_INSIGHTS:    return "insights";
        case PK_IO:          return "io";
        case PK_IO_PIN:      return "io_pin";
        default:             return "unknown";
    }
}

/* ---- Physical-control input (SELECTOR dial / TAB switches / ACTION buttons)
 * The author's nav model, decoupled from hardware: the web portal drives this
 * today via /cmd {"cmd":"input",...}; bsp_io will route the real knobs/switches
 * here once they're wired. Navigation lives in the view (this module), mirroring
 * prop_ui_goto - the engine stays a pure behavior model. */

/* SELECTOR rotation: context-dependent. */
static void nav_select_move(int dir)
{
    switch (s_cur_kind) {
        case PK_HOME:
            s_rail_sel = (s_rail_sel + dir + RAIL_COUNT) % RAIL_COUNT;
            set_rail_highlight();
            break;
        case PK_ARCHIVE:
            if (s_arch_row_count > 0) {
                s_archive_entry = (s_archive_entry + dir + s_arch_row_count) % s_arch_row_count;
                set_arch_row_highlight();
            }
            break;
        case PK_ARTICLE:
            if (s_article_scroll) {
                lv_obj_scroll_by(s_article_scroll, 0, dir > 0 ? -80 : 80, LV_ANIM_ON);
            }
            break;
        default:
            break;
    }
}

/* SELECTOR press / ACTION primary: open the highlighted thing, or step back. */
static void nav_select_press(void)
{
    switch (s_cur_kind) {
        case PK_HOME:    open_panel(s_rail[s_rail_sel].kind); break;
        case PK_ARCHIVE: open_panel(PK_ARTICLE); break;
        case PK_ARTICLE: open_panel(PK_ARCHIVE); break;
        case PK_NONE:    open_panel(PK_HOME); break;   /* from the SCANNER readout */
        default:         open_panel(PK_HOME); break;
    }
}

/* TAB switch: archive section selection (opens/switches the ARCHIVE). */
static void nav_tab(int n)
{
    s_archive_section = clamp_section(n);
    s_archive_entry = 0;
    open_panel(PK_ARCHIVE);
}

void prop_ui_input(const char *control, int arg)
{
    if (!control || !lvgl_port_lock(300)) {
        return;
    }
    /* Only the dial *rotation* needs a sound here — it changes the highlight/scroll
     * without a screen swap. Press / back / tab all route through open_panel(), which
     * plays the screen-change clack, so emitting a tone here too would double up. */
    int sfx = -1;                          /* feedback tone, played after the lock drops */
    if (strcmp(control, "selector") == 0) {
        if (arg == 0) {
            nav_select_press();
        } else {
            nav_select_move(arg > 0 ? 1 : -1);
            sfx = PA_DIAL_TICK;
        }
    } else if (strcmp(control, "tab") == 0) {
        nav_tab(arg);
    } else if (strcmp(control, "action") == 0) {
        if (arg == 2) {
            open_panel(PK_HOME);      /* universal back-to-console */
        } else {
            nav_select_press();        /* action 1 = primary / select */
        }
    }
    lvgl_port_unlock();
    /* Play outside the LVGL lock (prop_audio only enqueues, but the spec keeps audio
     * off the lock; harmless if dropped when audio is unavailable). */
    if (sfx >= 0) {
        prop_audio_play((prop_audio_event_t)sfx);
    }
    ESP_LOGI(UI_TAG, "input %s %d", control, arg);
}

/* ---- FPS meter (optional dev HUD, top-right) ------------------------------
 * Counts genuinely rendered frames via the display monitor_cb (called once per
 * refresh that draws). When active it drops the refresh-timer period from the
 * 30 ms default (33 fps cap) to 8 ms, so animations that invalidate small regions
 * can run up to ~60 fps. It deliberately does NOT force a full-screen redraw every
 * frame: a whole-screen software render at 1024x600 takes ~250 ms (≈4 fps) and
 * saturates the CPU, so forcing it is counter-productive. The counter therefore
 * reflects real render activity - high during motion, low when the screen is idle.
 *
 * The HUD is an OPAQUE child of the active screen, NOT lv_layer_top - a
 * translucent top-layer object made LVGL recomposite the whole layer each frame
 * and thrashed lv_mem_buf_get into a watchdog hang.
 *
 * It is a PASSIVE counter: it does not force a faster refresh. Dropping the
 * refresh period was tried and lifted nothing (the framerate is bound by the
 * full-screen double-buffered pipeline + Wi-Fi CPU contention, not the refresh
 * cap) while making the device network-sluggish. See
 * docs/superpowers/specs/2026-06-20-framerate-investigation-findings.md. */
static lv_obj_t *s_fps_label;
static volatile uint32_t s_fps_frames;
static lv_timer_t *s_fps_timer;     /* 1 Hz: publishes the count */

/* v9 removed lv_disp_drv_t / monitor_cb; count completed renders via a display event. */
static void fps_render_ready_cb(lv_event_t *e)
{
    (void)e;
    s_fps_frames++;
}

static void fps_tick(lv_timer_t *t)
{
    (void)t;
    if (s_fps_label && !lv_obj_has_flag(s_fps_label, LV_OBJ_FLAG_HIDDEN)) {
        lv_label_set_text_fmt(s_fps_label, "FPS %u", (unsigned)s_fps_frames);
    }
    s_fps_frames = 0;   /* refreshes counted in the last ~1 s */
}

/* Build the HUD + install the frame counter. Call once, under the LVGL lock. */
static void fps_init(void)
{
    lv_display_t *d = lv_display_get_default();
    if (d) {
        lv_display_add_event_cb(d, fps_render_ready_cb, LV_EVENT_RENDER_READY, NULL);
    }
    s_fps_label = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_font(s_fps_label, FONT_BODY, 0);
    lv_obj_set_style_text_color(s_fps_label, COL_BG, 0);
    lv_obj_set_style_bg_color(s_fps_label, COL_AMBER, 0);
    lv_obj_set_style_bg_opa(s_fps_label, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_fps_label, 4, 0);
    lv_label_set_text(s_fps_label, "FPS --");
    lv_obj_align(s_fps_label, LV_ALIGN_TOP_RIGHT, -6, 6);
    s_fps_timer = lv_timer_create(fps_tick, 1000, NULL);

    uint32_t on = 0;
    prop_settings_get_u32("fps_on", &on, 0);
    if (!on) {
        lv_obj_add_flag(s_fps_label, LV_OBJ_FLAG_HIDDEN);
    }
}

void prop_ui_set_fps(bool on)
{
    if (!lvgl_port_lock(200)) {
        return;
    }
    if (s_fps_label) {
        if (on) lv_obj_clear_flag(s_fps_label, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_add_flag(s_fps_label, LV_OBJ_FLAG_HIDDEN);
    }
    lvgl_port_unlock();
    prop_settings_set_u32("fps_on", on ? 1 : 0);
}

esp_err_t prop_ui_init(void)
{
    if (!lvgl_port_lock(1000)) {
        ESP_LOGE(UI_TAG, "could not lock LVGL to build UI");
        return ESP_FAIL;
    }
    build_screen();
    fps_init();
    lvgl_port_unlock();

    esp_err_t err = prop_engine_add_observer(ui_observer, NULL);
    ESP_LOGI(UI_TAG, "cassette-futurism UI ready");
    return err;
}
