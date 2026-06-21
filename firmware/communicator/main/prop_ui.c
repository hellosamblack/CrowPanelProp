/* prop_ui — cassette-futurism LVGL readout, driven by prop_engine state. */
#include "prop_ui.h"
#include "prop_engine.h"
#include "prop_net.h"
#include "prop_settings.h"
#include "prop_fx.h"
#include "prop_mic.h"
#include "prop_content.h"
#include "bsp_io.h"
#include "bsp_illuminate.h"
#include <string.h>
#include <stdio.h>
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

/* Eurostile (cassette-futurism typeface) — generated from the TTFs in resources/
 * with the LVGL FontAwesome symbol range merged in, so LV_SYMBOL_* glyphs still
 * render. Regular at 14 px (body), Bold at 24 px (headings/status). */
LV_FONT_DECLARE(eurostile_14);
LV_FONT_DECLARE(eurostile_24);
LV_FONT_DECLARE(eurostile_40);   /* resting status headline */
LV_FONT_DECLARE(eurostile_56);   /* status punch-in (SIGNAL DETECTED / ALERT) */
#define FONT_BODY (&eurostile_14)
#define FONT_HEAD (&eurostile_24)
#define FONT_STATUS (&eurostile_40)   /* the big readout status line */
#define FONT_PUNCH (&eurostile_56)    /* the scene-entry slam */

#define UI_TAG "PROP_UI"
#define SCAN_MAX 20

/* Cassette-futurism palette: amber phosphor on near-black. */
#define COL_BG     lv_color_hex(0x0A0A06)
#define COL_AMBER  lv_color_hex(0xE0B000)
#define COL_DIM    lv_color_hex(0x6B5300)
#define COL_ALERT  lv_color_hex(0xFF3030)
/* Secondary text that still has to read on camera (footer, SETUP, IP): brighter
 * than COL_DIM so it survives a 7" panel + lens, while staying below COL_AMBER. */
#define COL_MUTE   lv_color_hex(0xB58A00)

#define SCREEN_W 1024           /* physical panel width */
#define RAIL_W   76             /* persistent icon nav rail (left edge, all screens) */
/* SCAN_W is the CONTENT width (right of the rail). All panels/readouts lay out
 * relative to this, so redefining it here insets every screen with no per-panel edits. */
#define SCAN_W (SCREEN_W - RAIL_W)
#define SCAN_TRACK_Y 330        /* taller, lower trace — the hero waveform */
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
 * LVGL's heap (LV_MEM, 32 KB) cannot grow — esp_hosted's SDIO DMA mempool needs
 * the internal RAM, so a bigger LV_MEM boot-loops ("HS_MP: mempool ... no mem").
 * Therefore panels are NOT all pre-built: exactly one is alive at a time, built
 * on navigation and torn down on leave. This caps LVGL usage at "main screen +
 * one panel" regardless of how many panel types exist (and scales to new ones). */

#define COL_PANEL_ITEM lv_color_hex(0x141008)

typedef enum {
    PK_NONE = 0,   /* no panel: the bare SCANNER readout on the root screen */
    PK_HOME,       /* the in-world console (default landing screen) */
    PK_MENU, PK_WIFI, PK_DISPLAY, PK_AUDIO, PK_LEDS, PK_ABOUT, PK_VITALS,
    PK_SCAN, PK_SPECTRUM,
    PK_ARCHIVE,    /* data-archive browser (tabs = sections) */
    PK_ARTICLE,    /* a single archive entry */
    PK_CASSETTE,   /* cassette deck (stub) */
    PK_INSIGHTS,   /* insight engine (stub) */
} panel_kind_t;

static lv_obj_t *s_root;          /* the CONTENT container (panels are built on it) */
static lv_obj_t *s_rail_strip;    /* persistent icon nav rail on the real screen (left edge) */
static bool s_ui_ready;           /* true once boot build is done — gates screen transitions */
static lv_obj_t *s_cur_panel;     /* the one live setup panel, or NULL (main) */
static panel_kind_t s_cur_kind;

/* Live value readouts updated by panels / the observer — valid ONLY while the
 * owning panel is the current one (NULLed on teardown). */
static lv_obj_t *s_disp_bright_val, *s_audio_vol_val;
static lv_obj_t *s_fx_scan_val, *s_fx_phos_val, *s_fx_vign_val, *s_fx_refr_val;
static lv_obj_t *s_about_ip, *s_about_uptime;

/* VITALS instrument live readouts (valid only while PK_VITALS is current). */
static lv_obj_t *s_vit_temp, *s_vit_ram, *s_vit_uptime, *s_vit_cell;
static lv_obj_t *s_vit_temp_bar, *s_vit_ram_bar, *s_vit_cell_bar;

/* SIGNAL SCAN instrument (valid only while PK_SCAN is current). */
static lv_obj_t *s_sig_list, *s_sig_status;
static volatile bool s_sig_scanning;

/* SPECTRUM instrument (valid only while PK_SPECTRUM is current). */
static lv_obj_t *s_spec_bars[PROP_MIC_BANDS];
static lv_obj_t *s_spec_db, *s_spec_db_bar, *s_spec_status;
static float s_spec_decay[PROP_MIC_BANDS];   /* peak-hold / slow decay (UI-side) */

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
static lv_obj_t *build_archive_panel(lv_obj_t *parent);
static lv_obj_t *build_article_panel(lv_obj_t *parent);
static lv_obj_t *build_cassette_panel(lv_obj_t *parent);
static lv_obj_t *build_insights_panel(lv_obj_t *parent);
static void wifi_panel_opened(void);
static void start_signal_scan(void);
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
    IC_SPECTRUM, IC_CASSETTE, IC_INSIGHTS, IC_SETUP,
} icon_id_t;

static const struct {
    const char *label;
    panel_kind_t kind;
    icon_id_t    icon;
} s_rail[] = {
    { "CONSOLE",     PK_HOME,     IC_HOME     },
    { "ARCHIVE",     PK_ARCHIVE,  IC_ARCHIVE  },
    { "SCANNER",     PK_NONE,     IC_SCANNER  },   /* reveals the bare readout on the root screen */
    { "VITALS",      PK_VITALS,   IC_VITALS   },
    { "SIGNAL SCAN", PK_SCAN,     IC_SIGNAL   },
    { "SPECTRUM",    PK_SPECTRUM, IC_SPECTRUM },
    { "CASSETTE",    PK_CASSETTE, IC_CASSETTE },
    { "INSIGHTS",    PK_INSIGHTS, IC_INSIGHTS },
    { "SETUP",       PK_MENU,     IC_SETUP    },
};
#define RAIL_COUNT ((int)(sizeof(s_rail) / sizeof(s_rail[0])))

static int s_rail_sel;                 /* highlighted function on the console */
static int s_archive_section;          /* current ARCHIVE tab (0..prop_section_count-1) */
static int s_archive_entry;            /* selected entry within the section */

/* Console (PK_HOME) live readouts — valid only while PK_HOME is the live panel. */
static lv_obj_t *s_home_clock, *s_home_temp, *s_home_link;
static lv_obj_t *s_rail_btns[RAIL_COUNT];   /* rail rows, for dial highlighting */

/* ARCHIVE browser widgets — valid only while PK_ARCHIVE is live. */
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
    s_sig_list = NULL; s_sig_status = NULL;
    s_spec_db = NULL; s_spec_db_bar = NULL; s_spec_status = NULL;
    s_home_clock = NULL; s_home_temp = NULL; s_home_link = NULL;
    /* s_rail_btns are the persistent rail cells on the real screen — NOT children
     * of the torn-down panel, so they survive close_panel and are never nulled. */
    for (int i = 0; i < (int)(sizeof(s_arch_tabs) / sizeof(s_arch_tabs[0])); i++) s_arch_tabs[i] = NULL;
    for (int i = 0; i < (int)(sizeof(s_arch_rows) / sizeof(s_arch_rows[0])); i++) s_arch_rows[i] = NULL;
    s_arch_row_count = 0;
    s_article_scroll = NULL;
    s_connect_pending = false;
}

/* Light the rail cell for the function `kind` belongs to. Sub-panels map to their
 * top-level function (SETUP sub-screens -> SETUP, an article -> ARCHIVE). Runs for
 * every open including PK_NONE (the SCANNER readout), so it precedes the switch. */
static void rail_sync(panel_kind_t kind)
{
    panel_kind_t want = kind;
    switch (kind) {
        case PK_WIFI: case PK_DISPLAY: case PK_AUDIO:
        case PK_LEDS: case PK_ABOUT: case PK_MENU: want = PK_MENU; break;
        case PK_ARTICLE: want = PK_ARCHIVE; break;
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
        case PK_ARCHIVE: s_cur_panel = build_archive_panel(s_root); break;
        case PK_ARTICLE: s_cur_panel = build_article_panel(s_root); break;
        case PK_CASSETTE: s_cur_panel = build_cassette_panel(s_root); break;
        case PK_INSIGHTS: s_cur_panel = build_insights_panel(s_root); break;
        default: break;    /* PK_NONE: no panel — the SCANNER readout shows through */
    }
    s_cur_kind = kind;
    if (kind == PK_WIFI) {
        wifi_panel_opened();
    } else if (kind == PK_SCAN) {
        start_signal_scan();   /* auto-scan on open */
    }
    /* Mask the swap with the configured channel-change transition (skipped during
     * the initial boot build; the overlay was created above so the next render
     * shows it over the already-swapped screen). */
    if (s_ui_ready) {
        prop_fx_transition_play();
    }
}

/* Cassette-futurism styling helpers: amber-on-black, square corners. */
static void style_btn(lv_obj_t *b)
{
    lv_obj_set_style_bg_color(b, COL_PANEL_ITEM, 0);
    lv_obj_set_style_bg_color(b, COL_DIM, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_color(b, COL_AMBER, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_radius(b, 0, 0);
    lv_obj_set_style_text_color(b, COL_AMBER, 0);
}

static void style_field(lv_obj_t *f)
{
    lv_obj_set_style_bg_color(f, COL_PANEL_ITEM, 0);
    lv_obj_set_style_border_color(f, COL_DIM, 0);
    lv_obj_set_style_border_width(f, 1, 0);
    lv_obj_set_style_radius(f, 0, 0);
    lv_obj_set_style_text_color(f, COL_AMBER, 0);
}

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
    if (xTaskCreate(scan_task, "wifi_scan", 4096, NULL, 4, NULL) != pdPASS) {
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

/* One row in the global SETUP menu. */
static void menu_item(lv_obj_t *menu, const char *text, int idx, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_btn_create(menu);
    lv_obj_set_size(b, SCAN_W - 120, 56);
    lv_obj_align(b, LV_ALIGN_TOP_MID, 0, 80 + idx * 66);
    style_btn(b);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, FONT_HEAD, 0);
    lv_obj_align(l, LV_ALIGN_LEFT_MID, 20, 0);
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

/* A themed amber-on-black slider, full-ish width, centred at y. */
static lv_obj_t *make_slider(lv_obj_t *parent, int min, int max, int val,
                             lv_coord_t y, lv_event_cb_t cb)
{
    lv_obj_t *s = lv_slider_create(parent);
    lv_obj_set_width(s, SCAN_W - 320);
    lv_slider_set_range(s, min, max);
    lv_slider_set_value(s, val, LV_ANIM_OFF);
    lv_obj_align(s, LV_ALIGN_TOP_LEFT, 40, y);
    lv_obj_set_style_bg_color(s, COL_PANEL_ITEM, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(s, COL_DIM, LV_PART_MAIN);
    lv_obj_set_style_border_width(s, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(s, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, COL_AMBER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, COL_AMBER, LV_PART_KNOB);
    lv_obj_set_style_radius(s, 0, LV_PART_KNOB);
    if (cb) {
        lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, NULL);
    }
    return s;
}

/* A themed switch (square, amber when on). */
static lv_obj_t *make_switch(lv_obj_t *parent, bool on, lv_coord_t x, lv_coord_t y,
                             lv_event_cb_t cb, void *ud)
{
    lv_obj_t *sw = lv_switch_create(parent);
    lv_obj_align(sw, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_bg_color(sw, COL_PANEL_ITEM, LV_PART_MAIN);
    lv_obj_set_style_border_color(sw, COL_DIM, LV_PART_MAIN);
    lv_obj_set_style_border_width(sw, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(sw, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, COL_AMBER, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_radius(sw, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sw, COL_AMBER, LV_PART_KNOB);
    lv_obj_set_style_radius(sw, 0, LV_PART_KNOB);
    if (on) {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    if (cb) {
        lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, ud);
    }
    return sw;
}

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

/* One labelled FX slider row at vertical position y; returns the value label. */
static lv_obj_t *fx_row(lv_obj_t *p, const char *name, lv_coord_t y,
                        uint8_t val, lv_event_cb_t cb)
{
    panel_label(p, name, 40, y);
    lv_obj_t *v = lv_label_create(p);
    lv_label_set_text_fmt(v, "%u%%", val);
    lv_obj_set_style_text_color(v, COL_MUTE, 0);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, -40, y);
    make_slider(p, 0, 100, val, y + 34, cb);
    return v;
}

static lv_obj_t *build_display_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "DISPLAY", back_to_menu_cb);
    lv_obj_set_scroll_dir(p, LV_DIR_VER);   /* rows can exceed panel height */

    uint32_t b = 80;
    prop_settings_get_u32("brightness", &b, 80);
    panel_label(p, "BACKLIGHT", 40, 92);
    s_disp_bright_val = lv_label_create(p);
    lv_label_set_text_fmt(s_disp_bright_val, "%u%%", (unsigned)b);
    lv_obj_set_style_text_color(s_disp_bright_val, COL_MUTE, 0);
    lv_obj_align(s_disp_bright_val, LV_ALIGN_TOP_RIGHT, -40, 92);
    make_slider(p, 5, 100, b, 126, disp_bright_cb);

    panel_label(p, "CRT EFFECTS", 40, 188);
    make_switch(p, prop_fx_enabled(), SCAN_W - 140, 184, fx_toggle_cb, NULL);

    s_fx_scan_val = fx_row(p, "SCANLINES",     232, prop_fx_scanlines(), fx_scan_cb);
    s_fx_phos_val = fx_row(p, "PHOSPHOR",      300, prop_fx_phosphor(),  fx_phos_cb);
    s_fx_vign_val = fx_row(p, "VIGNETTE",      368, prop_fx_vignette(),  fx_vign_cb);
    s_fx_refr_val = fx_row(p, "REFRESH SWEEP", 436, prop_fx_refresh(),   fx_refr_cb);

    /* Screen-change transition flavor (the "old TV channel change"). */
    panel_label(p, "TRANSITION", 40, 508);
    lv_obj_t *dd = lv_dropdown_create(p);
    lv_dropdown_set_options(dd, "OFF\nSNOW\nROLL\nCOLLAPSE\nSNOW+COLLAPSE");
    uint32_t tr = 1;
    prop_settings_get_u32("fx_trans", &tr, 1);
    lv_dropdown_set_selected(dd, tr > 4 ? 1 : tr);
    lv_obj_set_width(dd, 360);
    lv_obj_align(dd, LV_ALIGN_TOP_RIGHT, -40, 502);
    style_field(dd);
    lv_obj_add_event_cb(dd, fx_trans_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* FPS meter HUD toggle (top-right perf readout). */
    panel_label(p, "FPS METER", 40, 560);
    uint32_t fps_on = 0;
    prop_settings_get_u32("fps_on", &fps_on, 0);
    make_switch(p, fps_on != 0, SCAN_W - 140, 556, fps_toggle_cb, NULL);
    return p;
}

/* LEDS: discrete lamps (on/off only) — per-lamp toggles + a lamp test. */
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
    for (int i = 0; i < LED_COUNT && i < 3; i++) {
        lv_coord_t y = 100 + i * 70;
        lv_obj_t *l = lv_label_create(p);
        char name[16];
        strlcpy(name, bsp_io_led_name((prop_led_t)i), sizeof(name));
        for (char *c = name; *c; c++) *c = (char)toupper((unsigned char)*c);
        lv_label_set_text(l, name);
        lv_obj_set_style_text_color(l, COL_AMBER, 0);
        lv_obj_set_style_text_font(l, FONT_HEAD, 0);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, 60, y);
        make_switch(p, bsp_io_led_get((prop_led_t)i), SCAN_W - 160, y - 4,
                    led_sw_cb, (void *)(intptr_t)i);
    }
    make_btn(p, "LAMP TEST", 200, LV_ALIGN_TOP_LEFT, 60, 100 + 3 * 70 + 10, lamp_test_cb);

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "Lamps are on/off (no brightness). Scene animation resumes control.");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, 60, -30);
    return p;
}

/* AUDIO: volume/mute stub (persisted; consumed when speaker SFX is promoted). */
static void audio_vol_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_settings_set_u32("audio_vol", v);
    lv_label_set_text_fmt(s_audio_vol_val, "%d%%", v);
}
static void audio_mute_cb(lv_event_t *e)
{
    prop_settings_set_u32("audio_mute",
                          lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED) ? 1 : 0);
}

static lv_obj_t *build_audio_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "AUDIO", back_to_menu_cb);
    uint32_t vol = 60, mute = 0;
    prop_settings_get_u32("audio_vol", &vol, 60);
    prop_settings_get_u32("audio_mute", &mute, 0);

    panel_label(p, "VOLUME", 40, 100);
    s_audio_vol_val = lv_label_create(p);
    lv_label_set_text_fmt(s_audio_vol_val, "%u%%", (unsigned)vol);
    lv_obj_set_style_text_color(s_audio_vol_val, COL_MUTE, 0);
    lv_obj_align(s_audio_vol_val, LV_ALIGN_TOP_RIGHT, -40, 100);
    make_slider(p, 0, 100, vol, 134, audio_vol_cb);

    panel_label(p, "MUTE", 40, 210);
    make_switch(p, mute != 0, SCAN_W - 140, 206, audio_mute_cb, NULL);

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "Output stage idle - SFX not yet wired.");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_TOP_LEFT, 40, 300);
    return p;
}

/* ABOUT: firmware identity + live IP / uptime (refreshed by the observer). */
static lv_obj_t *build_about_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "ABOUT", back_to_menu_cb);
    const esp_app_desc_t *app = esp_app_get_description();

    lv_obj_t *rows = lv_label_create(p);
    lv_obj_set_style_text_color(rows, COL_AMBER, 0);
    lv_obj_align(rows, LV_ALIGN_TOP_LEFT, 50, 100);
    lv_label_set_text_fmt(rows,
        "UNIT      COMM // SCANNER UNIT-7\n"
        "FIRMWARE  %s\n"
        "BUILD     %s  %s\n"
        "IDF       %s",
        app->version, app->date, app->time, app->idf_ver);

    s_about_ip = lv_label_create(p);
    lv_obj_set_style_text_color(s_about_ip, COL_MUTE, 0);
    lv_obj_align(s_about_ip, LV_ALIGN_TOP_LEFT, 50, 230);
    lv_label_set_text(s_about_ip, "IP        ...");

    s_about_uptime = lv_label_create(p);
    lv_obj_set_style_text_color(s_about_uptime, COL_MUTE, 0);
    lv_obj_align(s_about_uptime, LV_ALIGN_TOP_LEFT, 50, 262);
    lv_label_set_text(s_about_uptime, "UPTIME    00:00:00");
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
static lv_obj_t *make_meter_bar(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w)
{
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, w, 18);
    lv_obj_align(bar, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(bar, COL_PANEL_ITEM, 0);
    lv_obj_set_style_border_color(bar, COL_DIM, 0);
    lv_obj_set_style_border_width(bar, 2, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *fill = lv_obj_create(bar);
    lv_obj_remove_style_all(fill);
    lv_obj_set_height(fill, 18 - 8);
    lv_obj_set_width(fill, 0);
    lv_obj_align(fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(fill, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(fill, COL_AMBER, 0);
    return fill;
}

static void set_meter(lv_obj_t *fill, int pct, lv_color_t col)
{
    if (!fill) {
        return;
    }
    lv_coord_t w = lv_obj_get_content_width(lv_obj_get_parent(fill));
    if (w <= 1) w = 100;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    lv_obj_set_width(fill, pct * w / 100);
    lv_obj_set_style_bg_color(fill, col, 0);
}

/* A VITALS readout row: caption + right-aligned value + a meter bar below. */
static lv_obj_t *vitals_row(lv_obj_t *p, const char *cap, lv_coord_t y,
                            lv_obj_t **value_out, lv_obj_t **bar_out)
{
    panel_label(p, cap, 40, y);
    lv_obj_t *v = lv_label_create(p);
    lv_label_set_text(v, "--");
    lv_obj_set_style_text_color(v, COL_MUTE, 0);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, -40, y);
    *value_out = v;
    if (bar_out) {
        *bar_out = make_meter_bar(p, 40, y + 28, SCAN_W - 360);
    }
    return v;
}

static lv_obj_t *build_vitals_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "VITALS", back_to_home_cb);
    vitals_row(p, "CORE TEMP", 92,  &s_vit_temp, &s_vit_temp_bar);
    vitals_row(p, "FREE RAM",  166, &s_vit_ram,  &s_vit_ram_bar);
    vitals_row(p, "CELL",      240, &s_vit_cell, &s_vit_cell_bar);
    vitals_row(p, "UPTIME",    314, &s_vit_uptime, NULL);

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "Reactor / system telemetry - live.");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, 40, -30);
    return p;
}

/* ---- SIGNAL SCAN instrument (WiFi APs rendered as detected contacts) ------ */

#define SIG_MAX_ROWS 8
#define SIG_ROW_H    46

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

                    int pct = rssi_to_pct(s_aps[i].rssi);
                    lv_obj_t *fill = make_meter_bar(s_sig_list, 6, y + 24, SCAN_W - 180);
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
    if (xTaskCreate(signal_scan_task, "sig_scan", 4096, NULL, 4, NULL) != pdPASS) {
        s_sig_scanning = false;
        lv_label_set_text(s_sig_status, "scan busy");
    }
}

static void signal_rescan_cb(lv_event_t *e) { (void)e; start_signal_scan(); }

static lv_obj_t *build_signal_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "SIGNAL SCAN", back_to_home_cb);

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

/* ---- SPECTRUM instrument (live mic FFT bars + dB meter) ------------------- */

#define SPEC_BW      30     /* bar width */
#define SPEC_GAP     8      /* gap between bars */
#define SPEC_X0      52     /* left margin */
#define SPEC_BASE    118    /* baseline offset from panel bottom */
#define SPEC_MAXH    312    /* full-scale bar height */

static lv_obj_t *build_spectrum_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "SPECTRUM", back_to_home_cb);

    if (!prop_mic_available()) {
        s_spec_status = lv_label_create(p);
        lv_label_set_text(s_spec_status, "-- MIC OFFLINE --");
        lv_obj_set_style_text_color(s_spec_status, COL_DIM, 0);
        lv_obj_set_style_text_font(s_spec_status, FONT_HEAD, 0);
        lv_obj_center(s_spec_status);
        return p;
    }

    /* dB level meter (top). */
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
    }

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "Live acoustic spectrum  -  0 to 8 kHz");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, SPEC_X0, -30);
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

/* lv_line keeps a pointer to its points (no copy) — each polyline needs a live
 * buffer. The rail is built once, so one buffer per glyph is enough. */
static lv_point_precise_t s_ic_scan[9];
static lv_point_precise_t s_ic_vit[6];
static lv_point_precise_t s_ic_sig[3][3];
static lv_point_precise_t s_ic_ins[2][2];

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
 * muted glyphs on transparent — camera-legible, matches the old rail. */
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

/* A framed inset box — the common visual container. */
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

/* CLIMATE — a diurnal thermal trace (oscilloscope/recorder character). */
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

/* MAP — a survey map of the territory: a faint graticule, dune-field contours, the
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

/* WILDLIFE — a bio-scan dossier: a reticle portrait + a stat block. */
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

/* PLANTS — a field-guide plate: a drawn cactus + spec tags. */
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
static lv_obj_t *build_menu_panel(lv_obj_t *parent)
{
    /* SETUP holds CONFIGURATION only; instruments (VITALS/SCAN/SPECTRUM) live on
     * the console rail. BACK returns to the console. */
    lv_obj_t *m = make_panel(parent, "SETUP", close_setup_cb);
    menu_item(m, "WI-FI",   0, menu_open_cb, (void *)(intptr_t)PK_WIFI);
    menu_item(m, "DISPLAY", 1, menu_open_cb, (void *)(intptr_t)PK_DISPLAY);
    menu_item(m, "AUDIO",   2, menu_open_cb, (void *)(intptr_t)PK_AUDIO);
    menu_item(m, "LEDS",    3, menu_open_cb, (void *)(intptr_t)PK_LEDS);
    menu_item(m, "ABOUT",   4, menu_open_cb, (void *)(intptr_t)PK_ABOUT);
    return m;
}

/* Signal level is measured in quarter-cells. The gauge deliberately never tops
 * out (cassette-futurism: a perfect reading is suspicious) — SIG_QMAX is one
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
 * "SENS [====    ] 65%" — reflects the sensitivity the web slider drives, which
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
     * an overlay panel shown on top of it by default — see the open_panel(PK_HOME)
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

    /* IP on the second header line — kept above the y=64 divider so it isn't clipped. */
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
     * console home is shown on top as the default landing screen. */
    open_panel(PK_HOME);
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
     * Repainting it at 20 Hz then is pure waste — worse, each invisible
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
         * waveform logic — it just maps the signed columns to screen coordinates. */
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
         * real cost behind the ~73 ms frames — far more than the waveform. In steady
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
        /* LVGL's printf has no %f support — format floats with stdio snprintf. */
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
            lv_obj_align(s_spec_bars[i], LV_ALIGN_BOTTOM_LEFT,
                         SPEC_X0 + i * (SPEC_BW + SPEC_GAP), -SPEC_BASE);
            lv_obj_set_style_bg_color(s_spec_bars[i],
                                      pct > 85 ? COL_ALERT : (pct > 35 ? COL_AMBER : COL_MUTE), 0);
        }
        if (st->tick % 4 == 0 && s_spec_db) {
            lv_label_set_text_fmt(s_spec_db, "%d dB", prop_mic_get_db());
            set_meter(s_spec_db_bar, (prop_mic_get_db() + 60) * 100 / 60, COL_AMBER);
        }
    }

    /* Refresh the ABOUT panel's live fields (~1 Hz) only while it's the live panel. */
    if (s_cur_kind == PK_ABOUT && s_about_ip && (st->tick % 20 == 0)) {
        char ip[16];
        prop_net_get_ip(ip, sizeof(ip));
        lv_label_set_text_fmt(s_about_ip, "IP        %s", ip[0] ? ip : "(no link)");
        uint32_t s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        lv_label_set_text_fmt(s_about_uptime, "UPTIME    %02u:%02u:%02u",
                              (unsigned)(s / 3600), (unsigned)((s / 60) % 60),
                              (unsigned)(s % 60));
    }

    /* While a connection attempt is in flight, reflect its real outcome in the
     * setup panel status — but only while the WiFi panel is still the live one. */
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
    else if (strcmp(screen, "archive") == 0) open_panel(PK_ARCHIVE);
    else if (strcmp(screen, "cassette") == 0) open_panel(PK_CASSETTE);
    else if (strcmp(screen, "insights") == 0) open_panel(PK_INSIGHTS);
    lvgl_port_unlock();
    ESP_LOGI(UI_TAG, "goto screen: %s", screen);
}

/* ---- Physical-control input (SELECTOR dial / TAB switches / ACTION buttons)
 * The author's nav model, decoupled from hardware: the web portal drives this
 * today via /cmd {"cmd":"input",...}; bsp_io will route the real knobs/switches
 * here once they're wired. Navigation lives in the view (this module), mirroring
 * prop_ui_goto — the engine stays a pure behavior model. */

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
    if (strcmp(control, "selector") == 0) {
        if (arg == 0) {
            nav_select_press();
        } else {
            nav_select_move(arg > 0 ? 1 : -1);
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
    ESP_LOGI(UI_TAG, "input %s %d", control, arg);
}

/* ---- FPS meter (optional dev HUD, top-right) ------------------------------
 * Counts genuinely rendered frames via the display monitor_cb (called once per
 * refresh that draws). When active it drops the refresh-timer period from the
 * 30 ms default (33 fps cap) to 8 ms, so animations that invalidate small regions
 * can run up to ~60 fps. It deliberately does NOT force a full-screen redraw every
 * frame: a whole-screen software render at 1024x600 takes ~250 ms (≈4 fps) and
 * saturates the CPU, so forcing it is counter-productive. The counter therefore
 * reflects real render activity — high during motion, low when the screen is idle.
 *
 * The HUD is an OPAQUE child of the active screen, NOT lv_layer_top — a
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
