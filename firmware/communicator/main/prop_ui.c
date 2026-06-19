/* prop_ui — cassette-futurism LVGL readout, driven by prop_engine state. */
#include "prop_ui.h"
#include "prop_engine.h"
#include "prop_net.h"
#include "prop_settings.h"
#include "prop_fx.h"
#include "prop_mic.h"
#include "bsp_io.h"
#include "bsp_illuminate.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
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

#define SCAN_W 1024
#define SCAN_TRACK_Y 330        /* taller, lower trace — the hero waveform */
#define SCAN_TRACK_H 160

static lv_obj_t *s_status_label;
static lv_obj_t *s_channel_label;
static lv_obj_t *s_link_label;
static lv_obj_t *s_ip_label;        /* STA IP, dim, under the LINK line */
static lv_obj_t *s_scan_track;
static lv_obj_t *s_scan_blip;
static lv_obj_t *s_wave_line;                       /* recorder trail across the track */
static lv_point_t s_wave_pts[PROP_WAVE_SAMPLES];    /* persistent points buffer for lv_line */

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
    PK_NONE = 0, PK_MENU, PK_WIFI, PK_DISPLAY, PK_AUDIO, PK_LEDS, PK_ABOUT, PK_VITALS,
    PK_SCAN, PK_SPECTRUM,
} panel_kind_t;

static lv_obj_t *s_root;          /* the main screen (panels are built on it) */
static lv_obj_t *s_cur_panel;     /* the one live setup panel, or NULL (main) */
static panel_kind_t s_cur_kind;

/* Live value readouts updated by panels / the observer — valid ONLY while the
 * owning panel is the current one (NULLed on teardown). */
static lv_obj_t *s_disp_bright_val, *s_fx_int_val, *s_audio_vol_val;
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
static lv_obj_t *build_menu_panel(lv_obj_t *parent);
static lv_obj_t *build_wifi_panel(lv_obj_t *parent);
static lv_obj_t *build_display_panel(lv_obj_t *parent);
static lv_obj_t *build_audio_panel(lv_obj_t *parent);
static lv_obj_t *build_leds_panel(lv_obj_t *parent);
static lv_obj_t *build_about_panel(lv_obj_t *parent);
static lv_obj_t *build_vitals_panel(lv_obj_t *parent);
static lv_obj_t *build_signal_panel(lv_obj_t *parent);
static lv_obj_t *build_spectrum_panel(lv_obj_t *parent);
static void wifi_panel_opened(void);
static void start_signal_scan(void);

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
    s_disp_bright_val = NULL; s_fx_int_val = NULL; s_audio_vol_val = NULL;
    s_about_ip = NULL; s_about_uptime = NULL;
    s_vit_temp = NULL; s_vit_ram = NULL; s_vit_uptime = NULL; s_vit_cell = NULL;
    s_vit_temp_bar = NULL; s_vit_ram_bar = NULL; s_vit_cell_bar = NULL;
    s_sig_list = NULL; s_sig_status = NULL;
    s_spec_db = NULL; s_spec_db_bar = NULL; s_spec_status = NULL;
    s_connect_pending = false;
}

/* Switch to a panel (or PK_NONE for the main screen). */
static void open_panel(panel_kind_t kind)
{
    close_panel();
    switch (kind) {
        case PK_MENU:    s_cur_panel = build_menu_panel(s_root); break;
        case PK_WIFI:    s_cur_panel = build_wifi_panel(s_root); break;
        case PK_DISPLAY: s_cur_panel = build_display_panel(s_root); break;
        case PK_AUDIO:   s_cur_panel = build_audio_panel(s_root); break;
        case PK_LEDS:    s_cur_panel = build_leds_panel(s_root); break;
        case PK_ABOUT:   s_cur_panel = build_about_panel(s_root); break;
        case PK_VITALS:  s_cur_panel = build_vitals_panel(s_root); break;
        case PK_SCAN:    s_cur_panel = build_signal_panel(s_root); break;
        case PK_SPECTRUM: s_cur_panel = build_spectrum_panel(s_root); break;
        default: return;   /* PK_NONE: just closed to main */
    }
    s_cur_kind = kind;
    if (kind == PK_WIFI) {
        wifi_panel_opened();
    } else if (kind == PK_SCAN) {
        start_signal_scan();   /* auto-scan on open */
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
static void close_setup_cb(lv_event_t *e)  { (void)e; open_panel(PK_NONE); }
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
static void fx_int_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_intensity(v);
    lv_label_set_text_fmt(s_fx_int_val, "%d%%", v);
}

static lv_obj_t *build_display_panel(lv_obj_t *parent)
{
    lv_obj_t *p = make_panel(parent, "DISPLAY", back_to_menu_cb);

    uint32_t b = 80;
    prop_settings_get_u32("brightness", &b, 80);
    panel_label(p, "BACKLIGHT", 40, 92);
    s_disp_bright_val = lv_label_create(p);
    lv_label_set_text_fmt(s_disp_bright_val, "%u%%", (unsigned)b);
    lv_obj_set_style_text_color(s_disp_bright_val, COL_MUTE, 0);
    lv_obj_align(s_disp_bright_val, LV_ALIGN_TOP_RIGHT, -40, 92);
    make_slider(p, 5, 100, b, 126, disp_bright_cb);

    panel_label(p, "CRT EFFECTS", 40, 196);
    make_switch(p, prop_fx_enabled(), SCAN_W - 140, 192, fx_toggle_cb, NULL);

    panel_label(p, "FX INTENSITY", 40, 268);
    s_fx_int_val = lv_label_create(p);
    lv_label_set_text_fmt(s_fx_int_val, "%u%%", prop_fx_intensity());
    lv_obj_set_style_text_color(s_fx_int_val, COL_MUTE, 0);
    lv_obj_align(s_fx_int_val, LV_ALIGN_TOP_RIGHT, -40, 268);
    make_slider(p, 0, 100, prop_fx_intensity(), 302, fx_int_cb);

    lv_obj_t *note = lv_label_create(p);
    lv_label_set_text(note, "Effects are a top-layer CRT overlay (panel only).");
    lv_obj_set_style_text_color(note, COL_MUTE, 0);
    lv_obj_align(note, LV_ALIGN_TOP_LEFT, 40, 360);
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
    lv_obj_t *p = make_panel(parent, "VITALS", back_to_menu_cb);
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
    lv_obj_t *p = make_panel(parent, "SIGNAL SCAN", back_to_menu_cb);

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
    lv_obj_t *p = make_panel(parent, "SPECTRUM", back_to_menu_cb);

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

/* Global SETUP menu (its own lazily-built panel). Rows open sub-panels by kind. */
static lv_obj_t *build_menu_panel(lv_obj_t *parent)
{
    lv_obj_t *m = make_panel(parent, "SETUP", close_setup_cb);   /* BACK exits to main */
    menu_item(m, "WI-FI",   0, menu_open_cb, (void *)(intptr_t)PK_WIFI);
    menu_item(m, "DISPLAY", 1, menu_open_cb, (void *)(intptr_t)PK_DISPLAY);
    menu_item(m, "AUDIO",   2, menu_open_cb, (void *)(intptr_t)PK_AUDIO);
    menu_item(m, "LEDS",        3, menu_open_cb, (void *)(intptr_t)PK_LEDS);
    menu_item(m, "VITALS",      4, menu_open_cb, (void *)(intptr_t)PK_VITALS);
    menu_item(m, "SIGNAL SCAN", 5, menu_open_cb, (void *)(intptr_t)PK_SCAN);
    menu_item(m, "SPECTRUM",    6, menu_open_cb, (void *)(intptr_t)PK_SPECTRUM);
    menu_item(m, "ABOUT",       7, menu_open_cb, (void *)(intptr_t)PK_ABOUT);
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
    lv_obj_t *scr = lv_scr_act();
    s_root = scr;   /* lazily-built panels are created on this screen */
    lv_obj_set_style_bg_color(scr, COL_BG, LV_PART_MAIN);
    lv_obj_set_style_text_font(scr, FONT_BODY, 0);   /* Eurostile everywhere (inherited) */

    /* Title bar */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "COMM // SCANNER  UNIT-7");
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
     * column. Built before the blip so the bright write-head draws on top of it. */
    s_wave_line = lv_line_create(s_scan_track);
    lv_obj_set_style_line_color(s_wave_line, COL_AMBER, 0);
    lv_obj_set_style_line_width(s_wave_line, 2, 0);
    lv_obj_set_style_line_rounded(s_wave_line, false, 0);
    lv_obj_align(s_wave_line, LV_ALIGN_TOP_LEFT, 0, 0);

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

    /* SETUP menu + sub-screens are built lazily on navigation (open_panel). */
}

static const char *link_text(prop_link_t link)
{
    switch (link) {
        case LINK_STA: return "LINK: NET";
        case LINK_AP:  return "LINK: LOCAL";
        default:       return "LINK: ----";
    }
}

/* Observer: runs in engine task context -> must take the LVGL lock. */
static void ui_observer(const prop_state_t *st, void *ctx)
{
    (void)ctx;
    if (!lvgl_port_lock(50)) {
        return;  /* skip this frame rather than block the engine */
    }

    lv_label_set_text(s_status_label, st->status);
    lv_label_set_text(s_channel_label, st->channel);
    lv_label_set_text(s_link_label, link_text(st->link));
    lv_obj_set_style_text_color(s_link_label, st->link == LINK_DOWN ? COL_DIM : COL_AMBER, 0);
    /* Keep the meter pinned to the (variable-width) LINK label. */
    lv_obj_align_to(s_sig_box, s_link_label, LV_ALIGN_OUT_LEFT_MID, -12, 0);

    /* IP + signal strength alongside the LINK indicator. prop_net_get_rssi() is a
     * cheap cached read (a background task does the SDIO poll), so this stays off
     * the critical path and never stalls the display. */
    if (st->link == LINK_STA) {
        char ip[16];
        prop_net_get_ip(ip, sizeof(ip));
        lv_label_set_text(s_ip_label, ip);
        set_signal_bars(rssi_to_quarters(prop_net_get_rssi()), COL_AMBER);
    } else if (st->link == LINK_AP) {
        lv_label_set_text(s_ip_label, "AP 192.168.4.1");
        set_signal_bars(0, COL_AMBER);
    } else {
        lv_label_set_text(s_ip_label, "");
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
    lv_line_set_points(s_wave_line, s_wave_pts, PROP_WAVE_SAMPLES);
    lv_obj_set_style_line_color(s_wave_line, status_col, 0);

    /* The bright blip rides the write head (the "now" edge of the recorder); when
     * SIGNAL_ACQUIRED freezes the sweep it parks on the eruption as a peak marker. */
    int head_x = (int)st->wave_head * (cw - 1) / (PROP_WAVE_SAMPLES - 1);
    lv_obj_align(s_scan_blip, LV_ALIGN_LEFT_MID, head_x, 0);
    lv_obj_set_style_bg_color(s_scan_blip, status_col, 0);

    /* Status punch-in: entering SIGNAL_ACQUIRED / ALERT slams the headline in big
     * (FONT_PUNCH) and bright, then drops to the normal heading and settles. v8
     * labels don't upscale via transform, so the punch is a font swap + flash. */
    bool punching = st->scene_tick < 6 && (st->scene == SCENE_SIGNAL_ACQUIRED || alert);
    lv_color_t headline_col = status_col;
    if (punching) {
        lv_obj_set_style_text_font(s_status_label, FONT_PUNCH, 0);
        if (st->scene_tick & 1) {   /* flash a brighter tint while it lands */
            headline_col = alert ? lv_color_hex(0xFF9090) : lv_color_hex(0xFFE060);
        }
    } else {
        lv_obj_set_style_text_font(s_status_label, FONT_STATUS, 0);
    }
    lv_obj_set_style_text_color(s_status_label, headline_col, 0);

    /* Channel gauge marker tracks the tuned position (scrambles while SCANNING,
     * parks on lock). The waveform below is the noise at this channel. */
    lv_coord_t band_w = lv_obj_get_content_width(s_chan_band);
    if (band_w <= 1) band_w = CHAN_BAND_W - 4;
    int marker_x = (int)st->chan_pos * (band_w - 4) / 100;
    lv_obj_align(s_chan_marker, LV_ALIGN_LEFT_MID, marker_x, 0);
    lv_obj_set_style_bg_color(s_chan_marker, alert ? COL_ALERT : COL_AMBER, 0);

    /* SENS meter reflects the receiver gain the web slider drives. */
    lv_coord_t sens_w = lv_obj_get_content_width(lv_obj_get_parent(s_sens_fill));
    if (sens_w <= 1) sens_w = SENS_W - 4;
    lv_obj_set_width(s_sens_fill, (int)st->sensitivity * sens_w / 100);
    lv_label_set_text_fmt(s_sens_val, "%d%%", st->sensitivity);

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
    if (strcmp(screen, "home") == 0)         open_panel(PK_NONE);
    else if (strcmp(screen, "menu") == 0)    open_panel(PK_MENU);
    else if (strcmp(screen, "wifi") == 0)    open_panel(PK_WIFI);
    else if (strcmp(screen, "display") == 0) open_panel(PK_DISPLAY);
    else if (strcmp(screen, "audio") == 0)   open_panel(PK_AUDIO);
    else if (strcmp(screen, "leds") == 0)    open_panel(PK_LEDS);
    else if (strcmp(screen, "about") == 0)   open_panel(PK_ABOUT);
    else if (strcmp(screen, "vitals") == 0)  open_panel(PK_VITALS);
    else if (strcmp(screen, "scan") == 0)    open_panel(PK_SCAN);
    else if (strcmp(screen, "spectrum") == 0) open_panel(PK_SPECTRUM);
    lvgl_port_unlock();
    ESP_LOGI(UI_TAG, "goto screen: %s", screen);
}

esp_err_t prop_ui_init(void)
{
    if (!lvgl_port_lock(1000)) {
        ESP_LOGE(UI_TAG, "could not lock LVGL to build UI");
        return ESP_FAIL;
    }
    build_screen();
    lvgl_port_unlock();

    esp_err_t err = prop_engine_add_observer(ui_observer, NULL);
    ESP_LOGI(UI_TAG, "cassette-futurism UI ready");
    return err;
}
