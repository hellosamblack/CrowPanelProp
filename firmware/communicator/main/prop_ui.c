/* prop_ui — cassette-futurism LVGL readout, driven by prop_engine state. */
#include "prop_ui.h"
#include "prop_engine.h"
#include "prop_net.h"
#include "prop_settings.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#define UI_TAG "PROP_UI"
#define SCAN_MAX 20

/* Cassette-futurism palette: amber phosphor on near-black. */
#define COL_BG     lv_color_hex(0x0A0A06)
#define COL_AMBER  lv_color_hex(0xE0B000)
#define COL_DIM    lv_color_hex(0x6B5300)
#define COL_ALERT  lv_color_hex(0xFF3030)

#define SCAN_W 1024
#define SCAN_TRACK_Y 360
#define SCAN_TRACK_H 40

static lv_obj_t *s_status_label;
static lv_obj_t *s_channel_label;
static lv_obj_t *s_link_label;
static lv_obj_t *s_scan_track;
static lv_obj_t *s_scan_blip;

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

/* ---- Setup screens: global menu + per-section sub-screens --------------- */

#define COL_PANEL_ITEM lv_color_hex(0x141008)
#define SETUP_PANEL_MAX 8

static lv_obj_t *s_setup_menu;                    /* global SETUP page */
static lv_obj_t *s_setup_panels[SETUP_PANEL_MAX]; /* every full-screen setup panel */
static int s_setup_panel_count;
static lv_obj_t *s_panel_display, *s_panel_audio, *s_panel_leds, *s_panel_about;

/* Register a panel so the navigation helpers can manage it (starts hidden). */
static void register_panel(lv_obj_t *p)
{
    if (s_setup_panel_count < SETUP_PANEL_MAX) {
        s_setup_panels[s_setup_panel_count++] = p;
    }
    lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
}

/* Show exactly one setup panel (or NULL to return to the main prop screen). */
static void show_only(lv_obj_t *panel)
{
    for (int i = 0; i < s_setup_panel_count; i++) {
        if (s_setup_panels[i] == panel) {
            lv_obj_clear_flag(s_setup_panels[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_setup_panels[i], LV_OBJ_FLAG_HIDDEN);
        }
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
    lv_obj_set_style_text_font(kb, &lv_font_montserrat_24, LV_PART_ITEMS);
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

static void open_wifi(void)
{
    char saved_pass[65];
    prop_settings_get_str("sta_pass", saved_pass, sizeof(saved_pass), "");
    lv_textarea_set_text(s_pass_ta, "");
    lv_textarea_set_password_mode(s_pass_ta, true);
    /* If a password is already stored, hint that instead of "password". */
    lv_textarea_set_placeholder_text(s_pass_ta, saved_pass[0] ? "[ saved ]" : "password");
    s_pass_shown = false;
    lv_label_set_text(lv_obj_get_child(s_show_btn, 0), LV_SYMBOL_EYE_OPEN);
    lv_obj_add_state(s_remember_cb, LV_STATE_CHECKED);   /* default: remember */
    lv_obj_add_flag(s_forget_btn, LV_OBJ_FLAG_HIDDEN);
    s_connect_pending = false;
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_setup_status, "");
    show_only(s_setup_panel);
    start_scan();   /* auto-scan on open so the list is ready */
}

/* Navigation between the main screen, the SETUP menu, and sub-screens. */
static void open_menu_cb(lv_event_t *e)    { (void)e; show_only(s_setup_menu); }
static void close_setup_cb(lv_event_t *e)  { (void)e; show_only(NULL); }
static void back_to_menu_cb(lv_event_t *e) { (void)e; show_only(s_setup_menu); }
static void open_wifi_cb(lv_event_t *e)    { (void)e; open_wifi(); }
static void open_stub_cb(lv_event_t *e)    { show_only((lv_obj_t *)lv_event_get_user_data(e)); }
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
    register_panel(p);

    make_btn(p, LV_SYMBOL_LEFT " BACK", 130, LV_ALIGN_TOP_LEFT, 20, 12, back_cb);

    lv_obj_t *t = lv_label_create(p);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_color(t, COL_AMBER, 0);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 18);
    return p;
}

/* A placeholder sub-screen for sections not built yet. */
static lv_obj_t *build_stub(lv_obj_t *parent, const char *title)
{
    lv_obj_t *p = make_panel(parent, title, back_to_menu_cb);
    lv_obj_t *msg = lv_label_create(p);
    lv_label_set_text(msg, "-- COMING SOON --");
    lv_obj_set_style_text_color(msg, COL_DIM, 0);
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_24, 0);
    lv_obj_center(msg);
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
    lv_obj_set_style_text_font(l, &lv_font_montserrat_24, 0);
    lv_obj_align(l, LV_ALIGN_LEFT_MID, 20, 0);
}

static lv_obj_t *build_settings_panel(lv_obj_t *parent)
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

/* Global SETUP menu with section rows; WI-FI is live, the rest are stubs. */
static void build_setup_screens(lv_obj_t *parent)
{
    build_settings_panel(parent);                              /* sets s_setup_panel (WI-FI) */
    s_panel_display = build_stub(parent, "DISPLAY");
    s_panel_audio   = build_stub(parent, "AUDIO");
    s_panel_leds    = build_stub(parent, "LEDS");
    s_panel_about   = build_stub(parent, "ABOUT");

    s_setup_menu = make_panel(parent, "SETUP", close_setup_cb);   /* BACK here exits to main */
    menu_item(s_setup_menu, "WI-FI",   0, open_wifi_cb, NULL);
    menu_item(s_setup_menu, "DISPLAY", 1, open_stub_cb, s_panel_display);
    menu_item(s_setup_menu, "AUDIO",   2, open_stub_cb, s_panel_audio);
    menu_item(s_setup_menu, "LEDS",    3, open_stub_cb, s_panel_leds);
    menu_item(s_setup_menu, "ABOUT",   4, open_stub_cb, s_panel_about);
}

static void build_screen(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_BG, LV_PART_MAIN);

    /* Title bar */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "COMM // SCANNER  UNIT-7");
    lv_obj_set_style_text_color(title, COL_AMBER, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 24, 20);

    /* A divider line under the title (thin amber bar). */
    lv_obj_t *rule = lv_obj_create(scr);
    lv_obj_set_size(rule, SCAN_W - 48, 3);
    lv_obj_set_style_bg_color(rule, COL_DIM, 0);
    lv_obj_set_style_border_width(rule, 0, 0);
    lv_obj_align(rule, LV_ALIGN_TOP_MID, 0, 64);

    /* LINK indicator (top-right) */
    s_link_label = lv_label_create(scr);
    lv_label_set_text(s_link_label, "LINK: ----");
    lv_obj_set_style_text_color(s_link_label, COL_DIM, 0);
    lv_obj_set_style_text_font(s_link_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_link_label, LV_ALIGN_TOP_RIGHT, -24, 28);

    /* Big status line */
    s_status_label = lv_label_create(scr);
    lv_label_set_text(s_status_label, "STANDBY");
    lv_obj_set_style_text_color(s_status_label, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_24, 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 150);

    /* Channel / frequency readout */
    s_channel_label = lv_label_create(scr);
    lv_label_set_text(s_channel_label, "CH -- / --- MHz");
    lv_obj_set_style_text_color(s_channel_label, COL_AMBER, 0);
    lv_obj_set_style_text_font(s_channel_label, &lv_font_montserrat_24, 0);
    lv_obj_align(s_channel_label, LV_ALIGN_TOP_MID, 0, 240);

    /* Scanner track + sweeping blip */
    s_scan_track = lv_obj_create(scr);
    lv_obj_set_size(s_scan_track, SCAN_W - 48, SCAN_TRACK_H);
    lv_obj_set_style_bg_color(s_scan_track, lv_color_hex(0x141008), 0);
    lv_obj_set_style_border_color(s_scan_track, COL_DIM, 0);
    lv_obj_set_style_border_width(s_scan_track, 2, 0);
    lv_obj_set_style_radius(s_scan_track, 0, 0);
    lv_obj_align(s_scan_track, LV_ALIGN_TOP_MID, 0, SCAN_TRACK_Y);
    lv_obj_clear_flag(s_scan_track, LV_OBJ_FLAG_SCROLLABLE);

    s_scan_blip = lv_obj_create(s_scan_track);
    lv_obj_set_size(s_scan_blip, 12, SCAN_TRACK_H - 12);
    lv_obj_set_style_bg_color(s_scan_blip, COL_AMBER, 0);
    lv_obj_set_style_border_width(s_scan_blip, 0, 0);
    lv_obj_set_style_radius(s_scan_blip, 0, 0);
    lv_obj_align(s_scan_blip, LV_ALIGN_LEFT_MID, 0, 0);

    /* Footer hint */
    lv_obj_t *foot = lv_label_create(scr);
    lv_label_set_text(foot, "AUTHORIZED PERSONNEL ONLY  -  SYS REV 0.1");
    lv_obj_set_style_text_color(foot, COL_DIM, 0);
    lv_obj_set_style_text_font(foot, &lv_font_montserrat_14, 0);
    lv_obj_align(foot, LV_ALIGN_BOTTOM_MID, 0, -20);

    /* SETUP button (bottom-right, dim) opens the WiFi config panel. */
    lv_obj_t *setup_btn = lv_btn_create(scr);
    lv_obj_set_size(setup_btn, 110, 44);
    lv_obj_align(setup_btn, LV_ALIGN_BOTTOM_RIGHT, -16, -12);
    lv_obj_set_style_bg_color(setup_btn, lv_color_hex(0x141008), 0);
    lv_obj_set_style_border_color(setup_btn, COL_DIM, 0);
    lv_obj_set_style_border_width(setup_btn, 1, 0);
    lv_obj_set_style_radius(setup_btn, 0, 0);
    lv_obj_add_event_cb(setup_btn, open_menu_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *setup_lbl = lv_label_create(setup_btn);
    lv_label_set_text(setup_lbl, "SETUP");
    lv_obj_set_style_text_color(setup_lbl, COL_DIM, 0);
    lv_obj_center(setup_lbl);

    /* The SETUP menu + sub-screens (all hidden until SETUP tapped). */
    build_setup_screens(scr);
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

    /* Alert scene tints the status red. */
    lv_color_t status_col = (st->scene == SCENE_ALERT) ? COL_ALERT : COL_AMBER;
    lv_obj_set_style_text_color(s_status_label, status_col, 0);

    /* Sweep the blip: position derived from the engine tick so motion speed
     * tracks the scene without the UI owning any timer. */
    lv_coord_t track_w = lv_obj_get_width(s_scan_track);
    if (track_w <= 0) {
        track_w = SCAN_W - 48;
    }
    int span = track_w - 12;
    int phase = st->tick % (2 * span);
    int x = (phase < span) ? phase : (2 * span - phase);  /* ping-pong */
    lv_obj_align(s_scan_blip, LV_ALIGN_LEFT_MID, x, 0);
    lv_obj_set_style_bg_color(s_scan_blip, status_col, 0);

    /* While a connection attempt is in flight, reflect its real outcome in the
     * setup panel status (this runs ~10 Hz, so feedback is near-immediate). */
    if (s_connect_pending) {
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
    if (strcmp(screen, "home") == 0)         show_only(NULL);
    else if (strcmp(screen, "menu") == 0)    show_only(s_setup_menu);
    else if (strcmp(screen, "wifi") == 0)    open_wifi();
    else if (strcmp(screen, "display") == 0) show_only(s_panel_display);
    else if (strcmp(screen, "audio") == 0)   show_only(s_panel_audio);
    else if (strcmp(screen, "leds") == 0)    show_only(s_panel_leds);
    else if (strcmp(screen, "about") == 0)   show_only(s_panel_about);
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
