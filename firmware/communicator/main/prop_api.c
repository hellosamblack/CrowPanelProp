/* prop_api — HTTP server: operator console, REST, WebSocket, OTA. See prop_api.h. */
#include "prop_api.h"
#include "prop_engine.h"
#include "prop_net.h"
#include "prop_ui.h"
#include "prop_fx.h"
#include "bsp_io.h"
#include <string.h>
#include <strings.h>   /* strcasecmp */
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "cJSON.h"

#define API_TAG "PROP_API"
#define OTA_BUF_SIZE 1460

static httpd_handle_t s_server;

/* ---- State -> JSON ------------------------------------------------------- */
static char *state_to_json(void)
{
    prop_state_t st;
    prop_engine_get_state(&st);
    char ip[16];
    prop_net_get_ip(ip, sizeof(ip));

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "scene", prop_scene_name(st.scene));
    cJSON_AddStringToObject(root, "status", st.status);
    cJSON_AddStringToObject(root, "channel", st.channel);
    cJSON_AddNumberToObject(root, "link", st.link);
    cJSON_AddNumberToObject(root, "sensitivity", st.sensitivity);
    cJSON_AddNumberToObject(root, "channel_pos", st.chan_pos);
    cJSON_AddStringToObject(root, "ip", ip);
    cJSON *leds = cJSON_AddArrayToObject(root, "leds");
    for (int i = 0; i < LED_COUNT; i++) {
        cJSON *led = cJSON_CreateObject();
        cJSON_AddStringToObject(led, "name", bsp_io_led_name((prop_led_t)i));
        cJSON_AddBoolToObject(led, "on", (st.led_mask >> i) & 0x1);
        cJSON_AddItemToArray(leds, led);
    }
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;   /* caller frees */
}

/* ---- Command dispatch (shared by /cmd and /ws) -------------------------- */
/* Returns ESP_OK if the command was understood. */
static esp_err_t dispatch_command(const char *json, int len)
{
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ESP_ERR_INVALID_ARG;
    const cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(cmd)) {
        const char *c = cmd->valuestring;
        const cJSON *value = cJSON_GetObjectItem(root, "value");

        if (strcmp(c, "scene") == 0 && cJSON_IsString(value)) {
            prop_scene_t s = prop_scene_from_name(value->valuestring);
            err = (s < SCENE_COUNT) ? prop_engine_set_scene(s) : ESP_ERR_INVALID_ARG;
        } else if (strcmp(c, "next_scene") == 0) {
            err = prop_engine_next_scene();
        } else if (strcmp(c, "status") == 0 && cJSON_IsString(value)) {
            err = prop_engine_set_status(value->valuestring);
        } else if (strcmp(c, "channel") == 0 && cJSON_IsString(value)) {
            err = prop_engine_set_channel(value->valuestring);
        } else if (strcmp(c, "sens") == 0 && cJSON_IsNumber(value)) {
            int v = value->valueint;
            err = prop_engine_set_sensitivity((uint8_t)(v < 0 ? 0 : v));
        } else if (strcmp(c, "fx") == 0) {
            const cJSON *on = cJSON_GetObjectItem(root, "on");
            if (cJSON_IsBool(on)) {
                prop_fx_set_enabled(cJSON_IsTrue(on));
            }
            const cJSON *j;
            bool any = false;
            if ((j = cJSON_GetObjectItem(root, "scan"))     && cJSON_IsNumber(j)) { prop_fx_set_scanlines((uint8_t)(j->valueint < 0 ? 0 : j->valueint)); any = true; }
            if ((j = cJSON_GetObjectItem(root, "phosphor")) && cJSON_IsNumber(j)) { prop_fx_set_phosphor ((uint8_t)(j->valueint < 0 ? 0 : j->valueint)); any = true; }
            if ((j = cJSON_GetObjectItem(root, "vignette")) && cJSON_IsNumber(j)) { prop_fx_set_vignette ((uint8_t)(j->valueint < 0 ? 0 : j->valueint)); any = true; }
            if ((j = cJSON_GetObjectItem(root, "refresh"))  && cJSON_IsNumber(j)) { prop_fx_set_refresh  ((uint8_t)(j->valueint < 0 ? 0 : j->valueint)); any = true; }
            err = (cJSON_IsBool(on) || any) ? ESP_OK : ESP_ERR_INVALID_ARG;
        } else if (strcmp(c, "led") == 0) {
            const cJSON *on = cJSON_GetObjectItem(root, "on");
            const cJSON *index = cJSON_GetObjectItem(root, "index");
            const cJSON *name = cJSON_GetObjectItem(root, "name");
            int idx = -1;
            if (cJSON_IsNumber(index)) {
                idx = index->valueint;
            } else if (cJSON_IsString(name)) {
                for (int i = 0; i < LED_COUNT; i++) {
                    if (strcasecmp(name->valuestring, bsp_io_led_name((prop_led_t)i)) == 0) {
                        idx = i;
                        break;
                    }
                }
            }
            if (idx >= 0 && cJSON_IsBool(on)) {
                err = prop_engine_set_led(idx, cJSON_IsTrue(on));
            }
        } else if (strcmp(c, "ui") == 0) {
            const cJSON *screen = cJSON_GetObjectItem(root, "screen");
            if (cJSON_IsString(screen)) {        /* remote nav for testing/screenshots */
                prop_ui_goto(screen->valuestring);
                err = ESP_OK;
            }
        } else if (strcmp(c, "input") == 0) {
            /* Simulated physical controls (SELECTOR dial / TAB switch / ACTION).
             * arg accepts a number, or words: cw/ccw/press for the dial. */
            const cJSON *control = cJSON_GetObjectItem(root, "control");
            const cJSON *arg = cJSON_GetObjectItem(root, "arg");
            if (cJSON_IsString(control)) {
                int a = 0;
                if (cJSON_IsNumber(arg)) {
                    a = arg->valueint;
                } else if (cJSON_IsString(arg)) {
                    const char *s = arg->valuestring;
                    if (strcasecmp(s, "cw") == 0 || strcasecmp(s, "next") == 0) a = 1;
                    else if (strcasecmp(s, "ccw") == 0 || strcasecmp(s, "prev") == 0) a = -1;
                    else if (strcasecmp(s, "press") == 0 || strcasecmp(s, "select") == 0) a = 0;
                    else a = atoi(s);
                }
                prop_ui_input(control->valuestring, a);
                err = ESP_OK;
            }
        } else if (strcmp(c, "wifi") == 0) {
            const cJSON *ssid = cJSON_GetObjectItem(root, "ssid");
            const cJSON *pass = cJSON_GetObjectItem(root, "pass");
            const cJSON *remember = cJSON_GetObjectItem(root, "remember");
            if (cJSON_IsString(ssid)) {
                bool rem = remember ? cJSON_IsTrue(remember) : true;  /* default: persist */
                err = prop_net_set_sta_credentials(ssid->valuestring,
                                                   cJSON_IsString(pass) ? pass->valuestring : "",
                                                   rem);
            }
        }
    }
    cJSON_Delete(root);
    return err;
}

/* ---- WebSocket broadcast ----------------------------------------------- */
/* Broadcast only on meaningful changes (scene/status/channel/link), not on every
 * 10 Hz animation tick, to avoid flooding clients. */
static void broadcast_observer(const prop_state_t *st, void *ctx)
{
    (void)ctx;
    static prop_scene_t last_scene = SCENE_COUNT;
    static prop_link_t last_link = -1;
    static char last_status[PROP_TEXT_MAX];
    static char last_channel[PROP_TEXT_MAX];

    if (st->scene == last_scene && st->link == last_link &&
        strcmp(st->status, last_status) == 0 && strcmp(st->channel, last_channel) == 0) {
        return;
    }
    last_scene = st->scene;
    last_link = st->link;
    strlcpy(last_status, st->status, sizeof(last_status));
    strlcpy(last_channel, st->channel, sizeof(last_channel));

    if (!s_server) {
        return;
    }
    char *json = state_to_json();
    if (!json) {
        return;
    }
    size_t num = CONFIG_LWIP_MAX_SOCKETS;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    if (httpd_get_client_list(s_server, &num, fds) == ESP_OK) {
        for (size_t i = 0; i < num; i++) {
            if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                httpd_ws_frame_t frame = {
                    .type = HTTPD_WS_TYPE_TEXT,
                    .payload = (uint8_t *)json,
                    .len = strlen(json),
                };
                httpd_ws_send_frame_async(s_server, fds[i], &frame);
            }
        }
    }
    free(json);
}

/* ---- Handlers ----------------------------------------------------------- */
static esp_err_t state_get_handler(httpd_req_t *req)
{
    char *json = state_to_json();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json ? json : "{}");
    free(json);
    return ESP_OK;
}

static esp_err_t cmd_post_handler(httpd_req_t *req)
{
    char buf[256];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad length");
        return ESP_FAIL;
    }
    int received = httpd_req_recv(req, buf, total);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
        return ESP_FAIL;
    }
    buf[received] = '\0';
    esp_err_t err = dispatch_command(buf, received);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (err == ESP_OK) {
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown command");
    }
    return err;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* Handshake; also send a state snapshot once connected. */
        ESP_LOGI(API_TAG, "ws client connected");
        return ESP_OK;
    }
    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_TEXT };
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);  /* get length */
    if (ret != ESP_OK || frame.len == 0 || frame.len > 255) {
        return ret;
    }
    uint8_t buf[256];
    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        return ret;
    }
    buf[frame.len] = '\0';
    dispatch_command((char *)buf, frame.len);

    /* Reply to this client with the fresh state. */
    char *json = state_to_json();
    if (json) {
        httpd_ws_frame_t out = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)json,
            .len = strlen(json),
        };
        httpd_ws_send_frame(req, &out);
        free(json);
    }
    return ESP_OK;
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    /* Gate on token. */
    char query[64], token[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "token", token, sizeof(token)) != ESP_OK ||
        strcmp(token, PROP_OTA_TOKEN) != 0) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "bad token");
        return ESP_FAIL;
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no ota partition");
        return ESP_FAIL;
    }
    ESP_LOGI(API_TAG, "OTA -> partition '%s' (%d bytes incoming)", target->label, req->content_len);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota begin failed");
        return ESP_FAIL;
    }

    char *buf = malloc(OTA_BUF_SIZE);
    if (!buf) {
        esp_ota_abort(handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_FAIL;
    }
    int remaining = req->content_len;
    while (remaining > 0) {
        int r = httpd_req_recv(req, buf, remaining < OTA_BUF_SIZE ? remaining : OTA_BUF_SIZE);
        if (r <= 0) {
            free(buf);
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
            return ESP_FAIL;
        }
        if (esp_ota_write(handle, buf, r) != ESP_OK) {
            free(buf);
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota write failed");
            return ESP_FAIL;
        }
        remaining -= r;
    }
    free(buf);

    if (esp_ota_end(handle) != ESP_OK || esp_ota_set_boot_partition(target) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota finalize failed");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "{\"ok\":true,\"rebooting\":true}");
    ESP_LOGW(API_TAG, "OTA complete, rebooting into '%s'", target->label);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* Minimal operator "cue board": connects over WS, buttons send scene commands. */
static const char s_console_html[] =
"<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>PROP CUE BOARD</title><style>body{background:#111;color:#e0b000;font-family:monospace;text-align:center}"
"button{background:#222;color:#e0b000;border:2px solid #e0b000;padding:14px 18px;margin:6px;font:inherit;font-size:18px}"
"#st{margin:16px;font-size:20px;white-space:pre}"
"#sens{margin:18px auto;max-width:520px;border:2px solid #e0b000;padding:14px}"
"#sens label{font-size:18px}#sr{width:100%;accent-color:#e0b000;height:28px}"
"</style></head><body><h2>PROP CUE BOARD</h2><div id=st>connecting...</div>"
"<div id=nav><b>CONSOLE</b><br></div>"
"<div id=scenes><b>SCANNER SCENES</b><br></div>"
"<div id=sens><label>SENSITIVITY <span id=sv>--</span>%</label><br>"
"<input id=sr type=range min=0 max=100 value=65></div><script>"
"var ws=new WebSocket('ws://'+location.host+'/ws');"
"var scenes=['IDLE','SCANNING','SIGNAL_ACQUIRED','COMMS','ALERT'];"
"var d=document.getElementById('scenes');scenes.forEach(function(s){var b=document.createElement('button');"
"b.textContent=s;b.onclick=function(){ws.send(JSON.stringify({cmd:'scene',value:s}))};d.appendChild(b)});"
/* Console nav: SELECTOR dial / PRESS / HOME / archive TAB switches. */
"var navs=[['\\u25C0 SEL','selector','ccw'],['PRESS','selector','press'],['SEL \\u25B6','selector','cw'],"
"['HOME','action',2],['TAB1','tab',0],['TAB2','tab',1],['TAB3','tab',2],['TAB4','tab',3]];"
"var nd=document.getElementById('nav');navs.forEach(function(s){var b=document.createElement('button');"
"b.textContent=s[0];b.onclick=function(){ws.send(JSON.stringify({cmd:'input',control:s[1],arg:s[2]}))};nd.appendChild(b)});"
"var sr=document.getElementById('sr'),sv=document.getElementById('sv'),drag=false,st=0;"
"function sendSens(){ws.send(JSON.stringify({cmd:'sens',value:+sr.value}))}"
"sr.oninput=function(){sv.textContent=sr.value;drag=true;var n=Date.now();"
"if(n-st>50){st=n;sendSens()}};"   /* throttle drags to ~20/s */
"sr.onchange=function(){drag=false;sendSens()};"   /* always send the final value */
"ws.onmessage=function(e){var o=JSON.parse(e.data);document.getElementById('st').textContent="
"'SCENE: '+o.scene+'\\nSTATUS: '+o.status+'\\n'+o.channel+'\\nLINK:'+o.link+'  IP:'+o.ip;"
"if(o.sensitivity!=null&&!drag){sr.value=o.sensitivity;sv.textContent=o.sensitivity}};"
"ws.onopen=function(){ws.send(JSON.stringify({cmd:'next_scene'}));ws.send(JSON.stringify({cmd:'scene',value:'IDLE'}))};"
"</script></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, s_console_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* GET /screenshot — capture the live screen as raw RGB565 (little-endian).
 * Dimensions come back in X-Width/X-Height headers. Decode host-side with
 * tools/screenshot.py. This is the remote "see the UI" path for graphics work. */
static esp_err_t screenshot_get_handler(httpd_req_t *req)
{
    void *buf = NULL;
    lv_img_dsc_t dsc = { 0 };
    bool ok = false;

    if (lvgl_port_lock(1000)) {
        lv_obj_t *scr = lv_scr_act();
        uint32_t sz = lv_snapshot_buf_size_needed(scr, LV_IMG_CF_TRUE_COLOR);
        if (sz > 0) {
            buf = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
            if (!buf) {
                ESP_LOGE(API_TAG, "snapshot: malloc %u failed (psram_largest=%u)",
                         (unsigned)sz,
                         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
            } else {
                lv_res_t r = lv_snapshot_take_to_buf(scr, LV_IMG_CF_TRUE_COLOR, &dsc, buf, sz);
                if (r == LV_RES_OK) {
                    ok = true;
                } else {
                    ESP_LOGE(API_TAG, "snapshot: take_to_buf returned %d", (int)r);
                }
            }
        }
        lvgl_port_unlock();
    } else {
        ESP_LOGE(API_TAG, "snapshot: could not lock LVGL");
    }
    if (!ok) {
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "snapshot failed");
        return ESP_FAIL;
    }

    char w_str[8], h_str[8];
    snprintf(w_str, sizeof(w_str), "%d", (int)dsc.header.w);
    snprintf(h_str, sizeof(h_str), "%d", (int)dsc.header.h);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Width", w_str);
    httpd_resp_set_hdr(req, "X-Height", h_str);
    httpd_resp_set_hdr(req, "X-Format", "RGB565LE");

    const char *p = (const char *)dsc.data;
    size_t remaining = dsc.data_size;
    esp_err_t err = ESP_OK;
    while (remaining > 0) {
        size_t chunk = remaining > 8192 ? 8192 : remaining;
        if (httpd_resp_send_chunk(req, p, chunk) != ESP_OK) {
            err = ESP_FAIL;
            break;
        }
        p += chunk;
        remaining -= chunk;
    }
    httpd_resp_send_chunk(req, NULL, 0);   /* end response */
    free(buf);
    return err;
}

esp_err_t prop_api_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    config.lru_purge_enable = true;
    config.stack_size = 8192;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(API_TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t root = { .uri = "/",     .method = HTTP_GET,  .handler = root_get_handler };
    httpd_uri_t st   = { .uri = "/state", .method = HTTP_GET,  .handler = state_get_handler };
    httpd_uri_t cmd  = { .uri = "/cmd",  .method = HTTP_POST, .handler = cmd_post_handler };
    httpd_uri_t ota  = { .uri = "/ota",  .method = HTTP_POST, .handler = ota_post_handler };
    httpd_uri_t ws   = { .uri = "/ws",   .method = HTTP_GET,  .handler = ws_handler, .is_websocket = true };
    httpd_uri_t shot = { .uri = "/screenshot", .method = HTTP_GET, .handler = screenshot_get_handler };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &st);
    httpd_register_uri_handler(s_server, &cmd);
    httpd_register_uri_handler(s_server, &ota);
    httpd_register_uri_handler(s_server, &ws);
    httpd_register_uri_handler(s_server, &shot);

    prop_engine_add_observer(broadcast_observer, NULL);
    ESP_LOGI(API_TAG, "HTTP API up (/, /state, /cmd, /ws, /ota, /screenshot)");
    return ESP_OK;
}
