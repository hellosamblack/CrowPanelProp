/* prop_settings — NVS-backed persistent key/value store. See prop_settings.h. */
#include "prop_settings.h"
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#define SET_TAG "PROP_SET"
#define NVS_NAMESPACE "propcfg"

/* NVS caps key names at NVS_KEY_NAME_MAX_SIZE-1 = 15 chars; a longer key is
 * silently rejected by the underlying nvs_set_u32/nvs_get_str/etc. calls
 * (ESP_ERR_NVS_KEY_TOO_LONG), and every accessor below only special-cases
 * ESP_ERR_NVS_NOT_FOUND, so an over-length key used to just look like "this
 * setting always resets to its default" with nothing in the log to explain
 * why. Catch it up front and say so loudly instead — this bit a real feature
 * once ("audio_ping_voice", 16 chars, one over the limit). */
static bool key_len_ok(const char *key)
{
    if (strlen(key) >= NVS_KEY_NAME_MAX_SIZE) {
        ESP_LOGE(SET_TAG, "key '%s' is %d chars, NVS caps keys at %d -- rejected",
                 key, (int)strlen(key), NVS_KEY_NAME_MAX_SIZE - 1);
        return false;
    }
    return true;
}

esp_err_t prop_settings_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Only triggered on a blank/format-changed NVS partition — wipes NVS data
         * (not user values written later), then re-inits. Not hit by normal flashes. */
        ESP_LOGW(SET_TAG, "NVS needs erase (%s), reformatting", esp_err_to_name(ret));
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(SET_TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t prop_settings_get_str(const char *key, char *out, size_t out_len, const char *def)
{
    if (!key || !out || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Seed with default so callers always get a usable value. */
    strlcpy(out, def ? def : "", out_len);
    if (!key_len_ok(key)) {
        return ESP_ERR_NVS_KEY_TOO_LONG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return ESP_OK;   /* namespace not created yet -> default is fine */
    }
    size_t len = out_len;
    err = nvs_get_str(h, key, out, &len);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strlcpy(out, def ? def : "", out_len);
        return ESP_OK;
    }
    return err;
}

esp_err_t prop_settings_set_str(const char *key, const char *value)
{
    if (!key) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!key_len_ok(key)) {
        return ESP_ERR_NVS_KEY_TOO_LONG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, key, value ? value : "");
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(SET_TAG, "saved '%s'", key);
    }
    return err;
}

esp_err_t prop_settings_get_u32(const char *key, uint32_t *out, uint32_t def)
{
    if (!key || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = def;
    if (!key_len_ok(key)) {
        return ESP_ERR_NVS_KEY_TOO_LONG;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return ESP_OK;
    }
    esp_err_t err = nvs_get_u32(h, key, out);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *out = def;
        return ESP_OK;
    }
    return err;
}

esp_err_t prop_settings_set_u32(const char *key, uint32_t value)
{
    if (!key) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!key_len_ok(key)) {
        return ESP_ERR_NVS_KEY_TOO_LONG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u32(h, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

bool prop_settings_has(const char *key)
{
    if (!key) {
        return false;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = 0;
    esp_err_t err = nvs_get_str(h, key, NULL, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        uint32_t tmp;
        err = nvs_get_u32(h, key, &tmp);
    }
    nvs_close(h);
    return err == ESP_OK;
}
