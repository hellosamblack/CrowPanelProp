#ifndef _PROP_SETTINGS_H_
#define _PROP_SETTINGS_H_

/* prop_settings — persistent key/value store for prop configuration.
 *
 * Backed by NVS in a dedicated namespace, so values survive reboots AND firmware
 * reflashes: `idf.py flash` only rewrites the bootloader/partition-table/app, it
 * never touches the `nvs` data partition. (Only `idf.py erase-flash` wipes it.)
 *
 * This is the single home for anything that should persist: WiFi credentials now,
 * plus whatever else we add later (default scene, brightness, AP name, ...).
 * Add a key string, call get/set — no schema changes needed.
 *
 * Known keys:
 *   "sta_ssid"  (str)  upstream WiFi SSID
 *   "sta_pass"  (str)  upstream WiFi password
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* Initialize NVS and the settings namespace. Call once, early in boot, before any
 * other module reads settings (prop_net, prop_ui). */
esp_err_t prop_settings_init(void);

/* String get/set. get copies the stored value (or `def` if absent) into out. */
esp_err_t prop_settings_get_str(const char *key, char *out, size_t out_len, const char *def);
esp_err_t prop_settings_set_str(const char *key, const char *value);

/* Unsigned-int get/set for numeric settings. */
esp_err_t prop_settings_get_u32(const char *key, uint32_t *out, uint32_t def);
esp_err_t prop_settings_set_u32(const char *key, uint32_t value);

/* True if a value is stored for key. */
bool prop_settings_has(const char *key);

#endif /* _PROP_SETTINGS_H_ */
