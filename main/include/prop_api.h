#ifndef _PROP_API_H_
#define _PROP_API_H_

/* prop_api — live control surface over WiFi (the rapid-prototyping core).
 *
 * Starts an HTTP server exposing:
 *   GET  /           -> operator "cue board" web console (single HTML page)
 *   GET  /state      -> current prop state as JSON
 *   GET  /telemetry  -> live sensor/instrument snapshot as JSON
 *   GET  /screenshot -> raw RGB565 framebuffer grab (whole panel)
 *   GET  /ld2450     -> LD2450 radar config read (live UART queries)
 *   POST /cmd        -> one JSON command (REST/curl-friendly)
 *   WS   /ws         -> bidirectional JSON: push commands, receive state + telemetry
 *   POST /ota        -> stream a new firmware .bin to the inactive OTA slot, reboot
 *
 * Command schema (same for /cmd and /ws):
 *   {"cmd":"scene","value":"SCANNING"}
 *   {"cmd":"next_scene"}
 *   {"cmd":"led","name":"alert","on":true}     (or "index":2)
 *   {"cmd":"status","value":"SIGNAL ACQUIRED"}
 *   {"cmd":"channel","value":"CH 04 / 147.55 MHz"}
 *   {"cmd":"wifi","ssid":"...","pass":"..."}
 *
 * Registers itself as a prop_engine observer and broadcasts state changes to all
 * connected WebSocket clients so every console stays in sync.
 */

#include "esp_err.h"

/* OTA is gated by this token (sent as ?token=... on POST /ota). Change it. */
#define PROP_OTA_TOKEN "prop-ota-2024"

esp_err_t prop_api_init(void);

#endif /* _PROP_API_H_ */
