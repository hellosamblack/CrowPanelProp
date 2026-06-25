/*
 * prop_traffic — host-side CSI traffic generator.
 *
 * ESPectre needs a steady stream of received Wi-Fi frames to compute CSI, but
 * the CSI capture lives on the C6 while the IP stack (lwip) lives on the P4 —
 * the C6 slave can't generate IP traffic itself. So the P4 pings the gateway at
 * a configurable rate; the replies are RX frames at the C6 → a steady HT20 CSI
 * stream for the detector. Mirrors ESPectre's traffic_generator (ping mode).
 *
 * Rate/mode come from the runtime config (traffic_generator_rate / _mode),
 * pushed here by prop_coproc — so it's tunable live, no reflash.
 */
#ifndef PROP_TRAFFIC_H
#define PROP_TRAFFIC_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the generator task. Idle until a rate>0 is set and STA is up. */
esp_err_t prop_traffic_init(void);

/* Packets/sec to the gateway (0 = off). */
void prop_traffic_set_rate(int packets_per_sec);

/* 0 = ping (ICMP), 1 = dns. v1 implements ping; dns falls back to ping. */
void prop_traffic_set_mode(int mode);

#ifdef __cplusplus
}
#endif

#endif /* PROP_TRAFFIC_H */
