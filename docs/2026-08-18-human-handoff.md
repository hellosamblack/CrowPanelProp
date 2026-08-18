# Human Handoff — CrowPanel prop needs a power cycle (2026-08-18)

## TL;DR

**Unplug the prop, wait 5 seconds, plug it back in.** That's the only thing blocking.
Everything else (the audit's bug fixes, performance work, and LIDAR UI redesign) is
implemented, builds clean, and a fixed firmware image is ready to flash over OTA.

## What happened

The first OTA of the new firmware uploaded fine, the board said "rebooting", and then never
reappeared on the network (~30+ min of polling, mDNS, ping, and a full subnet sweep). Two
possible reasons, and a power cycle fixes both:

1. **The new firmware hung during boot** in a hardware-accelerated graphics call (my prime
   suspect — that call has since been removed and replaced with a safe, equally fast
   approach). Because it hung rather than crashed, the automatic OTA rollback never
   triggered — rollback needs a reset, and a power cycle IS that reset. You'll get the
   **previous working firmware** back.
2. **The C6 WiFi chip wedged on the warm reboot** (a known quirk of this board). The new
   firmware would actually be running fine with a dead radio. A cold boot fixes it.

**Helpful datapoint:** before power-cycling, glance at the screen and note what it shows —
a frozen or partial display means scenario 1; a normal live SCANNER screen means scenario 2.

## After the power cycle

- If a Claude session is still running with the recovery watcher, it will detect the board
  and push the fixed firmware automatically — nothing more to do.
- Otherwise, from `~/git/personal/CrowPanelProp` on the Linux box:

  ```bash
  # wait until the board answers (up to ~90 s after power-on)
  curl -s http://comm-unit-7.local/state
  # push the already-built fixed firmware
  curl -X POST "http://comm-unit-7.local/ota?token=prop-ota-2024" \
       --data-binary @build/communicator.bin --fail
  ```

  (or hand it to a fresh Claude session — it should read
  `docs/2026-08-18-agent-handoff-recovery.md` and take it from there, including the
  remaining on-device smoke tests.)

## If the board does NOT come back after a power cycle

Then it's boot-looping or the radio is truly broken, and we need serial to see why:
connect the board's USB (CH340 → shows as `/dev/ttyUSB*` on Linux or COM7-ish on Windows)
to whichever machine will run the next session. The flash coredump partition will hold the
first panic if there was one (`idf.py coredump-info` — see the `crash-forensics` skill).

## State of the code

- All changes are **uncommitted** in the working tree at `~/git/personal/CrowPanelProp`
  (13 files; `git diff` to inspect). Don't reset/clean the tree.
- `build/communicator.bin` is the fixed build (the risky PPA call removed).
- Full technical detail + remaining verification steps:
  `docs/2026-08-18-agent-handoff-recovery.md`.
