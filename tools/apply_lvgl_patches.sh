#!/usr/bin/env bash
# Re-apply the three local LVGL 9.4.0 patches after ANY fresh managed_components
# download (fresh clone, `idf.py fullclean` + re-resolve, lvgl version re-pin).
#
# Why this exists: commit b68e7cf9 fixed 3 real bugs in LVGL 9.4's *experimental*
# Espressif PPA draw unit (CONFIG_LV_USE_PPA=y) and tracked the patched files in
# git for durability. A later repo reorg (0ad5e09b) untracked managed_components/
# as "regenerable", silently discarding that durability. A stock re-download then
# reintroduces the bugs: every sub-region PPA fill garbles (missing left rail,
# grey/fragmented buttons, corrupted chrome, flickering) while full-area fills
# look fine — see the 2026-08-18 recovery session.
#
# The patched files live forever in git history, so this script just restores
# them from the commit that fixed them:
#   - env_support/cmake/esp.cmake        (esp_driver_ppa/esp_mm deps unconditional,
#                                         else `fatal error: driver/ppa.h` at build)
#   - src/draw/espressif/ppa/lv_draw_ppa.c       (seed draw_area before cache inval)
#   - src/draw/espressif/ppa/lv_draw_ppa_fill.c  (stride/offset/blocking-mode fixes)
#
# Symptom this cures: rail/buttons/chrome render grey/garbled/missing on hardware.
set -euo pipefail

PATCH_COMMIT=b68e7cf9
LVGL_DIR="$(cd "$(dirname "$0")/.." && pwd)/managed_components/lvgl__lvgl"
EXPECTED_VER="9.4.0"

if [[ ! -d "$LVGL_DIR" ]]; then
    echo "error: $LVGL_DIR not found — run idf.py reconfigure first to download components" >&2
    exit 1
fi

ver=$(grep -m1 '^version:' "$LVGL_DIR/idf_component.yml" | awk '{print $2}' | tr -d '"')
if [[ "$ver" != "$EXPECTED_VER" ]]; then
    echo "error: lvgl component is $ver, patches are against $EXPECTED_VER — re-diff the" >&2
    echo "       three files in commit $PATCH_COMMIT against the new version before applying" >&2
    exit 1
fi

cd "$(dirname "$0")/.."
for f in env_support/cmake/esp.cmake \
         src/draw/espressif/ppa/lv_draw_ppa.c \
         src/draw/espressif/ppa/lv_draw_ppa_fill.c; do
    git show "$PATCH_COMMIT:managed_components/lvgl__lvgl/$f" > "$LVGL_DIR/$f"
    echo "patched: $f"
done
echo "done — rebuild with idf.py build"
