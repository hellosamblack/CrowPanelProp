# ESPectre CSI motion-detection core (GPL-3.0), vendored from
# github.com/francescopace/espectre — algorithm files only; ESPHome glue dropped.
# The `include/` dir holds the esphome/core/log.h shim. See LICENSE (GPL-3.0):
# this component makes the C6 slave image a GPL-3.0 derivative.
idf_component_register(
    SRCS "base_detector.cpp"
         "mvs_detector.cpp"
         "nbvi_calibrator.cpp"
         "gain_controller.cpp"
         "csi_manager.cpp"
         "csi_filters.cpp"
         "calibration_file_buffer.cpp"
         "prop_espectre.cpp"
    INCLUDE_DIRS "." "include"
    REQUIRES esp_wifi esp_timer spiffs
)

# Vendored third-party code, kept verbatim — don't fail the build on its style
# warnings (member-init order, ++ on volatile, missing-field-initializers, …).
# Scoped to this component only; the rest of the slave keeps -Werror.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
