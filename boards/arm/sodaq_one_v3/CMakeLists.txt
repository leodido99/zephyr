zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)

if(CONFIG_PINMUX_SAM0)
  zephyr_library()
  zephyr_library_sources(pinmux.c)
endif()