idf_component_register(
    SRCS
        "main.c"
        "ota.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        driver
        nvs_flash
        app_update
        espressif__esp-zigbee-lib
        espressif__led_strip
)

# Gli header di compatibilità 1.x di esp-zigbee-lib 2.x (include/compat/*.h) emettono
# '#warning "This file is deprecated..."' a ogni inclusione. -Wno-cpp nasconde solo i #warning
# e solo per i file di questo componente; gli altri warning (con -Werror) restano attivi.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-cpp)
