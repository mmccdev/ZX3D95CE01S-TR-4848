idf_component_get_property(LVGL_DIR lvgl COMPONENT_DIR)

file(GLOB LVGL_DEMOS 
    ${LVGL_DIR}/demos/widgets
    ${LVGL_DIR}/demos/widgets/assets
)

idf_component_register(
    SRC_DIRS ${DIRS} ${LVGL_DEMOS} "."
    INCLUDE_DIRS "." ${LVGL_DEMOS} ${DIRS}
    REQUIRES esp_lcd driver lvgl esp_timer bus touch_panel sht20 
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
littlefs_create_partition_image(littlefs ../flash_data FLASH_IN_PROJECT)
#add_subdirectory(ui)
