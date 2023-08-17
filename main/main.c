#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lvgl.h"
#include "board.h"
#include "esp_timer.h"
#include "ui.h"

#define TAG "MAIN"

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

extern void screen_init(void);
extern void sht20_test_start(void);
extern void epever_modbus_start(void);

void lvgl_task(void* arg) {

    lv_init();
    screen_init();
    ui_init();

    // Tick interface for LVGL
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = increase_lvgl_tick,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, portTICK_PERIOD_MS * 1000);
#if LV_USE_DEMO_WIDGETS
    extern void lv_demo_widgets(void);
    lv_demo_widgets();
 #endif   
 #ifdef CONFIG_ZX3D95CE01S_TR_4848 
    sht20_test_start();
 #endif   
    epever_modbus_start();

    for (;;) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    xTaskCreatePinnedToCore(lvgl_task, NULL, 16 * 1024, NULL, 4, NULL, 1);
}
