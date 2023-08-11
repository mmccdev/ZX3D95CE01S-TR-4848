#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "board.h"
#include "sht20.h"
#include "ui.h"

#define TAG "SHT20 TeST"

void __sht20_test_task(void* user_data)
{    
    sht20_init(TP_SDA, TP_SCL);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    while(1){
        float get_tem_data = sht20_get_temperature();
        float get_hum_data = sht20_get_humidity();
        char label[22];
        sprintf(&label[0],"Temp %5.3f Hum %5.3f",get_tem_data,get_hum_data);        
        //printf("sht20: %f/%f\n", get_tem_data, get_hum_data);
        lv_label_set_text(ui_BattLabelRctn, &label[0]); //22
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void sht20_test_start(void)
{
    xTaskCreatePinnedToCore(__sht20_test_task, "sht20", 4 * 1024, NULL, 5, NULL, 0);
}