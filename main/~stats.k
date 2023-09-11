#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "stats.h"

#define TAG "NVS"
#define STORAGE_NAMESPACE "statsnvs"
// #define STORAGE_NAMESPACE "storage"

void savestat(struct watts Watts)
{
    size_t wattsSize = sizeof(struct watts);
    nvs_handle_t handle;
    esp_err_t err;
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
    // time_t secs;
    struct timeval displaytimeval = {.tv_sec = 0};
    struct tm *tm_info;
    char key[8];
    displaytimeval.tv_sec = Watts.since70_usect / 1000000;
    tm_info = localtime(&displaytimeval.tv_sec);
    strftime(key, sizeof(key), "%w%H%M", tm_info);
            
    ESP_LOGI(TAG, "%s e: %12lld, s %12lld, I %12lld, O %12lld", key, Watts.elapsed_usect,Watts.since70_usect,Watts.millijouleIn, Watts.millijouleOu);

    ESP_ERROR_CHECK(nvs_set_blob(handle, key, (void *)&Watts, wattsSize));

    ESP_ERROR_CHECK(nvs_commit(handle));
}

void liststats()
{

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    
    struct watts Watts;
    size_t wattsSize = sizeof(struct watts);
    nvs_handle_t handle;
    // esp_err_t err;
    char key[8];
    // Open
    // err =
    nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);

    for (int weeknum = 0; weeknum < 7; weeknum++)
    {
        for (int hour = 0; hour < 24; hour++)
        {
            for (int min = 0; min < 60; min += 5)
            {
                sprintf(key, "%1d%2d%2d", weeknum, hour, min);
                esp_err_t result = nvs_get_blob(handle, key, (void *)&Watts, &wattsSize);
                switch (result)
                {
                case ESP_ERR_NOT_FOUND:
                case ESP_ERR_NVS_NOT_INITIALIZED:

                case ESP_ERR_NVS_NOT_FOUND:
                    //ESP_LOGE(TAG, "Value not set yet");
                    break;
                case ESP_OK:
            
                    ESP_LOGI(TAG, "%s e: %12lld, s %12lld, I %12lld, O %12lld", key, Watts.elapsed_usect,Watts.since70_usect,Watts.millijouleIn, Watts.millijouleOu);
                    break;
                default:
                    ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(result));
                    break;
                }
            }
        }
    }
}

void app_spare_main()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(nvs_flash_init_partition("statsnvs"));

    nvs_handle handle;
    ESP_ERROR_CHECK(nvs_open_from_partition("statsnvs", "stats_store", NVS_READWRITE, &handle));

    char catKey[50];
    struct watts Watts;
    size_t catSize = sizeof(struct watts);
    for (int i = 0; i < 5; i++)
    {
        sprintf(catKey, "cat_%d", i);
        esp_err_t result = nvs_get_blob(handle, catKey, (void *)&Watts, &catSize);
        // printf("actual cat size=%d returned from NVS =%d\n", sizeof(Cat), catSize);

        switch (result)
        {
        case ESP_ERR_NOT_FOUND:
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "Value not set yet");
            break;
        case ESP_OK:
            ESP_LOGI(TAG, "elapsed_usect: %d, millijouleIn %d, millijouleOu %d", Watts.elapsed_usect, Watts.millijouleIn, Watts.millijouleOu);
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(result));
            break;
        }
    }
    /*
    for (int i = 0; i < 5; i++)
    {
      sprintf(catKey, "cat_%d", i);
      Cat newCat;
      sprintf(newCat.name,"Mr cat %d", i);
      newCat.age = i + 2;
      newCat.id = i;
      ESP_ERROR_CHECK(nvs_set_blob(handle, catKey, (void *) &newCat, sizeof(Cat) ));
      ESP_ERROR_CHECK(nvs_commit(handle));
    }
  */
    nvs_close(handle);
}