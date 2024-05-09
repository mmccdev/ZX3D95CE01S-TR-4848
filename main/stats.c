/* Demo ESP LittleFS Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "esp_idf_version.h"
#include "esp_flash.h"
#include "stats.h"


#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_chip_info.h"
#include "spi_flash_mmap.h"
#endif
 
#include <esp_littlefs.h>

static const char *TAG = "demo_esp_littlefs";

FILE *initfile(int wday)
{
    char fname[128];
    sprintf(fname,"/littlefs/%d.data",wday);    
    //ESP_LOGI(TAG, "Init file"); 
    struct stat st;
    stat(fname, &st);
    if (st.st_size!=(LOG_COUNT_DAY*sizeof(struct watts)))
    {
        // initizialize file
        ESP_LOGI(TAG, "Initializing file %s",fname);
        FILE *fp = fopen(fname, "w");
        if (fp == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file for write");
            return NULL;
        }        
        struct watts Wattsinit = {0};
        for (int i=0;i<(LOG_COUNT_DAY);i++)
        {   
            fwrite((void *)&Wattsinit, sizeof(struct watts), 1, fp);
        }
        fclose(fp);
        //fp = fopen(fname, "r+");
    }
    FILE *fp = fopen(fname, "r+");
    if (fp == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for rw");
        return NULL;
    }
    return fp;
}

void savestat(struct watts Wattswrite)
{
    //  Open
    //struct watts Wattsread;
    struct timeval curtimeval = {.tv_sec = 0};
    struct tm *tm_info;

    curtimeval.tv_sec = Wattswrite.since70_usect / 1000000;
    tm_info = localtime(&curtimeval.tv_sec);
    ESP_LOGI(TAG, "e: %12lld, s %12lld, I %12lld, O %12lld", Wattswrite.elapsed_usect, Wattswrite.since70_usect, Wattswrite.millijouleIn, Wattswrite.millijouleOu);
    FILE *fp = initfile(tm_info->tm_wday);
    int measpos=(((tm_info->tm_hour * 60) + tm_info->tm_min) * 60 ) / LOG_INTERVAL_SEC;
    fseek(fp, (measpos*sizeof(struct watts)), SEEK_SET);
    fwrite((void *)&Wattswrite, sizeof(struct watts), 1, fp);
    fclose(fp);
    ESP_LOGI(TAG, "struct written to file");
}

short int *getstatsa(int wday)
{
    static short int ret[LOG_INTERVAL_SEC];    
    struct watts Wattsread;
    int i = 0;
    ESP_LOGI(TAG, "Reading file");
    FILE *fp = initfile(wday);
    //fp = fopen(fname, "r");
    if (fp == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return NULL;
    }
    while (!feof(fp))
    {
        fread((void *)&Wattsread, sizeof(struct watts), 1, fp);
        if (feof(fp))
            break;
        else 
        {
            //ESP_LOGI(TAG, "e: %12lld, s %12lld, I %12lld, O %12lld", Wattsread.elapsed_usect,Wattsread.since70_usect,Wattsread.millijouleIn, Wattsread.millijouleOu);
            ret[i]=(short int)(Wattsread.millijouleIn/(long long)(1000*LOG_INTERVAL_SEC));  
            //3600/LOG_INTERVAL_SEC
            //solar_sincetodaystart = (int)(Wattsread.millijouleIn/3600000);
            //LOG_INTERVAL_SEC
        }
        i++;
    }
    fclose(fp);
    return ret;

}

void initstats(void)
{
    printf("Demo LittleFs implementation by esp_littlefs!\n");
    printf("   https://github.com/joltwallet/esp_littlefs\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    uint32_t size_flash_chip = 0;
    esp_flash_get_size(NULL, &size_flash_chip);
    printf("%uMB %s flash\n", (unsigned int)size_flash_chip >> 20,
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %u\n", (unsigned int)esp_get_free_heap_size());

    printf("Now we are starting the LittleFs Demo ...\n");

    ESP_LOGI(TAG, "Initializing LittleFS");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

}
#ifdef NOTDEFINED
void dummy(void)
{
    printf("Demo LittleFs implementation by esp_littlefs!\n");
    printf("   https://github.com/joltwallet/esp_littlefs\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    uint32_t size_flash_chip = 0;
    esp_flash_get_size(NULL, &size_flash_chip);
    printf("%uMB %s flash\n", (unsigned int)size_flash_chip >> 20,
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %u\n", (unsigned int)esp_get_free_heap_size());

    printf("Now we are starting the LittleFs Demo ...\n");

    ESP_LOGI(TAG, "Initializing LittleFS");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen("/littlefs/hello.txt", "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "LittleFS Rocks!\n");
    fclose(f);
    ESP_LOGI(TAG, "File written");

    // Check if destination file exists before renaming
    struct stat st;
    if (stat("/littlefs/foo.txt", &st) == 0)
    {
        // Delete it if it exists
        unlink("/littlefs/foo.txt");
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file");
    if (rename("/littlefs/hello.txt", "/littlefs/foo.txt") != 0)
    {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen("/littlefs/foo.txt", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    char line[128];
    char *pos;

    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    pos = strchr(line, '\n');
    if (pos)
    {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    ESP_LOGI(TAG, "Reading from flashed filesystem example.txt");
    f = fopen("/littlefs/example.txt", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    pos = strchr(line, '\n');
    if (pos)
    {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    // All done, unmount partition and disable LittleFS
    esp_vfs_littlefs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "LittleFS unmounted");
}
#endif