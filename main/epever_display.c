#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"

#include "ui.h"
#include "epever.h"
#include "board.h"

#define TAG "epever modbus"
struct measures
{
    signed long long PInCC;
    signed long long POutCC;
    signed long long POutInv;
};
volatile epever_load_inverter_t epever_load_g;
volatile epever_realtime_cc_t epever_realtime_1_g;
volatile epever_realtime_cc_t epever_realtime_2_g;
volatile epever_statistical_cc_t epever_statistical_1_g;
volatile epever_statistical_cc_t epever_statistical_2_g;
volatile epever_rated_cc_t epever_rated_g;
volatile bool setinverter = true;

void ui_event_Inverterswitch(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    // lv_obj_t * target = lv_event_get_target(e);
    if (event_code == LV_EVENT_VALUE_CHANGED)
    {
        //  lv_obj_has_state(ui_Inverterswitch, LV_STATE_CHECKED)
        if (setinverter == false)
        {
            // it looks like esp-modbus is not thread safe so pass a variable
            // master_set_status_inverter(1);
            setinverter = true;
        }
        else
        {
            // master_set_status_inverter(0);
            setinverter = false;
        }
    }
}

void __epever_modbus_task(void *user_data)
{
    uint32_t loop = 1;
    int idx = loop;
    unsigned long long elapsed_usec = 0;
    unsigned long long interval_usec = 0;
    unsigned long long cdentijoulein = 0;
    unsigned long long centijouleout = 0;

    unsigned long long full_elapsed_usec = 0;
    // unsigned long long interval_usec=0;
    unsigned long long full_cdentijoulein = 0;
    unsigned long long full_centijouleout = 0;
    // bool inverteron = true;
#define measurecount CONFIG_MAIN_AVGBUF
    int swperiod = measurecount * 2;
    // int blankcount=0;
    struct timeval start, stop; //, inverterarm={0};
    struct measures sum = {0};
    volatile struct measures avg = {0};
    struct measures measurebuf[measurecount] = {0};
    ESP_ERROR_CHECK(master_init(BOARD_MB_UART_PORT_NUM,BOARD_MB_UART_RXD,BOARD_MB_UART_TXD));
    epever_rated_cc_t epever_rated_cc = {};
    epever_setting_cc_t epever_setting_cc = {};
    epever_status_inverter_t epever_status_inverter = {};
    epever_discrete_cc_t epever_discrete_cc = {};
    struct timeval elapsedtimeval = {.tv_sec = 0};

    master_read_rated_cc(&epever_rated_cc);
    master_read_setting_cc(&epever_setting_cc);
    struct timeval now = {.tv_sec = epever_setting_cc.RealTimeClock};
    settimeofday(&now, NULL);
    master_read_discrete_cc(&epever_discrete_cc);
    gettimeofday(&start, NULL);

    lv_obj_add_event_cb(ui_Inverterbutton, ui_event_Inverterswitch, LV_EVENT_ALL, NULL);

    while (1)
    {
        ESP_LOGI("LOOP ", "%d", loop);
        // master_set_status_inverter(0);
        //  read all significant
        struct tm *tm_info;
        char buffer[16];
        master_read_realtime_cc1(&epever_realtime_1_g);
        master_read_realtime_cc2(&epever_realtime_2_g);
        master_read_load_inverter(&epever_load_g);
        master_read_status_inverter(&epever_status_inverter);
        if (epever_status_inverter.InverterOnoff != setinverter)
        {
            master_set_status_inverter(setinverter);
            master_read_status_inverter(&epever_status_inverter);
        }
        if (epever_status_inverter.InverterOnoff == false)
            //setled(1);
            lv_imgbtn_set_state(ui_Inverterbutton, LV_IMGBTN_STATE_RELEASED); 
        else
            lv_imgbtn_set_state(ui_Inverterbutton, LV_IMGBTN_STATE_CHECKED_PRESSED);
             //setled(0);
        master_read_statistical_cc1(&epever_statistical_1_g);
        master_read_statistical_cc2(&epever_statistical_2_g);
        gettimeofday(&stop, NULL);
        interval_usec = ((stop.tv_sec - start.tv_sec) * 1000000) + (stop.tv_usec - start.tv_usec);
        elapsed_usec = elapsed_usec + interval_usec;
        cdentijoulein += ((epever_realtime_1_g.ChargingOutputPower + epever_realtime_2_g.ChargingOutputPower) * (interval_usec)) / 1000000;
        centijouleout += ((epever_load_g.LoadOutputPower + epever_realtime_1_g.DischargingOutputPower + epever_realtime_2_g.DischargingOutputPower) * (interval_usec)) / 1000000;
        idx = loop % measurecount;
        {
            sum.PInCC -= measurebuf[idx].PInCC;
            sum.POutCC -= measurebuf[idx].POutCC;
            sum.POutInv -= measurebuf[idx].POutInv;
            measurebuf[idx].PInCC = epever_realtime_1_g.ChargingOutputPower + epever_realtime_2_g.ChargingOutputPower;
            measurebuf[idx].POutCC = epever_realtime_1_g.DischargingOutputPower + epever_realtime_2_g.DischargingOutputPower;
            measurebuf[idx].POutInv = epever_load_g.LoadOutputPower;
            sum.PInCC += measurebuf[idx].PInCC;
            sum.POutCC += measurebuf[idx].POutCC;
            sum.POutInv += measurebuf[idx].POutInv;
            int divider = ((loop + 1) > measurecount ? measurecount : (idx + 1));
            avg.PInCC = sum.PInCC / divider;
            avg.POutCC = sum.POutCC / divider;
            avg.POutInv = sum.POutInv / divider;
        }
        start = stop;
        if ((((elapsed_usec - full_elapsed_usec) / 1000000) > 3600) && (epever_load_g.LoadInputVoltage > 1400))
        // min 1 hour after last reset and more than 14 volts on the inverter (battery)
        {
            // propose  epever_load_g.LoadInputVoltage>1430 but need to check for a good idea
            // 1400 spotted in operation
            full_elapsed_usec = elapsed_usec;
            full_cdentijoulein = cdentijoulein;
            full_centijouleout = centijouleout;
        }
        // alternative logic try to keep it simple
        if ((loop % swperiod) == 0) // switch every swperiod loops
        {
            if (epever_status_inverter.InverterOnoff == false)
            {
                setinverter = true;          // set it back to on so the avarage can be measured
                swperiod = measurecount * 2; // measurecount is the average buffer size. Times 2 because the first measurements might be inacurate
            }
            else if ((avg.PInCC < 15000) && (avg.POutInv < 2400)) // if less than 150 watt from the panels and below 24 watt demand
            // else if (avg.POutInv<2400)  // below 24 watt
            {
                setinverter = false;
                swperiod = 1000; // based on a loop time of 0.5 a second 500 seconds
            }
            else
            {
                setinverter = epever_status_inverter.InverterOnoff; // if it was 0 it would have been caught in the first if
                swperiod = measurecount * 2;                        // measurecount is the average buffer size. Times 2 because the first measurements might be inacurate
            }
        }
        // display the stuff
        //lv_bar_set_mode(ui_Barchargedischarge, LV_BAR_MODE_SYMMETRICAL);
        //lv_bar_set_value(ui_Barchargedischarge, ((float)(((int64_t)epever_realtime_1_g.ChargingOutputPower + (int64_t)epever_realtime_2_g.ChargingOutputPower) - ((int64_t)epever_load_g.LoadOutputPower + (int64_t)epever_realtime_1_g.DischargingOutputPower + (int64_t)epever_realtime_2_g.DischargingOutputPower))) / 100, LV_ANIM_OFF);
        lv_label_set_text_fmt(ui_LabelSunnosun, "%7.2f",((float)(int64_t)epever_realtime_1_g.ChargingOutputPower+(int64_t)epever_realtime_2_g.ChargingOutputPower)/100);
        lv_bar_set_value(ui_SliderSunnosun, ((float)(int64_t)epever_realtime_1_g.ChargingOutputPower+(int64_t)epever_realtime_2_g.ChargingOutputPower)/100, LV_ANIM_OFF);
        
        lv_label_set_text_fmt(ui_Labelchargedischarge, "%7.2f",((float)(((int64_t)epever_realtime_1_g.ChargingOutputPower+(int64_t)epever_realtime_2_g.ChargingOutputPower)-((int64_t)epever_load_g.LoadOutputPower+(int64_t)epever_realtime_1_g.DischargingOutputPower+(int64_t)epever_realtime_2_g.DischargingOutputPower)))/100);
        lv_bar_set_value(ui_SliderChargeDischarge, ((float)(((int64_t)epever_realtime_1_g.ChargingOutputPower+(int64_t)epever_realtime_2_g.ChargingOutputPower)-((int64_t)epever_load_g.LoadOutputPower+(int64_t)epever_realtime_1_g.DischargingOutputPower+(int64_t)epever_realtime_2_g.DischargingOutputPower)))/100, LV_ANIM_OFF);
        // need to reset tm_info after this useless to use a seperate struct because it will be overwritten
        {

        }
        {
            //char label[8];
            //sprintf(&label[0], "Dis%cKWh", (full_cdentijoulein > 0) ? ':' : '.');
            //ssd1306_display_mytext(&dev_g, 5, "%7.3f", &label[0], (((float)(cdentijoulein - full_cdentijoulein) - (float)(centijouleout - full_centijouleout)) / 360000000));
            float battdischarge = (((float)(cdentijoulein-full_cdentijoulein)-(float)(centijouleout-full_centijouleout))/360000000);
            elapsedtimeval.tv_sec = elapsed_usec / 1000000;
            tm_info = localtime(&elapsedtimeval.tv_sec);
            // time and date
            strftime(buffer, 17, "Dlt %j %H:%M:%S", tm_info);
            // ssd1306_display_mytext(&dev_g, 0, "", &buffer[0], 0);
            //lv_label_set_text(ui_BattLabelRctn, &buffer[0]);

            lv_label_set_text_fmt(ui_BattLabelval,"%s\nDif.KWh %7.3f\nLoop %07X\nDlt.sec %7.6f",&buffer[0],battdischarge,loop,(float)interval_usec/1000000);


            lv_bar_set_value(ui_Batterybar,(int)(battdischarge*1000), LV_ANIM_OFF);
        }
        tm_info = localtime(&start.tv_sec);
        //sprintf(&label[0],"%02d.%02d  %1d",tm_info->tm_hour,tm_info->tm_min,disppage);
        lv_label_set_text_fmt(ui_Labeltime,"%02d:%02d",tm_info->tm_hour,tm_info->tm_min);
        // ssd1306_display_mytext(&dev_g,5,"%7.2f","TOT.W",((fl oat)(((int64_t)epever_realtime_1_g.ChargingOutputPower+(int64_t)epever_realtime_2_g.ChargingOutputPower)-((int64_t)epever_load_g.LoadOutputPower+(int64_t)epever_realtime_1_g.DischargingOutputPower+(int64_t)epever_realtime_2_g.DischargingOutputPower)))/100);
        /*
        switch (disppage)
        {
            case 0:
                ssd1306_display_mytext(&dev_g,0,"%7.2f","Inv.W",(-(float)epever_load_g.LoadOutputPower)/100);
                ssd1306_display_mytext(&dev_g,1,"%7.2f","Inv.V",((float)epever_load_g.LoadInputVoltage)/100);
                ssd1306_display_mytext(&dev_g,2,"%7.2f","CC1,2.W",((float)(int64_t)epever_realtime_1_g.ChargingOutputPower+(int64_t)epever_realtime_2_g.ChargingOutputPower)/100);
                ssd1306_display_mytext(&dev_g,3,"%7.2f","       ",(-(float)((int64_t)epever_realtime_1_g.DischargingOutputPower+(int64_t)epever_realtime_2_g.DischargingOutputPower))/100);
                //ssd1306_display_mytext(&dev_g,4,"","",0);
                ssd1306_display_mytext(&dev_g,4,"%7.2f","CC2.V",((float)epever_realtime_2_g.DischargingOutputVoltage)/100);
                ssd1306_display_mytext(&dev_g,5,"%7.2f","TOT.W",((float)(((int64_t)epever_realtime_1_g.ChargingOutputPower+(int64_t)epever_realtime_2_g.ChargingOutputPower)-((int64_t)epever_load_g.LoadOutputPower+(int64_t)epever_realtime_1_g.DischargingOutputPower+(int64_t)epever_realtime_2_g.DischargingOutputPower)))/100);
                ssd1306_display_mytext(&dev_g,6,"","",0);
                break;
            case 1:
                ssd1306_display_mytext(&dev_g,0,"%7.2f","Inv.W",(-(float)avg.POutInv)/100);
                ssd1306_display_mytext(&dev_g,1,"%7.2f","Inv.V",((float)epever_load_g.LoadInputVoltage)/100);
                ssd1306_display_mytext(&dev_g,2,"%7.2f","CC1,2.W",((float)(int64_t)avg.PInCC)/100);
                ssd1306_display_mytext(&dev_g,3,"%7.2f","       ",(-(float)(int64_t)avg.POutCC)/100);
                ssd1306_display_mytext(&dev_g,4,"","",0);
                ssd1306_display_mytext(&dev_g,5,"%7.2f","TOT.W",((float)(((int64_t)avg.PInCC)-((int64_t)avg.POutInv+(int64_t)avg.POutCC)))/100);
                ssd1306_display_mytext(&dev_g,6,"","",0);
                break;
            case 2:
                {
                    tm_info = localtime(&start.tv_sec);
                    // time and date
                    strftime(buffer, 17, "%d%b%y %H:%M:%S", tm_info);
                    ssd1306_display_mytext(&dev_g,0,"",&buffer[0],0);
                }
                //interval_usec
                ssd1306_display_mytext(&dev_g,1,"%7.6f","Dlt.sec",(float)interval_usec/1000000);

                if ((loop%2)==0)
                {
                    ssd1306_display_mytext(&dev_g,2,"%7.0f","CC1.D.W",((float)(int64_t)epever_statistical_1_g.TodayGeneratedEnergy)*10);
                    ssd1306_display_mytext(&dev_g,5,"%7.2f","Bat1.T",((float)(int64_t)epever_realtime_1_g.BatteryTemperature)/100);
                    ssd1306_display_mytext(&dev_g,6,"%7.2f","CC1.T",((float)(int64_t)epever_realtime_1_g.ChargerCaseTemperature)/100);
                }
                else
                {
                    ssd1306_display_mytext(&dev_g,2,"%7.0f","CC2.D.W",((float)(int64_t)epever_statistical_2_g.TodayGeneratedEnergy)*10);
                    ssd1306_display_mytext(&dev_g,5,"%7.2f","Bat2.T",((float)(int64_t)epever_realtime_2_g.BatteryTemperature)/100);
                    ssd1306_display_mytext(&dev_g,6,"%7.2f","CC2.T",((float)(int64_t)epever_realtime_2_g.ChargerCaseTemperature)/100);
                }
                ssd1306_display_mytext(&dev_g,3,"","",0);
                ssd1306_display_mytext(&dev_g,4,"","",0);
                break;
            case 3:
                elapsedtimeval.tv_sec = elapsed_usec/1000000;
                // need to reset tm_info after this useless to use a seperate struct because it will be overwritten
                {
                    tm_info = localtime(&elapsedtimeval.tv_sec);
                    // time and date
                    strftime(buffer, 17, "Dlt %j %H:%M:%S", tm_info);
                    ssd1306_display_mytext(&dev_g,0,"",&buffer[0],0);
                }

                // usec is 1000000
                // 1h = 3600 sec
                // all powers in W but centiwatts so also centijoules
                ssd1306_display_mytext(&dev_g,2,"%7.3f","In .KWh",((float)cdentijoulein/360000000));
                ssd1306_display_mytext(&dev_g,3,"%7.3f","Out.KWh",((float)centijouleout/360000000));
                ssd1306_display_mytext(&dev_g,4,"%7.3f","Dif.KWh",(((float)cdentijoulein-(float)centijouleout)/360000000));
                {
                    char label[8];
                    sprintf(&label[0],"Dis%cKWh",(full_cdentijoulein>0) ? ':':'.');
                    ssd1306_display_mytext(&dev_g,5,"%7.3f",&label[0],(((float)(cdentijoulein-full_cdentijoulein)-(float)(centijouleout-full_centijouleout))/360000000));
                }

            // full_elapsed_usec=elapsed_usec;
            // full_cdentijoulein=cdentijoulein;
            // full_centijouleout=centijouleout;

                //ssd1306_display_mytext(&dev_g,5,"%7.2f","In .W",(((float)cdentijoulein*10000)/(float)elapsed_usec));

                //ssd1306_display_mytext(&dev_g,6,"%7.2f","Out.KJ",((float)centijouleout/100000));

                //ssd1306_display_mytext(&dev_g,6,"%7.2f","Out.W",(((float)centijouleout*10000)/(float)elapsed_usec));

                ssd1306_display_mytext(&dev_g,1,"","",0);

                //ssd1306_display_mytext(&dev_g,5,"","",0);
                ssd1306_display_mytext(&dev_g,6,"","",0);
                //ssd1306_display_mytext(&dev_g,1,"","",0);
                //ssd1306_display_mytext(&dev_g,4,"","",0);
                break;

            default:
                break;
        }
        // always displayed
        {
            char label[18];
            // somehow tm_info
            tm_info = localtime(&start.tv_sec);
            sprintf(&label[0],"%02d.%02d  %1d",tm_info->tm_hour,tm_info->tm_min,disppage);
            sprintf(&label[0],"%1d %07X  %02d%c%02d",disppage,loop,tm_info->tm_hour,(setinverter==false) ? '.':':',tm_info->tm_min);
            ssd1306_display_mytext(&dev_g,7,"",&label[0],0);
        }
        */
        // loop = (loop+1)%100000000;
        loop = (loop + 1) % 0xFFFFFFF;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void epever_modbus_start(void)
{
    xTaskCreatePinnedToCore(__epever_modbus_task, "epever modbus", 4 * 1024, NULL, 5, NULL, 1);
}
