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
#include "driver/ledc.h"
#include "ui.h"
#include "epever.h"
#include "board.h"

#define TAG "epever display"
#define LEDC_DUTY_0             (0) // Set duty to 0%. 
#define LEDC_DUTY_12            (1023) // Set duty to 12%. 
#define LEDC_DUTY_25            (2047) // Set duty to 25%. 
#define LEDC_DUTY_50            (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_DUTY_100           (8191) // Set duty to 100%.
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

struct measures
{
    signed long long PInCC;
    signed long long POutCC;
    signed long long POutInv;
};

struct watts 
{
    unsigned long long elapsed_usect;
    unsigned long long millijoulein;
    unsigned long long millijouleout;
};
volatile epever_load_inverter_t epever_load_g;
volatile epever_realtime_cc_t epever_realtime_1_g;
volatile epever_realtime_cc_t epever_realtime_2_g;
volatile epever_statistical_cc_t epever_statistical_1_g;
volatile epever_statistical_cc_t epever_statistical_2_g;
volatile epever_rated_cc_t epever_rated_g;
volatile bool setinverter = true;
volatile int sundisplaytoggle = 0,battdisplaytoggle = 0,displayontoggle = true;
#define measurecount CONFIG_MAIN_AVGBUF
volatile int swperiod = measurecount * 2;


void display_set_backlight(int percentage) 
{
    int duty = (percentage * 0xFF) / 100;
    if (duty != ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1)) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

static void display_backlight_init() 
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,        // resolution of PWM duty
        .freq_hz         = 2000,                    // frequency of PWM signal
        .speed_mode      = LEDC_LOW_SPEED_MODE,     // timer mode
        .timer_num       = LEDC_TIMER_2,            // timer index
        .clk_cfg         = LEDC_AUTO_CLK,           // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {.channel    = LEDC_CHANNEL_1,
                                          .duty       = 0xFF,
                                          .gpio_num   = LCD_PIN_BK_LIGHT,
                                          .speed_mode = LEDC_LOW_SPEED_MODE,
                                          .hpoint     = 0,
                                          .timer_sel  = LEDC_TIMER_2};
    ledc_channel_config(&ledc_channel);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    display_set_backlight(40);
}

void prepareinverteroff()
{
    setinverter = false;
    swperiod = 1000; // based on a loop time of 0.5 a second 500 seconds
}

void ui_event_Inverterswitch(lv_event_t *e)
{
    if (displayontoggle == true)
    {
        lv_event_code_t event_code = lv_event_get_code(e);
        if (event_code == LV_EVENT_VALUE_CHANGED)
        {
            if (setinverter == false)
            {
                // it looks like esp-modbus is not thread safe so pass a variable
                setinverter = true;
            }
            else
            {
                prepareinverteroff();
            }
        }
    }
}
void ui_event_SliderSunnosundraw(lv_event_t *e)
{
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->type != LV_OBJ_DRAW_PART_RECTANGLE)
        return;
    lv_obj_set_x(ui_LabelSunnosun, dsc->draw_area->x1-8);
}

void ui_event_SliderChargeDischargedraw(lv_event_t *e)
{
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->type != LV_OBJ_DRAW_PART_RECTANGLE)
        return;
    lv_obj_set_x(ui_LabelChargeDischarge, dsc->draw_area->x1-8);
}


void ui_event_LabelSunnosunclicked(lv_event_t *e)
{
    sundisplaytoggle = !sundisplaytoggle;
}

void ui_event_LabelChargeDischargeclicked(lv_event_t *e)
{
    battdisplaytoggle = !battdisplaytoggle;
}
void ui_event_ImageNoSunclicked(lv_event_t *e)
{
    displayontoggle = false;
}

void ui_event_ScreenOn(lv_event_t *e)
{
    displayontoggle = true;
}

#define SECS_IN_HOUR (60 * 60)
#define SECS_IN_DAY (24 * SECS_IN_HOUR)
int daynum(long long usect, int utc_offset_min)
{
    return ((usect/1000000)+utc_offset_min*60)/SECS_IN_DAY;
}


void __epever_modbus_task(void *user_data)
{
    uint32_t loop = 1;
    int idx = loop;
    unsigned long long interval_usec = 0;
    float sunbar,battbar;
    struct timeval start, stop; 
    struct measures sum = {0};
    struct watts running = {0}, sincefull ={0}, todaystart ={0} ;
    volatile struct measures avg = {0};
    struct measures measurebuf[measurecount] = {0};
    ESP_ERROR_CHECK(master_init(BOARD_MB_UART_PORT_NUM,BOARD_MB_UART_RXD,BOARD_MB_UART_TXD));
    epever_rated_cc_t epever_rated_cc = {};
    epever_setting_cc_t epever_setting_cc = {};
    epever_status_inverter_t epever_status_inverter = {};
    epever_discrete_cc_t epever_discrete_cc = {};
    struct timeval displaytimeval = {.tv_sec = 0};
    display_backlight_init();
    display_set_backlight(100);
    master_read_rated_cc(&epever_rated_cc);
    master_read_setting_cc(&epever_setting_cc);
    struct timeval now = {.tv_sec = epever_setting_cc.RealTimeClock};
    settimeofday(&now, NULL);
    master_read_discrete_cc(&epever_discrete_cc);
    gettimeofday(&start, NULL);

    lv_obj_add_event_cb(ui_Inverterbutton, ui_event_Inverterswitch, LV_EVENT_ALL, NULL);

    lv_obj_add_event_cb(ui_SliderChargeDischarge, ui_event_SliderChargeDischargedraw, LV_EVENT_DRAW_PART_BEGIN , NULL);
    lv_obj_add_event_cb(ui_SliderSunnosun, ui_event_SliderSunnosundraw, LV_EVENT_DRAW_PART_BEGIN , NULL);

    lv_obj_add_event_cb(ui_LabelChargeDischarge, ui_event_LabelChargeDischargeclicked, LV_EVENT_CLICKED , NULL);
    lv_obj_add_event_cb(ui_LabelSunnosun, ui_event_LabelSunnosunclicked, LV_EVENT_CLICKED , NULL);

    lv_obj_add_event_cb(ui_ImageNoSun, ui_event_ImageNoSunclicked, LV_EVENT_CLICKED , NULL);
    lv_obj_add_event_cb(ui_ImageSun, ui_event_ScreenOn, LV_EVENT_CLICKED , NULL);

    lv_obj_add_event_cb(ui_Screen1, ui_event_ScreenOn, LV_EVENT_CLICKED , NULL);

    while (1)
    {
        ESP_LOGV("LOOP ", "%d", loop);
        // master_set_status_inverter(0);
        //  read all significant
        struct tm *tm_info;
        char buffer[512];
        //char buffer2[18];
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
            lv_imgbtn_set_state(ui_Inverterbutton, LV_IMGBTN_STATE_RELEASED); 
        else
            lv_imgbtn_set_state(ui_Inverterbutton, LV_IMGBTN_STATE_CHECKED_PRESSED);
        master_read_statistical_cc1(&epever_statistical_1_g);
        master_read_statistical_cc2(&epever_statistical_2_g);
        gettimeofday(&stop, NULL);
        interval_usec = ((stop.tv_sec - start.tv_sec) * 1000000) + (stop.tv_usec - start.tv_usec);

        running.elapsed_usect = running.elapsed_usect + interval_usec;

        running.millijoulein += ((epever_realtime_1_g.ChargingOutputPower + epever_realtime_2_g.ChargingOutputPower) * (interval_usec)) / 100000;
        running.millijouleout += ((epever_load_g.LoadOutputPower + epever_realtime_1_g.DischargingOutputPower + epever_realtime_2_g.DischargingOutputPower) * (interval_usec)) / 100000;
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
        //backlight
        if (displayontoggle == false)
        {
            display_set_backlight(0);
        }
        else if (avg.PInCC < 50) // .5 amp
        {
            // could be because full battery
            if (epever_load_g.LoadInputVoltage > 1340)
            {
                display_set_backlight(100);
            }
            else
            {
                display_set_backlight(1);
            }
        }
        else if (avg.PInCC < 20000) // 300 amp
        {
            display_set_backlight(50);
        }
        else
        {
            display_set_backlight(100);
        }
        if ((((running.elapsed_usect - sincefull.elapsed_usect) / 1000000) > 3600) && (epever_load_g.LoadInputVoltage > 1400))
        // min 1 hour after last reset and more than 14 volts on the inverter (battery)
        {
            // propose  epever_load_g.LoadInputVoltage>1430 but need to check for a good idea
            // 1400 spotted in operation
            sincefull.elapsed_usect = running.elapsed_usect;
            sincefull.millijoulein = running.millijoulein;
            sincefull.millijouleout = running.millijouleout;
        }

        if (daynum(running.elapsed_usect,0)>daynum(todaystart.elapsed_usect,0))
        {
            // new day reset all running numbers / should be written to non volatile memory
            todaystart.elapsed_usect = running.elapsed_usect;
            todaystart.millijoulein = running.millijoulein;
            todaystart.millijouleout = running.millijouleout;
        }

        if ((loop % swperiod) == 0) // switch every swperiod loops
        {
            if (epever_status_inverter.InverterOnoff == false)
            {
                setinverter = true;          // set it back to on so the avarage can be measured
                swperiod = measurecount * 2; // measurecount is the average buffer size. Times 2 because the first measurements might be inacurate
            }
            else if ((avg.PInCC < 15000) && (avg.POutInv < 2400) && (epever_load_g.LoadInputVoltage<1400)) // if less than 150 watt from the panels and below 24 watt demand (the consumption of the inverter)
            // else if (avg.POutInv<2400)  // below 24 watt (the consumption of the inverter itself)
            {
                prepareinverteroff();
            }
            else
            {
                setinverter = epever_status_inverter.InverterOnoff; // if it was 0 it would have been caught in the first if
                swperiod = measurecount * 2;                        // measurecount is the average buffer size. Times 2 because the first measurements might be inacurate
            }
        }
        // display the stuff
        // the panels
        if (sundisplaytoggle==0) sunbar = (float)avg.PInCC/100;
        else sunbar = (float)measurebuf[idx].PInCC/100;
        lv_label_set_text_fmt(ui_LabelSunnosun, "%c%7.2f",((sundisplaytoggle==0 )? 'A': ' '),sunbar);
        lv_bar_set_value(ui_SliderSunnosun, sunbar, LV_ANIM_OFF);
       // the battery
        if (battdisplaytoggle==0) battbar = (float)(avg.PInCC - avg.POutCC - avg.POutInv)/100;
        else battbar = (float)(measurebuf[idx].PInCC - measurebuf[idx].POutCC - measurebuf[idx].POutInv)/100;
        lv_label_set_text_fmt(ui_LabelChargeDischarge, "%c%7.2f",((battdisplaytoggle==0 )? 'A': ' '),battbar);
        lv_bar_set_value(ui_SliderChargeDischarge, battbar, LV_ANIM_OFF);
        {

            
            displaytimeval.tv_sec = running.elapsed_usect / 1000000;
            tm_info = localtime(&displaytimeval.tv_sec);
            strftime(buffer,sizeof(buffer) , "Dlt %j %H:%M:%S\n", tm_info);

            displaytimeval.tv_sec = (running.elapsed_usect - sincefull.elapsed_usect)/1000000;
            tm_info = localtime(&displaytimeval.tv_sec);
            strftime(buffer + strlen(buffer) ,sizeof(buffer) , "D B %j %H:%M:%S\n", tm_info);

            displaytimeval.tv_sec = (running.elapsed_usect - todaystart.elapsed_usect)/1000000;
            tm_info = localtime(&displaytimeval.tv_sec);
            strftime(buffer + strlen(buffer) ,sizeof(buffer) , "D D %j %H:%M:%S\n", tm_info);

            //todaystart.elapsed_usect


            float battdischarge = (((float)(running.millijoulein-sincefull.millijoulein)-(float)(running.millijouleout-sincefull.millijouleout))/3600000000);
            sprintf(buffer + strlen(buffer),"Dif.T.KWh %7.3f\n",battdischarge);

            float battdischarge_sincetodaystart = (((float)(running.millijoulein-todaystart.millijoulein)-(float)(running.millijouleout-todaystart.millijouleout))/3600000000);
            sprintf(buffer + strlen(buffer),"Dif.d.KWh %7.3f\n",battdischarge_sincetodaystart);

            sprintf(buffer + strlen(buffer),"Dlt.sec %7.6f\n",(float)interval_usec/1000000);
            sprintf(buffer + strlen(buffer),"Inv.V %7.2f\n",((float)epever_load_g.LoadInputVoltage)/100);
            sprintf(buffer + strlen(buffer),"CC2.V %7.2f\n",((float)epever_realtime_2_g.DischargingOutputVoltage)/100);
            sprintf(buffer + strlen(buffer),"In 1 %4d 2 %4d\n",epever_statistical_1_g.TodayGeneratedEnergy*10,epever_statistical_2_g.TodayGeneratedEnergy*10);
            lv_label_set_text_fmt(ui_BattLabelval,"%s",&buffer[0]);
            //lv_label_set_text_fmt(ui_BattLabelval,"%s\n%s\nDif.KWh %7.3f\nLoop %07X\nDlt.sec %7.6f\nInv.V %7.2f\nCC2.V %7.2f\nIn 1 %4d 2 %4d ",
             //           &buffer[0],&buffer2[0],battdischarge,loop,(float)interval_usec/1000000,((float)epever_load_g.LoadInputVoltage)/100,((float)epever_realtime_2_g.DischargingOutputVoltage)/100,epever_statistical_1_g.TodayGeneratedEnergy*10,epever_statistical_2_g.TodayGeneratedEnergy*10);
//            lv_label_set_text_fmt(ui_BattLabelval,"%s\n%s\nDif.KWh %7.3f\nLoop %07X\nDlt.sec %7.6f\nInv.V %7.2f\nCC2.V %7.2f\nIn 1 %4d 2 %4d ",
 //                       &buffer[0],&buffer2[0],battdischarge,loop,(float)interval_usec/1000000,((float)epever_load_g.LoadInputVoltage)/100,((float)epever_realtime_2_g.DischargingOutputVoltage)/100,epever_statistical_1_g.TodayGeneratedEnergy*10,epever_statistical_2_g.TodayGeneratedEnergy*10);
            lv_bar_set_value(ui_Batterybar,(int)(battdischarge*1000), LV_ANIM_OFF);
        }
        tm_info = localtime(&start.tv_sec);
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

                ssd1306_display_mytext(&dev_g,1,"%7.2f","Inv.V",((float)epever_load_g.LoadInputVoltage)/100);
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
    xTaskCreatePinnedToCore(__epever_modbus_task, TAG, 8 * 1024, NULL, 5, NULL, 0);
}
