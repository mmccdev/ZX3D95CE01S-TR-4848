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
#include "esp_log.h"
#include "esp_debug_helpers.h"
#include "stats.h"

#define TAG "epever display"
#define LEDC_DUTY_0             (0) // Set duty to 0%. 
#define LEDC_DUTY_12            (1023) // Set duty to 12%. 
#define LEDC_DUTY_25            (2047) // Set duty to 25%. 
#define LEDC_DUTY_50            (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_DUTY_100           (8191) // Set duty to 100%.
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

#define CANVAS_WIDTH  288
#define CANVAS_HEIGHT 112
#define HEAT_LINE_HEIGHT 12

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

void ui_event_BatteryImage(lv_event_t * e)
{
    // lv_event_code_t event_code = lv_event_get_code(e);
    // if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_BattLabelval, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        ESP_LOGI(TAG, "Toggle battery");
        //esp_backtrace_print(20); 
    //}
}


#define SECS_IN_HOUR (60 * 60)
#define SECS_IN_DAY (24 * SECS_IN_HOUR)
int daynum(long long usect, int utc_offset_min)
{
    return ((usect/1000000)+utc_offset_min*60)/SECS_IN_DAY;
}
void drawmespoint(lv_obj_t * canvas,int weekday,int x,int point)
{
    
    //if (x > 40)
    {
        uint32_t y;
        for (y = 0 + (weekday * HEAT_LINE_HEIGHT); y < HEAT_LINE_HEIGHT + (weekday * HEAT_LINE_HEIGHT); y++) // one measurement is a line
        {
            lv_canvas_set_px_color(canvas, x, y, lv_color_make((uint8_t)(point / 2), (uint8_t)(point / 2), 0x80 - (uint8_t)(point / 4)));
        }
    }
}
void setweeklabel(lv_obj_t * canvas,int * totalday )
{
    char weeklabelarr[128] = {0};    
    for (int weekday = 0; weekday < 7; weekday++)
    {
        sprintf(weeklabelarr + strlen(weeklabelarr),"%6d\n",(totalday[weekday]/(12)));
    }
    ESP_LOGI(TAG, "%s",weeklabelarr);
    //lv_label_set_text_fmt(ui_WeekLabel, "%s","weeklabelarr");
    lv_label_set_text(ui_WeekLabel,weeklabelarr);
}

void initlvgl()
{
    lv_obj_add_event_cb(ui_Inverterbutton, ui_event_Inverterswitch, LV_EVENT_ALL, NULL);

    lv_obj_add_event_cb(ui_SliderChargeDischarge, ui_event_SliderChargeDischargedraw, LV_EVENT_DRAW_PART_BEGIN , NULL);
    lv_obj_add_event_cb(ui_SliderSunnosun, ui_event_SliderSunnosundraw, LV_EVENT_DRAW_PART_BEGIN , NULL);

    lv_obj_add_event_cb(ui_LabelChargeDischarge, ui_event_LabelChargeDischargeclicked, LV_EVENT_CLICKED , NULL);
    lv_obj_add_event_cb(ui_LabelSunnosun, ui_event_LabelSunnosunclicked, LV_EVENT_CLICKED , NULL);

    lv_obj_add_event_cb(ui_ImageNoSun, ui_event_ImageNoSunclicked, LV_EVENT_CLICKED , NULL);
    lv_obj_add_event_cb(ui_ImageSun, ui_event_ScreenOn, LV_EVENT_CLICKED , NULL);

    lv_obj_add_event_cb(ui_Screen1, ui_event_ScreenOn, LV_EVENT_CLICKED , NULL);

    lv_obj_add_event_cb(ui_BatteryImage, ui_event_BatteryImage, LV_EVENT_CLICKED, NULL);

    lv_chart_set_point_count( ui_SunChart, LOG_COUNT_DAY);

    lv_obj_move_foreground(ui_WeekLabel);
}

void copywatts(struct watts *to,struct watts *from)
{
    //to->elapsed_usect
    to->elapsed_usect = from->elapsed_usect;
    to->since70_usect = from->since70_usect;
    to->millijouleIn = from->millijouleIn;
    to->millijouleOu = from->millijouleOu;
}

void __epever_modbus_task(void *user_data)
{
    const long loginterval_usec = (1000000 * LOG_INTERVAL_SEC);
    uint32_t loop = 1;
    int idx = loop;
    unsigned long long interval_usec = 0;
    //long long since70_usect_ls =0;
    float sunbar,battbar;
    struct timeval start, stop; 
    struct measures sum = {0};
    struct watts running = {0}, sincefull ={0}, todaystart ={0},save = {0};
    volatile struct measures avg = {0};
    struct measures measurebuf[measurecount] = {0};
    ESP_ERROR_CHECK(master_init(BOARD_MB_UART_PORT_NUM,BOARD_MB_UART_RXD,BOARD_MB_UART_TXD));
    epever_rated_cc_t epever_rated_cc = {};
    epever_setting_cc_t epever_setting_cc = {};
    epever_status_inverter_t epever_status_inverter = {};
    epever_discrete_cc_t epever_discrete_cc = {};
    struct timeval displaytimeval = {.tv_sec = 0};
    struct tm *tm_info;    

    display_backlight_init();
    display_set_backlight(100);

    master_read_rated_cc(&epever_rated_cc);
    master_read_setting_cc(&epever_setting_cc);
    struct timeval now = {.tv_sec = epever_setting_cc.RealTimeClock};
    settimeofday(&now, NULL);
    master_read_discrete_cc(&epever_discrete_cc);
    gettimeofday(&start, NULL);
    
    lv_obj_t * canvas = lv_canvas_create(ui_WeekPanel);
    static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT)];
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);    

    initlvgl();
    initstats();
    
    tm_info = localtime(&now.tv_sec);

    short int *pointsa ;//= getstatsa(tm_info->tm_wday);
    

//ui_WeekPanel
//    int CANVAS_Width = lv_obj_get_width(ui_WeekPanel);
//    int CANVAS_Height = lv_obj_get_height(ui_WeekPanel);
    //lv_obj_move_foreground(ui_WeekLabel);
    uint32_t x;
    uint32_t weekday;
    int totalday[7]={0,0,0,0,0,0,0};
    for (weekday = 0; weekday < 7; weekday++)
    {
        pointsa = getstatsa(weekday);
        for (x = 0; x < CANVAS_WIDTH; x++)
        {
            totalday[weekday]+=pointsa[x];
            drawmespoint(canvas, weekday, x,pointsa[x]);
        }
       
    }
    setweeklabel(ui_WeekLabel,totalday);

    pointsa = getstatsa(tm_info->tm_wday);
    /*
    for (x=0;x<256;x++)
    {
        lv_canvas_set_palette(canvas,x,lv_color_make((uint8_t)(x/2), (uint8_t)(x/2), 0x80-(uint8_t)(x/4)));
    }
    */
    lv_chart_series_t *ui_SunChart_series_1 = lv_chart_add_series(ui_SunChart, lv_color_hex(0x808080), LV_CHART_AXIS_SECONDARY_Y);
    // static lv_coord_t ui_SunChart_series_1_array[] = { 0,10,20,40,80,80,40,20,10,0 };
    lv_chart_set_ext_y_array(ui_SunChart, ui_SunChart_series_1, pointsa);
    while (1)
    {
        ESP_LOGV("LOOP ", "%d", loop);
        //  read all significant
        char buffer[1024];
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
        running.since70_usect = (stop.tv_sec* 1000000) + stop.tv_usec;
        running.millijouleIn += ((epever_realtime_1_g.ChargingOutputPower + epever_realtime_2_g.ChargingOutputPower) * (interval_usec)) / 100000;
        running.millijouleOu += ((epever_load_g.LoadOutputPower + epever_realtime_1_g.DischargingOutputPower + epever_realtime_2_g.DischargingOutputPower) * (interval_usec)) / 100000;
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

            //epever_realtime_1_g.ChargingInputVoltage
            //epever_realtime_2_g.ChargingInputVoltage

        }
        start = stop;
        //backlight
        if (displayontoggle == false)
        {
            display_set_backlight(0);
        }
        else if (avg.PInCC < 1) // .01 W
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
        else if (avg.PInCC < 20000) // 200 W
        {
            display_set_backlight(50);
        }
        else
        {
            display_set_backlight(100);
        }
        displaytimeval.tv_sec = running.since70_usect / 1000000;
        tm_info = localtime(&displaytimeval.tv_sec);

        if ((((running.elapsed_usect - sincefull.elapsed_usect) / 1000000) > 3600) && (epever_load_g.LoadInputVoltage > 1400))
        // min 1 hour after last reset and more than 14 volts on the inverter (battery)
        {
            // propose  epever_load_g.LoadInputVoltage>1430 but need to check for a good idea
            // 1400 spotted in operation
            copywatts(&sincefull,&running);
            // sincefull.elapsed_usect = running.elapsed_usect;
            // sincefull.since70_usect = running.since70_usect;
            // sincefull.millijouleIn = running.millijouleIn;
            // sincefull.millijouleOu = running.millijouleOu;
        }

        if (daynum((running.since70_usect * 1000000) ,0)>daynum(todaystart.since70_usect,0)) 
        {
            // new day reset all running numbers / should be written to non volatile memory
            pointsa = getstatsa(tm_info->tm_wday);    
            copywatts(&todaystart,&running);
            // todaystart.elapsed_usect = running.elapsed_usect;
            // todaystart.since70_usect = running.since70_usect;
            // todaystart.millijouleIn = running.millijouleIn;
            // todaystart.millijouleOu = running.millijouleOu;
            //totalday[tm_info->tm_wday]=0;
        }
        
        if (((running.since70_usect)/loginterval_usec)>((save.since70_usect)/loginterval_usec))
        {
            if (save.since70_usect>0)
            {
                struct watts Watts;
                //sprintf(buffer,"\nsavetime\tusect\tmillijouleIn\tmillijouleOu\n");
                //displaytimeval.tv_sec = running.since70_usect / 1000000;
                //tm_info = localtime(&displaytimeval.tv_sec);
                // strftime(buffer + strlen(buffer),sizeof(buffer) , "%Y-%m-%d %H:%M:%S\t", tm_info);            
                // sprintf(buffer + strlen(buffer),"%12d\t",running.elapsed_usect-save.elapsed_usect);
                // sprintf(buffer + strlen(buffer),"%12d\t",running.millijouleIn-save.millijouleIn);
                // sprintf(buffer + strlen(buffer),"%12d\n",running.millijouleOu-save.millijouleOu);
                //ESP_LOGI("stats","%s", buffer);
                //copywatts(&todaystart,&running);
                Watts.elapsed_usect= running.elapsed_usect-save.elapsed_usect;
                Watts.since70_usect = running.since70_usect;
                Watts.millijouleIn = running.millijouleIn-save.millijouleIn;
                Watts.millijouleOu = running.millijouleOu-save.millijouleOu;
                
                int measpos=(((tm_info->tm_hour * 60) + tm_info->tm_min) * 60 ) / LOG_INTERVAL_SEC;

                totalday[tm_info->tm_wday] -=pointsa[measpos];
                pointsa[measpos]=(short int)(Watts.millijouleIn/(long long)(1000*LOG_INTERVAL_SEC));
                totalday[tm_info->tm_wday] +=pointsa[measpos];
                drawmespoint(canvas, tm_info->tm_wday, measpos,pointsa[measpos]);
                setweeklabel(ui_WeekLabel,totalday);

                vTaskDelay(pdMS_TO_TICKS(10));
                savestat(Watts);
                vTaskDelay(pdMS_TO_TICKS(10));
                //vTaskDelay(pdMS_TO_TICKS(10));

                //vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            copywatts(&save,&running);
            // save.elapsed_usect = running.elapsed_usect;
            // save.since70_usect = running.since70_usect;
            // save.millijouleIn = running.millijouleIn;
            // save.millijouleOu = running.millijouleOu; 
        }
        
        if ((loop % swperiod) == 0) // switch every swperiod loops
        {
            if (epever_status_inverter.InverterOnoff == false)
            {
                setinverter = true;          // set it back to on so the avarage can be measured
                swperiod = measurecount * 2; // measurecount is the average buffer size. Times 2 because the first measurements might be inacurate
            }
            else if ((avg.PInCC < 20000) && (avg.POutInv < 2600) && (epever_load_g.LoadInputVoltage<1400)) // if less than 150 watt from the panels and below 24 watt demand (the consumption of the inverter)
            // else if (avg.POutInv<2400)  // below 26 watt (the consumption of the inverter itself)
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
            strftime(buffer,sizeof(buffer) , "DT\t%j %H:%M:%S\n", tm_info);

            displaytimeval.tv_sec = (running.elapsed_usect - sincefull.elapsed_usect)/1000000;
            tm_info = localtime(&displaytimeval.tv_sec);
            strftime(buffer + strlen(buffer) ,sizeof(buffer) , "DF\t%j %H:%M:%S\n", tm_info);

            displaytimeval.tv_sec = (running.elapsed_usect - todaystart.elapsed_usect)/1000000;
            tm_info = localtime(&displaytimeval.tv_sec);
            strftime(buffer + strlen(buffer) ,sizeof(buffer) , "DD\t%j %H:%M:%S\n", tm_info);

            //todaystart.elapsed_usect

            float battdischarge = (((float)(running.millijouleIn-sincefull.millijouleIn)-(float)(running.millijouleOu-sincefull.millijouleOu))/3600000000);
            sprintf(buffer + strlen(buffer),"Bat.KWh\t%7.3f\n",battdischarge);

            float solar_sincetodaystart = (((float)(running.millijouleIn-todaystart.millijouleIn))/3600000000);
            sprintf(buffer + strlen(buffer),"Sol.KWh\t%7.3f\n",solar_sincetodaystart);

            sprintf(buffer + strlen(buffer),"Dlt.sec\t%5.6f\n",(float)interval_usec/1000000);
            sprintf(buffer + strlen(buffer),"Inv.V\t%7.2f\n",((float)epever_load_g.LoadInputVoltage)/100);
            sprintf(buffer + strlen(buffer),"CC2.V\t%7.2f\n",((float)epever_realtime_2_g.DischargingOutputVoltage)/100);
            sprintf(buffer + strlen(buffer),"In 1\t%4d 2 %4d\n",epever_statistical_1_g.TodayGeneratedEnergy*10,epever_statistical_2_g.TodayGeneratedEnergy*10);
            sprintf(buffer + strlen(buffer),"Solar\t%7.2f\n",((float)sunbar*10000));
            //sunbar)
            lv_label_set_text_fmt(ui_BattLabelval,"%s",buffer);
            //lv_label_set_text_fmt(ui_BattLabelval,"%s\n%s\nDif.KWh %7.3f\nLoop %07X\nDlt.sec %7.6f\nInv.V %7.2f\nCC2.V %7.2f\nIn 1 %4d 2 %4d ",
             //           &buffer[0],&buffer2[0],battdischarge,loop,(float)interval_usec/1000000,((float)epever_load_g.LoadInputVoltage)/100,((float)epever_realtime_2_g.DischargingOutputVoltage)/100,epever_statistical_1_g.TodayGeneratedEnergy*10,epever_statistical_2_g.TodayGeneratedEnergy*10);
//            lv_label_set_text_fmt(ui_BattLabelval,"%s\n%s\nDif.KWh %7.3f\nLoop %07X\nDlt.sec %7.6f\nInv.V %7.2f\nCC2.V %7.2f\nIn 1 %4d 2 %4d ",
 //                       &buffer[0],&buffer2[0],battdischarge,loop,(float)interval_usec/1000000,((float)epever_load_g.LoadInputVoltage)/100,((float)epever_realtime_2_g.DischargingOutputVoltage)/100,epever_statistical_1_g.TodayGeneratedEnergy*10,epever_statistical_2_g.TodayGeneratedEnergy*10);
            lv_bar_set_value(ui_Batterybar,(int)(battdischarge*1000), LV_ANIM_OFF);
        }
        tm_info = localtime(&start.tv_sec);
        lv_label_set_text_fmt(ui_Labeltime,"%02d:%02d",tm_info->tm_hour,tm_info->tm_min);
        loop = (loop + 1) % 0xFFFFFFF;
        //lv_task_handler();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void epever_modbus_start(void)
{
    xTaskCreatePinnedToCore(__epever_modbus_task, TAG, 8 * 1024, NULL, 5, NULL, 0);
}
