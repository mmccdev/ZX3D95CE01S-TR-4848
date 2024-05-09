// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t * ui_Screen1;
extern lv_obj_t * ui_BattPanel;
extern lv_obj_t * ui_Batterybar;
extern lv_obj_t * ui_BattLabelval;
extern lv_obj_t * ui_BatteryImage;
extern lv_obj_t * ui_Inverterbutton;
extern lv_obj_t * ui_SunnosunPanel;
extern lv_obj_t * ui_SliderSunnosun;
extern lv_obj_t * ui_LabelSunnosun;
extern lv_obj_t * ui_ImageSun;
extern lv_obj_t * ui_ImageNoSun;
extern lv_obj_t * ui_PanelChargeDisCharge;
extern lv_obj_t * ui_SliderChargeDischarge;
extern lv_obj_t * ui_LabelChargeDischarge;
extern lv_obj_t * ui_ImageCharge;
extern lv_obj_t * ui_WeekPanel;
extern lv_obj_t * ui_Labeltime;
extern lv_obj_t * ui_SunChart;
extern lv_obj_t * ui_Heatmap;
extern lv_obj_t * ui_WeekLabel;
extern lv_obj_t * ui____initial_actions0;


LV_IMG_DECLARE(ui_img_battery_png);    // assets/battery.png
LV_IMG_DECLARE(ui_img_offb_png);    // assets/offb.png
LV_IMG_DECLARE(ui_img_onb_png);    // assets/onb.png
LV_IMG_DECLARE(ui_img_sun_png);    // assets/sun.png
LV_IMG_DECLARE(ui_img_dark_png);    // assets/dark.png
LV_IMG_DECLARE(ui_img_charge_png);    // assets/charge.png






void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
