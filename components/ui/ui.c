// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.1
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_BattPanel;
lv_obj_t * ui_Batterybar;
lv_obj_t * ui_BattLabelval;
void ui_event_BatteryImage(lv_event_t * e);
lv_obj_t * ui_BatteryImage;
lv_obj_t * ui_Inverterbutton;
lv_obj_t * ui_EnvironLabelRctn;
lv_obj_t * ui_SunnosunPanel;
lv_obj_t * ui_SliderSunnosun;
lv_obj_t * ui_LabelSunnosun;
lv_obj_t * ui_ImageSun;
lv_obj_t * ui_ImageNoSun;
lv_obj_t * ui_PanelChargeDisCharge;
lv_obj_t * ui_SliderChargeDischarge;
lv_obj_t * ui_LabelChargeDischarge;
lv_obj_t * ui_ImageCharge;
lv_obj_t * ui_Labeltime;
lv_obj_t * ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_BatteryImage(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_BattLabelval, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_basic_init(dispp);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen1);
}
