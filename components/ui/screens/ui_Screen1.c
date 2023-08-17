// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.1
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BattPanel = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_BattPanel, 150);
    lv_obj_set_height(ui_BattPanel, 280);
    lv_obj_set_x(ui_BattPanel, 324);
    lv_obj_set_y(ui_BattPanel, 180);
    lv_obj_clear_flag(ui_BattPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BattPanel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BattPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Batterybar = lv_bar_create(ui_BattPanel);
    lv_bar_set_range(ui_Batterybar, -2500, 0);
    lv_bar_set_value(ui_Batterybar, -7, LV_ANIM_OFF);
    lv_obj_set_width(ui_Batterybar, 145);
    lv_obj_set_height(ui_Batterybar, 238);
    lv_obj_set_x(ui_Batterybar, 0);
    lv_obj_set_y(ui_Batterybar, 6);
    lv_obj_set_align(ui_Batterybar, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Batterybar, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Batterybar, lv_color_hex(0x7F0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Batterybar, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Batterybar, lv_color_hex(0x007F00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Batterybar, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_BattLabelval = lv_label_create(ui_BattPanel);
    lv_obj_set_width(ui_BattLabelval, 130);
    lv_obj_set_height(ui_BattLabelval, 200);
    lv_obj_set_x(ui_BattLabelval, 0);
    lv_obj_set_y(ui_BattLabelval, 1);
    lv_obj_set_align(ui_BattLabelval, LV_ALIGN_CENTER);
    lv_label_set_long_mode(ui_BattLabelval, LV_LABEL_LONG_CLIP);
    lv_label_set_text(ui_BattLabelval, "x.xxx\nfdsf\nsffsf");
    lv_obj_add_flag(ui_BattLabelval, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_BattLabelval, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                      LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_scrollbar_mode(ui_BattLabelval, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_text_color(ui_BattLabelval, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_BattLabelval, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BatteryImage = lv_img_create(ui_BattPanel);
    lv_img_set_src(ui_BatteryImage, &ui_img_battery_png);
    lv_obj_set_width(ui_BatteryImage, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_BatteryImage, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_BatteryImage, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BatteryImage, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_BatteryImage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_BatteryImage, 500);

    ui_Inverterbutton = lv_imgbtn_create(ui_Screen1);
    lv_imgbtn_set_src(ui_Inverterbutton, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_offb_png, NULL);
    lv_imgbtn_set_src(ui_Inverterbutton, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_offb_png, NULL);
    lv_imgbtn_set_src(ui_Inverterbutton, LV_IMGBTN_STATE_CHECKED_PRESSED, NULL, &ui_img_onb_png, NULL);
    lv_imgbtn_set_src(ui_Inverterbutton, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, &ui_img_onb_png, NULL);
    lv_obj_set_height(ui_Inverterbutton, 50);
    lv_obj_set_width(ui_Inverterbutton, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_Inverterbutton, 160);
    lv_obj_set_y(ui_Inverterbutton, -100);
    lv_obj_set_align(ui_Inverterbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Inverterbutton, LV_OBJ_FLAG_CHECKABLE);     /// Flags
    lv_obj_clear_flag(ui_Inverterbutton,
                      LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                      LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_scrollbar_mode(ui_Inverterbutton, LV_SCROLLBAR_MODE_OFF);

    ui_EnvironLabelRctn = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_EnvironLabelRctn, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_EnvironLabelRctn, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_EnvironLabelRctn, 160);
    lv_obj_set_y(ui_EnvironLabelRctn, 128);
    lv_label_set_text(ui_EnvironLabelRctn, "Temp xx.x hum xx.x");
    lv_obj_set_style_text_color(ui_EnvironLabelRctn, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_EnvironLabelRctn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SunnosunPanel = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_SunnosunPanel, 480);
    lv_obj_set_height(ui_SunnosunPanel, 40);
    lv_obj_set_align(ui_SunnosunPanel, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_SunnosunPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SunnosunPanel, lv_color_hex(0x000080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SunnosunPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_SunnosunPanel, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_SunnosunPanel, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_SunnosunPanel, lv_color_hex(0x000080), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SunnosunPanel, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_SunnosunPanel, lv_color_hex(0xFFFF00), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_SunnosunPanel, LV_GRAD_DIR_HOR, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

    ui_SliderSunnosun = lv_slider_create(ui_SunnosunPanel);
    lv_slider_set_range(ui_SliderSunnosun, 0, 500);
    lv_slider_set_value(ui_SliderSunnosun, 500, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_SliderSunnosun) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_SliderSunnosun, 0,
                                                                                                   LV_ANIM_OFF);
    lv_obj_set_width(ui_SliderSunnosun, 400);
    lv_obj_set_height(ui_SliderSunnosun, 40);
    lv_obj_set_x(ui_SliderSunnosun, -1);
    lv_obj_set_y(ui_SliderSunnosun, 0);
    lv_obj_set_align(ui_SliderSunnosun, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_SliderSunnosun,
                      LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SNAPPABLE);     /// Flags
    lv_obj_set_style_bg_color(ui_SliderSunnosun, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SliderSunnosun, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_SliderSunnosun, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SliderSunnosun, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_SliderSunnosun, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SliderSunnosun, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_SliderSunnosun, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_SliderSunnosun, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_SliderSunnosun, 500, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_SliderSunnosun, 20, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_SliderSunnosun, LV_BLEND_MODE_NORMAL, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_SliderSunnosun, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_ImageSun = lv_img_create(ui_SunnosunPanel);
    lv_img_set_src(ui_ImageSun, &ui_img_sun_png);
    lv_obj_set_width(ui_ImageSun, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ImageSun, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_ImageSun, LV_ALIGN_BOTTOM_RIGHT);
    lv_obj_add_flag(ui_ImageSun, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ImageSun, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ImageDark = lv_img_create(ui_SunnosunPanel);
    lv_img_set_src(ui_ImageDark, &ui_img_dark_png);
    lv_obj_set_width(ui_ImageDark, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ImageDark, LV_SIZE_CONTENT);    /// 1
    lv_obj_add_flag(ui_ImageDark, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ImageDark, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_LabelSunnosun = lv_label_create(ui_SunnosunPanel);
    lv_obj_set_width(ui_LabelSunnosun, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelSunnosun, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelSunnosun, 0);
    lv_obj_set_y(ui_LabelSunnosun, 12);
    lv_label_set_long_mode(ui_LabelSunnosun, LV_LABEL_LONG_DOT);
    lv_label_set_text(ui_LabelSunnosun, "00000");
    lv_obj_clear_flag(ui_LabelSunnosun, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE);      /// Flags
    lv_obj_set_style_text_color(ui_LabelSunnosun, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelSunnosun, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_LabelSunnosun, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelSunnosun, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelSunnosun, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelSunnosun, 128, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PanelChargeDisCharge = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_PanelChargeDisCharge, 480);
    lv_obj_set_height(ui_PanelChargeDisCharge, 40);
    lv_obj_set_x(ui_PanelChargeDisCharge, 0);
    lv_obj_set_y(ui_PanelChargeDisCharge, 50);
    lv_obj_set_align(ui_PanelChargeDisCharge, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_PanelChargeDisCharge, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_PanelChargeDisCharge, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_PanelChargeDisCharge, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_PanelChargeDisCharge, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_PanelChargeDisCharge, 80, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_PanelChargeDisCharge, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_PanelChargeDisCharge, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_PanelChargeDisCharge, lv_color_hex(0xFFFFFF), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_PanelChargeDisCharge, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_PanelChargeDisCharge, lv_color_hex(0x00FF00), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_PanelChargeDisCharge, 80, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_PanelChargeDisCharge, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_PanelChargeDisCharge, LV_GRAD_DIR_HOR, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

    ui_SliderChargeDischarge = lv_slider_create(ui_PanelChargeDisCharge);
    lv_slider_set_range(ui_SliderChargeDischarge, -1000, 500);
    lv_obj_set_width(ui_SliderChargeDischarge, 400);
    lv_obj_set_height(ui_SliderChargeDischarge, 40);
    lv_obj_set_align(ui_SliderChargeDischarge, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_SliderChargeDischarge,
                      LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SNAPPABLE);     /// Flags
    lv_obj_set_style_bg_color(ui_SliderChargeDischarge, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SliderChargeDischarge, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_SliderChargeDischarge, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SliderChargeDischarge, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_SliderChargeDischarge, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SliderChargeDischarge, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_SliderChargeDischarge, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_SliderChargeDischarge, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_SliderChargeDischarge, 500, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_SliderChargeDischarge, 20, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_SliderChargeDischarge, LV_BLEND_MODE_NORMAL, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_SliderChargeDischarge, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_ImageCharge = lv_img_create(ui_PanelChargeDisCharge);
    lv_img_set_src(ui_ImageCharge, &ui_img_charge_png);
    lv_obj_set_width(ui_ImageCharge, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ImageCharge, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_ImageCharge, LV_ALIGN_BOTTOM_RIGHT);
    lv_obj_add_flag(ui_ImageCharge, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ImageCharge, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Labelchargedischarge = lv_label_create(ui_PanelChargeDisCharge);
    lv_obj_set_width(ui_Labelchargedischarge, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Labelchargedischarge, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Labelchargedischarge, 20);
    lv_obj_set_y(ui_Labelchargedischarge, 12);
    lv_label_set_long_mode(ui_Labelchargedischarge, LV_LABEL_LONG_DOT);
    lv_label_set_text(ui_Labelchargedischarge, "00000");
    lv_obj_clear_flag(ui_Labelchargedischarge, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE);      /// Flags
    lv_obj_set_style_text_color(ui_Labelchargedischarge, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Labelchargedischarge, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Labelchargedischarge, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Labelchargedischarge, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Labelchargedischarge, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Labelchargedischarge, 64, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Labeltime = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Labeltime, LV_SIZE_CONTENT);   /// 20
    lv_obj_set_height(ui_Labeltime, LV_SIZE_CONTENT);    /// 10
    lv_obj_set_x(ui_Labeltime, 20);
    lv_obj_set_y(ui_Labeltime, 120);
    lv_label_set_long_mode(ui_Labeltime, LV_LABEL_LONG_DOT);
    lv_label_set_text(ui_Labeltime, "19:00");
    lv_obj_set_style_text_color(ui_Labeltime, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Labeltime, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Labeltime, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Labeltime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Labeltime, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_BatteryImage, ui_event_BatteryImage, LV_EVENT_ALL, NULL);

}
