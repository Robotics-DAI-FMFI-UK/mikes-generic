#ifndef _CONFIG_MIKES_H_
#define _CONFIG_MIKES_H_

#include "../mikes-common/config/config.h"

#define MIKES_CONFIG "sick.cfg"

typedef struct {
    int autostart;
    int with_gui;
    int print_all_logs_to_console;
    int print_debug_logs;
    int use_ncurses_control;
    int start_x;
    int start_y;
    int use_rplidar;
    int use_ust10lx;
    int use_tim571;
    int use_xtion;
    int use_rfid;
    int use_tim_hough_transform;
    int use_tim_segment;
    int use_line_filter;
    int use_tim_corner;
    double map_azimuth;
    char *line_map_file;
    char *xtion_samples_config;

    int use_sick_localization;
    int use_sick_cart_align;
    int use_sick_strategy;

    double localization_start_x;
    double localization_start_y;
    double localization_start_heading;

    double localization_waiting1_x;
    double localization_waiting1_y;
    double localization_waiting1_heading;

    double localization_waiting2_x;
    double localization_waiting2_y;
    double localization_waiting2_heading;

    double localization_base1_x;
    double localization_base1_y;
    double localization_base1_heading;

    double localization_base2_x;
    double localization_base2_y;
    double localization_base2_heading;

    double localization_base3_x;
    double localization_base3_y;
    double localization_base3_heading;
} mikes_config_t;

extern mikes_config_t mikes_config;

void load_config();

#endif
