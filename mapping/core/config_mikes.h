#ifndef _CONFIG_MIKES_H_
#define _CONFIG_MIKES_H_

#include "../mikes-common/config/config.h"

#define MIKES_CONFIG "mapping.cfg"

typedef struct {
    int autostart;
    int with_gui;
    int print_all_logs_to_console;
    int print_debug_logs;
    int use_ncurses_control;
    int use_tim571;
    int use_xtion;
    char *xtion_samples_config;
    int gridmap_width;
    int gridmap_height;
    double map_azimuth;
} mikes_config_t;

extern mikes_config_t mikes_config;

void load_config();

#endif
