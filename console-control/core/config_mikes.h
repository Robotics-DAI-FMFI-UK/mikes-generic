#ifndef _CONFIG_MIKES_H_
#define _CONFIG_MIKES_H_

#include "../mikes-common/config/config.h"

#define MIKES_CONFIG "console-control.cfg"

typedef struct {
    int print_all_logs_to_console;
    int print_debug_logs;
    int use_ncurses_control;
    double map_azimuth;
} mikes_config_t;

extern mikes_config_t mikes_config;

void load_config();

#endif
