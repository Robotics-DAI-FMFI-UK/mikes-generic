#include <stdio.h>

#include "config_mikes.h"
#include "../mikes-common/bites/mikes.h"

mikes_config_t mikes_config;

void load_config()
{
    config_data cfg = read_config(MIKES_CONFIG);
    if (cfg == 0)
    {
       printf("Could not open config file %s\n", MIKES_CONFIG);
       program_runs = 0;
    }
    mikes_config.print_all_logs_to_console = config_get_intval(cfg, "print_all_logs_to_console", 0);
    mikes_config.print_debug_logs = config_get_intval(cfg, "print_debug_logs", 0);
    mikes_config.use_ncurses_control = config_get_intval(cfg, "use_ncurses_control", 0);

    mikes_config.map_azimuth = config_get_doubleval(cfg, "map_azimuth", mikes_config.map_azimuth);

    config_dispose(cfg);
}
