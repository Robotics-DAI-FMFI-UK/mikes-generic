#include <stdio.h>

#include "config_mikes.h"
#include "../mikes-common/bites/mikes.h"

mikes_config_t mikes_config;

mikes_config_t default_mikes_config = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
mikes_config_t mikes_config;

void load_config()
{
    printf("load_config()\n");

    config_data cfg = read_config(MIKES_CONFIG);
    if (cfg == 0)
    {
       printf("Could not open config file %s\n", MIKES_CONFIG);
       program_runs = 0;
    }
    mikes_config = default_mikes_config;
    mikes_config.autostart = config_get_intval(cfg, "autostart", mikes_config.autostart);
    mikes_config.with_gui = config_get_intval(cfg, "show_gui", mikes_config.with_gui);

    mikes_config.print_all_logs_to_console = config_get_intval(cfg, "print_all_logs_to_console",
                                                               mikes_config.print_all_logs_to_console);
    mikes_config.print_debug_logs = config_get_intval(cfg, "print_debug_logs", mikes_config.print_debug_logs);

    mikes_config.use_ncurses_control = config_get_intval(cfg, "use_ncurses_control", mikes_config.use_ncurses_control);
	mikes_config.use_tim571 = config_get_intval(cfg, "use_tim571", mikes_config.use_tim571);
	mikes_config.use_t265 = config_get_intval(cfg, "use_t265", mikes_config.use_t265);
    mikes_config.use_xtion = config_get_intval(cfg, "use_xtion", mikes_config.use_xtion);
  
	mikes_config.xtion_samples_config = config_get_alloc_strval(cfg, "xtion_samples_config", "file_missing_in_config");
	
	mikes_config.gridmap_width = config_get_intval(cfg, "gridmap_width", mikes_config.gridmap_width);
	
	mikes_config.gridmap_height = config_get_intval(cfg, "gridmap_height", mikes_config.gridmap_height);

	mikes_config.map_azimuth = config_get_doubleval(cfg, "map_azimuth", mikes_config.map_azimuth);

    config_dispose(cfg);
}
